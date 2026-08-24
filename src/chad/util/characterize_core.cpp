#include "chad/util/characterize_core.hpp"

#include <cmath>
#include <limits>

namespace chad::tuning {

void FeedforwardAccumulator::reset() {
    *this = FeedforwardAccumulator{};
}

void FeedforwardAccumulator::add(float speed, float accel, float volts) {
    const double x[3] = {1.0, static_cast<double>(speed),
                         static_cast<double>(accel)};
    const double y = static_cast<double>(volts);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) mXtX[i][j] += x[i] * x[j];
        mXty[i] += x[i] * y;
    }
    mYty += y * y;
    mSy += y;
    mN++;
}

bool FeedforwardAccumulator::solve(FeedforwardFitStats& out) const {
    out = FeedforwardFitStats{};
    out.samples = mN;
    if (mN < kMinSamples) return false;

    // [XtX | I | Xty], so one elimination yields both the fit and the
    // covariance factor the standard errors need.
    double aug[3][7] = {};
    double scale = 0.0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) aug[i][j] = mXtX[i][j];
        aug[i][3 + i] = 1.0;
        aug[i][6] = mXty[i];
        if (mXtX[i][i] > scale) scale = mXtX[i][i];
    }
    if (!(scale > 0.0)) return false;

    for (int c = 0; c < 3; c++) {
        int piv = c;
        for (int r = c + 1; r < 3; r++) {
            if (std::fabs(aug[r][c]) > std::fabs(aug[piv][c])) piv = r;
        }
        // Relative to the largest diagonal: XtX entries grow with the sample
        // count and with speed squared, so a fixed floor is a different test on
        // every run.
        if (std::fabs(aug[piv][c]) < 1e-12 * scale) return false;
        if (piv != c) {
            for (int k = 0; k < 7; k++) {
                const double t = aug[c][k];
                aug[c][k] = aug[piv][k];
                aug[piv][k] = t;
            }
        }
        const double d = aug[c][c];
        for (int k = 0; k < 7; k++) aug[c][k] /= d;
        for (int r = 0; r < 3; r++) {
            if (r == c) continue;
            const double f = aug[r][c];
            for (int k = 0; k < 7; k++) aug[r][k] -= f * aug[c][k];
        }
    }

    const double beta[3] = {aug[0][6], aug[1][6], aug[2][6]};
    out.kS = static_cast<float>(beta[0]);
    out.kV = static_cast<float>(beta[1]);
    out.kA = static_cast<float>(beta[2]);

    const double n = static_cast<double>(mN);
    const double ssTot = mYty - mSy * mSy / n;
    double ssRes = mYty;
    for (int i = 0; i < 3; i++) ssRes -= beta[i] * mXty[i];
    if (ssRes < 0.0) ssRes = 0.0;
    out.r2 = ssTot > 0.0 ? static_cast<float>(1.0 - ssRes / ssTot) : 0.0f;

    const double sigma2 = ssRes / (n - 3.0);
    const double se[3] = {std::sqrt(sigma2 * aug[0][3]),
                          std::sqrt(sigma2 * aug[1][4]),
                          std::sqrt(sigma2 * aug[2][5])};
    out.seKS = static_cast<float>(se[0]);
    out.seKV = static_cast<float>(se[1]);
    out.seKA = static_cast<float>(se[2]);
    // An exact fit leaves no residual to scale the standard error by, which is
    // kA at its most identified, not its least.
    out.kaT = se[2] > 0.0
                  ? static_cast<float>(std::fabs(beta[2]) / se[2])
                  : std::numeric_limits<float>::infinity();

    out.solved = true;
    return true;
}

RampSampler::RampSampler(int span, float dtSec) : mSpan(1), mDt(dtSec) {
    int s = span;
    if (s < 1) s = 1;
    if (s > kMaxSpan) s = kMaxSpan;
    mSpan = s;
}

void RampSampler::reset() {
    mCount = 0;
}

bool RampSampler::push(float speed, float volts, Sample& out) {
    if (mCount == mSpan + 1) {
        for (int i = 0; i < mSpan; i++) {
            mSpeed[i] = mSpeed[i + 1];
            mVolts[i] = mVolts[i + 1];
        }
        mSpeed[mSpan] = speed;
        mVolts[mSpan] = volts;
    } else {
        mSpeed[mCount] = speed;
        mVolts[mCount] = volts;
        mCount++;
        if (mCount < mSpan + 1) return false;
    }
    // Trapezoid, matching the integral the window means come from.
    double sv = 0.5 * (mSpeed[0] + mSpeed[mSpan]);
    double su = 0.5 * (mVolts[0] + mVolts[mSpan]);
    for (int i = 1; i < mSpan; i++) {
        sv += mSpeed[i];
        su += mVolts[i];
    }
    const double w = static_cast<double>(mSpan);
    out.speed = static_cast<float>(sv / w);
    out.volts = static_cast<float>(su / w);
    out.accel = (mSpeed[mSpan] - mSpeed[0]) /
                (mDt * static_cast<float>(mSpan));
    return true;
}

}  // namespace chad::tuning
