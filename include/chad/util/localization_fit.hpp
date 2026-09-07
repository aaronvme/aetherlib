#pragma once

#include <cmath>

namespace chad::tuning {
constexpr float BEAM_SPAN_SIGMAS = 8.0f;

struct BeamMixture {
    float hitWeight{1.0f};
    float shortWeight{0.50f};
    float shortDecaySigmas{1.2f};
    float randWeight{0.0008f};
};

struct BeamMixtureFit {
    BeamMixture params;
    float hitFraction{0.0f};
    float shortFraction{0.0f};
    float randFraction{0.0f};
    float meanLogLik{0.0f};
    int samples{0};
    int iterations{0};
    bool valid{false};
};

inline float opposingPairSum(float fieldSpanIn, float alongOffsetA,
                            float alongOffsetB) {
    return fieldSpanIn - alongOffsetA + alongOffsetB;
}

inline float mountAlongAxis(float mountX, float mountY, float robotThetaRad,
                           bool xAxis) {
    const float c = std::cos(robotThetaRad);
    const float s = std::sin(robotThetaRad);
    return xAxis ? mountX * c - mountY * s : mountX * s + mountY * c;
}

inline float beamSigmaBase(float readingIn, float closeIn, float prop) {
    constexpr float kCloseIn = 200.0f / 25.4f;
    return readingIn <= kCloseIn ? closeIn : readingIn * prop;
}

namespace detail {
inline float erfApprox(float x) {
    const float sign = x < 0.0f ? -1.0f : 1.0f;
    const float ax = std::fabs(x);
    const float t = 1.0f / (1.0f + 0.3275911f * ax);
    const float y = 1.0f - (((((1.061405429f * t - 1.453152027f) * t) +
                              1.421413741f) * t - 0.284496736f) * t +
                            0.254829592f) * t * std::exp(-ax * ax);
    return sign * y;
}

inline float hitNorm(float span) {
    return 2.5066282746f * erfApprox(span * 0.7071067812f);
}

inline float shortNorm(float lambda, float span) {
    if (lambda <= 0.0f) return 0.0f;
    return lambda * (1.0f - std::exp(-span / lambda));
}
}

inline BeamMixtureFit fitBeamMixture(const float* u, int n,
                                     float span = BEAM_SPAN_SIGMAS,
                                     int maxIters = 200, float tol = 1e-7f) {
    BeamMixtureFit fit;
    if (u == nullptr || n < 12 || span <= 1.0f) return fit;

    const float zHit = detail::hitNorm(span);
    const float zRand = 2.0f * span;
    if (zHit <= 0.0f) return fit;

    float pHit = 0.70f, pShort = 0.28f, pRand = 0.02f;
    float lambda = 1.2f;
    float prevLL = -1e30f;

    for (int iter = 1; iter <= maxIters; iter++) {
        const float zShort = detail::shortNorm(lambda, span);
        if (zShort <= 0.0f) break;

        double sHit = 0.0, sShort = 0.0, sRand = 0.0;
        double sShortV = 0.0;
        double ll = 0.0;
        int used = 0;

        for (int k = 0; k < n; k++) {
            const float x = u[k];
            if (!std::isfinite(x) || x < -span || x > span) continue;
            const float fHit = std::exp(-0.5f * x * x) / zHit;
            const float fShort = x < 0.0f ? std::exp(x / lambda) / zShort : 0.0f;
            const float fRand = 1.0f / zRand;

            const float aHit = pHit * fHit;
            const float aShort = pShort * fShort;
            const float aRand = pRand * fRand;
            const float tot = aHit + aShort + aRand;
            if (!(tot > 0.0f)) continue;

            sHit += aHit / tot;
            sShort += aShort / tot;
            sRand += aRand / tot;
            sShortV += static_cast<double>(aShort / tot) * (-x);
            ll += std::log(tot);
            used++;
        }
        if (used < 12) return fit;

        const double inv = 1.0 / static_cast<double>(used);
        pHit = static_cast<float>(sHit * inv);
        pShort = static_cast<float>(sShort * inv);
        pRand = static_cast<float>(sRand * inv);

        if (sShort > 1e-6) {
            float lam = static_cast<float>(sShortV / sShort);
            for (int p = 0; p < 2; p++) {
                if (lam <= 1e-4f) break;
                const float r = span / lam;
                const float e = std::exp(-r);
                lam = static_cast<float>(sShortV / sShort) + span * e / (1.0f - e);
            }
            lambda = lam;
        }
        if (lambda < 0.05f) lambda = 0.05f;
        if (lambda > span) lambda = span;

        fit.meanLogLik = static_cast<float>(ll * inv);
        fit.samples = used;
        fit.iterations = iter;
        if (std::fabs(fit.meanLogLik - prevLL) < tol) break;
        prevLL = fit.meanLogLik;
    }
    if (fit.samples < 12 || pHit <= 1e-6f) return fit;

    const float zShort = detail::shortNorm(lambda, span);
    fit.hitFraction = pHit;
    fit.shortFraction = pShort;
    fit.randFraction = pRand;
    fit.params.hitWeight = 1.0f;
    fit.params.shortDecaySigmas = lambda;
    fit.params.shortWeight =
        zShort > 0.0f ? (pShort / pHit) * (zHit / zShort) : 0.0f;
    fit.params.randWeight = (pRand / pHit) * (zHit / zRand);
    fit.valid = true;
    return fit;
}

inline BeamMixture perBeamFromPair(const BeamMixture& pair) {
    BeamMixture out = pair;
    out.shortDecaySigmas = pair.shortDecaySigmas * 1.41421356f;
    return out;
}

struct SigmaCalibration {
    float scale{1.0f};
    float floorIn{0.0f};
    float meanNees{0.0f};
    int samples{0};
    bool valid{false};
};

inline SigmaCalibration calibrateMclSigma(const float* reported,
                                          const float* errorIn, int n,
                                          float floorQuantile = 0.9f) {
    SigmaCalibration cal;
    if (reported == nullptr || errorIn == nullptr || n < 8) return cal;

    double acc = 0.0;
    int used = 0;
    float sorted[128];
    int m = 0;
    for (int k = 0; k < n; k++) {
        const float s = reported[k];
        const float e = errorIn[k];
        if (!std::isfinite(s) || !std::isfinite(e) || s <= 1e-4f) continue;
        acc += static_cast<double>(e) * e / (static_cast<double>(s) * s);
        used++;
        if (m < 128) sorted[m++] = std::fabs(e);
    }
    if (used < 8 || m < 8) return cal;

    cal.meanNees = static_cast<float>(acc / used);
    cal.scale = std::sqrt(cal.meanNees);
    if (cal.scale < 1.0f) cal.scale = 1.0f;

    for (int i = 1; i < m; i++) {
        const float x = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > x) { sorted[j + 1] = sorted[j]; j--; }
        sorted[j + 1] = x;
    }
    float q = floorQuantile;
    if (q < 0.0f) q = 0.0f;
    if (q > 1.0f) q = 1.0f;
    int idx = static_cast<int>(q * static_cast<float>(m - 1) + 0.5f);
    if (idx < 0) idx = 0;
    if (idx >= m) idx = m - 1;
    cal.floorIn = sorted[idx];
    cal.samples = used;
    cal.valid = true;
    return cal;
}

struct RecoveryTrial {
    float settleMs{0.0f};
    float finalErrorIn{0.0f};
    bool converged{false};
};

struct RecoveryScore {
    float medianMs{0.0f};
    float worstMs{0.0f};
    float medianErrorIn{0.0f};
    int converged{0};
    int trials{0};
    bool valid{false};
};

inline RecoveryScore scoreRecovery(const RecoveryTrial* t, int n) {
    RecoveryScore s;
    if (t == nullptr || n <= 0) return s;

    float ms[32], err[32];
    int m = 0;
    for (int k = 0; k < n && m < 32; k++) {
        s.trials++;
        if (!t[k].converged) continue;
        s.converged++;
        ms[m] = t[k].settleMs;
        err[m] = t[k].finalErrorIn;
        m++;
    }
    if (m == 0) return s;

    for (int i = 1; i < m; i++) {
        const float a = ms[i], b = err[i];
        int j = i - 1;
        while (j >= 0 && ms[j] > a) { ms[j + 1] = ms[j]; err[j + 1] = err[j]; j--; }
        ms[j + 1] = a; err[j + 1] = b;
    }
    s.medianMs = (m % 2 == 1) ? ms[m / 2] : 0.5f * (ms[m / 2 - 1] + ms[m / 2]);
    s.worstMs = ms[m - 1];

    for (int i = 1; i < m; i++) {
        const float x = err[i];
        int j = i - 1;
        while (j >= 0 && err[j] > x) { err[j + 1] = err[j]; j--; }
        err[j + 1] = x;
    }
    s.medianErrorIn =
        (m % 2 == 1) ? err[m / 2] : 0.5f * (err[m / 2 - 1] + err[m / 2]);
    s.valid = true;
    return s;
}

inline bool recoveryBetterThan(const RecoveryScore& a, const RecoveryScore& b) {
    if (!a.valid) return false;
    if (!b.valid) return true;
    const float ra = static_cast<float>(a.converged) / static_cast<float>(a.trials);
    const float rb = static_cast<float>(b.converged) / static_cast<float>(b.trials);
    if (std::fabs(ra - rb) > 0.01f) return ra > rb;
    return a.medianMs < b.medianMs;
}

struct WallSolve {
    float value{0.0f};
    float bracketIn{0.0f};
    bool trusted{false};
};

inline float solveAxisSingle(float readingIn, float dirComponent,
                            float alongOffset, float halfSpanIn) {
    const float wall = dirComponent >= 0.0f ? halfSpanIn : -halfSpanIn;
    return wall - readingIn * dirComponent - alongOffset;
}

inline WallSolve solveAxisBracketed(float readingPlus, float dirPlus,
                                    float alongPlus, float readingMinus,
                                    float dirMinus, float alongMinus,
                                    float halfSpanIn, float tolIn) {
    WallSolve out;
    const float hi = halfSpanIn - readingPlus * dirPlus - alongPlus;
    const float lo = -halfSpanIn - readingMinus * dirMinus - alongMinus;
    out.bracketIn = hi - lo;
    out.value = 0.5f * (hi + lo);
    out.trusted = std::fabs(out.bracketIn) <= tolIn;
    return out;
}

constexpr int MAX_BIAS_GROUPS = 8;

struct BiasLeg {
    float lengthIn{0.0f};
    float alongErrIn{0.0f};
    float crossErrIn{0.0f};
};

struct BiasFit {
    float vertBiasProp{0.0f};
    float slipBiasProp{0.0f};

    float alongPerRootIn{0.0f};
    float crossPerRootIn{0.0f};
    float growthExponent{0.0f};
    int legs{0};
    int groups{0};
    bool valid{false};
    bool shapeSuspect{false};
};

inline BiasFit fitOdomBias(const BiasLeg* legs, int n, float biasDecorrIn) {
    BiasFit fit;
    if (legs == nullptr || n < 4 || biasDecorrIn <= 0.0f) return fit;

    double sAlong = 0.0, sCross = 0.0;
    int used = 0;

    float glen[MAX_BIAS_GROUPS] = {};
    double gAlong[MAX_BIAS_GROUPS] = {};
    int gN[MAX_BIAS_GROUPS] = {};
    int groups = 0;

    for (int k = 0; k < n; k++) {
        const float L = legs[k].lengthIn;
        const float a = legs[k].alongErrIn;
        const float c = legs[k].crossErrIn;
        if (!(L > 0.5f) || !std::isfinite(a) || !std::isfinite(c)) continue;
        sAlong += static_cast<double>(a) * a / L;
        sCross += static_cast<double>(c) * c / L;
        used++;

        int g = -1;
        for (int i = 0; i < groups; i++) {
            if (std::fabs(glen[i] - L) < 0.5f) { g = i; break; }
        }
        if (g < 0 && groups < MAX_BIAS_GROUPS) {
            g = groups++;
            glen[g] = L;
        }
        if (g >= 0) {
            gAlong[g] += static_cast<double>(a) * a;
            gN[g]++;
        }
    }
    if (used < 4) return fit;

    const double invN = 1.0 / static_cast<double>(used);
    fit.alongPerRootIn = static_cast<float>(std::sqrt(sAlong * invN));
    fit.crossPerRootIn = static_cast<float>(std::sqrt(sCross * invN));

    const float denom = std::sqrt(biasDecorrIn);
    fit.vertBiasProp = fit.alongPerRootIn / denom;
    fit.slipBiasProp = fit.crossPerRootIn / denom;
    fit.legs = used;
    fit.groups = groups;

    int fitted = 0;
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (int i = 0; i < groups; i++) {
        if (gN[i] < 2) continue;
        const double rms = std::sqrt(gAlong[i] / gN[i]);
        if (!(rms > 0.0)) continue;
        const double x = std::log(glen[i]);
        const double y = std::log(rms);
        sx += x; sy += y; sxx += x * x; sxy += x * y;
        fitted++;
    }
    if (fitted >= 2) {
        const double det = fitted * sxx - sx * sx;
        if (std::fabs(det) > 1e-9) {
            fit.growthExponent =
                static_cast<float>((fitted * sxy - sx * sy) / det);
            fit.shapeSuspect = std::fabs(fit.growthExponent - 0.5f) > 0.25f;
        }
    }
    fit.valid = fit.vertBiasProp > 0.0f;
    return fit;
}
}
