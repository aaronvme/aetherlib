#pragma once

#include <cmath>
#include <cstdint>

#include "chad/util/util.hpp"

namespace chad {
// Mean per-sample travel of a group of motor encoders that can drop out
// (PROS_ERR_F from an unplugged motor) or re-zero (a motor that browns out
// comes back at 0). Differencing the mean of absolute positions steps by
// about (L - R) / n whenever the set of readable encoders changes, and the
// sides of a drive are thousands of degrees apart after a few turns. Averaging
// each encoder's own delta, over encoders readable in both samples, does not.
class EncoderDeltaMean {
public:
    static constexpr int kMaxEncoders = 8;

    // A V5 motor's output shaft tops out at 600 rpm on the blue cartridge.
    static constexpr double kMaxMotorDegPerSec = 600.0 * 6.0;
    // A tracking wheel at 6 in offset turning at the gyro's limit, plus drive
    // speed, stays under this.
    static constexpr double kMaxWheelInPerSec = 200.0;

    // Device telemetry arrives on its own ~10 ms cadence, so one sample can
    // carry up to a period more motion than the elapsed time suggests. Twice
    // the top rate covers being shoved.
    static double maxStep(double maxRatePerSec, float dtSec) {
        const double dt = (dtSec > 0.0f) ? dtSec : 0.0;
        return 2.0 * maxRatePerSec * (dt + 0.01);
    }
    static double maxStepDeg(float dtSec) { return maxStep(kMaxMotorDegPerSec, dtSec); }
    static double maxStepIn(float dtSec) { return maxStep(kMaxWheelInPerSec, dtSec); }

    static bool readable(double v) {
        return std::isfinite(v) && std::fabs(v) < 2147483647.0;
    }

    void rebase(const double* pos, int n) {
        mCount = (n < kMaxEncoders) ? ((n > 0) ? n : 0) : kMaxEncoders;
        for (int i = 0; i < mCount; i++) {
            mHave[i] = readable(pos[i]);
            if (mHave[i]) mPrev[i] = pos[i];
        }
        mContributing = 0;
    }

    // Mean change since the last sample over encoders readable in both whose
    // step is within maxStep. An encoder coming back only rebases. 0 when no
    // encoder qualifies; contributing() tells that apart from standing still.
    double update(const double* pos, int n, double maxStep) {
        if (n > kMaxEncoders) n = kMaxEncoders;
        if (n != mCount) {
            rebase(pos, n);
            return 0.0;
        }
        double sum = 0.0;
        int count = 0;
        for (int i = 0; i < n; i++) {
            if (!readable(pos[i])) {
                mHave[i] = false;
                continue;
            }
            if (mHave[i]) {
                const double d = pos[i] - mPrev[i];
                if (std::fabs(d) <= maxStep) {
                    sum += d;
                    count++;
                } else {
                    mRejected++;
                }
            }
            mPrev[i] = pos[i];
            mHave[i] = true;
        }
        mContributing = count;
        return (count > 0) ? sum / count : 0.0;
    }

    int contributing() const { return mContributing; }
    std::uint32_t rejectedSteps() const { return mRejected; }

private:
    double mPrev[kMaxEncoders]{};
    bool mHave[kMaxEncoders]{};
    int mCount{0};
    int mContributing{0};
    std::uint32_t mRejected{0};
};

// Centre travel of a differential drive. arc is (trackWidth / 2) * dTheta,
// CCW positive; a side with nothing readable is rebuilt from the other side
// and the heading change rather than letting one side's arc stand in for the
// centre during a turn.
inline double differentialCentreTravel(double left, bool leftValid,
                                       double right, bool rightValid,
                                       double arc) {
    if (leftValid && rightValid) return 0.5 * (left + right);
    if (leftValid) return left + arc;
    if (rightValid) return right - arc;
    return 0.0;
}

// Field heading from an IMU that can drop out and restart at zero after a
// brown-out. A step farther than the gyro can turn in the time since the last
// good read, or any reading after a gap long enough for a restart, re-anchors
// the offset so the heading carries on from the last accepted value. A short
// gap (a missed read) is bridged: the turn made across it is kept.
struct ImuHeadingGuard {
    // The V5 gyro saturates near 1000 deg/s; ScaledIMU adds a percent or so.
    static constexpr float kMaxTurnRadPerSec = 1100.0f * DEG_TO_RAD;
    // One IMU data period: a sample can be that much fresher than the last.
    static constexpr float kSampleSlackSec = 0.01f;
    // A brown-out restart recalibrates for about two seconds; a few missed
    // reads cannot hide one.
    static constexpr float kMaxBridgedGapSec = 0.1f;

    float offsetRad{0.0f};      // raw + offset = field heading
    float headingRad{0.0f};     // last accepted field heading
    bool haveRaw{false};        // a finite raw read has been seen in this frame
    float sinceRawSec{0.0f};    // time since that read

    // Take rawRad as the reference for the current headingRad.
    void seed(float rawRad) {
        haveRaw = std::isfinite(rawRad);
        sinceRawSec = 0.0f;
        if (haveRaw) headingRad = wrapRad(rawRad + offsetRad);
    }

    // A non-finite fieldRad is ignored.
    void anchor(float rawRad, float fieldRad) {
        if (!std::isfinite(fieldRad)) return;
        headingRad = wrapRad(fieldRad);
        haveRaw = std::isfinite(rawRad);
        sinceRawSec = 0.0f;
        if (haveRaw) offsetRad = wrapRad(fieldRad - rawRad);
    }

    // False when rawRad was unreadable or re-anchored rather than followed.
    bool update(float rawRad, float dtSec) {
        const float dt = (dtSec > 0.0f) ? dtSec : 0.0f;
        sinceRawSec += dt;
        if (!std::isfinite(rawRad)) return false;

        const float candidate = wrapRad(rawRad + offsetRad);
        const bool plausible =
            haveRaw && sinceRawSec <= kMaxBridgedGapSec &&
            std::fabs(wrapRad(candidate - headingRad)) <=
                kMaxTurnRadPerSec * (sinceRawSec + kSampleSlackSec);
        haveRaw = true;
        sinceRawSec = 0.0f;
        if (!plausible) {
            offsetRad = wrapRad(headingRad - rawRad);
            return false;
        }
        headingRad = candidate;
        return true;
    }
};
}  // namespace chad
