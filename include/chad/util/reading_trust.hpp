#pragma once

#include <cmath>
#include <cstdint>

namespace chad {
// Per-sensor trust in [0, 1] from how a distance sensor's readings have agreed
// with the wall the pose predicts.
//
// Disagreement accrues in milliseconds of real sample spacing, not in cycles,
// so the device rate and loop overruns do not change the time constants. Trust
// stays at 1 until the accrual passes graceMs (one stray sample costs nothing),
// then falls linearly to 0 over rampMs. It is paid back only after holdMs of
// unbroken consistent readings, a full accrual (graceMs + rampMs) per
// recoverMs, so a sensor that flickers between an object and the wall stays
// out.
//
// Short and long disagreement are not symmetric. A return short of the wall is
// what an object in the beam produces (a game element, a robot); it accrues at
// full rate, scaled up with how far short it is, so a gross one drops the
// sensor on its first sample. A return long of the wall cannot come from an
// obstruction: inside the innovation gate it is a plausible pose correction and
// is neutral, beyond it it is a beam miss (or a displacement, which the caller
// handles) and accrues at longWeight.
//
// No heap, no trig, O(1) per sample.
class ReadingTrust {
public:
    struct Config {
        /// Consistent band: |innovation| <= tolIn + tolFrac * range.
        float tolIn{2.0f};
        float tolFrac{0.05f};
        /// Long readings inside this are neutral; beyond it they accrue.
        float longTolIn{8.0f};
        float graceMs{40.0f};
        float rampMs{90.0f};
        /// Short accrual is multiplied by 1 + excess / shortSeverityIn, at most
        /// kMaxShortScale. <= 0 disables the scaling.
        float shortSeverityIn{8.0f};
        float longWeight{0.25f};
        float holdMs{150.0f};
        float recoverMs{300.0f};
        /// Below this the sensor is not used. <= 0 never excludes.
        float floor{0.5f};
    };

    enum class Verdict { None, Consistent, Short, Long };

    /// Nominal V5 refresh; the interval charged for the first sample, and half
    /// the most one sample can be charged after a gap.
    static constexpr float kSamplePeriodMs = 30.0f;
    static constexpr float kMaxShortScale = 4.0f;

    static float tolerance(float rangeIn, const Config& c) {
        return c.tolIn + c.tolFrac * (rangeIn > 0.0f ? rangeIn : 0.0f);
    }

    /// Classify one innovation (reading minus predicted wall range, inches).
    /// Non-finite input carries no information.
    static Verdict judge(float innovationIn, float rangeIn, const Config& c) {
        if (!std::isfinite(innovationIn) || !std::isfinite(rangeIn)) return Verdict::None;
        const float tol = tolerance(rangeIn, c);
        if (innovationIn < -tol) return Verdict::Short;
        if (innovationIn <= tol) return Verdict::Consistent;
        const float longTol = c.longTolIn > tol ? c.longTolIn : tol;
        return innovationIn > longTol ? Verdict::Long : Verdict::None;
    }

    /// Fold one judged sample taken at nowMs. Returns the new trust. None
    /// changes nothing, including the sample clock.
    float observe(Verdict v, float innovationIn, float rangeIn, std::uint32_t nowMs,
                  const Config& c) {
        if (v == Verdict::None) return mTrust;

        float dt = kSamplePeriodMs;
        if (mHave) {
            const std::int32_t e = static_cast<std::int32_t>(nowMs - mLastMs);
            dt = e <= 0 ? 0.0f : static_cast<float>(e);
            if (dt > 2.0f * kSamplePeriodMs) dt = 2.0f * kSamplePeriodMs;
        }
        mHave = true;
        mLastMs = nowMs;

        const float grace = c.graceMs > 0.0f ? c.graceMs : 0.0f;
        const float ramp = c.rampMs > 1.0f ? c.rampMs : 1.0f;
        const float full = grace + ramp;

        switch (v) {
            case Verdict::Consistent: {
                const float before = mGoodMs;
                mGoodMs += dt;
                const float hold = c.holdMs > 0.0f ? c.holdMs : 0.0f;
                if (mGoodMs > hold) {
                    const float paidMs = mGoodMs - (before > hold ? before : hold);
                    const float rec = c.recoverMs > 1.0f ? c.recoverMs : 1.0f;
                    mBadMs -= paidMs * full / rec;
                }
                break;
            }
            case Verdict::Short: {
                float scale = 1.0f;
                if (c.shortSeverityIn > 0.0f && std::isfinite(innovationIn)) {
                    const float excess = -innovationIn - tolerance(rangeIn, c);
                    if (excess > 0.0f) scale += excess / c.shortSeverityIn;
                    if (scale > kMaxShortScale) scale = kMaxShortScale;
                }
                mGoodMs = 0.0f;
                mBadMs += dt * scale;
                break;
            }
            case Verdict::Long:
                mGoodMs = 0.0f;
                mBadMs += dt * (c.longWeight > 0.0f ? c.longWeight : 0.0f);
                break;
            case Verdict::None:
                break;
        }

        if (mBadMs < 0.0f) mBadMs = 0.0f;
        if (mBadMs > full) mBadMs = full;
        const float t = 1.0f - (mBadMs - grace) / ramp;
        mTrust = t > 1.0f ? 1.0f : (t < 0.0f ? 0.0f : t);
        return mTrust;
    }

    float trust() const { return mTrust; }
    bool usable(const Config& c) const { return mTrust >= c.floor; }
    void reset() { *this = ReadingTrust{}; }

private:
    float mTrust{1.0f};
    float mBadMs{0.0f};
    float mGoodMs{0.0f};
    std::uint32_t mLastMs{0};
    bool mHave{false};
};

/// V5 distance sensor rated accuracy, inches: +-15 mm under 200 mm, 5 %
/// beyond. The max of the two is continuous and covers both.
inline float distanceAccuracyIn(float rangeIn) {
    constexpr float kFloorIn = 15.0f / 25.4f;
    const float prop = 0.05f * (rangeIn > 0.0f ? rangeIn : 0.0f);
    return prop > kFloorIn ? prop : kFloorIn;
}

/// Weight of one vote: trust, optionally over the rated variance at its range.
inline float readingWeight(float trust, float rangeIn, bool byRange) {
    if (!std::isfinite(trust) || trust <= 0.0f) return 0.0f;
    if (!byRange) return trust;
    const float s = distanceAccuracyIn(std::isfinite(rangeIn) ? rangeIn : 0.0f);
    return trust / (s * s);
}

/// Weighted mean of n values; the plain mean when the weights sum to nothing.
inline float weightedMean(const float* v, const float* w, int n) {
    if (n <= 0) return 0.0f;
    float sw = 0.0f, swv = 0.0f, sv = 0.0f;
    for (int k = 0; k < n; k++) {
        sv += v[k];
        const float wk = (std::isfinite(w[k]) && w[k] > 0.0f) ? w[k] : 0.0f;
        sw += wk;
        swv += wk * v[k];
    }
    return sw > 1e-12f ? swv / sw : sv / static_cast<float>(n);
}
}  // namespace chad
