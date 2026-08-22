#pragma once

#include <cmath>

namespace chad {

// Rate limit on a command, in units per second.
//
// Asymmetric on purpose: a drive may accelerate slowly to keep the wheels from
// breaking traction while still being allowed to brake instantly, and a limit
// that applied both ways would make every stop longer than it needs to be.
//
// Measured worse than no limit on every path tried, and it cannot be validated
// where it would matter -- the simulator has no traction to break. Shaping
// acceleration in the velocity profile, where it is expressed in real units
// ahead of the controller, beat rate limiting the voltage behind it.
class SlewLimiter {
public:
    SlewLimiter() = default;
    SlewLimiter(float risePerSec, float fallPerSec)
        : mRise(risePerSec), mFall(fallPerSec) {}

    void setRates(float risePerSec, float fallPerSec) {
        mRise = risePerSec;
        mFall = fallPerSec;
    }

    /// 0 or negative disables that direction.
    ///
    /// A reversal is a brake to zero followed by an acceleration away, so it is
    /// governed by both rates in turn -- hence the two segments. Classifying the
    /// whole step by relative magnitude instead waves +50 -> -40 straight
    /// through while rate limiting the harmless +50 -> 0.
    float update(float target, float dt) {
        if (dt <= 0.0f) return mValue;
        float budget = dt;
        for (int seg = 0; seg < 2; ++seg) {
            const float delta = target - mValue;
            if (delta == 0.0f) break;
            const bool toZero = (mValue > 0.0f && delta < 0.0f) ||
                                (mValue < 0.0f && delta > 0.0f);
            const float rate = toZero ? mFall : mRise;
            // Reversals stop at zero; the opposite rate governs what follows.
            const float limit = (toZero && std::fabs(delta) > std::fabs(mValue))
                                ? -mValue : delta;
            if (rate <= 0.0f) { mValue += limit; continue; }
            if (budget <= 0.0f) break;
            const float step = rate * budget;
            if (std::fabs(limit) <= step) {
                mValue += limit;
                budget -= std::fabs(limit) / rate;
            } else {
                mValue += (limit > 0.0f) ? step : -step;
                budget = 0.0f;
            }
        }
        return mValue;
    }

    void reset(float value = 0.0f) { mValue = value; }
    float value() const { return mValue; }

private:
    float mRise{0.0f};
    float mFall{0.0f};
    float mValue{0.0f};
};

}  // namespace chad
