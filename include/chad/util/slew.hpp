#pragma once

#include <cmath>

namespace chad {
class SlewLimiter {
public:
    SlewLimiter() = default;
    SlewLimiter(float risePerSec, float fallPerSec)
        : mRise(risePerSec), mFall(fallPerSec) {}

    void setRates(float risePerSec, float fallPerSec) {
        mRise = risePerSec;
        mFall = fallPerSec;
    }

    float update(float target, float dt) {
        if (dt <= 0.0f) return mValue;
        float budget = dt;
        for (int seg = 0; seg < 2; ++seg) {
            const float delta = target - mValue;
            if (delta == 0.0f) break;
            const bool toZero = (mValue > 0.0f && delta < 0.0f) ||
                                (mValue < 0.0f && delta > 0.0f);
            const float rate = toZero ? mFall : mRise;

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
}
