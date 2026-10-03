#pragma once

#include <cmath>
#include <cstdint>

namespace chad {
struct PIDGains {
    float kP = 0.0f;
    float kI = 0.0f;
    float kD = 0.0f;
};

class PID {
public:
    PID(float kP, float kI, float kD, float windupRange = 0.0f, bool flipReset = false);
    PID(const PIDGains& gains, float windupRange = 0.0f, bool flipReset = false);

    PIDGains getGains() const;

    void setGains(const PIDGains& gains);

    float update(float error);
    void reset();

    /// update() with the interval given rather than read off the clock.
    float step(float error, float dt);

    void setFlipReset(bool flipReset);
    void setWindupRange(float windupRange);
    float getWindupRange() const;

    void setFixedDt(float seconds);

    void setDerivativeFilter(float alpha);
    float derivativeFilter() const { return mDerivativeFilter; }

    // Cap on |kI * integral| when no windup range is set. The chassis clamps
    // its outputs to +-127, so integral past that is stored windup.
    static constexpr float kIntegralOutputLimit = 127.0f;

    // windupRange > 0 bounds the integral itself; otherwise the kI term is
    // held to kIntegralOutputLimit, and kI == 0 banks nothing.
    static float accumulateIntegral(float integral, float error, float dt,
                                    float kI, float windupRange) {
        if (windupRange > 0.0f) {
            integral += error * dt;
            if (integral > windupRange) return windupRange;
            if (integral < -windupRange) return -windupRange;
            return integral;
        }
        if (kI == 0.0f) return integral;
        const float limit = kIntegralOutputLimit / std::fabs(kI);
        integral += error * dt;
        if (integral > limit) return limit;
        if (integral < -limit) return -limit;
        return integral;
    }

private:
    void clearState();

    PIDGains mGains;
    float mWindupRange;
    bool mFlipReset;
    float mFixedDt = 0.0f;
    float mIntegral = 0.0f;
    float mPreviousError = 0.0f;
    bool mHavePreviousError = false;
    float mFilteredDerivative = 0.0f;
    float mDerivativeFilter = 0.1f;
    uint32_t mPreviousTime = 0;
};
}
