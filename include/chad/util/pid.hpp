#pragma once

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

    void setFlipReset(bool flipReset);
    void setWindupRange(float windupRange);
    float getWindupRange() const;

    void setFixedDt(float seconds);

    void setDerivativeFilter(float alpha);
    float derivativeFilter() const { return mDerivativeFilter; }

private:
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
