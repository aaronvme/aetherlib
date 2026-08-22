// Ported from FBLib - https://github.com/jiazegao/FBLib
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

    /// Smoothing on the derivative term: 1 = raw, small = heavily damped. The
    /// EMA's time constant is dt/alpha, so 0.1 at a 10 ms tick lags the true
    /// derivative by ~100 ms -- long enough that kD cannot brake a fast turn
    /// until it is already past the target.
    void setDerivativeFilter(float alpha);
    float derivativeFilter() const { return mDerivativeFilter; }

private:
    PIDGains mGains;
    float mWindupRange;
    bool mFlipReset;
    float mFixedDt = 0.0f;
    float mIntegral = 0.0f;
    float mPreviousError = 0.0f;
    float mFilteredDerivative = 0.0f;
    float mDerivativeFilter = 0.1f;  
    uint32_t mPreviousTime = 0; 
};
}
