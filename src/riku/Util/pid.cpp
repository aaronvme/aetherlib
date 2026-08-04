// Ported from FBLib - https://github.com/jiazegao/FBLib
#include "riku/Util/Util.hpp"
#include "riku/Util/pid.hpp"
#include "pros/rtos.hpp"

using namespace riku;
    // PID 
PID::PID(float kP,          // Proportional gain
         float kI,          // Integral gain
         float kD,          // Derivative gain
         float windupRange, // Anti-windup clamp limit
         bool flipReset)    // Negate vs. zero integral on reset
    : mGains{kP, kI, kD}, mWindupRange(windupRange), mFlipReset(flipReset), mPreviousTime(pros::millis()) {}

    PID::PID(const PIDGains& gains, float windupRange, bool flipReset)
        : mGains(gains), mWindupRange(windupRange), mFlipReset(flipReset), mPreviousTime(pros::millis()) {}

    PIDGains PID::getGains() const {
        return mGains;
    }

    void PID::setGains(const PIDGains& gains) {
        mGains = gains;
    }

    float PID::update(float error) {
        uint32_t currentTime = pros::millis();
        float dt = (currentTime - mPreviousTime) / 1000.0f; 

        if (dt > 0.1f) dt = 0.1f;
        mPreviousTime = currentTime;

        if (mFixedDt > 0.0f) dt = mFixedDt;

        // proportional
        float p = mGains.kP * error;

        // integral 
        mIntegral += error * dt;
        if (mWindupRange > 0.0f) {
            if (mIntegral > mWindupRange) {
                mIntegral = mWindupRange;
            } else if (mIntegral < -mWindupRange) {
                mIntegral = -mWindupRange;
            }
        }
        float i = mGains.kI * mIntegral;

        // derivative 
        float d = 0.0f;
        if (dt > 1e-6f) {  
            float derivative = (error - mPreviousError) / dt;

            mFilteredDerivative = mDerivativeFilter * derivative + (1.0f - mDerivativeFilter) * mFilteredDerivative;
            d = mGains.kD * mFilteredDerivative;
        }


        mPreviousError = error;

        return p + i + d;
    }

    void PID::reset() {
        if (mFlipReset) {
            // Negate the integral so it opposes the opposite direction
            mIntegral = -mIntegral;
        } else {
            mIntegral = 0.0f;
        }

        mPreviousError = 0.0f;
        mFilteredDerivative = 0.0f;
        mPreviousTime = pros::millis();  // Reset time to current
    }

    void PID::setFlipReset(bool flipReset) {
        mFlipReset = flipReset;
    }

    void PID::setWindupRange(float windupRange) {
        mWindupRange = windupRange;
    }

    void PID::setFixedDt(float seconds) {
        mFixedDt = seconds;
    }

    float PID::getWindupRange() const {
        return mWindupRange;
    }
    