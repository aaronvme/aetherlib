#include "chad/util/util.hpp"
#include "chad/util/pid.hpp"
#include "pros/rtos.hpp"

using namespace chad;

PID::PID(float kP,
         float kI,
         float kD,
         float windupRange,
         bool flipReset)
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

        float p = mGains.kP * error;

        mIntegral += error * dt;
        if (mWindupRange > 0.0f) {
            if (mIntegral > mWindupRange) {
                mIntegral = mWindupRange;
            } else if (mIntegral < -mWindupRange) {
                mIntegral = -mWindupRange;
            }
        }
        float i = mGains.kI * mIntegral;

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
            mIntegral = -mIntegral;
        } else {
            mIntegral = 0.0f;
        }

        mPreviousError = 0.0f;
        mFilteredDerivative = 0.0f;
        mPreviousTime = pros::millis();
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

    void PID::setDerivativeFilter(float alpha) {
        if (alpha < 0.01f) alpha = 0.01f;
        if (alpha > 1.0f) alpha = 1.0f;
        mDerivativeFilter = alpha;
    }

    float PID::getWindupRange() const {
        return mWindupRange;
    }
