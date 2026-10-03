#include <cmath>

#include "chad/util/util.hpp"
#include "chad/util/pid.hpp"

using namespace chad;

PID::PID(float kP,
         float kI,
         float kD,
         float windupRange,
         bool flipReset)
    : mGains{kP, kI, kD}, mWindupRange(windupRange), mFlipReset(flipReset) {}

    PID::PID(const PIDGains& gains, float windupRange, bool flipReset)
        : mGains(gains), mWindupRange(windupRange), mFlipReset(flipReset) {}

    PIDGains PID::getGains() const {
        return mGains;
    }

    void PID::setGains(const PIDGains& gains) {
        mGains = gains;
    }

    float PID::step(float error, float dt) {
        // A NaN error would sit in the integral and derivative state forever.
        if (!std::isfinite(error)) return 0.0f;

        float p = mGains.kP * error;

        mIntegral = accumulateIntegral(mIntegral, error, dt, mGains.kI, mWindupRange);
        float i = mGains.kI * mIntegral;

        float d = 0.0f;
        if (dt > 1e-6f && mHavePreviousError) {
            float derivative = (error - mPreviousError) / dt;

            mFilteredDerivative = mDerivativeFilter * derivative + (1.0f - mDerivativeFilter) * mFilteredDerivative;
            d = mGains.kD * mFilteredDerivative;
        }

        // mPreviousTime only advances on a real tick, so the baseline has
        // to as well or the next derivative divides by the wrong interval.
        if (dt > 1e-6f) {
            mPreviousError = error;
            mHavePreviousError = true;
        }

        return p + i + d;
    }

    void PID::clearState() {
        if (mFlipReset) {
            mIntegral = -mIntegral;
        } else {
            mIntegral = 0.0f;
        }

        mPreviousError = 0.0f;
        // The first error after a reset is the whole distance to the target.
        // Differentiated against a zero nothing measured, it spikes kD into
        // the clamp on tick one of every motion.
        mHavePreviousError = false;
        mFilteredDerivative = 0.0f;
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
