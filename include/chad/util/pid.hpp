// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include <cmath>
#include <cstdint>

namespace chad {

struct PIDGains {
    float kP = 0.0f;
    float kI = 0.0f;
    float kD = 0.0f;
};

/// The loop itself, with time passed in. No clock, no hardware, so it can be
/// exercised off-robot; `PID` below is the same loop against the PROS clock.
class PidCore {
public:
    PidCore() = default;
    explicit PidCore(const PIDGains& gains, float integralClamp = 0.0f,
                     bool flipReset = false)
        : mGains(gains), mIntegralClamp(integralClamp), mFlipReset(flipReset) {}

    float update(float error, float dt) {
        // A dead sensor reports PROS_ERR_F; integrating it poisons the loop for
        // the rest of the motion.
        if (!std::isfinite(error)) return mOutput;

        if (mResetOnSignFlip && mPrimed && error * mPreviousError < 0.0f) mIntegral = 0.0f;

        const float p = mGains.kP * error;

        if (dt > 0.0f && (mIntegralZone <= 0.0f || std::fabs(error) <= mIntegralZone)) {
            mIntegral += error * dt;
            if (mIntegralClamp > 0.0f) {
                if (mIntegral > mIntegralClamp) mIntegral = mIntegralClamp;
                if (mIntegral < -mIntegralClamp) mIntegral = -mIntegralClamp;
            }
        }
        const float i = mGains.kI * mIntegral;

        // The first sample has nothing to difference against. Differencing
        // against a zeroed previous error reports error/dt, which on a 24 inch
        // move is a kD spike of thousands on tick one.
        if (dt > 1e-6f) {
            const float raw = mPrimed ? (error - mPreviousError) / dt : 0.0f;
            mFilteredDerivative =
                mDerivativeFilter * raw + (1.0f - mDerivativeFilter) * mFilteredDerivative;
        }
        const float d = mGains.kD * mFilteredDerivative;

        mPreviousError = error;
        mPrimed = true;

        float out = p + i + d;
        if (mOutputLimit > 0.0f) {
            if (out > mOutputLimit) out = mOutputLimit;
            if (out < -mOutputLimit) out = -mOutputLimit;
        }
        mOutput = out;
        return out;
    }

    void reset() {
        // Flipping rather than zeroing keeps the accumulated term working
        // against the direction the next motion travels.
        mIntegral = mFlipReset ? -mIntegral : 0.0f;
        mPreviousError = 0.0f;
        mFilteredDerivative = 0.0f;
        mOutput = 0.0f;
        mPrimed = false;
    }

    PIDGains getGains() const { return mGains; }
    void setGains(const PIDGains& gains) { mGains = gains; }

    /// Hard clamp on the accumulator, in error-seconds. <= 0 disables.
    void setIntegralClamp(float clamp) { mIntegralClamp = clamp; }
    float integralClamp() const { return mIntegralClamp; }

    /// Only accumulate inside this error magnitude, so the integrator does its
    /// job near the target instead of winding up across the whole approach.
    /// <= 0 always accumulates.
    void setIntegralZone(float zone) { mIntegralZone = zone; }
    float integralZone() const { return mIntegralZone; }

    /// Saturate the returned output. <= 0 leaves it unbounded.
    void setOutputLimit(float limit) { mOutputLimit = limit; }
    float outputLimit() const { return mOutputLimit; }

    /// Dump the integral when the error changes sign, i.e. on overshoot.
    void setResetIntegralOnSignFlip(bool enable) { mResetOnSignFlip = enable; }

    void setFlipReset(bool flipReset) { mFlipReset = flipReset; }

    /// Smoothing on the derivative term: 1 = raw, small = heavily damped. The
    /// EMA's time constant is dt/alpha, so 0.1 at a 10 ms tick lags the true
    /// derivative by ~100 ms -- long enough that kD cannot brake a fast turn
    /// until it is already past the target.
    void setDerivativeFilter(float alpha) {
        mDerivativeFilter = alpha < 0.01f ? 0.01f : (alpha > 1.0f ? 1.0f : alpha);
    }
    float derivativeFilter() const { return mDerivativeFilter; }

    float integral() const { return mIntegral; }
    float derivative() const { return mFilteredDerivative; }
    float output() const { return mOutput; }

private:
    PIDGains mGains{};
    float mIntegralClamp{0.0f};
    float mIntegralZone{0.0f};
    float mOutputLimit{0.0f};
    bool mFlipReset{false};
    bool mResetOnSignFlip{false};

    float mIntegral{0.0f};
    float mPreviousError{0.0f};
    float mFilteredDerivative{0.0f};
    float mDerivativeFilter{0.1f};
    float mOutput{0.0f};
    bool mPrimed{false};
};

/// PidCore timed against the PROS clock.
class PID {
public:
    PID(float kP, float kI, float kD, float windupRange = 0.0f, bool flipReset = false);
    PID(const PIDGains& gains, float windupRange = 0.0f, bool flipReset = false);

    PIDGains getGains() const { return mCore.getGains(); }
    void setGains(const PIDGains& gains) { mCore.setGains(gains); }

    float update(float error);
    void reset();

    void setFlipReset(bool flipReset) { mCore.setFlipReset(flipReset); }
    void setWindupRange(float windupRange) { mCore.setIntegralClamp(windupRange); }
    float getWindupRange() const { return mCore.integralClamp(); }

    /// Ignore the measured tick length and use this instead. Zero uses the
    /// clock. Only for replaying a fixed-step trace.
    void setFixedDt(float seconds) { mFixedDt = seconds; }

    void setDerivativeFilter(float alpha) { mCore.setDerivativeFilter(alpha); }
    float derivativeFilter() const { return mCore.derivativeFilter(); }

    void setIntegralZone(float zone) { mCore.setIntegralZone(zone); }
    void setOutputLimit(float limit) { mCore.setOutputLimit(limit); }
    void setResetIntegralOnSignFlip(bool enable) { mCore.setResetIntegralOnSignFlip(enable); }

    PidCore& core() { return mCore; }
    const PidCore& core() const { return mCore; }

private:
    PidCore mCore;
    float mFixedDt = 0.0f;
    uint32_t mPreviousTime = 0;
};

}  // namespace chad
