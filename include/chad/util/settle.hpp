#pragma once

#include <cmath>
#include <cstdint>

namespace chad {

/// When a motion is allowed to stop trying. Every field is optional except the
/// error tolerance; zero means "do not use this test".
struct SettleSpec {
    float errorTolerance{1.0f};
    /// Speed must also be inside this before the motion counts as arrived.
    /// Without it a motion exits while still carrying velocity through target.
    float speedTolerance{0.0f};
    /// How long error and speed must hold together. Zero exits on first sample.
    uint32_t holdMs{0};
    uint32_t timeoutMs{0};
    /// Exit as soon as error is inside this, carrying speed into the next
    /// motion. This is motion chaining: it trades final accuracy for time.
    float earlyExitRange{0.0f};
    /// Below this the drivetrain has stopped, so whatever error remains is what
    /// it is going to get. Burning the rest of the timeout gains nothing, and
    /// on a robot pinned against a wall it costs a brownout.
    float stallSpeed{0.0f};
    uint32_t stallHoldMs{0};
    /// Stall detection is suppressed for this long after the start, otherwise
    /// every standing start reads as a stall.
    uint32_t graceMs{150};
    /// Run the whole timeout window and nothing else: a step-response capture
    /// needs the entire response, and cutting the trace short silently changes
    /// what an autotuner is optimising.
    bool runFullWindow{false};
};

/// One sample. `speed` is measured along the error, in the motion's own units;
/// `driveSpeed` is how fast the drivetrain is actually turning and is a
/// different signal, because a robot rotating in place makes no progress toward
/// a point yet is plainly not stalled.
struct SettleInput {
    float error{0.0f};
    float speed{0.0f};
    float driveSpeed{0.0f};
    /// The motion's own primary target, met on its own terms and separate from
    /// `error`, which may fold in a secondary term: a path inside its position
    /// tolerance but off its final tangent is atTarget. Stopping here is
    /// arrival -- the residual is what the motion is going to get. Stopping
    /// without it is a stall.
    bool atTarget{false};
    /// False while the reference is still running, or is legitimately stopped
    /// mid-motion: the motion cannot end yet and a stopped drivetrain is not a
    /// stall.
    bool referenceDone{true};
};

enum class SettleState {
    Running,
    Settled,
    EarlyExit,
    TimedOut,
    Stalled,
};

/// The single authority on when a motion is finished. Time is passed in rather
/// than read, so it is testable off-robot and cannot drift between callers.
class SettleMonitor {
public:
    SettleMonitor() = default;
    explicit SettleMonitor(const SettleSpec& spec) : mSpec(spec) {}

    void reset(uint32_t nowMs) {
        mStartMs = nowMs;
        mInToleranceSince = 0;
        mInTolerance = false;
        mStalledSince = 0;
        mStalling = false;
        mState = SettleState::Running;
    }

    void setSpec(const SettleSpec& spec) { mSpec = spec; }
    const SettleSpec& spec() const { return mSpec; }

    /// @param error  signed distance still to cover, in the motion's own units
    /// @param speed  current speed along that error, same units per second
    SettleState update(uint32_t nowMs, float error, float speed) {
        return update(nowMs, SettleInput{error, speed, speed, false, true});
    }

    SettleState update(uint32_t nowMs, const SettleInput& in) {
        if (mState != SettleState::Running) return mState;

        const uint32_t elapsed = nowMs - mStartMs;

        if (mSpec.runFullWindow) {
            // Only the clock ends it, so the whole response is captured.
            if (mSpec.timeoutMs > 0 && elapsed >= mSpec.timeoutMs) {
                return mState = SettleState::TimedOut;
            }
            return mState;
        }

        const bool errorValid = std::isfinite(in.error);
        const bool speedValid = std::isfinite(in.speed);
        const float absError = errorValid ? std::fabs(in.error) : 0.0f;
        const float absSpeed = speedValid ? std::fabs(in.speed) : 0.0f;

        if (errorValid && mSpec.earlyExitRange > 0.0f && absError <= mSpec.earlyExitRange) {
            return mState = SettleState::EarlyExit;
        }

        // A non-finite reading is a dead sensor, not an arrival.
        const bool arrived = in.referenceDone && errorValid &&
                             absError <= mSpec.errorTolerance &&
                             (mSpec.speedTolerance <= 0.0f ||
                              (speedValid && absSpeed <= mSpec.speedTolerance));
        if (arrived) {
            if (!mInTolerance) {
                mInTolerance = true;
                mInToleranceSince = nowMs;
            }
            // Sitting inside tolerance is arrival, not a stall, however long
            // the hold is. Otherwise a holdMs above stallHoldMs never settles.
            mStalling = false;
            if (nowMs - mInToleranceSince >= mSpec.holdMs) {
                return mState = SettleState::Settled;
            }
        } else {
            mInTolerance = false;

            const bool driveValid = std::isfinite(in.driveSpeed);
            const bool atRest = in.referenceDone && mSpec.stallSpeed > 0.0f &&
                                mSpec.stallHoldMs > 0 && elapsed >= mSpec.graceMs &&
                                driveValid &&
                                std::fabs(in.driveSpeed) < mSpec.stallSpeed;
            if (atRest && in.atTarget) {
                // The reference has run out and the drivetrain has stopped
                // inside the motion's own target, so the secondary error left
                // over is what it is going to get. That is an arrival: routing
                // it through Stalled reported every curved path as a failure,
                // half a second late.
                mStalling = false;
                return mState = SettleState::Settled;
            }
            if (atRest) {
                if (!mStalling) {
                    mStalling = true;
                    mStalledSince = nowMs;
                }
                if (nowMs - mStalledSince >= mSpec.stallHoldMs) {
                    return mState = SettleState::Stalled;
                }
            } else {
                mStalling = false;
            }
        }

        if (mSpec.timeoutMs > 0 && elapsed >= mSpec.timeoutMs) {
            return mState = SettleState::TimedOut;
        }

        return mState;
    }

    SettleState state() const { return mState; }
    bool done() const { return mState != SettleState::Running; }

    /// True while the last sample was inside every arrival test, whether or not
    /// the hold has run out yet.
    bool inTolerance() const { return mInTolerance; }

    /// How long that has held continuously. 0 the moment it breaks, so a caller
    /// can tell a settled motion from one jitter sample touching tolerance.
    uint32_t inToleranceMs(uint32_t nowMs) const {
        return mInTolerance ? (nowMs - mInToleranceSince) : 0u;
    }

    /// True only if the motion reached what it was asked to reach.
    bool reachedTarget() const {
        return mState == SettleState::Settled || mState == SettleState::EarlyExit;
    }

    uint32_t elapsedMs(uint32_t nowMs) const { return nowMs - mStartMs; }

private:
    SettleSpec mSpec{};
    uint32_t mStartMs{0};
    uint32_t mInToleranceSince{0};
    uint32_t mStalledSince{0};
    SettleState mState{SettleState::Running};
    bool mInTolerance{false};
    bool mStalling{false};
};

/// Low-passed rate of change of a signal. A raw per-tick difference of a pose
/// estimate is mostly jitter — under MCL, ±0.5 in over a 10 ms tick reads as
/// 50 in/s — so the speed tests read this instead.
class RateEstimator {
public:
    explicit RateEstimator(float alpha = 0.1f) { setAlpha(alpha); }

    void setAlpha(float alpha) {
        mAlpha = alpha < 0.01f ? 0.01f : (alpha > 1.0f ? 1.0f : alpha);
    }

    /// A single sample this many times the signal's own running derivative scale
    /// is a discontinuity -- an MCL relocalisation, or a setpoint that moved --
    /// not a speed, so it re-baselines instead of being differentiated. One that
    /// persists into a second sample is believed. <= 0 differentiates
    /// everything, which reads 200 in/s off a stationary robot whose error
    /// stepped 2 in over one 10 ms tick.
    void setJumpFactor(float factor) { mJumpFactor = factor; }

    void reset() {
        mPrimed = false;
        mScaled = false;
        mHeldLast = false;
        mScale = 0.0f;
        mRate = 0.0f;
    }

    /// @return the filtered derivative, in value-units per second
    float update(float value, float dt) {
        if (!std::isfinite(value) || !(dt > 0.0f)) return mRate;
        if (!mPrimed) {
            mPrevious = value;
            mPrimed = true;
            return mRate;
        }

        const float raw = (value - mPrevious) / dt;
        mPrevious = value;                  // re-baselined either way
        const float mag = std::fabs(raw);

        if (!mScaled) {
            mScale = mag;                   // the first derivative sets the scale
            mScaled = true;
        } else if (mJumpFactor > 0.0f && !mHeldLast && mag > mJumpFactor * mScale) {
            mHeldLast = true;
            return mRate;
        }

        mHeldLast = false;
        mScale += mAlpha * (mag - mScale);
        mRate += mAlpha * (raw - mRate);
        return mRate;
    }

    float rate() const { return mRate; }

private:
    float mAlpha{0.1f};
    float mJumpFactor{8.0f};
    float mPrevious{0.0f};
    float mScale{0.0f};
    float mRate{0.0f};
    bool mPrimed{false};
    bool mScaled{false};
    bool mHeldLast{false};
};

}  // namespace chad
