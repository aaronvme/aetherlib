#pragma once

#include <cmath>
#include <cstdint>

#include "chad/util/pid.hpp"

namespace chad {
struct IndexedRotatorConfig {
    // Motor degrees per index step.
    double degPerStep{180.0};
    PIDGains gains{1.2f, 0.0f, 0.04f};
    float derivativeFilter{0.5f};
    float maxPower{127.0f};
    // Floor on a positive output while short of the target, so the last few
    // degrees are not left to static friction.
    float minPower{15.0f};
    // Drive stops and settling starts inside this of the target.
    double toleranceDeg{2.0};
    // Within this of an index counts as on it: where a manual spin or a jam
    // is left decides the next flip's index, and a jam this close is done.
    double indexSnapDeg{10.0};
    float settleSec{0.04f};
    // Short of the target and moved less than stallMovedDeg in stallSec.
    double stallMovedDeg{3.0};
    float stallSec{0.25f};
};

// One-way position indexer. The target is base + steps * degPerStep with an
// integer step count, so it never drifts. Output is a power in
// [0, maxPower]: it never reverses -- past the target it outputs 0 and the
// caller's brake mode holds. A stall drops the flip in progress, and the next
// advance() retries it. Cancel, manual and a stall park it: idle where it is
// until the next advance(). Not thread-safe; one task owns an instance.
class IndexedRotator {
public:
    using Config = IndexedRotatorConfig;

    explicit IndexedRotator(const Config& config = Config()) : mConfig(config) {
        resetPid();
        mHeldSec = mConfig.settleSec;
    }

    const Config& config() const { return mConfig; }

    // Re-zero: baseDeg is index 0, nothing queued.
    void reset(double baseDeg = 0.0) {
        mBase = baseDeg;
        mSteps = 0;
        mPowerCap = mConfig.maxPower;
        mLastPos = baseDeg;
        mStalled = false;
        mParked = false;
        mSettled = true;
        mHeldSec = mConfig.settleSec;
        mStallRef = baseDeg;
        mStallTimer = 0.0f;
        resetPid();
    }

    // Queue steps more. Counts from the last index reached if the shaft is
    // ahead of the target, so nothing is ever caught up. maxPower > 0 caps
    // the move until it settles; the latest request's cap wins.
    void advance(int steps, float maxPower = 0.0f) {
        if (steps <= 0) return;
        const std::int32_t reached = reachedIndex(mLastPos);
        if (reached > mSteps) mSteps = reached;
        mSteps += steps;
        mPowerCap = cappedPower(maxPower);
        mStalled = false;
        mParked = false;
        mSettled = false;
        mHeldSec = 0.0f;
        mStallRef = mLastPos;
        mStallTimer = 0.0f;
        resetPid();
    }

    // Replace whatever is queued with the first index that is a multiple of
    // stepMultiple at or ahead of the shaft. Up to indexSnapDeg past one (its
    // own overshoot) is already there, so repeating it does nothing; further
    // past goes on round to the next. Short of one by toleranceDeg or less
    // does not drive either. maxPower > 0 caps this move only.
    void home(int stepMultiple, float maxPower = 0.0f) {
        if (stepMultiple <= 0 || !(mConfig.degPerStep > 0.0)) return;
        mPowerCap = cappedPower(maxPower);
        const double period = mConfig.degPerStep * stepMultiple;
        const double turns =
            std::ceil((mLastPos - mBase - mConfig.indexSnapDeg) / period);
        mSteps = static_cast<std::int32_t>(turns) * stepMultiple;
        mStalled = false;
        mParked = false;
        mSettled = false;
        mHeldSec = 0.0f;
        mStallRef = mLastPos;
        mStallTimer = 0.0f;
        resetPid();
    }

    // Drop the step in progress and anything queued; stops where it is.
    void cancel() {
        const std::int32_t reached = reachedIndex(mLastPos);
        if (reached < mSteps) mSteps = reached;
        mStalled = false;
        mParked = true;
        mSettled = true;
        mHeldSec = mConfig.settleSec;
        mStallTimer = 0.0f;
    }

    // Driver override: cancels and resyncs to the shaft every call, so release
    // leaves it idle where it stopped. Returns the power clamped to one way.
    float manual(double positionDeg, float power) {
        if (std::isfinite(positionDeg)) mLastPos = positionDeg;
        cancel();
        if (!(power > 0.0f)) return 0.0f;
        return power > mConfig.maxPower ? mConfig.maxPower : power;
    }

    // positionDeg non-finite (unplugged motor) outputs 0 and changes nothing.
    float update(double positionDeg, float dtSec) {
        if (!(dtSec > 0.0f) || !std::isfinite(dtSec)) dtSec = 0.0f;
        if (!std::isfinite(positionDeg)) return 0.0f;
        mLastPos = positionDeg;
        if (mStalled || mParked) return 0.0f;

        const double error = target() - positionDeg;
        if (error <= mConfig.toleranceDeg) {
            mHeldSec += dtSec;
            if (mHeldSec >= mConfig.settleSec) mSettled = true;
            mStallRef = positionDeg;
            mStallTimer = 0.0f;
            return 0.0f;
        }
        mSettled = false;
        mHeldSec = 0.0f;

        if (std::fabs(positionDeg - mStallRef) > mConfig.stallMovedDeg) {
            mStallRef = positionDeg;
            mStallTimer = 0.0f;
        } else {
            mStallTimer += dtSec;
            if (mStallTimer >= mConfig.stallSec) {
                const std::int32_t reached = reachedIndex(positionDeg);
                if (reached < mSteps) mSteps = reached;
                // Jammed within the snap band of the target: done, not stalled.
                mStalled = error > mConfig.indexSnapDeg;
                mParked = true;
                mSettled = !mStalled;
                return 0.0f;
            }
        }

        float out = mPid.step(static_cast<float>(error), dtSec);
        if (!std::isfinite(out) || out <= 0.0f) return 0.0f;
        if (out < mConfig.minPower) out = mConfig.minPower;
        if (out > mPowerCap) out = mPowerCap;
        return out;
    }

    double target() const {
        return mBase + static_cast<double>(mSteps) * mConfig.degPerStep;
    }
    double error() const { return target() - mLastPos; }
    std::int32_t steps() const { return mSteps; }
    bool settled() const { return mSettled; }
    bool stalled() const { return mStalled; }
    bool busy() const { return !mSettled && !mStalled; }

private:
    float cappedPower(float maxPower) const {
        return (maxPower > 0.0f && maxPower < mConfig.maxPower) ? maxPower : mConfig.maxPower;
    }

    std::int32_t reachedIndex(double positionDeg) const {
        if (!(mConfig.degPerStep > 0.0)) return mSteps;
        return static_cast<std::int32_t>(std::floor(
            (positionDeg - mBase + mConfig.indexSnapDeg) / mConfig.degPerStep));
    }

    void resetPid() {
        mPid = PID(mConfig.gains);
        mPid.setDerivativeFilter(mConfig.derivativeFilter);
    }

    Config mConfig;
    PID mPid{0.0f, 0.0f, 0.0f};
    float mPowerCap{mConfig.maxPower};
    double mBase{0.0};
    std::int32_t mSteps{0};
    double mLastPos{0.0};
    bool mStalled{false};
    bool mParked{false};
    bool mSettled{true};
    float mHeldSec{0.0f};
    double mStallRef{0.0};
    float mStallTimer{0.0f};
};
}  // namespace chad
