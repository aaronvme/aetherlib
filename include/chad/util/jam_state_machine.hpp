#pragma once

#include <cstdint>

namespace chad {

/// Timing thresholds for JamStateMachine. Deciding "jammed right now" (the
/// efficiency band) is the caller's job; this only tracks how long that signal
/// has held and drives detect -> reverse -> settle.
struct JamTimingConfig {
    uint32_t jamTriggerMs{150};
    uint32_t reverseDurationMs{250};
    uint32_t settleDurationMs{150};
    /// Suppresses the jam timer for this long after motion starts, so a
    /// mechanism's spin-up transient (low efficiency before it reaches speed)
    /// does not read as a jam. Mirrors SettleSpec::graceMs.
    uint32_t graceMs{150};
};

enum class JamState { Normal, Reversing, Settling };

/// PROS-free detect/reverse/settle cycle, time and the jammed signal injected
/// so it is testable off the robot. JamDetector drives this against real
/// motors and applies the motor commands on each transition.
class JamStateMachine {
public:
    JamStateMachine() = default;
    explicit JamStateMachine(const JamTimingConfig& config) : mConfig(config) {}

    void setConfig(const JamTimingConfig& config) { mConfig = config; }
    const JamTimingConfig& config() const { return mConfig; }

    void reset(uint32_t nowMs) {
        mState = JamState::Normal;
        mStateStartMs = nowMs;
        mJamStartMs = 0;
        mMotionStartMs = nowMs;
    }

    /// Call when the monitored motors go from all-stopped to any-moving.
    /// Restarts the grace window without otherwise touching the state.
    void noteMotionStart(uint32_t nowMs) { mMotionStartMs = nowMs; }

    /// A stopped mechanism cannot be jammed; clears the accumulator without
    /// touching state.
    void clearJamTimer() { mJamStartMs = 0; }

    /// @param jammed  this tick's efficiency reading is in the jam band
    JamState update(uint32_t nowMs, bool jammed) {
        switch (mState) {
            case JamState::Normal: {
                const bool inGrace = (nowMs - mMotionStartMs) < mConfig.graceMs;
                if (jammed && !inGrace) {
                    if (mJamStartMs == 0) {
                        mJamStartMs = (nowMs == 0) ? 1 : nowMs;  // 0 is the sentinel
                    } else if (nowMs - mJamStartMs >= mConfig.jamTriggerMs) {
                        mState = JamState::Reversing;
                        mStateStartMs = nowMs;
                        mJamStartMs = 0;
                    }
                } else {
                    mJamStartMs = 0;
                }
                break;
            }
            case JamState::Reversing:
                if (nowMs - mStateStartMs >= mConfig.reverseDurationMs) {
                    mState = JamState::Settling;
                    mStateStartMs = nowMs;
                }
                break;
            case JamState::Settling:
                if (nowMs - mStateStartMs >= mConfig.settleDurationMs) {
                    mState = JamState::Normal;
                    mStateStartMs = nowMs;
                    mJamStartMs = 0;
                }
                break;
        }
        return mState;
    }

    JamState state() const { return mState; }

private:
    JamTimingConfig mConfig{};
    JamState mState{JamState::Normal};
    uint32_t mStateStartMs{0};
    uint32_t mJamStartMs{0};
    uint32_t mMotionStartMs{0};
};

}  // namespace chad
