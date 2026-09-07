#pragma once

#include <cstdint>

namespace chad {
struct JamTimingConfig {
    uint32_t jamTriggerMs{150};
    uint32_t reverseDurationMs{250};
    uint32_t settleDurationMs{150};

    uint32_t graceMs{150};
};

enum class JamState { Normal, Reversing, Settling };

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

    void noteMotionStart(uint32_t nowMs) { mMotionStartMs = nowMs; }

    void clearJamTimer() { mJamStartMs = 0; }

    JamState update(uint32_t nowMs, bool jammed) {
        switch (mState) {
            case JamState::Normal: {
                const bool inGrace = (nowMs - mMotionStartMs) < mConfig.graceMs;
                if (jammed && !inGrace) {
                    if (mJamStartMs == 0) {
                        mJamStartMs = (nowMs == 0) ? 1 : nowMs;
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
}
