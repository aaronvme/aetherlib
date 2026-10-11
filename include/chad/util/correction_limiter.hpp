#pragma once

#include <cmath>
#include <cstdint>

namespace chad {
// Token bucket on correction distance over wall-clock time. Whatever is asked
// for, the total distance applied over any window of T seconds is at most
// burstIn + rateInPerSec * T, so a run of wrong readings can drag the pose at
// a bounded speed but never teleport it. Each step keeps its direction and is
// only shortened. No heap, no trig: one sqrt and a divide per clamped step.
class CorrectionLimiter {
public:
    struct Config {
        /// Sustained correction speed. <= 0 (or non-finite) disables the guard.
        float rateInPerSec{30.0f};
        /// Distance that may be applied at once from a full bucket. <= 0
        /// (or non-finite) disables the guard.
        float burstIn{2.0f};
    };

    CorrectionLimiter() = default;
    explicit CorrectionLimiter(const Config& config) : mConfig(config) {}

    /// Keeps the held budget, clamped to the new burst.
    void setConfig(const Config& config) {
        mConfig = config;
        if (enabled() && mBudget > mConfig.burstIn) mBudget = mConfig.burstIn;
    }
    const Config& config() const { return mConfig; }

    /// The next call starts from a full bucket.
    void reset() { mHave = false; }

    bool enabled() const {
        return std::isfinite(mConfig.rateInPerSec) && std::isfinite(mConfig.burstIn) &&
               mConfig.rateInPerSec > 0.0f && mConfig.burstIn > 0.0f;
    }

    /// Distance that could be applied at nowMs, without spending it.
    float available(std::uint32_t nowMs) const {
        if (!mHave) return mConfig.burstIn;
        return refilled(nowMs);
    }

    /// Shortens (dx, dy) in place to what the budget allows at nowMs and
    /// spends it. Returns the length applied. A non-finite request is zeroed.
    /// nowMs is compared by unsigned difference, so a millis() wrap is an
    /// ordinary period.
    float limit(std::uint32_t nowMs, float& dx, float& dy) {
        if (!std::isfinite(dx) || !std::isfinite(dy)) {
            dx = dy = 0.0f;
            return 0.0f;
        }
        const float dist = std::sqrt(dx * dx + dy * dy);
        if (!enabled()) return dist;

        mBudget = mHave ? refilled(nowMs) : mConfig.burstIn;
        mHave = true;
        mLastMs = nowMs;

        if (dist <= mBudget) {
            mBudget -= dist;
            return dist;
        }
        const float applied = mBudget;
        const float scale = applied / dist;     // dist > budget >= 0, so > 0
        dx *= scale;
        dy *= scale;
        mBudget = 0.0f;
        return applied;
    }

private:
    float refilled(std::uint32_t nowMs) const {
        const float elapsedSec = static_cast<float>(nowMs - mLastMs) * 0.001f;
        const float b = mBudget + mConfig.rateInPerSec * elapsedSec;
        return b < mConfig.burstIn ? b : mConfig.burstIn;
    }

    Config mConfig{};
    float mBudget{0.0f};
    std::uint32_t mLastMs{0};
    bool mHave{false};
};
}  // namespace chad
