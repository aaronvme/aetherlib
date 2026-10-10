#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace chad {
// Decides when to shut a gripper from a range sensor polled faster than it
// refreshes. Each physical sample counts once, an object is confirmed by a
// second sample landing where the robot's travel says it should, and the
// decision leads the object by the sensor-plus-actuator latency.
class GrabTrigger {
public:
    struct Config {
        float grabRangeIn{4.5f};
        // Sensor latency plus the time the jaws take to shut.
        float leadMs{50.0f};
        // A repeated value this old is a fresh sample of something not moving.
        std::uint32_t samplePeriodMs{40};
        // How far a sample may land from the previous one moved by the robot.
        float consistencyIn{1.5f};
        float consistencyFrac{0.08f};
    };

    explicit GrabTrigger(const Config& config) : mConfig(config) {}

    void reset() {
        mHave = false;
        mConfirmed = false;
    }

    // distIn: NaN when there is no return. closingInPerSec: how fast the
    // robot closes on the object; non-finite falls back to the plain range.
    bool update(std::uint32_t nowMs, float distIn, float closingInPerSec) {
        if (!std::isfinite(distIn)) {
            reset();
            return false;
        }
        const float closing = std::isfinite(closingInPerSec) ? closingInPerSec : 0.0f;

        if (!mHave || distIn != mDistIn || nowMs - mSampleMs >= mConfig.samplePeriodMs) {
            if (mHave) {
                const float dtSec = static_cast<float>(nowMs - mSampleMs) * 0.001f;
                const float expected = mDistIn - closing * dtSec;
                const float tol = mConfig.consistencyIn + mConfig.consistencyFrac * distIn;
                mConfirmed = std::fabs(distIn - expected) <= tol;
            }
            mHave = true;
            mDistIn = distIn;
            mSampleMs = nowMs;
        }
        if (!mConfirmed) return false;

        const std::uint32_t ageMs = std::min(nowMs - mSampleMs, mConfig.samplePeriodMs);
        const float aheadSec = (static_cast<float>(ageMs) + mConfig.leadMs) * 0.001f;
        return mDistIn - closing * aheadSec <= mConfig.grabRangeIn;
    }

private:
    Config mConfig;
    bool mHave{false};
    bool mConfirmed{false};
    float mDistIn{0.0f};
    std::uint32_t mSampleMs{0};
};
}  // namespace chad
