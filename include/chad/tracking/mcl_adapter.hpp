#pragma once

#include <array>
#include <atomic>

#include "chad/tracking/mcl_core.hpp"
#include "chad/util/util.hpp"

namespace chad {

/// The parts of MclTracking that are not PROS: the published-estimate handoff,
/// the sensor enable/disable bookkeeping, the raw-reading classification and the
/// odometry writeback arithmetic. MclTracking is 273 lines of concurrency,
/// hardware reads and odometry writeback with no host coverage at all, and this
/// is the seam that gives most of it a test.
namespace mcladapter {

/// Single-writer, many-reader handoff for a small POD, without a lock.
///
/// The writer fills the slot the readers are not using and then publishes the
/// index; a reader copies the current slot and re-checks the index. Two slots
/// are enough because the writer would have to publish twice inside one 24-byte
/// copy to overtake a reader, and the writer runs once per filter tick.
///
/// A mutex would do the same job, but on a target with no watchdog a worker
/// killed while holding it -- which is exactly what MclTracking::stopTracking()
/// does when the task will not exit -- deadlocks every reader for good.
template <typename T>
class Published {
public:
    void store(const T& value) {
        const std::uint32_t next = mSeq.load(std::memory_order_relaxed) + 1u;
        mSlot[next & 1u] = value;
        mSeq.store(next, std::memory_order_release);
    }

    T load() const {
        for (int attempt = 0; attempt < 4; attempt++) {
            const std::uint32_t seq = mSeq.load(std::memory_order_acquire);
            T out = mSlot[seq & 1u];
            if (mSeq.load(std::memory_order_acquire) == seq) return out;
        }
        return T{};
    }

    /// How many times store() has run. A reader that sees this stop moving knows
    /// the worker has stopped publishing.
    std::uint32_t generation() const { return mSeq.load(std::memory_order_acquire); }

private:
    std::atomic<std::uint32_t> mSeq{0};
    T mSlot[2]{};
};

/// Which slots are muted, and for how long. `disableFor` is a countdown in
/// milliseconds; `disable` has no timer and holds until `enable`.
template <int N>
struct SensorGate {
    std::array<bool, N> disabled{};
    std::array<float, N> timers{};

    void enable(int i) {
        if (i < 0 || i >= N) return;
        disabled[i] = false;
        timers[i] = 0.0f;
    }

    void disable(int i) {
        if (i < 0 || i >= N) return;
        disabled[i] = true;
        timers[i] = 0.0f;
    }

    void disableFor(int i, float ms) {
        if (i < 0 || i >= N) return;
        disabled[i] = true;
        // A non-positive duration would expire on the next tick, which reads as
        // "disable briefly" but is really "do nothing". Treat it as permanent,
        // matching disable().
        timers[i] = (ms > 0.0f) ? ms : 0.0f;
    }

    /// Age the countdowns by one update. Slots with no timer are untouched.
    void tick(float periodMs) {
        for (int i = 0; i < N; i++) {
            if (timers[i] <= 0.0f) continue;
            timers[i] -= periodMs;
            if (timers[i] <= 0.0f) {
                timers[i] = 0.0f;
                disabled[i] = false;
            }
        }
    }
};

/// A disconnected V5 sensor answers PROS_ERR, which is INT32_MAX and would sail
/// past any bound expressed in millimetres. Anything at or above this is a fault
/// code, not a range.
inline constexpr int kSensorFaultMm = 20000;

/// One hardware read reduced to what the filter consumes. `mm` and `confidence`
/// are as the V5 reports them; a slot with no sensor fitted passes present=false.
inline MclCore::RawSensor classifyRaw(bool fitted, int mm, int confidence,
                                      bool suppressed) {
    MclCore::RawSensor r{};
    if (!fitted) return r;
    const bool live = mm >= 0 && mm < kSensorFaultMm;
    r.mm = live ? mm : 0;
    r.confidence = (confidence >= 0 && confidence <= 63) ? confidence : 0;
    r.present = live;
    r.suppressed = suppressed;
    return r;
}

/// Where the beam-validity and sigma decisions are judged from: last tick's
/// position with this tick's heading. `imuValid` false means the IMU is absent
/// or faulted and the value it offers is a hold, so the cloud's own heading is
/// the better answer.
inline Pose referencePose(const Pose& rawEstimate, float imuHeadingRad, bool imuValid) {
    Pose p = rawEstimate;
    if (imuValid && std::isfinite(imuHeadingRad)) p.theta = imuHeadingRad;
    return p;
}

/// One lerp of the odometry pose toward the filter estimate. Heading is moved
/// along the angular difference, not by subtraction, so a pair straddling the
/// +/-pi cut does not swing the robot the long way round.
inline Pose syncStep(const Pose& odomPose, const Pose& estimate, float distProp,
                     float thetaProp) {
    const float d = clamp(distProp, 0.0f, 1.0f);
    const float t = clamp(thetaProp, 0.0f, 1.0f);
    return {odomPose.x + d * (estimate.x - odomPose.x),
            odomPose.y + d * (estimate.y - odomPose.y),
            wrapRad(odomPose.theta +
                    t * angleDiffRad(odomPose.theta, estimate.theta))};
}

}  // namespace mcladapter

}  // namespace chad
