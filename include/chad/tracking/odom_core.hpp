#pragma once

#include <cstdint>

#include "chad/util/util.hpp"

namespace chad {

// ============================================================================
// OdomCore — dead reckoning, with no hardware and no clock
// ============================================================================
//
// Takes accumulated wheel distances and a heading, integrates the tick's motion
// along the arc it actually traced, and keeps one delta accumulator per
// consumer. `OdomTracking` is this class fed by tracking wheels, motor encoders
// and a pros::Imu.
// ============================================================================

/// Mean of a redundant distance source, live readings only.
struct DistanceAverage {
    float value{0.0f};
    int live{0};
    int faulted{0};
    bool valid() const { return live > 0; }
};

/// Averaging a faulted sensor's HELD value with its live neighbours halves the
/// reported distance per dead sensor: every per-tick delta is scaled, none of
/// them jumps, so no jump guard fires and a 2x scale error is invisible. Live
/// readings only, and the count of the ones dropped.
DistanceAverage averageLiveDistance(const float* values, const bool* faulted,
                                    int count);

class OdomCore {
public:
    /// One independent accumulator per consumer. Two filters sharing a channel
    /// would each see only the motion the other had not yet taken.
    enum DeltaChannel { CHANNEL_MCL = 0, CHANNEL_EKF = 1, DELTA_CHANNELS = 2 };

    struct Config {
        /// Largest per-tick wheel delta accepted as real motion, inches. A
        /// replugged Rotation sensor reports position zero, and a drivetrain
        /// that loses every motor reports zero degrees; both look like driving
        /// the whole accumulated distance backwards in a single tick. The tick
        /// is dropped and the baseline re-taken, so the outage costs one tick
        /// rather than freezing odometry. Zero disables the guard.
        ///
        /// It also caps how long an encoder outage can be recovered from. The
        /// hold buys back the whole blackout only while the travel accumulated
        /// during it stays under this. Measured at 31.5 in/s: 300/500/700 ms
        /// outages recover exactly, a 900 ms one (33 in) has its catch-up
        /// rejected and loses 28.7 in. At this drivetrain's full 63 in/s that
        /// ceiling is about 380 ms.
        float maxDeltaIn{24.0f};
        float maxDeltaRad{1.5f};

        /// True pins the pose heading to the IMU, so a heading written by
        /// setPose() or corrected by a filter is overwritten on the next tick.
        /// False integrates dTheta instead and keeps those corrections, which is
        /// what makes setHeading() and the EKF's thetaSyncProp mean anything.
        ///
        /// True is the shipped value and nothing in the library changes it, so
        /// EkfCore::Config::thetaSyncProp is inert as shipped: the blended
        /// heading EkfTracking writes back survives less than one 10 ms tick,
        /// including after a relocation. Read EkfTracking::getEstimate() for the
        /// fused heading; Chassis::getPose() reports this one.
        bool headingFromImu{true};
    };

    const Config& config() const { return mConfig; }
    void setConfig(const Config& config) { mConfig = config; }

    /// Re-take the sensor baseline without touching the pose. Only correct after
    /// the encoders really have been zeroed, or at construction.
    void rebaseline(float vertIn, float horizIn, float headingRad);

    /// Move the pose. Deliberately does NOT re-take the baseline: distance
    /// travelled since the last tick is real, and re-baselining here silently
    /// deletes it -- which, with a filter writing the pose back every cycle, is
    /// most of the motion.
    void setPose(const Pose& pose) { mPose = pose; }
    Pose pose() const { return mPose; }

    /// One tick. `headingValid` false means `headingRad` is a held value, not a
    /// measurement, so the tick contributes no rotation.
    OdomDelta step(float vertIn, float horizIn, float headingRad, bool headingValid);

    OdomDelta lastDelta() const { return mLastDelta; }

    /// The accumulated delta on `channel` since the previous call, reset.
    OdomDelta consumeDelta(int channel);
    void clearDeltas();

    float prevHeadingRad() const { return mPrevHeadingRad; }

    /// Ticks dropped by the jump guards. Climbing means a sensor is dropping out.
    std::uint32_t jumpRejects() const { return mJumpRejects; }
    std::uint32_t headingJumpRejects() const { return mHeadingJumpRejects; }
    /// Ticks whose heading was a held value rather than a reading.
    std::uint32_t headingHolds() const { return mHeadingHolds; }

private:
    Config mConfig{};
    Pose mPose{};

    float mPrevVertDist{0.0f};
    float mPrevHorizDist{0.0f};
    float mPrevHeadingRad{0.0f};

    OdomDelta mLastDelta{};
    OdomDelta mAccumDelta[DELTA_CHANNELS]{};

    std::uint32_t mJumpRejects{0};
    std::uint32_t mHeadingJumpRejects{0};
    std::uint32_t mHeadingHolds{0};
};

}  // namespace chad
