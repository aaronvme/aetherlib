#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"

#include "chad/tracking/ekf_core.hpp"
#include "chad/tracking/mcl_tracking.hpp"
#include "chad/tracking/odom_tracking.hpp"
#include "chad/util/util.hpp"

namespace chad {

// ============================================================================
// EkfTracking — EkfCore against PROS devices and the PROS clock
// ============================================================================
//
// The numerics all live in EkfCore, which has no hardware and is host-tested
// (test/host/ekf_test.cpp). This class reads the IMU, the distance sensors and
// MCL, hands them to the core once per cycle, and writes the fused pose back
// into odometry.
//
// usage:
//   EkfTracking ekf(odom, distanceSensors, startPose, config);
//   ekf.attachMcl(&mcl);
//   ekf.startTracking();
// ============================================================================

class EkfTracking {
public:
    static constexpr int SENSOR_COUNT = MAX_DISTANCE_SENSORS;

    using Config = EkfCore::Config;
    using BeamState = EkfCore::BeamState;
    using Uncertainty = EkfCore::Uncertainty;

    EkfTracking(OdomTracking& odom,
                std::array<pros::Distance*, SENSOR_COUNT> distanceSensors,
                const Pose& startPose,
                const Config& config);
    ~EkfTracking();

    // ========================================================================
    // Lifecycle
    // ========================================================================

    void startTracking();
    void stopTracking();
    bool isTracking() const;

    /// One full cycle: predict, then every enabled measurement family.
    Pose update();

    // ========================================================================
    // State
    // ========================================================================

    Pose getEstimate() const;

    /// Position standard deviations (inches) and heading sigma (radians),
    /// straight off the diagonal of P.
    Uncertainty uncertainty() const;

    /// Reset the state to `pose` and the covariance to the initial-belief
    /// values. Use after a deliberate reposition, not to nudge the filter.
    void setPose(const Pose& pose);

    /// Reset the state to `pose` with an explicit position sigma, for handing
    /// over from a filter that knows how wrong it might be.
    void setPose(const Pose& pose, float posSigma, float thetaSigma);

    // ========================================================================
    // Wiring
    // ========================================================================

    /// Supply the particle filter used as the global-position measurement.
    /// Pass nullptr to run the EKF on ranges and the IMU alone.
    void attachMcl(MclTracking* mcl);

    void setSensorMounts(const std::array<Pose, SENSOR_COUNT>& mounts);
    void setDistanceSensors(const std::array<pros::Distance*, SENSOR_COUNT>& sensors);

    /// Obstacles whose returns must not be read as wall ranges. Copied into the
    /// core each cycle, so the caller may keep mutating the vector.
    void setObstacles(const std::vector<MclTracking::CircleObstacle>* circles);

    /// Override the millisecond clock. Only for replaying a fixed-step trace.
    void setClock(std::uint32_t (*clock)()) { mClock = clock; }

    // ========================================================================
    // Diagnostics
    // ========================================================================

    /// Per-beam outcome from the most recent cycle.
    std::array<BeamState, SENSOR_COUNT> beamStates() const;
    std::array<float, SENSOR_COUNT> innovations() const;

    /// Beams folded into the state on the last cycle.
    int beamsUsed() const;

    /// Consecutive cycles MCL has disagreed with the filter beyond its gate.
    int mclRejectStreak() const;

    /// Consecutive cycles in which beams returned and the gate took all of them.
    /// A filter rejecting everything looks healthy from the outside; this is how
    /// it does not.
    int rangeRejectStreak() const;

    /// Consecutive cycles in which a sensor was fitted and no beam was folded
    /// in, whatever the reason. A filter blinded by obstacle deadzones never
    /// reaches the gate, so it shows up here and nowhere else.
    int rangeBlindStreak() const;

    /// Cumulative gate rejections since construction, per family.
    std::uint32_t rangeRejects() const;
    std::uint32_t headingRejects() const;
    std::uint32_t mclRejects() const;

    /// Cumulative beams discarded before the gate saw them.
    std::uint32_t rangeNoReturns() const;
    std::uint32_t rangeDeadzoned() const;
    std::uint32_t rangeGrazed() const;

    /// Cycles in which the IMU could not be read, so heading went unmeasured.
    std::uint32_t imuSkips() const { return mImuSkips; }

    /// Times P had to be clamped back to positive-semidefinite. Nonzero means
    /// float32 is losing the fight and the covariance is worth logging.
    std::uint32_t covarianceRepairs() const;

    /// Times the filter has been forced onto MCL's estimate since construction.
    int relocateCount() const;

    Config config() const;
    void setConfig(const Config& config);

    /// The filter itself, for callers that want the covariance or want to drive
    /// it by hand. Not mutex-guarded: only touch it with tracking stopped.
    EkfCore& core() { return mCore; }
    const EkfCore& core() const { return mCore; }

private:
    void run();
    std::uint32_t nowMs() const;

    EkfCore mCore;
    OdomTracking& mOdom;
    MclTracking* mMcl{nullptr};
    std::array<pros::Distance*, SENSOR_COUNT> mDistanceSensors;
    const std::vector<MclTracking::CircleObstacle>* mCircleObstacles{nullptr};

    std::uint32_t (*mClock)(){nullptr};
    std::uint32_t mImuSkips{0};

    // Guards the core: the background task writes it while getEstimate() and
    // uncertainty() are called from motion code on another task.
    mutable pros::Mutex mMutex;

    pros::Task* mTask{nullptr};
    std::atomic<bool> mRunning{false};
};

}  // namespace chad
