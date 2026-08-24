// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <utility>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"

#include "chad/tracking/mcl_adapter.hpp"
#include "chad/tracking/mcl_core.hpp"
#include "chad/tracking/odom_tracking.hpp"
#include "chad/util/util.hpp"

namespace chad {

/// MclCore driven by V5 distance sensors on a background task, with the
/// odometry writeback. All of the numerics live in MclCore so they can be
/// host-tested; this class only reads hardware, keeps time, and publishes.
///
///   MclTracking mcl(odom, distanceSensors, startPose, config);
///   mcl.setSensorMounts(mounts);
///   mcl.startTracking();
///   Pose est = mcl.getEstimate().first;
///
/// Locking, since a background task owns the cloud: every method that touches
/// the core takes one mutex for the whole call, so any of them is safe to call
/// while the worker runs -- at the cost of waiting up to one filter tick. The
/// exceptions, each of which says so at its own declaration, are:
///   - estimate() / getEstimate() / estimateGeneration(), which read a
///     lock-free published copy while the worker runs and never wait;
///   - snapshotParticles(), core(), sensorReadings(), deadzoneFraction() and
///     validSensors(), which read live state unlocked and may be torn;
///   - startTracking() / stopTracking(), which own the worker's lifetime and
///     must not wait on a lock the worker itself may be holding.
class MclTracking {
public:
    using Config = MclCore::Config;
    using Estimate = MclCore::Estimate;
    using LineObstacle = MclCore::LineObstacle;
    using CircleObstacle = MclCore::CircleObstacle;

    static constexpr int SENSOR_COUNT = MclCore::SENSOR_COUNT;
    static constexpr int PARTICLE_MAX = MclCore::PARTICLE_MAX;

    MclTracking(OdomTracking& odom,
                std::array<pros::Distance*, SENSOR_COUNT> distanceSensors,
                const Pose& startPose,
                const Config& config);

    ~MclTracking();

    void startTracking();
    void stopTracking();
    bool isTracking() const;

    /// Motion update from the odometry delta accumulated since the last call.
    void predict();
    /// Sensor update from a fresh read of the distance sensors.
    void updateWeights();
    /// Systematic low-variance resampling, unconditionally.
    void resample();

    /// {weighted-mean pose, effective sample size}. Lock-free.
    std::pair<Pose, float> getEstimate() const;
    /// The same, plus the weighted spread of the cloud, which is what lets a
    /// downstream filter weigh this estimate instead of guessing at it.
    /// Lock-free while tracking, and coherent: never x from one tick with y from
    /// the next. With the worker stopped it reads the cloud under a bounded
    /// wait, falling back to the last published tick, so it can never block a
    /// control loop indefinitely.
    Estimate estimate() const;
    /// Ticks published so far. Unchanged across two calls means the worker is
    /// not publishing.
    std::uint32_t estimateGeneration() const { return mPublished.generation(); }

    /// Reinitialise the cloud around this pose. Rewrites every particle and
    /// consumes the odometry delta, so it takes the lock: this is the natural
    /// call at the start of an auton, with the worker already running.
    void setPose(const Pose& pose);
    /// Scatter the cloud across the whole field.
    void uniformReset();
    /// predict, weight, resample if needed, publish, and sync. Returns the new
    /// estimate.
    Pose update();
    /// Lerp the estimate into the odometry pose.
    void syncToOdometry();

    void setDistanceSensors(const std::array<pros::Distance*, SENSOR_COUNT>& sensors);
    /// Robot-frame sensor poses: x forward, y left, theta the pointing angle.
    void setSensorMounts(const std::array<Pose, SENSOR_COUNT>& mounts);

    void enableSensor(int index);
    void disableSensor(int index);
    void disableSensorFor(int index, float durationMs);

    /// Line obstacles invalidate a beam that reaches them; circles mark a beam
    /// deadzoned for the particles it would have hit. Both are read every tick,
    /// so the caller may keep mutating them, but they must outlive this object.
    void setObstacles(const std::vector<LineObstacle>* lineObstacles = nullptr,
                      const std::vector<CircleObstacle>* circleObstacles = nullptr);

    /// Tracking-wheel drift, inches per second. Divided down internally.
    void setDrift(float verticalDrift, float horizontalDrift);

    Pose getRawEstimate() const;

    // UNLOCKED telemetry, all three: references into state the worker rewrites
    // every tick, so a reader may see a mix of two ticks. Fine for a screen,
    // not for a control decision.
    const std::array<float, SENSOR_COUNT>& sensorReadings() const { return mReadingsInch; }
    /// Share of the cloud that treated each beam as deadzoned (0..1).
    const std::array<float, SENSOR_COUNT>& deadzoneFraction() const {
        return mCore.deadzoneFraction();
    }
    const std::array<bool, SENSOR_COUNT>& validSensors() const { return mValidSensors; }

    void setDistSyncProp(float prop);

    /// Whether this filter writes its estimate back into odometry. Turn it off
    /// when a downstream filter owns the fused pose, or the two fight over it.
    void setAutoSync(bool enabled);

    /// Share of the cloud redrawn across the field on the last resample. Rises
    /// above 0 when the measurements stop fitting, i.e. after a kidnapping.
    float randomInjectFraction() const;
    /// Augmented-MCL fast/slow gap on the cloud's MEAN likelihood, 0 to 1.
    /// What randomInjectFraction() is capped from. Reads up to 0.36 during
    /// healthy driving, so it is not a lost/tracking test -- see
    /// MclCore::likelihoodMismatch().
    float likelihoodMismatch() const;
    /// RMS range residual of the published estimate, in sigmas. About 1 while
    /// tracking and tens once the filter is lost, and what the search
    /// roughening is driven by -- the one to watch to see the filter decide it
    /// is lost.
    float fitSigmas() const;
    /// fitSigmas() mapped onto 0..1 through MclCore::Config::lostFitSigmas,
    /// which is the scale searchMismatchFloor and searchMismatchFull are in.
    float searchMismatch() const;
    /// Resamples since the last setPose(). Flat across two calls while the robot
    /// drives means the filter has stopped incorporating evidence.
    std::uint32_t resampleCount() const;

    /// Writes 4 floats per particle (x, y, theta, weight) into `out`, up to
    /// `maxParticles`. Returns the number written. UNLOCKED telemetry: it reads
    /// the live cloud, so a snapshot taken mid-update is torn.
    int snapshotParticles(float* out, int maxParticles) const {
        return mCore.snapshotParticles(out, maxParticles);
    }

    /// By value: the worker may be replacing the core's copy.
    Config config() const;

    /// particleCount is held at its current value: resizing the cloud mid-run
    /// would invalidate the buffers the background task is walking.
    void setConfig(const Config& config);

    /// The filter's RNG is seeded from MclCore::kDefaultSeed, so a run is
    /// reproducible by default. Reseed for independent runs, or to replay a
    /// divergence with a known seed. Call before startTracking().
    void setSeed(std::uint32_t seed);

    /// UNLOCKED direct access to the filter, for a caller that drives it itself.
    /// Everything else here serialises against the worker; these do not, so use
    /// them only with the worker stopped.
    MclCore& core() { return mCore; }
    const MclCore& core() const { return mCore; }

private:
    static constexpr int BLOCKER_MAX = 16;
    static constexpr int CIRCLE_MAX = 16;

    // Every *Locked method requires mLock already held.
    /// Copy the externally-owned obstacle lists into storage the core can hold
    /// a pointer to for the duration of one tick.
    void refreshObstaclesLocked();
    Pose referencePoseLocked() const;
    int readSensorsLocked(MclCore::RawSensor* out);
    Pose updateLocked();
    void syncToOdometryLocked();
    void run();

    MclCore mCore;
    OdomTracking& mOdom;
    std::array<pros::Distance*, SENSOR_COUNT> mDistanceSensors;

    std::array<float, SENSOR_COUNT> mReadingsInch{};
    std::array<bool, SENSOR_COUNT> mValidSensors{};
    mcladapter::SensorGate<SENSOR_COUNT> mGate{};

    const std::vector<LineObstacle>* mLineObstacles{nullptr};
    const std::vector<CircleObstacle>* mCircleObstacles{nullptr};
    LineObstacle mBlockers[BLOCKER_MAX]{};
    CircleObstacle mCircles[CIRCLE_MAX]{};

    MclCore::Beam mBeams[SENSOR_COUNT]{};
    int mBeamCount{0};

    // Serialises the core against the worker, and held for a whole tick. The
    // published estimate deliberately does not use it: a control loop asking for
    // the pose must not wait on a filter update, and must not see x from one
    // tick with y from the next either.
    mutable pros::Mutex mLock;
    mcladapter::Published<Estimate> mPublished{};

    pros::Task* mTask{nullptr};
    // Written by stopTracking() on the caller's task and read by the loop in
    // run() on the worker: a cross-task flag.
    std::atomic<bool> mRunning{false};
};

}  // namespace chad
