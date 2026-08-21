// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include <array>
#include <atomic>
#include <random>
#include <utility>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"

#include "riku/tracking/odom_tracking.hpp"
#include "riku/util/fasttrig.hpp"
#include "riku/util/util.hpp"

namespace riku {

// ============================================================================
// MclTracking via particle filter
// ============================================================================
//
// Uses 1024 particles with a Gaussian sensor model to estimate the robot's
// absolute position on the field. 
//
// usage:
//   MclTracking mcl(odomSensors, distanceSensors, startPose);
//   mcl.startTracking();  // begins background task @ ~40 Hz
//   // ...during autonomous...
//   Pose est = mcl.getEstimate().first;
// ============================================================================

class MclTracking {
public:
    // ========================================================================
    // Config (oompaloompa i am a doompa)
    // ========================================================================

    struct Config {
        // Tags: [robot] phase 3 or 4 measures it and prints it -- paste it in.
        //       [yours] a fact about your hardware or CPU budget. Set it once.
        //       [model] a modelling choice or a sensor/statistics standard, not a
        //               property of your robot. Nothing to measure.
        //       [sim]   the VALUE came off simulator sweeps. Structure is sound,
        //               the number deserves checking on hardware.
        int particleCount = 1024;   // [yours]
        int resampleThreshold = 512;      // N_eff below this triggers resample [model]
        float maxSensorRange = 100.0f;     // inches [yours]
        // Do not lower this to tighten the estimate. It is what lets the cloud
        // explore, and it is the difference between recovering from a kidnap and
        // not: at 1.5 recovery lands 3 runs in 4, and at 1.0, 0.6 and 0.3 it
        // never recovers at all, while every smoothness metric improves. The
        // tempting direction is the broken one.
        float distResampleVariance = 1.5f; // inches, jitter added during resample [sim]
        float thetaResampleVariance = 0.01f;  // radians [sim]
        float maxThetaDeviation = 0.08f;   // radians, clamp particle theta vs IMU [model]

        // Per-sensor-type noise multipliers.
        // These scale the standard normal noise applied to each axis's motion delta.
        // Different sensor types have different noise characteristics:
        //   - Tracking wheels: direct ground contact, low slip
        //   - IME (motor encoders): wheel slip, gear backlash → higher variance
        //   - No sensor: pure random walk, no measurement at all
        float trackingWheelVariance = 0.08f;   // dedicated tracking (dead) wheels [model]
        float imeVariance = 0.12f;             // integrated motor encoders (more slip) [robot]
        float imuVariance = 0.005f;   // [model]

        // Horizontal noise when no lateral sensor exists.
        // Without a horizontal tracking wheel, lateral position is unmeasured.
        // Two noise terms combine:
        //   1. Movement-proportional: lateral slip scales with forward travel
        //   2. Constant: baseline random walk each update
        float horizDependentVarianceProp = 0.03f;  // multiplied by |dVert| [sim]
        float horizConstantNoise = 0.03f;          // inches/update baseline [robot]

        // Trades kidnap latency for smoothness, and only applies when the EKF is
        // off -- with it on, it owns the writeback and this is inert. 0.06 is
        // 1.75x smoother than 0.10 for 0.3 s more recovery; 0.03 is 3.1x for
        // 1.1 s. Raise it back toward 0.10 if recovery speed matters more.
        float distSyncProp = 0.06f;        // lerp factor for syncing to chassis [sim]
        float thetaSyncProp = 0.001f;   // [sim]
        float updatePeriodMs = 25.0f;      // background task interval [yours]
        int confidenceThreshold = 10;      // min sensor confidence [yours]
        float minSensorRange = 1.0f;       // mm — readings below this are invalid [yours]
                                           //   (increase if sensors are recessed)

        // A beam is dropped when the obstacle is what it actually RANGED: inside
        // the cone AND at the reported distance. Cone alone blanks a wide region
        // and discards good wall readings; a pencil ray misses most real hits.
        // Judged per particle (see updateWeights) -- deciding once from the
        // estimate would use the pose to choose which readings may correct it.
        float obstacleConeFrac = 0.22f;    // tan(12.5 deg), the V5 half-cone [model]
        float obstacleRangeTol = 8.0f;     // inches; reading vs obstacle surface [model]

        // Score for a beam blocked for this particle. Not 1.0, or particles would
        // win by hiding behind obstacles.
        float deadzoneLikelihood = 0.25f;   // [model]

        // Beam model mixture. A pure Gaussian says a reading 6 in short is
        // impossible, so one robot in the way annihilates the whole cloud; the
        // extra terms make such a reading uninformative instead of fatal, and
        // give the tails a slope so a badly placed particle still knows which
        // way to improve. Residuals are in sigmas, which folds the short term's
        // decay length into the range-dependent sigma.
        // The floor has to stay small. At 0.02 a beam reading 95 in away from
        // what a particle predicted costs the same as one reading 8 sigma away,
        // so a particle on the wrong side of the field is competitive with the
        // right one and the cloud migrates to a false mode. Together these bottom
        // out near 1e-3, matching the penalty the plain Gaussian used to apply,
        // while keeping a slope out to ~5 sigma where the old model had a cliff.
        // Beam sigma. Close readings carry roughly fixed noise, far ones scale
        // with the reading. Measure both with Characterizer::measureRangeNoise()
        // rather than trusting these: they came off one set of sensors.
        float rangeSigmaCloseIn = 0.787f;   // used at or below 200 mm [robot]
        float rangeSigmaProp = 0.05f;       // fraction of the reading beyond that [robot]

        float hitWeight = 1.0f;          // weight on the Gaussian [sim]
        float shortWeight = 0.50f;       // weight on the unexpected-obstacle tail [sim]
        float shortDecaySigmas = 1.2f;   // decay of that tail, in sigmas [sim]
        float randWeight = 0.0008f;      // floor; no single beam can zero a particle [sim]

        // Augmented MCL recovery (Probabilistic Robotics 8.3, in logs). A
        // kidnapped robot makes every particle equally wrong, so N_eff stays
        // high and plain resampling never fires. Comparing a fast and a slow
        // average of the measurement likelihood detects that, and the gap sets
        // how much of the cloud is redrawn across the field.
        // alphaFast must stay slow enough that ordinary measurement noise does
        // not reach injectResampleFloor: forcing a resample every cycle adds
        // distResampleVariance of jitter every cycle, which is a random walk
        // that dwarfs the drift the filter exists to correct.
        float alphaSlow = 0.005f;   // [model]
        float alphaFast = 0.10f;   // [model]
        //
        // OFF by default, and not yet safe to turn on. With injection enabled a
        // parked filter migrates to a false mode within ~1.5 s at poses where
        // only two beams are in range: an injected particle can point one beam
        // at a post, get it deadzoned for log(deadzoneLikelihood) = -1.4, and so
        // out-score the truth on its one remaining beam. deadzoneLikelihood has
        // to come down to about the beam-model floor before this can ship on --
        // hiding behind an obstacle must not be cheaper than measuring a wall.
        // Set to 0.10 to exercise it: recovery from a 36 in displacement goes
        // from never to 0.2 in within 3 s.
        float maxRandomFrac = 0.0f;      // cap on the share redrawn per resample [model]
        float injectResampleFloor = 0.03f;  // below this, do not force a resample [model]

        bool autoSync = true;              // automatically sync pose to odometry [model]
    };

    /// Pose estimate plus the cloud's own account of how sure it is.
    struct Estimate {
        Pose pose;
        float nEff{0.0f};
        float covXX{0.0f}, covXY{0.0f}, covYY{0.0f};  // inches^2
        float covTheta{0.0f};                         // radians^2
    };

    // ========================================================================
    // Compile-time constants (must be before first use as template arguments)
    // ========================================================================

    static constexpr int SENSOR_COUNT = MAX_DISTANCE_SENSORS;
    static constexpr int PARTICLE_MAX = 1024;
    static constexpr int NOISE_POOL_SIZE = 1024;
    static constexpr int NOISE_MASK = NOISE_POOL_SIZE - 1;
    static constexpr int GAUSSIAN_LUT_SIZE = 1024;

    // ========================================================================
    // Field geometry types (used for ray-casting)
    // ========================================================================

    struct LineObstacle {
        Pose p1, p2;
    };

    struct CircleObstacle {
        float x, y, radius;
    };

    // ========================================================================
    // Constructor / Destructor
    // ========================================================================

    MclTracking(OdomTracking& odom,
                std::array<pros::Distance*, SENSOR_COUNT> distanceSensors,
                const Pose& startPose,
                const Config& config);

    ~MclTracking();

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /// Start the background tracking task
    void startTracking();

    /// Stop the background tracking task
    void stopTracking();

    /// Check if tracking task is running
    bool isTracking() const;

    // ========================================================================
    // Core MCL operations
    // ========================================================================

    /// Motion update — propagate particles using decomposed odometry delta.
    /// Reads the per-axis (vertical, horizontal, angular) delta directly from
    /// OdomTracking::getLastDelta().  dVert and dHoriz are independent sensor
    /// measurements, allowing statistically correct per-axis noise.
    void predict();

    /// Sensor update — weight particles by how well they match sensor readings
    void updateWeights();

    /// Systematic low-variance resampling
    void resample();

    /// Get weighted-average pose estimate and effective sample size
    /// Returns {estimatedPose, N_eff}. N_eff below threshold triggers resample.
    std::pair<Pose, float> getEstimate() const;

    /// Weighted mean plus the weighted spread of the cloud. The spread is what
    /// lets a downstream filter weigh this estimate instead of guessing at it.
    Estimate estimate() const;

    // ========================================================================
    // Pose management
    // ========================================================================

    /// Set the robot pose (reinitializes particles around this pose)
    void setPose(const Pose& pose);

    /// Reset all particles uniformly across the field
    void uniformReset();

    /// Run one full MCL iteration (predict + updateWeights + resample if needed + sync)
    /// Returns the new estimated pose
    Pose update();

    /// Smoothly interpolate MCL estimate into odometry pose
    void syncToOdometry();

    // ========================================================================
    // Sensor management (8 sensors, indexed 0-7)
    // ========================================================================

    /// Replace all distance sensors (e.g. after construction)
    void setDistanceSensors(const std::array<pros::Distance*, SENSOR_COUNT>& sensors);

    /// Set sensor mount offsets (robot-frame: x=forward, y=left, theta=pointing angle).
    /// Call once after construction with your robot's measured sensor positions.
    void setSensorMounts(const std::array<Pose, SENSOR_COUNT>& mounts);

    /// Enable a specific sensor
    void enableSensor(int index);

    /// Disable a specific sensor
    void disableSensor(int index);

    /// Temporarily disable a sensor for a duration (milliseconds)
    void disableSensorFor(int index, float durationMs);

    // ========================================================================
    // Obstacles (for masking out known field elements)
    // ========================================================================

    /// Set dynamic obstacles that sensors should ignore
    void setObstacles(const std::vector<LineObstacle>* lineObstacles = nullptr,
                      const std::vector<CircleObstacle>* circleObstacles = nullptr);

    /// Set tracking wheel drift compensation
    void setDrift(float verticalDrift, float horizontalDrift);

    // ========================================================================
    // Accessors
    // ========================================================================

    /// Get the raw MCL estimate (before sync)
    Pose getRawEstimate() const;

    /// Get the distance sensor readings (inches), index 0-7
    const std::array<float, SENSOR_COUNT>& sensorReadings() const { return mSensorReadingsInch; }

    /// Share of the particle cloud that treated each beam as deadzoned (0..1).
    const std::array<float, SENSOR_COUNT>& deadzoneFraction() const { return mDeadzoneFrac; }

    /// Get sensor validity flags
    const std::array<bool, SENSOR_COUNT>& validSensors() const { return mValidSensors; }

    /// Set the distance sync proportion
    void setDistSyncProp(float prop) { mConfig.distSyncProp = prop; }

    /// Whether this filter writes its estimate back into odometry. Turn it off
    /// when a downstream filter owns the fused pose, or the two fight over it.
    void setAutoSync(bool enabled) { mConfig.autoSync = enabled; }

    /// Share of the cloud redrawn across the field on the last resample. Rises
    /// above 0 when the measurements stop fitting — i.e. after a kidnapping.
    float randomInjectFraction() const { return mInjectFrac; }

    /// Copy the live particle cloud out for visualisation or telemetry.
    /// Writes 4 floats per particle (x, y, theta, weight) into `out`, up to
    /// `maxParticles`. Returns the number actually written.
    int snapshotParticles(float* out, int maxParticles) const;

    /// Read-only view of the active configuration.
    const Config& config() const { return mConfig; }

    /// Replace the tuning parameters on a running filter. particleCount is held
    /// at its current value — resizing the cloud mid-run would invalidate the
    /// buffers the background task is walking.
    void setConfig(const Config& config) {
        int particles = mConfig.particleCount;
        mConfig = config;
        mConfig.particleCount = particles;
        buildGaussianLUT();   // the mixture weights are baked into it
    }

    /// Reseed the particle RNG and regenerate the noise pool.
    ///
    /// The constructor seeds from std::random_device, which is what you want on
    /// a robot. Pinning the seed makes a run reproducible instead — the same
    /// inputs then produce the same particle cloud every time, which is what
    /// lets a divergence be replayed in a simulator or a regression test.
    /// Call immediately after construction, before startTracking().
    void setSeed(std::uint32_t seed);

private:
    // ========================================================================
    // Internal types
    // ========================================================================

    struct Particle {
        Pose pose;
        float weight = 1.0f;
        // Weights are accumulated in logs and exponentiated relative to the
        // cloud maximum. Multiplying eight beam likelihoods directly underflows
        // to zero for every particle at once, and the recovery from that is a
        // uniform reset — throwing the filter away exactly when it was only
        // slightly wrong.
        float logWeight = 0.0f;
    };

    struct Trig {
        float cosVal, sinVal;
    };

    // ========================================================================
    // Field geometry (VRC field — 144" × 144", inner 140.4" × 140.4")
    //
    // MODIFY PER SEASON: Update the arrays below to match the current game's
    // field elements.  Non-disabling obstacles are targets that sensors CAN
    // detect (cast against during particle weighting).  Disabling obstacles
    // block sensor view of walls — if a sensor ray hits one, the reading is
    // marked invalid.
    // ========================================================================

    /// Field perimeter walls (never changes — 140.4" × 140.4" interior)
    static constexpr LineObstacle kFieldWalls[4] = {
        {{-70.2f, -70.2f}, { 70.2f, -70.2f}},  // bottom
        {{ 70.2f, -70.2f}, { 70.2f,  70.2f}},  // right
        {{ 70.2f,  70.2f}, {-70.2f,  70.2f}},  // top
        {{-70.2f,  70.2f}, {-70.2f, -70.2f}}   // left
    };

    /// Non-disabling line targets — field elements that sensors CAN detect.
    /// Ray-cast against these (in priority order before walls) when weighting
    /// particles.  Add goal legs, barriers, etc. for the current season.
    static constexpr LineObstacle kFieldTargets[] = {
        // Example — middle goal diagonal legs (2025-26 "High Stakes"):
        // {{-5.0f,  5.0f}, { 5.0f, -5.0f}},
        // {{-5.0f, -5.0f}, { 5.0f,  5.0f}},
        // TODO: add season-specific line targets here
    };

    /// Non-disabling circle targets — cylindrical field elements sensors can hit.
    /// Ray-cast against these (before walls) when weighting particles.
    static constexpr CircleObstacle kFieldCircles[] = {
        // Match loader posts (4 corners) — present on every VRC field
        // {-67.635f,  46.765f, 2.00f}, {-67.635f, -46.765f, 2.00f},
        // { 67.635f,  46.765f, 2.00f}, { 67.635f, -46.765f, 2.00f},
        // TODO: add season-specific circular targets here
    };

    /// Disabling line obstacles — if a sensor ray hits one of these BEFORE
    /// any wall/target, the reading is invalid (not a field-wall measurement).
    static constexpr LineObstacle kDisablingLines[] = {
        // Example — center goal structure that blocks sensors:
        // {{-14.0f, 14.0f}, {14.0f, -14.0f}},
        // {{-14.0f, -14.0f}, {14.0f, 14.0f}},
        // TODO: add season-specific disabling obstacles here
    };

    // ========================================================================
    // Ray-casting helpers
    // ========================================================================

    /// Intersect a ray with a line segment; returns distance or maxRange if no hit
    float intersectLine(float rayX, float rayY, const LineObstacle& wall,
                        float maxRange, float rayCos, float raySin) const;

    /// Intersect a ray with a circle; returns distance or maxRange if no hit
    float intersectCircle(float rayX, float rayY, const CircleObstacle& circle,
                          float maxRange, float dx, float dy) const;

    /// Cast a ray from a particle through a sensor and find nearest obstacle
    float castRay(const Particle& particle, int sensorIndex) const;

    /// Build the log-likelihood LUT over the SIGNED residual in sigmas, so one
    /// lookup covers the whole beam mixture.
    void buildGaussianLUT();

    /// Log-likelihood of a residual of `sigmas` (signed: negative = the reading
    /// came back shorter than the particle expected).
    float logLikelihood(float sigmas) const;

    // ========================================================================
    // Noise generation
    // ========================================================================

    float nextNoise();
    void regenerateNoise();

    // ========================================================================
    // Background task
    // ========================================================================

    static void taskLoop(void* param);
    void run();

    // ========================================================================
    // Member variables
    // ========================================================================

    Config mConfig;
    OdomTracking& mOdom;
    std::array<pros::Distance*, SENSOR_COUNT> mDistanceSensors;

    // Sensor state
    std::array<bool, SENSOR_COUNT> mValidSensors{};
    // Share of the cloud that ignored each beam as deadzoned (telemetry).
    std::array<float, SENSOR_COUNT> mDeadzoneFrac{};
    std::array<int, SENSOR_COUNT> mDeadzoneHits{};
    std::array<int, SENSOR_COUNT> mSensorReadingsMm{};
    std::array<float, SENSOR_COUNT> mSensorReadingsInch{};
    std::array<int, SENSOR_COUNT> mSensorConfs{};
    std::array<bool, SENSOR_COUNT> mDisabledSensors{};
    std::array<float, SENSOR_COUNT> mDisableTimers{};

    // Obstacle pointers (externally managed)
    const std::vector<LineObstacle>* mLineObstacles{nullptr};
    const std::vector<CircleObstacle>* mCircleObstacles{nullptr};

    // Particles (double-buffered for resampling)
    std::array<Particle, PARTICLE_MAX> mParticlesA;
    std::array<Particle, PARTICLE_MAX> mParticlesB;
    std::array<Particle, PARTICLE_MAX>* mParticles{&mParticlesA};
    std::array<Particle, PARTICLE_MAX>* mNewParticles{&mParticlesB};


    // Beam-model log-likelihood over the signed residual, +/- LUT_SIGMA_SPAN.
    static constexpr float LUT_SIGMA_SPAN = 8.0f;
    alignas(64) float mGaussianLUT[GAUSSIAN_LUT_SIZE]{};

    // Sensor mount poses (robot-frame offsets + pointing angles)
    std::array<Pose, SENSOR_COUNT> mSensorMounts{};
    std::array<Trig, SENSOR_COUNT> mMountTrigs{};

    // Noise pool
    alignas(64) std::array<float, NOISE_POOL_SIZE> mNoisePool{};
    int mNoiseIdx{0};
    int mRegenIdx{0};

    // RNG
    std::mt19937 mGen;
    std::normal_distribution<float> mDist;

    // Drift compensation (per-update values — setDrift divides by update rate)
    float mVertDrift{0.0f};
    float mHorizDrift{0.0f};

    // State
    Pose mLastResamplePose{};
    Pose mRawEstimate{};
    float mLastImuHeading{0.0f};
    float mLatestSpeed{0.0f};

    // Augmented-MCL recovery state. Slow and fast averages of the log
    // measurement likelihood; their gap is the evidence that the cloud is in
    // the wrong place rather than merely spread out.
    float mLogWSlow{0.0f};
    float mLogWFast{0.0f};
    bool  mHaveLogAvg{false};
    float mInjectFrac{0.0f};

    // Background task.
    // mRunning is atomic: written by stopTracking() (caller task) and read by
    // the loop in run() (worker task) — a cross-task flag.
    pros::Task* mTask{nullptr};
    std::atomic<bool> mRunning{false};
};

}  // namespace riku
