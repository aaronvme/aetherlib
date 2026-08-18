// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include <array>
#include <atomic>
#include <utility>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"

#include "riku/tracking/odom_tracking.hpp"
#include "riku/util/util.hpp"

namespace riku {

// ============================================================================
// RclTracking — Ray/Ceiling-Line Localization using V5 distance sensors
// ============================================================================
//
// Uses distance sensors pointed at field walls and ceiling bar to determine
// absolute position by intersecting sensor rays with known field geometry.
// This provides a direct global position estimate that can be used to:
//   - Initialize MCL particles at match start
//   - Correct odometry drift during autonomous
//   - Detect if the robot has been pushed/grabbed
//
// Typical configuration: 8 distance sensors around the robot perimeter.
// ============================================================================

/// Which coordinate a single sensor constrains.
/// A sensor pointed at an east/west wall determines X; a sensor pointed at a
/// north/south wall determines Y.  A single distance sensor cannot determine
/// both coordinates — the unconstrained coordinate must come from another
/// sensor or from odometry.
enum class CoordType { X, Y, INVALID };

class RclTracking {
public:
    // ========================================================================
    // RclSensor — a single distance sensor with known mount pose
    // ========================================================================

    struct RclSensor {
        pros::Distance* sensor{nullptr};
        Pose mountOffset;          // robot-frame offset (x=forward, y=left, theta=pointing angle)
        bool enabled{true};

        // Computed world-space values (updated each cycle)
        float worldX{0.0f}, worldY{0.0f};
        float rayCos{0.0f}, raySin{0.0f};
        float worldHeading{0.0f};  // world-frame ray direction (radians)
        float readingInch{0.0f};
        int confidence{0};
        bool valid{false};
    };

    // ========================================================================
    // Obstacle types (can block a sensor's view of field walls)
    // ========================================================================

    struct LineObstacle {
        float x1, y1, x2, y2;
        float lifetimeMs = 0.0f;  // 0 = permanent, >0 = temporary
    };

    struct CircleObstacle {
        float x, y, radius;
        float lifetimeMs = 0.0f;
    };

    // ========================================================================
    // Configuration
    // ========================================================================

    struct Config {
        int sensorCount = MAX_DISTANCE_SENSORS;
        float maxSyncDist = 5.0f;          // max distance (inches) to sync per update
        // A wall reading that passed every check is not an estimate to blend
        // toward -- it is the coordinate, to within sensor noise. hardSync writes
        // the measured axes outright, ignoring syncProp and maxSyncRate. The cost
        // is that a misclassified reading teleports the pose instead of nudging
        // it, so the gating ahead of it has to be trusted.
        bool hardSync = false;
        float syncProp = 0.15f;            // fraction of the RCL/odometry error
                                           //   applied per update (0-1)
        float maxSyncRate = 8.0f;          // inches/sec ceiling on ordinary
                                           //   correction, so a path follower
                                           //   never sees the pose step. The
                                           //   displacement path below is
                                           //   deliberately exempt.
        float accumulationAlpha = 0.3f;    // exponential moving average factor
        float updatePeriodMs = 10.0f;      // background task interval
        float angleTolerance = 10.0f;      // degrees — sensor ray must be within this
                                           //   of a wall axis (0°, 90°, 180°, 270°)
        float maxWallInnovation = 8.0f;    // inches — discard a reading that
                                           //   disagrees with the predicted wall
                                           //   distance by more than this
        float recoverySyncProp = 0.9f;     // sync fraction once readings prove
                                           //   the pose wrong — the measurement
                                           //   is absolute, so barely damp it
        int displaceCycles = 3;            // consecutive impossible readings
                                           //   before believing the robot moved
        float noReturnWallLimit = 35.0f;   // inches — a predicted wall closer
                                           //   than this must produce a return,
                                           //   so silence contradicts the pose
        int confidenceThreshold = 30;      // min V5 sensor confidence (0-63)

        // Circle obstacles are tested over the sensor's cone, not a pencil ray,
        // and matched against the reading -- the same rule MCL and the EKF use.
        // The V5 has a 25 degree field of view, so a post a foot off the axis
        // still owns the return, and a pencil ray misses most real post hits.
        float obstacleConeFrac = 0.22f;    // tan(12.5 deg), the V5 half-cone
        float obstacleRangeTol = 6.0f;     // inches; reading vs post surface
        bool autoSync = true;              // automatically sync pose to odometry
        bool useAccumulation = false;      // average multiple readings for stability
    };

    // ========================================================================
    // Constructor / Destructor
    // ========================================================================

    RclTracking(OdomTracking& odom, const Config& config);
    ~RclTracking();

    // ========================================================================
    // Sensor management
    // ========================================================================

    /// Add/replace a sensor at the given index
    void setSensor(int index, pros::Distance* sensor, const Pose& mountOffset);

    /// Configure all sensors from parallel arrays (distance sensors + mount offsets).
    /// Non-null distance entries are enabled with the corresponding mount offset;
    /// null entries are disabled. sensorCount is set to the count of
    /// non-null entries (minimum 1, maximum MAX_DISTANCE_SENSORS).
    void configureSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors,
                          const std::array<Pose, MAX_DISTANCE_SENSORS>& mounts);

    /// Enable/disable a specific sensor
    void enableSensor(int index);
    void disableSensor(int index);
    void disableSensorFor(int index, float durationMs);

    // ========================================================================
    // Obstacle management
    // ========================================================================

    /// Add an obstacle that can block sensor lines-of-sight
    void addLineObstacle(float x1, float y1, float x2, float y2, float lifetimeMs = 0.0f);
    void addCircleObstacle(float x, float y, float radius, float lifetimeMs = 0.0f);
    void clearObstacles();

    // ========================================================================
    // Core RCL operations
    // ========================================================================

    /// Run one localization update cycle (read sensors → compute position)
    void update();

    /// Update world-space positions of all sensors based on current robot pose
    void updateSensorPoses(const Pose& robotPose);

    /// What a reading means relative to the wall the pose says it should hit.
    enum class ReadingClass {
        Invalid,   ///< unusable: out of range, low confidence, skewed, occluded
        WallHit,   ///< agrees with the predicted wall — a usable measurement
        Blocked,   ///< nearer than the predicted wall. Under a pose believed
                   ///< good this is just an object in the beam and carries no
                   ///< information; once the pose is known to be wrong it is a
                   ///< candidate wall reading again.
        TooFar,    ///< further than the wall the current pose predicts. Nothing
                   ///< is visible through a wall, so this is proof the pose is
                   ///< wrong, not merely disagreement with it — an object can
                   ///< only ever shorten a reading, never lengthen it.
        Impossible ///< reports open space where the pose puts a near wall. Also
                   ///< proof the pose is wrong, but it pins down no coordinate.
    };

    /// Classify a reading and report the distance to the wall its ray would hit.
    ReadingClass classifyReading(int index, float& wallDist) const;

    /// Validate a sensor reading (range, confidence, obstacle intersection)
    bool isValidReading(int index) const;

    /// True when readings prove the pose is wrong — pushed, or picked up and
    /// put down somewhere else. RCL corrects hard while this holds.
    bool isDisplaced() const { return mDisplaced; }

    /// Compute robot coordinate from a single sensor intersecting a field wall.
    /// A sensor pointed at an east/west wall returns X; at a north/south wall
    /// returns Y.  A single distance sensor can only constrain one dimension.
    /// Returns {CoordType, coordinate} — CoordType::INVALID if no valid intersection.
    std::pair<CoordType, float> getBotCoordFromSensor(int index) const;

    /// Per-axis estimate.  X and Y are independently averaged from the sensors
    /// that constrain each axis; an axis with no sensor holds the odometry value
    /// and reports a count of 0, so callers can tell a measurement from a
    /// passthrough.
    struct AxisEstimate {
        float x{0.0f}, y{0.0f};
        int xCount{0}, yCount{0};
    };
    AxisEstimate estimateAxes() const;

    /// Derive robot position from all valid sensors.
    /// X and Y are independently averaged from sensors that constrain each axis.
    /// When no sensor constrains an axis, the odometry value is used.
    /// Returns {pose, numSensorsUsed}
    std::pair<Pose, int> computePose() const;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /// Start background localization task
    void startTracking();

    /// Stop background task
    void stopTracking();

    /// Check if running
    bool isTracking() const;

    // ========================================================================
    // Sync
    // ========================================================================

    /// Smoothly blend RCL estimate into odometry pose
    void syncUpdate();

    /// Directly set pose from the best available sensor reading (for initialization)
    bool updateBotPoseFromBestSensor();

    /// Set the RCL internal estimate (called by Chassis::setPose to keep estimates
    /// synchronized)
    void setRclPose(const Pose& pose) { mLatestEstimate = pose; }

    /// Read-only view of the active configuration.
    const Config& config() const { return mConfig; }

    /// Replace the tuning parameters on a running filter. sensorCount is held at
    /// its current value -- it is derived from the sensors actually wired up, not
    /// something to tune.
    void setConfig(const Config& config) {
        int count = mConfig.sensorCount;
        mConfig = config;
        mConfig.sensorCount = count;
    }

    /// Whether this filter writes its estimate back into odometry.
    void setAutoSync(bool enabled) { mConfig.autoSync = enabled; }

    // ========================================================================
    // Accessors
    // ========================================================================

    const std::array<RclSensor, MAX_DISTANCE_SENSORS>& sensors() const { return mSensors; }
    Pose latestEstimate() const { return mLatestEstimate; }
    int activeSensorCount() const { return mActiveSensorCount; }

private:
    // ========================================================================
    // Ray intersection helper
    // ========================================================================

    // Note: field wall geometry is defined in Util.hpp (Field::FIELD_HALF_WALL).
    // getBotCoordFromSensor() determines which wall the ray hits and derives
    // the constrained coordinate (X from east/west walls, Y from north/south).

    /// Line-line intersection: ray from (rx,ry) with direction (cos,sin) × line segment
    /// Returns distance along ray, or maxRange if no intersection
    float intersectLineSegment(float rx, float ry, float rCos, float rSin,
                                float x1, float y1, float x2, float y2,
                                float maxRange) const;

    /// First field wall along a sensor's ray.
    /// Returns {wall id (1=N, 2=E, 3=S, 4=W), distance}, or {-1, 0} if the ray
    /// reaches no wall.
    std::pair<int, float> nearestWall(const RclSensor& s) const;

    // ========================================================================
    // Background task
    // ========================================================================

    static void taskLoop(void* param);
    void run();

    // ========================================================================
    // Members
    // ========================================================================

    Config mConfig;
    OdomTracking& mOdom;
    std::array<RclSensor, MAX_DISTANCE_SENSORS> mSensors;
    std::vector<LineObstacle> mLineObstacles;
    std::vector<CircleObstacle> mCircleObstacles;

    Pose mLatestEstimate;
    int mActiveSensorCount{0};

    // Which axes the last update actually measured. syncUpdate() must not write
    // odometry on an axis whose "estimate" is only odometry echoed back.
    bool mXMeasured{false};
    bool mYMeasured{false};

    // Smoothed offset from odometry, when useAccumulation is set.
    float mResidualX{0.0f};
    float mResidualY{0.0f};

    // Displacement detection — readings that the current pose cannot explain
    bool mDisplaced{false};
    int mDisplaceCycles{0};

    // Timed disable support
    std::array<float, MAX_DISTANCE_SENSORS> mDisableTimers{};

    // mRunning is atomic: written by stopTracking() (caller task), read by the
    // loop in run() (worker task) — a cross-task flag.
    pros::Task* mTask{nullptr};
    std::atomic<bool> mRunning{false};
};

}  // namespace riku
