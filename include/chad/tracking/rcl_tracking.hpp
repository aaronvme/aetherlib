#pragma once

#include <array>
#include <atomic>
#include <utility>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"

#include "chad/tracking/odom_tracking.hpp"
#include "chad/util/util.hpp"

namespace chad {
enum class CoordType { X, Y, INVALID };

class RclTracking {
public:

    struct RclSensor {
        pros::Distance* sensor{nullptr};
        Pose mountOffset;
        bool enabled{true};

        float worldX{0.0f}, worldY{0.0f};
        float rayCos{0.0f}, raySin{0.0f};
        float worldHeading{0.0f};
        float readingInch{0.0f};
        int confidence{0};
        bool valid{false};
    };

    struct LineObstacle {
        float x1, y1, x2, y2;
        float lifetimeMs = 0.0f;
    };

    struct CircleObstacle {
        float x, y, radius;
        float lifetimeMs = 0.0f;
    };

    struct Config {
        int sensorCount = MAX_DISTANCE_SENSORS;
        float maxSyncDist = 5.0f;

        bool hardSync = false;
        float syncProp = 0.15f;

        float maxSyncRate = 8.0f;

        float accumulationAlpha = 0.3f;
        float updatePeriodMs = 10.0f;
        float angleTolerance = 10.0f;

        float maxWallInnovation = 8.0f;

        float recoverySyncProp = 0.9f;

        int displaceCycles = 3;

        float noReturnWallLimit = 35.0f;

        int confidenceThreshold = 30;

        float obstacleConeFrac = 0.22f;
        float obstacleRangeTol = 6.0f;
        bool autoSync = true;
        bool useAccumulation = false;
    };

    RclTracking(OdomTracking& odom, const Config& config);
    ~RclTracking();

    void setSensor(int index, pros::Distance* sensor, const Pose& mountOffset);

    void configureSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors,
                          const std::array<Pose, MAX_DISTANCE_SENSORS>& mounts);

    void enableSensor(int index);
    void disableSensor(int index);
    void disableSensorFor(int index, float durationMs);

    void addLineObstacle(float x1, float y1, float x2, float y2, float lifetimeMs = 0.0f);
    void addCircleObstacle(float x, float y, float radius, float lifetimeMs = 0.0f);
    void clearObstacles();

    void update();

    void updateSensorPoses(const Pose& robotPose);

    enum class ReadingClass {
        Invalid,
        WallHit,
        Blocked,

        TooFar,

        Impossible

    };

    ReadingClass classifyReading(int index, float& wallDist) const;

    bool isValidReading(int index) const;

    bool isDisplaced() const { return mDisplaced; }

    std::pair<CoordType, float> getBotCoordFromSensor(int index) const;

    struct AxisEstimate {
        float x{0.0f}, y{0.0f};
        int xCount{0}, yCount{0};
    };
    AxisEstimate estimateAxes() const;

    std::pair<Pose, int> computePose() const;

    void startTracking();

    void stopTracking();

    bool isTracking() const;

    void syncUpdate();

    bool updateBotPoseFromBestSensor();

    void setRclPose(const Pose& pose) { mLatestEstimate = pose; }

    const Config& config() const { return mConfig; }

    void setConfig(const Config& config) {
        int count = mConfig.sensorCount;
        mConfig = config;
        mConfig.sensorCount = count;
    }

    void setAutoSync(bool enabled) { mConfig.autoSync = enabled; }

    const std::array<RclSensor, MAX_DISTANCE_SENSORS>& sensors() const { return mSensors; }
    Pose latestEstimate() const { return mLatestEstimate; }
    int activeSensorCount() const { return mActiveSensorCount; }

private:

    float intersectLineSegment(float rx, float ry, float rCos, float rSin,
                                float x1, float y1, float x2, float y2,
                                float maxRange) const;

    std::pair<int, float> nearestWall(const RclSensor& s) const;

    static void taskLoop(void* param);
    void run();

    Config mConfig;
    OdomTracking& mOdom;
    std::array<RclSensor, MAX_DISTANCE_SENSORS> mSensors;
    std::vector<LineObstacle> mLineObstacles;
    std::vector<CircleObstacle> mCircleObstacles;

    Pose mLatestEstimate;
    int mActiveSensorCount{0};

    bool mXMeasured{false};
    bool mYMeasured{false};

    float mResidualX{0.0f};
    float mResidualY{0.0f};

    bool mDisplaced{false};
    int mDisplaceCycles{0};

    std::array<float, MAX_DISTANCE_SENSORS> mDisableTimers{};

    pros::Task* mTask{nullptr};
    std::atomic<bool> mRunning{false};
};
}
