#pragma once

#include <array>
#include <atomic>
#include <random>
#include <utility>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"

#include "chad/tracking/odom_tracking.hpp"
#include "chad/util/fasttrig.hpp"
#include "chad/util/util.hpp"

namespace chad {
class MclTracking {
public:

    struct Config {
        int particleCount = 1024;
        int resampleThreshold = 512;
        float maxSensorRange = 100.0f;

        float distResampleVariance = 1.5f;
        float thetaResampleVariance = 0.01f;
        float maxThetaDeviation = 0.08f;

        float trackingWheelVariance = 0.08f;
        float imeVariance = 0.12f;
        float imuVariance = 0.005f;

        float horizDependentVarianceProp = 0.03f;
        float horizConstantNoise = 0.03f;

        float distSyncProp = 0.06f;
        float thetaSyncProp = 0.001f;
        float updatePeriodMs = 25.0f;
        int confidenceThreshold = 10;
        float minSensorRange = 1.0f;

        float obstacleConeFrac = 0.22f;
        float obstacleRangeTol = 8.0f;

        float deadzoneLikelihood = 0.25f;

        float rangeSigmaCloseIn = 0.787f;
        float rangeSigmaProp = 0.05f;

        float hitWeight = 1.0f;
        float shortWeight = 0.50f;
        float shortDecaySigmas = 1.2f;
        float randWeight = 0.0008f;

        float alphaSlow = 0.005f;
        float alphaFast = 0.10f;

        float maxRandomFrac = 0.0f;
        float injectResampleFloor = 0.03f;

        // Above this the beams sweep further per cycle than the particle
        // spread can bracket, so the weights say only that every particle is
        // wrong. Coast on odometry instead. Zero disables the gate.
        float maxTurnRateRadS = 0.0f;

        bool autoSync = true;
    };

    struct Estimate {
        Pose pose;
        float nEff{0.0f};
        float covXX{0.0f}, covXY{0.0f}, covYY{0.0f};
        float covTheta{0.0f};
    };

    static constexpr int SENSOR_COUNT = MAX_DISTANCE_SENSORS;
    static constexpr int PARTICLE_MAX = 1024;
    static constexpr int NOISE_POOL_SIZE = 1024;
    static constexpr int NOISE_MASK = NOISE_POOL_SIZE - 1;
    static constexpr int GAUSSIAN_LUT_SIZE = 1024;

    struct LineObstacle {
        Pose p1, p2;
    };

    struct CircleObstacle {
        float x, y, radius;
    };

    MclTracking(OdomTracking& odom,
                std::array<pros::Distance*, SENSOR_COUNT> distanceSensors,
                const Pose& startPose,
                const Config& config);

    ~MclTracking();

    void startTracking();

    void stopTracking();

    bool isTracking() const;

    void predict();

    void updateWeights();

    void resample();

    std::pair<Pose, float> getEstimate() const;

    Estimate estimate() const;

    void setPose(const Pose& pose);

    void uniformReset();

    Pose update();

    void syncToOdometry();

    void setDistanceSensors(const std::array<pros::Distance*, SENSOR_COUNT>& sensors);

    void setSensorMounts(const std::array<Pose, SENSOR_COUNT>& mounts);

    void enableSensor(int index);

    void disableSensor(int index);

    void disableSensorFor(int index, float durationMs);

    void setObstacles(const std::vector<LineObstacle>* lineObstacles = nullptr,
                      const std::vector<CircleObstacle>* circleObstacles = nullptr);

    void setDrift(float verticalDrift, float horizontalDrift);

    Pose getRawEstimate() const;

    const std::array<float, SENSOR_COUNT>& sensorReadings() const { return mSensorReadingsInch; }

    const std::array<float, SENSOR_COUNT>& deadzoneFraction() const { return mDeadzoneFrac; }

    const std::array<bool, SENSOR_COUNT>& validSensors() const { return mValidSensors; }

    void setDistSyncProp(float prop) { mConfig.distSyncProp = prop; }

    void setAutoSync(bool enabled) { mConfig.autoSync = enabled; }

    float randomInjectFraction() const { return mInjectFrac; }

    float latestTurnRateRadS() const { return mLatestTurnRate; }

    // True while the turn-rate gate is holding the beams out of the filter.
    bool sensorsHeld() const { return mSensorsHeld; }

    int snapshotParticles(float* out, int maxParticles) const;

    const Config& config() const { return mConfig; }

    void setConfig(const Config& config) {
        int particles = mConfig.particleCount;
        mConfig = config;
        mConfig.particleCount = particles;
        buildGaussianLUT();
    }

    void setSeed(std::uint32_t seed);

private:

    struct Particle {
        Pose pose;
        float weight = 1.0f;

        float logWeight = 0.0f;
    };

    struct Trig {
        float cosVal, sinVal;
    };

    static constexpr LineObstacle kFieldWalls[4] = {
        {{-70.2f, -70.2f}, { 70.2f, -70.2f}},
        {{ 70.2f, -70.2f}, { 70.2f,  70.2f}},
        {{ 70.2f,  70.2f}, {-70.2f,  70.2f}},
        {{-70.2f,  70.2f}, {-70.2f, -70.2f}}
    };

    static constexpr LineObstacle kFieldTargets[] = {
    };

    static constexpr CircleObstacle kFieldCircles[] = {
    };

    static constexpr LineObstacle kDisablingLines[] = {
    };

    float intersectLine(float rayX, float rayY, const LineObstacle& wall,
                        float maxRange, float rayCos, float raySin) const;

    float intersectCircle(float rayX, float rayY, const CircleObstacle& circle,
                          float maxRange, float dx, float dy) const;

    float castRay(const Particle& particle, int sensorIndex) const;

    void buildGaussianLUT();

    float logLikelihood(float sigmas) const;

    float nextNoise();
    void regenerateNoise();

    static void taskLoop(void* param);
    void run();

    Config mConfig;
    OdomTracking& mOdom;
    std::array<pros::Distance*, SENSOR_COUNT> mDistanceSensors;

    std::array<bool, SENSOR_COUNT> mValidSensors{};

    std::array<float, SENSOR_COUNT> mDeadzoneFrac{};
    std::array<int, SENSOR_COUNT> mDeadzoneHits{};
    std::array<int, SENSOR_COUNT> mSensorReadingsMm{};
    std::array<float, SENSOR_COUNT> mSensorReadingsInch{};
    std::array<int, SENSOR_COUNT> mSensorConfs{};
    std::array<bool, SENSOR_COUNT> mDisabledSensors{};
    std::array<float, SENSOR_COUNT> mDisableTimers{};

    const std::vector<LineObstacle>* mLineObstacles{nullptr};
    const std::vector<CircleObstacle>* mCircleObstacles{nullptr};

    std::array<Particle, PARTICLE_MAX> mParticlesA;
    std::array<Particle, PARTICLE_MAX> mParticlesB;
    std::array<Particle, PARTICLE_MAX>* mParticles{&mParticlesA};
    std::array<Particle, PARTICLE_MAX>* mNewParticles{&mParticlesB};

    static constexpr float LUT_SIGMA_SPAN = 8.0f;
    alignas(64) float mGaussianLUT[GAUSSIAN_LUT_SIZE]{};

    std::array<Pose, SENSOR_COUNT> mSensorMounts{};
    std::array<Trig, SENSOR_COUNT> mMountTrigs{};

    alignas(64) std::array<float, NOISE_POOL_SIZE> mNoisePool{};
    int mNoiseIdx{0};
    int mRegenIdx{0};

    std::mt19937 mGen;
    std::normal_distribution<float> mDist;

    float mVertDrift{0.0f};
    float mHorizDrift{0.0f};

    Pose mLastResamplePose{};
    Pose mRawEstimate{};
    float mLastImuHeading{0.0f};
    float mLatestSpeed{0.0f};
    float mLatestTurnRate{0.0f};
    bool  mSensorsHeld{false};

    float mLogWSlow{0.0f};
    float mLogWFast{0.0f};
    bool  mHaveLogAvg{false};
    float mInjectFrac{0.0f};

    pros::Task* mTask{nullptr};
    std::atomic<bool> mRunning{false};
};
}
