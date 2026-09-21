#pragma once

#include <array>
#include <atomic>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"

#include "chad/tracking/mcl_tracking.hpp"
#include "chad/tracking/odom_tracking.hpp"
#include "chad/util/util.hpp"

namespace chad {
class EkfTracking {
public:
    static constexpr int SENSOR_COUNT = MAX_DISTANCE_SENSORS;

    struct Config {
        float vertNoiseProp{0.045f};
        float horizNoiseProp{0.045f};
        float slipNoiseProp{0.035f};
        float slipNoiseFloor{0.010f};

        float vertBiasProp{0.06f};
        float slipBiasProp{0.05f};
        float biasDecorrIn{40.0f};
        float thetaNoiseProp{0.020f};
        float thetaNoisePerInch{0.0015f};
        float posNoiseFloor{0.015f};
        float thetaNoiseFloor{0.0004f};

        bool  useRangeUpdates{true};
        float rangeSigmaBase{0.60f};
        float rangeSigmaProp{0.030f};
        float rangeGateChi2{9.0f};

        float maxIncidenceRad{1.15f};
        float maxRangeIn{100.0f};
        float sentinelMm{9000.0f};
        float minRangeMm{20.0f};
        int   minConfidence{10};

        float obstacleConeFrac{0.22f};
        float obstacleRangeTol{6.0f};
        float obstacleSigmaPad{2.0f};

        bool  useImuHeading{true};
        float imuSigmaRad{0.006f};

        bool  useMclUpdates{true};

        float mclSigmaScale{1.6f};
        float mclSigmaFloor{1.2f};
        float mclGateChi2{11.8f};

        int   mclResetCycles{10};
        float mclMinNeffFrac{0.20f};

        bool  autoSync{true};

        float syncProp{0.12f};
        float thetaSyncProp{0.20f};
        float updatePeriodMs{10.0f};
    };

    EkfTracking(OdomTracking& odom,
                std::array<pros::Distance*, SENSOR_COUNT> distanceSensors,
                const Pose& startPose,
                const Config& config);
    ~EkfTracking();

    void startTracking();
    void stopTracking();
    bool isTracking() const;

    Pose update();

    Pose getEstimate() const;

    struct Uncertainty {
        float sigmaX{0.0f}, sigmaY{0.0f}, sigmaTheta{0.0f};
    };
    Uncertainty uncertainty() const;

    void setPose(const Pose& pose);

    void setPose(const Pose& pose, float posSigma, float thetaSigma);

    void attachMcl(MclTracking* mcl) { mMcl = mcl; }

    void setSensorMounts(const std::array<Pose, SENSOR_COUNT>& mounts);
    void setDistanceSensors(const std::array<pros::Distance*, SENSOR_COUNT>& sensors);

    void setObstacles(const std::vector<MclTracking::CircleObstacle>* circles) {
        mCircleObstacles = circles;
    }

    enum class BeamState : std::uint8_t {
        Absent,
        NoReturn,
        Deadzone,
        Grazing,
        Rejected,
        Used,
    };
    const std::array<BeamState, SENSOR_COUNT>& beamStates() const { return mBeamStates; }
    const std::array<float, SENSOR_COUNT>& innovations() const { return mInnovations; }

    int beamsUsed() const { return mBeamsUsed; }

    int mclRejectStreak() const { return mMclRejectStreak; }

    int relocateCount() const { return mRelocateCount; }

    /// How long the last filter cycle took, against updatePeriodMs. At or
    /// over the period means the brain is not keeping up.
    std::uint32_t lastCycleMs() const { return mLastCycleMs.load(); }

    const Config& config() const { return mConfig; }
    void setConfig(const Config& config) { mConfig = config; }

private:

    void predict(const OdomDelta& delta);
    void updateRanges();
    void updateImuHeading();
    void updateMcl();

    bool applyScalarUpdate(const float H[3], float innovation, float R,
                           float gateChi2);

    struct WallHit {
        bool  valid{false};
        float range{0.0f};
        float nx{0.0f}, ny{0.0f};
        float cosIncidence{0.0f};
    };
    WallHit castWalls(float sx, float sy, float rayCos, float raySin) const;

    bool obstacleOwnsReading(float sx, float sy, float rayCos, float raySin,
                             float readingIn, float wallRange) const;

    static void taskLoop(void* param);
    void run();

    Config mConfig;
    OdomTracking& mOdom;
    MclTracking* mMcl{nullptr};
    std::array<pros::Distance*, SENSOR_COUNT> mDistanceSensors;
    const std::vector<MclTracking::CircleObstacle>* mCircleObstacles{nullptr};

    std::array<Pose, SENSOR_COUNT> mSensorMounts{};

    float mState[3]{0.0f, 0.0f, 0.0f};
    float mP[9]{};

    std::array<BeamState, SENSOR_COUNT> mBeamStates{};
    std::array<float, SENSOR_COUNT> mInnovations{};
    int mBeamsUsed{0};
    int mMclRejectStreak{0};
    int mRelocateCount{0};
    bool mSnapNextSync{false};

    mutable pros::Mutex mMutex;

    std::atomic<std::uint32_t> mLastCycleMs{0};

    pros::Task* mTask{nullptr};
    std::atomic<bool> mRunning{false};
};
}
