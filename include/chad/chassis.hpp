#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <vector>

#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

#include "chad/drivetrain.hpp"
#include "chad/movement_control/movetopoint.hpp"
#include "chad/movement_control/turntopoint.hpp"
#include "chad/movement_control/arc.hpp"
#include "chad/movement_control/boomerang.hpp"
#include "chad/movement_control/feedforward.hpp"
#include "chad/movement_control/ramsete.hpp"
#include "chad/util/field_mirror.hpp"
#include "chad/util/motion_slot.hpp"
#include "chad/movement_control/velocity_profiles.hpp"
#include "chad/tracking/ekf_tracking.hpp"
#include "chad/tracking/mcl_tracking.hpp"
#include "chad/tracking/odom_tracking.hpp"
#include "chad/tracking/rcl_tracking.hpp"
#include "chad/util/pid.hpp"
#include "chad/util/util.hpp"

namespace chad {
struct ChassisConfig {
    PIDGains lateralGains{10.0f, 0.0f, 0.3f};
    PIDGains angularGains{100.0f, 0.0f, 5.0f};

    float lateralWindupRange{0.0f};
    bool lateralFlipReset{false};

    float angularWindupRange{0.0f};
    bool angularFlipReset{false};

    float lateralDerivativeFilter{0.1f};
    float angularDerivativeFilter{0.1f};

    float lateralSlewPerSec{0.0f};

    DriveCurve throttleCurve;
    DriveCurve steerCurve;

    float targetToleranceIn{1.0f};
    float targetToleranceDeg{2.0f};
    float bearingHoldDist{1.5f};

    std::array<pros::Distance*, MAX_DISTANCE_SENSORS> distanceSensors{};

    std::array<Pose, MAX_DISTANCE_SENSORS> sensorMounts{};

    uint32_t settleTimeMs{0};

    bool useMclTracking{false};
    bool useRclTracking{false};
    bool useEkfTracking{false};
    MclTracking::Config mclConfig;
    RclTracking::Config rclConfig;
    EkfTracking::Config ekfConfig;
};

struct ChassisSample {
    int type{0};
    float x{0}, y{0}, thetaDeg{0};
    float targetX{0}, targetY{0};
    float lateralOut{0}, angularOut{0};
    bool settled{false};
};

class Chassis {
public:

    Chassis(Drivetrain& drivetrain, const OdomSensors& sensors,
            const ChassisConfig& config = {});

    ~Chassis();

    void calibrate();

    void setPose(const Pose& pose);

    // setPose without the mirror. Sensor fixes already read the real field,
    // so a mirrored routine must not transform them a second time.
    void setPoseFieldFrame(const Pose& pose);

    // Replays a routine written for one alliance on the other. Applied to
    // every motion target as it is queued, so the routine itself keeps its
    // original coordinates.
    void setMirror(FieldMirror mirror);
    FieldMirror mirror() const;

    void setPosition(float x, float y);

    void setHeading(float thetaDeg);

    Pose getPose() const;

    void tank(float left, float right);

    void arcade(float throttle, float steer);

    void curvature(float throttle, float steer);

    void moveDistance(float target, float timeout = 0, const MoveDistanceParams& params = {}, bool async = false);

    void moveToPoint(float x, float y, float timeout = 0, const MoveToPointParams& params = {}, bool async = false);

    void turnToHeading(float thetaDeg, float timeout = 0, const TurntoHeadingParams& params = {}, bool async = false);

    void turnToPoint(float x, float y, float timeout = 0, const TurnToPointParams& params = {}, bool async = false);

    void moveArc(float x, float y, float radius, float timeout = 0, const ArcParams& params = {}, bool async = false);

    void moveBoomerang(float x, float y, float thetaDeg, float timeout = 0, const BoomerangParams& params = {}, bool async = false);

    void moveRAMSETE(const std::vector<Pose>& path, float timeout = 0, const RAMSETEParams& params = {}, bool async = false);

    void swingToHeading(float thetaDeg, SwingSide side, float timeout = 0, const SwingParams& params = {}, bool async = false);

    bool isSettled() const;

    void cancelMotion();

    void abortMotion();

    void waitUntilSettled();

    void waitUntilDist(float dist);

    OdomTracking& odom();
    RclTracking& rcl();
    MclTracking& mcl();
    EkfTracking& ekf();

    void startTracking();
    void stopTracking();

    void setPidTuning(const ChassisConfig& config);

    void setLateralGains(const PIDGains& gains);
    void setAngularGains(const PIDGains& gains);

    void setSettleTolerances(float toleranceIn, float toleranceDeg,
                            float bearingHoldDist = -1.0f);

    void setLateralSlew(float unitsPerSec);
    void setThrottleCurve(const DriveCurve& curve);
    void setSteerCurve(const DriveCurve& curve);

    void setBrakeMode(pros::motor_brake_mode_e mode);

    void setDistanceSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors);

    void setDryRun(bool enabled);
    bool isDryRun() const;

    const std::vector<Pose>& dryRunPath() const;

    void resetDryRunPath();

    void setTelemetry(std::function<void(const ChassisSample&)> callback);

private:

    void startMotion();
    void endMotion(uint32_t seq);

    void resetPose(const Pose& pose);

    float computeLateralError(float targetX, float targetY) const;
    float computeAngularError(float targetHeadingRad) const;
    float computeAngularErrorToPoint(float targetX, float targetY) const;

    void runMotionTask();
    void executeCurrentMotion();
    void ensureMotionTask();

    enum class MotionType { None, Distance, Point, Heading, TurnPoint, Arc, Boomerang, RAMSETE, Swing };
    struct MotionRequest {
        MotionType type{MotionType::None};
        float targetX{0}, targetY{0}, targetHeadingRad{0}, targetRadius{0};
        float targetDistance{0};
        float timeout{0};
        uint32_t startTime{0};
        MoveDistanceParams moveDistParams;
        MoveToPointParams movePointParams;
        TurntoHeadingParams turnHeadingParams;
        TurnToPointParams turnPointParams;
        ArcParams arcParams;
        BoomerangParams boomerangParams;
        RAMSETEParams ramseteParams;
        SwingParams swingParams;
        SwingSide swingSide{SwingSide::Left};
        std::vector<Pose> ramsetePath;
    };

    void enqueueMotion(MotionRequest&& req, bool async);

    TurnDirection mirrorTurn(TurnDirection dir) const;
    void applyMirror(MotionRequest& req) const;

    FieldMirror mMirror{FieldMirror::None};

    Drivetrain& mDrivetrain;
    ChassisConfig mConfig;

    OdomTracking mOdom;
    RclTracking mRcl;
    MclTracking mMcl;
    EkfTracking mEkf;

    PID mLateralPID;
    PID mAngularPID;

    FeedforwardController mRamseteFf;

    std::atomic<bool> mMotionRunning{false};
    std::atomic<bool> mTaskShouldStop{false};
    MotionRequest mCurrentMotion;
    uint32_t mCurrentSeq{0};
    MotionSlot<MotionRequest> mMotionSlot;
    pros::Mutex mQueueMutex;
    pros::Task* mMotionTask{nullptr};

    DriveCurve mThrottleCurve;
    DriveCurve mSteerCurve;

    std::function<void(const ChassisSample&)> mTelemetry;

    bool mDryRun{false};
    std::vector<Pose> mDryRunPath;
    Pose mDryRunStartPose;
    Pose mDryRunPose;
};
}
