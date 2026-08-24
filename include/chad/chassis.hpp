// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <queue>
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
#include "chad/movement_control/primitives.hpp"
#include "chad/movement_control/ramsete.hpp"
#include "chad/movement_control/settle_spec.hpp"
#include "chad/movement_control/trajectory_follow.hpp"
#include "chad/movement_control/velocity_profiles.hpp"
#include "chad/tracking/ekf_tracking.hpp"
#include "chad/tracking/mcl_tracking.hpp"
#include "chad/tracking/odom_tracking.hpp"
#include "chad/tracking/rcl_tracking.hpp"
#include "chad/util/pid.hpp"
#include "chad/util/settle.hpp"
#include "chad/util/util.hpp"

namespace chad {

// MotionType and MotionOutcome live in movement_control/settle_spec.hpp, with
// the exit logic they belong to, so both are host-testable.

// ChassisConfig — all configuration needed to construct a Chassis

struct ChassisConfig {
    PIDGains lateralGains{10.0f, 0.0f, 0.3f};
    PIDGains angularGains{100.0f, 0.0f, 5.0f};

    // Lateral PID settings
    float lateralWindupRange{0.0f};
    bool lateralFlipReset{false};

    // Angular PID settings
    float angularWindupRange{0.0f};
    bool angularFlipReset{false};

    // Derivative smoothing for each loop. See PID::setDerivativeFilter -- this
    // is as much a damping control as kD is, because it decides how late kD
    // arrives.
    float lateralDerivativeFilter{0.1f};
    float angularDerivativeFilter{0.1f};

    /// Cap on how fast the lateral output may change, motor units per second.
    /// Applies to the fall as well as the rise -- the stop is what tips a tall
    /// robot, and softer gains only avoid that by never going fast. 0 disables.
    float lateralSlewPerSec{0.0f};

    // Drive curves (driver control)
    DriveCurve throttleCurve;
    DriveCurve steerCurve;

    // Robot-wide settle tolerances; every motion defaults to -1 meaning "use
    // these". targetToleranceIn must exceed the localization error or the settle
    // test can never pass. Phase 4 prints the value it implies.
    float targetToleranceIn{1.0f};    // moveDistance, moveToPoint, RAMSETE
    float targetToleranceDeg{2.0f};   // turnToHeading, turnToPoint, swings
    float bearingHoldDist{1.5f};      // moveToPoint steering hold radius

    // No default movement parameters here: there were four and nothing read
    // them. Change the defaults in movement_control/*.hpp instead.

    // Distance sensors for MCL/RCL tracking
    // Size controlled by MAX_DISTANCE_SENSORS in Util.hpp
    std::array<pros::Distance*, MAX_DISTANCE_SENSORS> distanceSensors{};

    // Sensor mount offsets (robot-frame: x=forward, y=left, theta=pointing angle)
    // Index corresponds to distanceSensors index. Leave zero for unused sensors.
    std::array<Pose, MAX_DISTANCE_SENSORS> sensorMounts{};

    // How long a motion's settle condition must hold continuously before the
    // motion exits. 0 exits on the first sample inside tolerance, which can
    // trigger while still carrying velocity through the target.
    uint32_t settleTimeMs{0};

    // Speed the robot must also be under to count as arrived, on top of the
    // position tolerance. 0 tests position alone, which lets a motion exit while
    // still carrying velocity through the target -- the next queued motion then
    // starts from a pose that is already moving.
    //
    // The signal is a filtered rate of change of the settle error, so pose
    // jitter sets a floor on what value can ever be met: too tight and the
    // motion runs out its timeout instead.
    float settleSpeedIn{0.0f};    // in/s, lateral motions
    float settleSpeedDeg{0.0f};   // deg/s, turns and swings

    // Below this the drivetrain has stopped turning, so the motion is not going
    // to arrive: it gives up rather than hold voltage into a wall for the rest
    // of its timeout. Measured on the wheels, not the pose, so a robot rotating
    // in place does not read as stalled. stallSpeedIn 0 disables both.
    float stallSpeedIn{0.5f};
    // A turn's rest test is its turn rate, because the same wheel speed means a
    // different thing about a turn: 0.5 in/s over an 11.9 in track is 4.8 deg/s,
    // and a turn closing at 4.8 deg/s is converging, not stopped.
    float stallSpeedDeg{2.0f};
    uint32_t stallTimeMs{500};
    // Suppressed for this long after a motion starts, or every standing start
    // reads as a stall.
    uint32_t stallGraceMs{300};

    // Tracking.
    //
    // Odometry always runs — it is the motion model every other filter predicts
    // with, and with no tracking wheels configured it is just the IMU and the
    // motor encoders. MCL and RCL correct it against the field walls. The EKF
    // sits downstream of both: enable it with MCL for a smooth fused pose that
    // still recovers from a displacement.
    bool useMclTracking{false};
    bool useRclTracking{false};
    bool useEkfTracking{false};
    MclTracking::Config mclConfig;
    RclTracking::Config rclConfig;
    EkfTracking::Config ekfConfig;
};

// One motion-loop tick. Angles in VEX degrees.
struct ChassisSample {
    int type{0};             // MotionType of the running motion
    float x{0}, y{0}, thetaDeg{0};
    float targetX{0}, targetY{0};
    float lateralOut{0}, angularOut{0};
    /// SettleMonitor::inTolerance(): inside every arrival test on THIS tick,
    /// whether or not the hold has run out. Not "the motion finished" -- a
    /// curved path can arrive with a heading residual outside tolerance, so it
    /// settles without this ever going true. Use MotionOutcome for that.
    bool inTolerance{false};
};

// Chassis 
//
// Composes tracking, motion control, and driver control into a single API.

class Chassis {
public:
    // Construction

    Chassis(Drivetrain& drivetrain, const OdomSensors& sensors,
            const ChassisConfig& config = {});

    ~Chassis();

    // Calibration & Pose

    /// Calibrate IMU (robot must be stationary for ~2 seconds)
    void calibrate();

    /// Set the robot's field pose
    void setPose(const Pose& pose);

    /// Set position only (heading unchanged)
    void setPosition(float x, float y);

    /// Set heading only (position unchanged)
    /// @param thetaDeg  heading in VEX degrees (0–360, clockwise positive)
    void setHeading(float thetaDeg);

    /// Get the current estimated pose
    Pose getPose() const;

    // Driver Control

    /// Tank drive: left and right joystick values (-127 to 127)
    void tank(float left, float right);

    /// Arcade drive: throttle and steer (-127 to 127)
    void arcade(float throttle, float steer);

    /// Curvature drive: like arcade but with better turn handling
    void curvature(float throttle, float steer);

    // Autonomous Movement (blocking by default, async overload available)
    //
    // All heading values are in VEX degrees (0–360, clockwise positive).
    // timeout in milliseconds — 0 = no timeout.

    /// Drive a specific distance forward/backward
    void moveDistance(float target, float timeout = 0, const MoveDistanceParams& params = {}, bool async = false);

    /// Drive to a field coordinate
    void moveToPoint(float x, float y, float timeout = 0, const MoveToPointParams& params = {}, bool async = false);

    /// Turn to an absolute heading (VEX degrees, 0–360 clockwise)
    void turnToHeading(float thetaDeg, float timeout = 0, const TurntoHeadingParams& params = {}, bool async = false);

    /// Turn to face a field coordinate
    void turnToPoint(float x, float y, float timeout = 0, const TurnToPointParams& params = {}, bool async = false);

    /// Drive an arc to a target point with a given radius
    void moveArc(float x, float y, float radius, float timeout = 0, const ArcParams& params = {}, bool async = false);

    /// Boomerang: curved approach to a pose
    /// @param thetaDeg  final heading in VEX degrees (0–360, clockwise positive)
    void moveBoomerang(float x, float y, float thetaDeg, float timeout = 0, const BoomerangParams& params = {}, bool async = false);

    /// Follow a path of poses using RAMSETE
    void moveRAMSETE(const std::vector<Pose>& path, float timeout = 0, const RAMSETEParams& params = {}, bool async = false);

    /// Follow a time-parameterized trajectory from `generateTrajectory`. The
    /// reference advances on the clock, so a trajectory whose speeds the
    /// drivetrain cannot deliver will be raced rather than followed -- generate
    /// it with the constraints from `Drivetrain::model()`.
    void followTrajectory(const Trajectory& trajectory, float timeout = 0,
                          const TrajectoryFollowParams& params = {}, bool async = false);

    /// Swing turn using one side of the drive
    /// @param thetaDeg  target heading in VEX degrees (0–360, clockwise positive)
    void swingToHeading(float thetaDeg, SwingSide side, float timeout = 0, const SwingParams& params = {}, bool async = false);

    // Motion Status

    /// True if the current movement has completed
    bool isSettled() const;

    /// How the most recently finished motion ended. Only meaningful once
    /// isSettled() is true for that motion.
    MotionOutcome lastOutcome() const;

    /// True if the last motion ended by reaching what it was asked to reach,
    /// rather than by timing out, stalling or being cancelled.
    bool lastMotionArrived() const;

    /// Cancel the current async movement
    void cancelMotion();

    /// Block until the current movement completes
    void waitUntilSettled();

    /// Wait until the robot has traveled at least `dist` inches
    void waitUntilDist(float dist);

    // Tracking Access (advanced use)

    OdomTracking& odom();
    RclTracking& rcl();
    MclTracking& mcl();
    EkfTracking& ekf();

    /// Start/stop all tracking systems
    void startTracking();
    void stopTracking();

    // Tuning

    /// Re-apply gains, derivative smoothing, windup limits and the settle
    /// debounce from a config in one call. Tuning any of these separately from
    /// the others is how a loop ends up damped for a gain it no longer has.
    void setPidTuning(const ChassisConfig& config);

    void setLateralGains(const PIDGains& gains);
    void setAngularGains(const PIDGains& gains);
    void setThrottleCurve(const DriveCurve& curve);
    void setSteerCurve(const DriveCurve& curve);

    /// Set brake mode for all drive motors
    void setBrakeMode(pros::motor_brake_mode_e mode);

    /// Set/replace distance sensors for MCL/RCL tracking
    void setDistanceSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors);

    // Dry-run mode (for Auton Selector path preview)

    /// Enable dry-run mode: movement commands record their paths but don't move motors
    void setDryRun(bool enabled);
    bool isDryRun() const;

    /// Get the recorded path from the last dry run
    const std::vector<Pose>& dryRunPath() const;

    /// Reset the dry-run path recording
    void resetDryRunPath();

    // Telemetry

    /// Empty by default. Set before starting motions - the motion task reads it.
    /// util/tuning.hpp has a hook that streams this to the SD card.
    void setTelemetry(std::function<void(const ChassisSample&)> callback);

private:
    // Internal motion helpers

    void startMotion();
    /// @param brake  false leaves the motors running, which is what makes
    ///               motion chaining a handover rather than a stop.
    void endMotion(bool brake = true);

    /// Scale a chassis twist into the drivetrain's wheel-speed budget and emit
    /// per-side motor output. The pair is scaled by one factor, never clipped
    /// independently, because the ratio between the sides IS the commanded
    /// curvature.
    void emitTwist(float v, float w, float maxOut, bool velocityController,
                   float vLeftMeas, float vRightMeas, float tickDt,
                   float& outLeft, float& outRight);
    // Motion task (for async operation)

    void runMotionTask();
    void executeCurrentMotion();
    void ensureMotionTask();

    /// The exit-test fields live in the PROS-free base, so settleSpecFor() can
    /// be tested off the robot against the very struct the loop runs on.
    struct MotionRequest : MotionSettleParams {
        uint32_t seq{0};                 // sequence id for blocking-completion wait
        float targetX{0}, targetY{0}, targetHeadingRad{0}, targetRadius{0};
        float targetDistance{0};         // raw distance for Distance motion (computed at exec time)
        uint32_t startTime{0};           // pros::millis() when motion began
        RAMSETEParams ramseteParams;
        SwingSide swingSide{SwingSide::Left};
        std::vector<Pose> ramsetePath;
        TrajectoryFollowParams trajParams;
        chad::Trajectory trajectory;
    };

    /// The subset of mConfig the exit test reads.
    SettleConfig settleConfig() const;

    /// Enqueue a motion for the single motion task to execute. When async is
    /// false, blocks the caller until THIS motion completes (or is cancelled).
    /// All motions — blocking and async — run on the one motion task, so the
    /// motors, PIDs, and mCurrentMotion are only ever touched by one thread.
    void enqueueMotion(MotionRequest& req, bool async);

    // Members

    Drivetrain& mDrivetrain;
    ChassisConfig mConfig;

    // Tracking. mEkf reads mMcl, so it is declared after it.
    OdomTracking mOdom;
    RclTracking mRcl;
    MclTracking mMcl;
    EkfTracking mEkf;

    // PID controllers
    PID mLateralPID;
    PID mAngularPID;

    // Closes RAMSETE's loop on measured wheel speed when the motion asks for
    // it. Reset at the start of every RAMSETE motion, so one instance is fine.
    FeedforwardController mRamseteFf;

    // Motion state
    std::atomic<bool> mMotionRunning{false};
    std::atomic<bool> mMotionCancelled{false};
    std::atomic<bool> mTaskShouldStop{false};
    std::atomic<uint32_t> mMotionSeqCounter{0};  // last seq assigned at enqueue
    std::atomic<uint32_t> mCompletedSeq{0};      // last seq the task finished
    std::atomic<MotionOutcome> mLastOutcome{MotionOutcome::None};
    MotionRequest mCurrentMotion;                // only touched by the motion task
    std::queue<MotionRequest> mMotionQueue;
    pros::Mutex mQueueMutex;
    pros::Task* mMotionTask{nullptr};

    // Drive curves
    DriveCurve mThrottleCurve;
    DriveCurve mSteerCurve;

    std::function<void(const ChassisSample&)> mTelemetry;

    // Dry-run mode
    bool mDryRun{false};
    std::vector<Pose> mDryRunPath;
    Pose mDryRunStartPose;
    Pose mDryRunPose;  // synthetic pose for dry-run simulation
};

}  // namespace chad
