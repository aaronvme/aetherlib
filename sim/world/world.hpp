// chad-sim — ground truth robot state and sensor synthesis.
//
// Motion is deliberately kinematic, not dynamic: commanded voltage maps to a
// target wheel speed through a first-order lag, and differential-drive
// kinematics are integrated from there. No mass, torque or friction.
//
// Everything ELSE aims to behave like the real hardware, because that is what
// the algorithms actually consume:
//
//   * Devices refresh on their own schedules (V5 distance ~30 ms, IMU and motor
//     encoders ~10 ms) and hold a stale value in between. Reading faster than
//     the device updates returns the previous sample, exactly as on a brain.
//   * Distance sensors integrate a ~25 deg cone, not a pencil ray, so a thin
//     goal post is detected from off-axis and the reading is the NEAREST thing
//     in the cone.
//   * Confidence falls with range and with glancing incidence, and the sensor
//     drops out entirely past a grazing threshold -- the dominant real-world
//     failure mode when a robot is near-parallel to a wall.
//   * Encoders and the IMU are quantised to their real resolutions.
//
// Sensor mount offsets are read straight from chad::sensorMounts in
// gamespecific/config.cpp, using the same robot-frame convention the library
// hands to MCL (x forward, y left, theta pointing angle), so the sim and the
// filter can never disagree about where a sensor is.
#pragma once

#include "chad/util/util.hpp"

namespace sim {

struct RobotParams {
    // Geometry is read from chad::drivetrain at init(); these mirror it.
    float wheelDiameterIn{3.25f};
    float trackWidthIn{10.4f};
    float maxWheelRpm{450.0f};

    // Milliseconds for wheel speed to reach ~63% of its commanded value.
    float responseTauMs{70.0f};

    // Multiplicative per-side error -- the classic "robot pulls left" defect.
    float leftScale{1.0f};
    float rightScale{1.0f};

    // Fraction of real travel the encoders fail to see (wheel slip).
    float encoderSlip{0.0f};

    // --- IMU ---------------------------------------------------------------
    // underReport reproduces the V5's habit of reading short. init() overwrites
    // this with the inverse of chad::imu's own scale factor so the two cancel;
    // the literal here only matters before init() runs. Change it afterwards
    // (the UI slider does) to model an IMU whose calibration is wrong.
    float imuUnderReport{354.25f / 360.0f};
    float imuDriftDegPerSec{0.0f};
    float imuNoiseDeg{0.0f};
    float imuPeriodMs{10.0f};

    // --- distance sensors --------------------------------------------------
    float distNoiseMm{5.0f};
    float distMinRangeMm{20.0f};
    float distMaxRangeMm{2000.0f};
    float distPeriodMs{30.0f};
    /// Full cone angle. The V5 sensor is roughly 25 degrees.
    float distFovDeg{25.0f};
    /// Rays sampled across the cone. Odd, so one lands on the axis.
    int   distConeRays{7};
    /// Incidence past this many degrees from the surface normal returns
    /// nothing. Real ToF sensors lose the return when grazing.
    float distMaxIncidenceDeg{75.0f};

    // --- encoders ----------------------------------------------------------
    float encoderPeriodMs{10.0f};
    /// V5 integrated encoder resolution in motor-shaft degrees.
    float encoderResolutionDeg{0.1f};

    /// Angular apodisation of the cone, as a fraction of the full FOV. The
    /// return is strongest on axis and falls off toward the cone edge; without
    /// this a flat wall is measured at the cone edge rather than on axis, which
    /// reads several inches short whenever the robot is not square to it.
    float distConeSigmaFrac{0.25f};

    /// Whether the robot collides with goal posts. The perimeter always stops it.
    bool collideObstacles{true};

    // Robot footprint, used for collision and drawing.
    float robotLengthIn{15.0f};
    float robotWidthIn{15.0f};
};

struct TrueState {
    float x{0.0f};          // inches
    float y{0.0f};
    float theta{0.0f};      // radians, standard math convention
    float vLinIn{0.0f};     // inches/sec
    float vAngRad{0.0f};    // radians/sec
    float leftVelIn{0.0f};
    float rightVelIn{0.0f};
    bool  colliding{false};
};

class World {
public:
    /// Binds to the drivetrain/sensor objects declared in
    /// gamespecific/config.cpp and installs the physics callback.
    void init();

    /// Teleport ground truth (the "kidnapped robot" case) without touching the
    /// library's estimate -- the point of the test.
    void teleport(const chad::Pose& pose);

    /// Reset ground truth, encoders and the IMU together.
    void resetTo(const chad::Pose& pose);

    void step(double dtMs);

    const TrueState& truth() const { return mTruth; }
    RobotParams& params() { return mParams; }
    const RobotParams& params() const { return mParams; }

    /// Last synthesised beam for sensor slot i, for drawing and diagnostics.
    struct BeamInfo {
        bool  active{false};        // slot is populated in config
        float originX{0.0f}, originY{0.0f};
        float dirDeg{0.0f};         // world-frame ray heading
        // Where the on-axis laser actually terminates. The reported range can
        // come from an off-axis cone ray, so drawing the beam to rangeIn along
        // the axis puts its end in empty space.
        float endX{0.0f}, endY{0.0f};
        float axisRangeIn{0.0f};
        // The return the sensor actually timed: the strongest ray in the cone.
        float hitX{0.0f}, hitY{0.0f};
        bool  hitIsObstacle{false};
        float rangeIn{0.0f};
        float fovDeg{25.0f};
        float incidenceDeg{0.0f};   // 0 = head-on
        int   confidence{0};
        bool  inRange{false};       // a usable reading was produced
    };
    const BeamInfo& beam(int i) const { return mBeams[i]; }

private:
    void updateDrive(double dtSec);
    void accumulateEncoders(double leftTravelIn, double rightTravelIn);
    void publishEncoders();
    void accumulateImu(double dThetaRad, double dtSec);
    void publishImu();
    void updateDistanceSensors();

    RobotParams mParams;
    TrueState   mTruth;

    float mLeftCmdIn{0.0f};    // lagged wheel speeds, inches/sec
    float mRightCmdIn{0.0f};

    // Continuous ground-truth sensor values. These advance every physics tick;
    // the publish* methods copy them onto the bus at device rate.
    double mMotorDegLeft{0.0};
    double mMotorDegRight{0.0};
    double mImuRotationDeg{0.0};

    // Dead-wheel travel, published to the rotation-sensor slots so the sim can
    // run with tracking wheels as well as without.
    double mPodVertIn{0.0};
    double mPodHorizIn{0.0};

    // Device refresh accumulators (ms).
    double mDistAccum{0.0};
    double mImuAccum{0.0};
    double mEncAccum{0.0};

    BeamInfo mBeams[chad::MAX_DISTANCE_SENSORS];
    bool mInitialised{false};
};

World& world();

}  // namespace sim
