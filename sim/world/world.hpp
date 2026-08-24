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
//     goal post is detected from off-axis. The reported range is the STRONGEST
//     return in the cone, not the nearest: nearest is the cone edge whenever a
//     flat wall fills the cone, which reads 39.07 in against 43.02 at 30 deg off
//     square. See updateDistanceSensors() for the strength model.
//   * Confidence falls with range and with glancing incidence, and the sensor
//     drops out entirely past a grazing threshold -- the dominant real-world
//     failure mode when a robot is near-parallel to a wall.
//   * Encoder POSITION and the IMU heading are quantised to their real
//     resolutions. Motor velocity is not -- it is published as an exact float.
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

    // Fraction of real travel the encoders fail to see. Modelled as encoder
    // under-report, NOT as slip: ground truth is untouched, so the robot still
    // covers the full distance and only odometry comes up short. Real slip on
    // traction wheels would lose the ground too.
    float encoderSlip{0.0f};

    // --- IMU ---------------------------------------------------------------
    // The V5's habit of reading short. init() overwrites this with the inverse
    // of config.cpp's ScaledIMU correction, so the default is a correctly
    // calibrated IMU and any value you set here is a calibration error.
    float imuUnderReport{354.25f / 360.0f};
    float imuDriftDegPerSec{0.0f};
    /// Random-WALK intensity in deg/sqrt(s), not white measurement noise: it is
    /// added to the accumulated rotation, so the error never returns to zero.
    /// At 2.0 the heading is 11.9 deg RMS out after 32 s at REST and stays out.
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

    /// Whether the robot collides with goal posts and editor structures. The
    /// LOADED perimeter always stops it; a field with no kind-0 geometry has no
    /// walls, which is what Field::clear() leaves behind.
    bool collideObstacles{true};

    /// Robot footprint, drawn as-is. Collision is an orientation-free disc of a
    /// QUARTER of the diagonal: 5.31 in here against a 7.50 in half-width, so the
    /// drawn robot overlaps a wall by 2.19 in where it stops. Kept because both
    /// numbers are guesses and the route suites are tuned to that clearance.
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
    // Wheel travel the encoders saw, inches -- per-motor shaft degrees are
    // derived at publish time from each port's own cartridge.
    double mWheelSeenLeft{0.0};
    double mWheelSeenRight{0.0};
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
