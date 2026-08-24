#include "gamespecific/config.hpp"

namespace chad {

pros::Controller master(pros::E_CONTROLLER_MASTER);



// ============================================================================
// Field obstacles

namespace {

struct FieldCircle { float x, y, r; };

constexpr FieldCircle kFieldPosts[] = {
    // alliance studs
    {-47.0f, -23.5f, 2.0f},   // red stud up
    {-23.5f, -47.0f, 2.0f},   // red stud down
    {23.5f,  47.0f, 2.0f},   // blue stud up
    {47.0f,  23.5f, 2.0f},   // blue stud down
    // neutral goals
    {0.0f,   0.0f, 2.0f},   // center goal
    {-47.0f,  23.5f, 2.0f},   // neutral red up
    {23.5f, -47.0f, 2.0f},   // neutral red down
    {-23.5f,  47.0f, 2.0f},   // neutral blue up
    {47.0f, -23.5f, 2.0f},   // neutral blue down 
};

std::vector<MclTracking::LineObstacle> gMclLineObstacles;
std::vector<MclTracking::CircleObstacle> gMclCircleObstacles;

}  // namespace

void registerFieldObstacles() {
    gMclCircleObstacles.clear();
    for (const auto& c : kFieldPosts) {
        chassis.rcl().addCircleObstacle(c.x, c.y, c.r);   // lifetime 0 = permanent
        gMclCircleObstacles.push_back({c.x, c.y, c.r});
    }
    chassis.mcl().setObstacles(&gMclLineObstacles, &gMclCircleObstacles);
    chassis.ekf().setObstacles(&gMclCircleObstacles);
}

void registerAutonDisableLine(float lifetimeMs) {
    chassis.rcl().addLineObstacle(Field::FIELD_HALF, -Field::FIELD_HALF,
                                  -Field::FIELD_HALF,  Field::FIELD_HALF,
                                  lifetimeMs);
}





//CHASSIS/MOTORS============================================================================

namespace {

constexpr std::int8_t kLeftGreenPort  = -12;
constexpr std::int8_t kRightGreenPort = 11;

const std::vector<std::int8_t> kLeftBluePorts{-18, -19};
const std::vector<std::int8_t> kRightBluePorts{16, 17};

std::vector<std::int8_t> plus(std::vector<std::int8_t> ports, std::int8_t extra) {
    ports.push_back(extra);
    return ports;
}

}  // namespace

pros::MotorGroup leftBlueMotors(kLeftBluePorts, pros::MotorGearset::blue);
pros::MotorGroup rightBlueMotors(kRightBluePorts, pros::MotorGearset::blue);
pros::MotorGroup leftGreenMotors({kLeftGreenPort}, pros::MotorGearset::green);
pros::MotorGroup rightGreenMotors({kRightGreenPort}, pros::MotorGearset::green);

pros::MotorGroup leftMotors(plus(kLeftBluePorts, kLeftGreenPort),
                            pros::MotorGearset::invalid);
pros::MotorGroup rightMotors(plus(kRightBluePorts, kRightGreenPort),
                             pros::MotorGearset::invalid);

pros::MotorGroup liftMotor({14, -15, 1, -2}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

ScaledIMU imu(13, 360.0, 359.0);

pros::adi::Pneumatics claw('D', false, false);
pros::adi::Pneumatics flip('C', true, true);


Drivetrain drivetrain(&leftMotors, &rightMotors,
                      11.918f,
                      2.6738f,
                      450.0f,
                      &leftBlueMotors,
                      &rightBlueMotors,
                      WheelType::Traction,
                      0.0f);


OdomSensors odomSensors = {
    .vertWheelCollection = {},
    .horizWheelCollection = {},
    .imuCollection = { &imu },
    .drivetrain = &drivetrain,
};

pros::Distance front_dist(7);
pros::Distance left_dist(9);
pros::Distance back_dist(10);
pros::Distance right_dist(8);

std::array<pros::Distance*, MAX_DISTANCE_SENSORS> distSensors = {
    &front_dist, &left_dist, &back_dist, &right_dist,
    nullptr, nullptr, nullptr, nullptr
};

// Sensor mount offsets are expressed in robot frame:
//   x = forward, y = left, theta = pointing angle in radians.
std::array<Pose, MAX_DISTANCE_SENSORS> sensorMounts = {{
    {  1.0f,  -3.75f,  0.0f        },//done
    {  0.5f,  6.25f,  HALF_PI    },//done
    { -0.5f,  5.0f,  PI         },//done
    {  -0.75f, -6.75f,  3.0f * HALF_PI },//done
}};


// Still on you. Nothing in the tuning autons measures any of this.
//
//   run a routine for it
//     drivetrain wheels         which wheels are on the base, not a guess
//     drivetrain grip limit     drive a known-radius circle until it slides
//     ScaledIMU actualReading   autonCalibrateImu, before anything else
//     sensorMounts[i].y         autonCalibrateMounts
//     drivetrain rpm            phase 1 prints the value its kV implies
//
//   no routine yet, but a bench measurement settles it
//     ekf.rangeLatencyMs 30     drive a straight leg at a known speed past a
//                               wall and sweep this until the front sensor's
//                               innovation stops depending on speed. It is a
//                               BIAS of speed*latency, so 60 in/s and a 30 ms
//                               error is 1.8 in of standing offset -- and zero,
//                               the old value, is the one setting guaranteed
//                               wrong. Wrong-but-plausible beats off.
//
//   came off simulator sweeps; the structure is sound, the number is a guess
//     mcl.hitWeight 1.0   mcl.shortWeight 0.50   mcl.shortDecaySigmas 1.2
//     mcl.randWeight 0.0008   settleTimeMs   mcl.thetaResampleVariance 0.01
//     distSyncProp and thetaSyncProp are inert while the EKF is on.
//     For ekf.mclSigmaScale/Floor the check is phase 4's "gate streak / reloc".
//
//   swept, not guessed: 648 paired kidnaps, 8 starts x 16 directions x 8 seeds
//     mcl.distResampleVariance 1.5 -- every lower roughening tried tracks
//     better and recovers worse, and the FUSED pose is what is driven on.
//     ekf.mclSigmaFloor 1.2 is the optimum AT that roughening; it is lowering
//     the roughening that would make a higher floor pay. Numbers in API.md.
//
//   no routine can measure it -- out-and-back cancels the effect by design
//     ekf.vertBiasProp 0.010   ekf.slipBiasProp 0.010   ekf.biasDecorrIn 40
//     Sized at 2.5x the 0.0040/in closure error phase 3 does measure, which is
//     the margin a tape-measured wheel diameter leaves. These set the range
//     gate width, so re-check phase 4's "gate streak / reloc" line after any
//     change: at the old 0.06 the gate stood open to +/-7.4 inches.
//
// Per-field tags for all 65 filter settings live on the Config structs, which
// are in chad/tracking/mcl_core.hpp and ekf_core.hpp -- the *_tracking.hpp
// headers only alias them.
ChassisConfig chassisConfig = {
    .lateralGains = { 9.012f, 0.0f, 1.15f },      // phase 2
    .angularGains = { 176.607f, 0.0f, 9.933f },   // phase 2
    .lateralWindupRange = 0.0f,
    .angularWindupRange = 0.0f,
    .lateralDerivativeFilter = 0.6f,
    .angularDerivativeFilter = 0.5f,
    .lateralSlewPerSec = 450.0f,   // 0 to full drive in ~280 ms
    // .throttleCurve = { .deadband = 10.0f, .minOutput = 15.0f, .curve = 1.2f },
    // .steerCurve = { .deadband = 10.0f, .minOutput = 10.0f, .curve = 1.3f },
    .targetToleranceIn = 1.0f,     // phase 4 floor, then your call
    .targetToleranceDeg = 1.5f,
    .bearingHoldDist = 1.0f,       // phase 4
    .distanceSensors = distSensors,
    .sensorMounts = sensorMounts,
    .settleTimeMs = 110,           // [sim]
    .useMclTracking = true,
    .useRclTracking = false,
    .useEkfTracking = true,
    
    .mclConfig = {
        .particleCount = 1024,
        .resampleThreshold = 512,
        .maxSensorRange = 140.0f,
        .distResampleVariance = 1.5f,      // [sim]
        .thetaResampleVariance = 0.01f,    // [sim]
        .maxThetaDeviation = 0.08f,
        .trackingWheelVariance = 0.08f,
        .imeVariance = 0.0040f,            // phase 3
        .imuVariance = 0.005f,
        .horizDependentVarianceProp = 0.0040f,  // phase 3
        .horizConstantNoise = 0.03f,       // [gap] per update, not per inch
        .distSyncProp = 0.10f,             // [sim]
        .thetaSyncProp = 0.001f,           // [sim]
        .updatePeriodMs = 25.0f,
        .confidenceThreshold = 10,
        .minSensorRange = 1.0f,
        .rangeSigmaCloseIn = 0.0680f,      // phase 4
        .rangeSigmaProp = 0.05f,           // phase 4 range sweep
        .autoSync = true,
    },
    // .rclConfig = {
    //     .sensorCount = 4,
    //     .maxSyncDist = 30.0f,
    //     .accumulationAlpha = 0.3f,
    //     .updatePeriodMs = 15.0f,
    //     .angleTolerance = 10.0f,
    //     .confidenceThreshold = 25,
    //     .autoSync = true,
    //     .useAccumulation = false,
    // },
    .ekfConfig = {
        .vertNoiseProp = 0.0040f,   // phase 3
        .slipNoiseProp = 0.0040f,   // phase 3
        .rangeSigmaBase = 0.0680f,  // phase 4 range sweep
        .rangeSigmaProp = 0.030f,   // phase 4 range sweep
        // Estimate, not a measurement: smart-port refresh plus half the ToF
        // integration window. CONFIRM ON THE ROBOT -- see the note above.
        .rangeLatencyMs = 30.0f,    // [gap]
        // Zero deliberately: the state's heading is propagated from this same
        // IMU, so measurement and prediction already share its port delay.
        .imuLatencyMs = 0.0f,       // [model]
    },
};

// Feedforward for RAMSETE's velocity controller. METRES and VOLTS -- the units
// characterisation phase 1 fits in, not the inches the rest of the API takes.
// kP and kI are not measurable: they came off the sim sweeps. Pass this as
// RAMSETEParams::ffGains.
FeedforwardGains ramseteFf = {
    .kV = 6.1287f,        // r2 0.996
    .kaStraight = 0.0f,   // phase 1 measures this now
    .kaTurn = 0.0202f,
    .ksStraight = 0.7020f,
    .ksTurn = 1.1607f,
    .kP = 13.0f,          // phase 1 derives this
    .integralClamp = 2.0f,
};
Chassis chassis(drivetrain, odomSensors, chassisConfig);

AutonSelector selector;



}  // namespace chad
