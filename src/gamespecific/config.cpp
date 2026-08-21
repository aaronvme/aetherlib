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
                      0.0f,
                      &leftBlueMotors,
                      &rightBlueMotors);


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
    {  -0.75f, -6.75f,  3.0f * HALF_PI },
}};


ChassisConfig chassisConfig = {
    .lateralGains = { 9.012f, 0.0f, 1.15f },
    .angularGains = { 176.607f, 0.0f, 9.933f },
    .lateralWindupRange = 0.0f,
    .angularWindupRange = 0.0f,
    .lateralDerivativeFilter = 0.6f,
    .angularDerivativeFilter = 0.5f,
    .lateralSlewPerSec = 450.0f,   // 0 to full drive in ~280 ms
    // .throttleCurve = { .deadband = 10.0f, .minOutput = 15.0f, .curve = 1.2f },
    // .steerCurve = { .deadband = 10.0f, .minOutput = 10.0f, .curve = 1.3f },
    .targetToleranceIn = 1.0f,
    .targetToleranceDeg = 1.5f,
    .bearingHoldDist = 1.0f,
    .distanceSensors = distSensors,
    .sensorMounts = sensorMounts,
    .settleTimeMs = 110,
    .useMclTracking = true,
    .useRclTracking = false,
    .useEkfTracking = true,
    
    .mclConfig = {
        .particleCount = 1024,
        .resampleThreshold = 512,
        .maxSensorRange = 140.0f,
        .distResampleVariance = 1.5f,
        .thetaResampleVariance = 0.01f,
        .maxThetaDeviation = 0.08f,
        .trackingWheelVariance = 0.08f,
        .imeVariance = 0.0040f,
        .imuVariance = 0.005f,
        .horizDependentVarianceProp = 0.01f,
        .horizConstantNoise = 0.0040f,
        .distSyncProp = 0.10f,
        .thetaSyncProp = 0.001f,
        .updatePeriodMs = 25.0f,
        .confidenceThreshold = 10,
        .minSensorRange = 1.0f,
        .rangeSigmaCloseIn = 0.0680f,
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
        .vertNoiseProp = 0.0040f,
        .slipNoiseProp = 0.0040f,
        .rangeSigmaBase = 0.0680f,
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
    .kP = 13.0f,
    .integralClamp = 2.0f,
};
Chassis chassis(drivetrain, odomSensors, chassisConfig);

AutonSelector selector;



}  // namespace chad
