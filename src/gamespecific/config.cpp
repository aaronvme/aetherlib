#include "gamespecific/config.hpp"

namespace chad {
pros::Controller master(pros::E_CONTROLLER_MASTER);

namespace {
struct FieldCircle { float x, y, r; };

constexpr FieldCircle kFieldPosts[] = {
    {-47.0f, -23.5f, 2.0f},
    {-23.5f, -47.0f, 2.0f},
    {23.5f,  47.0f, 2.0f},
    {47.0f,  23.5f, 2.0f},

    {0.0f,   0.0f, 2.0f},
    {-47.0f,  23.5f, 2.0f},
    {23.5f, -47.0f, 2.0f},
    {-23.5f,  47.0f, 2.0f},
    {47.0f, -23.5f, 2.0f},
};

std::vector<MclTracking::LineObstacle> gMclLineObstacles;
std::vector<MclTracking::CircleObstacle> gMclCircleObstacles;
}

void registerFieldObstacles() {
    gMclCircleObstacles.clear();
    for (const auto& c : kFieldPosts) {
        chassis.rcl().addCircleObstacle(c.x, c.y, c.r);
        gMclCircleObstacles.push_back({c.x, c.y, c.r});
    }
    chassis.mcl().setObstacles(&gMclLineObstacles, &gMclCircleObstacles);
    chassis.ekf().setObstacles(&gMclCircleObstacles);
}

const std::vector<MclTracking::CircleObstacle>& fieldCircleObstacles() {
    return gMclCircleObstacles;
}

void clearFieldObstacles() {
    gMclCircleObstacles.clear();
    gMclLineObstacles.clear();
    chassis.rcl().clearObstacles();
    chassis.mcl().setObstacles(&gMclLineObstacles, &gMclCircleObstacles);
    chassis.ekf().setObstacles(&gMclCircleObstacles);
}

void registerAutonDisableLine(float lifetimeMs) {
    chassis.rcl().addLineObstacle(Field::FIELD_HALF, -Field::FIELD_HALF,
                                  -Field::FIELD_HALF,  Field::FIELD_HALF,
                                  lifetimeMs);
}

namespace {
constexpr std::int8_t kLeftGreenPort  = -12;
constexpr std::int8_t kRightGreenPort = 11;

const std::vector<std::int8_t> kLeftBluePorts{-18, -19};
const std::vector<std::int8_t> kRightBluePorts{16, 17};

std::vector<std::int8_t> plus(std::vector<std::int8_t> ports, std::int8_t extra) {
    ports.push_back(extra);
    return ports;
}
}

pros::MotorGroup leftBlueMotors(kLeftBluePorts, pros::MotorGearset::blue);
pros::MotorGroup rightBlueMotors(kRightBluePorts, pros::MotorGearset::blue);
pros::MotorGroup leftGreenMotors({kLeftGreenPort}, pros::MotorGearset::green);
pros::MotorGroup rightGreenMotors({kRightGreenPort}, pros::MotorGearset::green);

pros::MotorGroup leftMotors(plus(kLeftBluePorts, kLeftGreenPort),
                            pros::MotorGearset::invalid);
pros::MotorGroup rightMotors(plus(kRightBluePorts, kRightGreenPort),
                             pros::MotorGearset::invalid);

pros::MotorGroup liftMotor({20, 15, -14}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

ScaledIMU imu(13, 360.0, 359.0);

pros::adi::Pneumatics claw('A', false, false);
// pros::adi::Pneumatics flip('C', true, true);

Drivetrain drivetrain(&leftMotors, &rightMotors,
                      11.50f,
                      2.75f,
                      450.0f,
                      &leftBlueMotors,
                      &rightBlueMotors);

OdomSensors odomSensors = {
    .vertWheelCollection = {},
    .horizWheelCollection = {},
    .imuCollection = { &imu },
    .drivetrain = &drivetrain,
};

pros::Distance front_dist(1);
pros::Distance left_dist(4);
pros::Distance back_dist(3);
pros::Distance right_dist(2);
pros::Distance claw_dist(5);

std::array<pros::Distance*, MAX_DISTANCE_SENSORS> distSensors = {
    &front_dist, &left_dist, &back_dist, &right_dist,
    nullptr, nullptr, nullptr, nullptr
};


std::array<Pose, MAX_DISTANCE_SENSORS> sensorMounts = {{
    {  -0.875f, -3.75f,  0.0f        },
    {  -0.50f,   4.625f, HALF_PI    },
    {  -3.125f,  5.50f,  PI         },
    {  -0.50f,  -4.875f, 3.0f * HALF_PI }
}};

ChassisConfig chassisConfig = {
    .lateralGains = { 9.5f, 0.0f, 0.65f},
    .angularGains = { 140.0, 0.0f, 8.25f },
    .lateralWindupRange = 0.0f,
    .angularWindupRange = 0.0f,
    .lateralDerivativeFilter = 0.6f,
    .angularDerivativeFilter = 0.5f,
    .lateralSlewPerSec = 450.0f,

    .targetToleranceIn = 0.2f,
    .targetToleranceDeg = 0.5f,
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
        .maxSensorRange = 78.0f,
        .distResampleVariance = 1.0f,
        .thetaResampleVariance = 0.01f,
        .maxThetaDeviation = 0.08f,
        .trackingWheelVariance = 0.08f,
        .imeVariance = 0.0040f,
        .imuVariance = 0.005f,
        .horizDependentVarianceProp = 0.0040f,
        .horizConstantNoise = 0.03f,
        .distSyncProp = 0.10f,
        .thetaSyncProp = 0.001f,
        .updatePeriodMs = 25.0f,
        .confidenceThreshold = 10,
        .minSensorRange = 1.0f,
        .rangeSigmaCloseIn = 0.0872f,
        .rangeSigmaProp = 0.020f,
        .maxRandomFrac = 0.25f,
        .injectResampleFloor = 0.20f,
        .maxTurnRateRadS = 2.0f,
        .autoSync = true
    },

    .ekfConfig = {
        .vertNoiseProp = 0.0040f,
        .slipNoiseProp = 0.0040f,
        .vertBiasProp = 0.0459f,
        .slipBiasProp = 0.0159f,
        .rangeSigmaBase = 0.0872f,
        .rangeSigmaProp = 0.030f,
        .maxRangeIn = 78.0f,
        .imuSigmaRad = 0.006f
    },
};

FeedforwardGains ramseteFf = {
    .kV = 6.2533f,
    .kaStraight = 0.0f,
    .kaTurn = 0.1377f,
    .ksStraight = 0.6611f,
    .ksTurn = 1.1607f,
    .kP = 12.51f,
    .integralClamp = 2.0f,
};
Chassis chassis(drivetrain, odomSensors, chassisConfig);

AutonSelector selector;
}
