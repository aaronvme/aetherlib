#include "gamespecific/config.hpp"

namespace chad {
pros::Controller master(pros::E_CONTROLLER_MASTER);

namespace {
struct FieldCircle { float x, y, r; };

// The nine Override Goals. 2.85 is half the 5.7 in across-flats of the
// octagon, measured off Figure FO-2 in the manual. Order matters: the sim
// colours them from the index, red first, then blue, the tall centre Goal,
// then the four short neutrals.
constexpr FieldCircle kFieldPosts[] = {
    {-47.0f, -23.5f, 2.85f},
    {-23.5f, -47.0f, 2.85f},
    {23.5f,  47.0f, 2.85f},
    {47.0f,  23.5f, 2.85f},

    {0.0f,   0.0f, 2.85f},
    {-47.0f,  23.5f, 2.85f},
    {23.5f, -47.0f, 2.85f},
    {-23.5f,  47.0f, 2.85f},
    {47.0f, -23.5f, 2.85f},
};

std::vector<MclTracking::LineObstacle> gMclLineObstacles;
std::vector<MclTracking::CircleObstacle> gMclCircleObstacles;
}

void registerFieldObstacles() {
    gMclCircleObstacles.clear();
    // RCL keeps its own copy, so clear that too. Without this a second call --
    // which the sim makes on every field reload -- left it carrying two
    // overlapping sets of posts and paying the cone test twice for each.
    chassis.rcl().clearObstacles();
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
    chassis.rcl().setFarSideGateFromLine(Field::FIELD_HALF, -Field::FIELD_HALF,
                                         -Field::FIELD_HALF,  Field::FIELD_HALF,
                                         lifetimeMs);
}

void clearAutonDisableLine() {
    chassis.rcl().clearFarSideGate();
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

namespace {
// Negate a port to reverse that side. Each must turn its toggle the one way
// it flips; the code never commands reverse.
constexpr std::int8_t kSideToggleLeftPort  = -7;
constexpr std::int8_t kSideToggleRightPort = 6;
}

// 5.5 W motors have a fixed 200 rpm output, which PROS treats as green.
pros::Motor sideToggleLeft(kSideToggleLeftPort, pros::v5::MotorGears::green,
                           pros::v5::MotorUnits::degrees);
pros::Motor sideToggleRight(kSideToggleRightPort, pros::v5::MotorGears::green,
                            pros::v5::MotorUnits::degrees);

ScaledIMU imu(13, 360.0, 360.0);

pros::adi::Pneumatics claw('A', false, false);
// pros::adi::Pneumatics flip('C', true, true);

Drivetrain drivetrain(&leftMotors, &rightMotors,
                      11.50f,
                      2.75f,
                      450.0f,
                      &leftBlueMotors,
                      &rightBlueMotors);

// Negate a port to reverse that sensor: the vertical wheel must count up
// driving forward, the horizontal one up when the robot moves to its left.
pros::Rotation vertRot(-9);
pros::Rotation horizRot(-10);

// TODO measure: wheel diameter, then offset from the turning centre
// (+right for the vertical wheel, +forward for the horizontal one).
TrackingWheel vertWheel(vertRot, 2.125f, 0.75f);
TrackingWheel horizWheel(horizRot, 2.75f, 4.75f);

OdomSensors odomSensors = {
    .vertWheelCollection = { &vertWheel },
    .horizWheelCollection = { &horizWheel },
    .imuCollection = { &imu },
    .drivetrain = &drivetrain,
};

pros::Distance front_dist(1);
pros::Distance left_dist(3);
pros::Distance back_dist(4);
pros::Distance right_dist(2);
pros::Distance claw_dist(5);

std::array<pros::Distance*, MAX_DISTANCE_SENSORS> distSensors = {
    &front_dist, &left_dist, &back_dist, &right_dist,
    nullptr, nullptr, nullptr, nullptr
};


std::array<Pose, MAX_DISTANCE_SENSORS> sensorMounts = {{
    {  0.25f,  -3.75f,  0.0f        }, //forwards done
    {  3.75f,  5.75f,  HALF_PI    }, //left done
    { -1.50f,  5.25f,  PI         }, //back done
    {  3.75f, -5.625f,  3.0f * HALF_PI } //right done
}};

ChassisConfig chassisConfig = {
    .lateralGains = { 9.5f, 0.0f, 0.65f},
    .angularGains = { 160.0, 0.0f, 8.25f },
    .lateralWindupRange = 0.0f,
    .angularWindupRange = 0.0f,
    .lateralDerivativeFilter = 0.6f,
    .angularDerivativeFilter = 0.5f,
    .lateralSlewPerSec = 450.0f,

    .targetToleranceIn = 0.2f,
    .targetToleranceDeg = 0.5f,
    .bearingHoldDist = 3.5f,
    .boomerangBearingHoldDist = 1.0f,
    .distanceSensors = distSensors,
    .sensorMounts = sensorMounts,
    .settleTimeMs = 110,
    .useMclTracking = false,
    .useRclTracking = false,
    .useEkfTracking = true,

    .mclConfig = {
        .particleCount = 1024,
        .resampleThreshold = 512,
        .maxSensorRange = 78.0f,
        .distResampleVariance = 0.7f,
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
        .minSensorRange = 20.0f,
        .rangeSigmaCloseIn = 0.0872f,
        .rangeSigmaProp = 0.020f,
        .maxRandomFrac = 0.25f,
        .injectResampleFloor = 0.20f,
        .injectRadiusIn = 25.0f,
        .minAxisObservability = 0.5f,
        .maxTurnRateRadS = 2.0f,
        // Half of a 12 in body; the drive is 11.5 in wheel to wheel.
        .bodyRadiusIn = 6.0f,
        .autoSync = true
    },

    // RCL owns the pose. Every number below that is not a stock default is
    // there because the sampling is now gated: a V5 distance sensor refreshes
    // about every 30 ms, so roughly one cycle in three carries a new reading
    // and the correction authority per second fell by the same factor.
    .rclConfig = {
        .maxSyncDist = 5.0f,
        .syncProp = 0.25f,
        .maxSyncRate = 12.0f,
        .updatePeriodMs = 10.0f,
        .angleTolerance = 10.0f,
        .maxWallInnovation = 8.0f,
        .recoverySyncProp = 0.9f,
        .displaceCycles = 3,
        .noReturnWallLimit = 35.0f,
        // The EKF and MCL both ran at 10. Thirty threw away every reading past
        // about 60 in on this sensor model, which is most of the field.
        .confidenceThreshold = 20,
        .obstacleConeFrac = 0.22f,
        .obstacleRangeTol = 6.0f,
        .autoSync = true,
        .useAccumulation = false,
        .staleHoldMs = 45.0f,
        .maxTurnRateRadS = 1.5f,
        .sensorLatencyMs = 30.0f,
        .axisAgreeIn = 2.0f,
        .pairSpanTolIn = 1.5f,
        .recoverCycles = 8,
        .recoveryMinVotes = 1,
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
