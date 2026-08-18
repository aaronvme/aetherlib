#include "gamespecific/config.hpp"

namespace riku {

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

// MCL keeps POINTERS to its obstacle lists â€” these must outlive the tracker
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
}

void registerAutonDisableLine(float lifetimeMs) {
    // Diagonal auton line: readings crossing it during autonomous see robots /
    // game objects, not walls. Temporary â€” expires after `lifetimeMs`.
    chassis.rcl().addLineObstacle(Field::FIELD_HALF, -Field::FIELD_HALF,
                                  -Field::FIELD_HALF,  Field::FIELD_HALF,
                                  lifetimeMs);
}





//CHASSIS/MOTORS============================================================================


pros::MotorGroup leftMotors({7, 6}, pros::MotorGearset::green);
pros::MotorGroup rightMotors({9, 10}, pros::MotorGearset::green);
pros::Motor frontMotor(17, pros::MotorGearset::blue);
pros::Motor leverMotor(-16, pros::MotorGearset::red);
pros::Motor armMotor(8,pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

ScaledIMU imu(15, 360.0, 354.25);
pros::Optical frontOptic(3);

pros::adi::Pneumatics matchLoadGate('D', false, false);
pros::adi::Pneumatics lift('C', true, true);
pros::adi::Pneumatics leftDescoreArm('A', false, false);
pros::adi::Pneumatics trapDoor('B', false, false);
pros::adi::Pneumatics intakeLift('G', true, true);

Drivetrain drivetrain(&leftMotors, &rightMotors,
                      10.4f,
                      3.25f,
                      450.0f,
                      2.0f);

OdomSensors odomSensors = {
    .vertWheelCollection = {},
    .horizWheelCollection = {},
    .imuCollection = { &imu },
    .drivetrain = &drivetrain,
};

pros::Distance front_dist(1);
pros::Distance left_dist(2);
pros::Distance back_dist(14);
pros::Distance right_dist(8);

std::array<pros::Distance*, MAX_DISTANCE_SENSORS> distSensors = {
    &front_dist, &left_dist, &back_dist, &right_dist,
    nullptr, nullptr, nullptr, nullptr
};

// Sensor mount offsets are expressed in robot frame:
//   x = forward, y = left, theta = pointing angle in radians.
std::array<Pose, MAX_DISTANCE_SENSORS> sensorMounts = {{
    {  6.0f,  0.0f,  0.0f        },
    {  0.0f,  3.5f,  HALF_PI    },
    { -5.0f,  0.0f,  PI         },
    {  0.0f, -3.5f,  3.0f * HALF_PI },
}};


ChassisConfig chassisConfig = {
    // Tuned in riku-sim: ~1.1 s to 0.75 in, vs 5 s at {5,0,5}/{50,0,6}. The sim
    // is kinematic and cannot show oscillation, so confirm on the robot;
    // {9,0,5}/{80,0,6} is about half as aggressive if it feels twitchy.
    .lateralGains = { 17.0f, 0.0f, 2.0f },
    .angularGains = { 140.0f, 0.0f, 4.0f },
    .lateralWindupRange = 0.0f,
    .angularWindupRange = 0.0f,
    .throttleCurve = { .deadband = 10.0f, .minOutput = 15.0f, .curve = 1.2f },
    .steerCurve = { .deadband = 10.0f, .minOutput = 10.0f, .curve = 1.3f },
    .distanceSensors = distSensors,
    .sensorMounts = sensorMounts,
    .useMclTracking = false,
    .useRclTracking = false,
    
    .mclConfig = {
        .particleCount = 1024,
        .resampleThreshold = 512,
        .maxSensorRange = 140.0f,
        .distResampleVariance = 1.5f,
        .thetaResampleVariance = 0.01f,
        .maxThetaDeviation = 0.08f,
        .trackingWheelVariance = 0.08f,
        .imeVariance = 0.12f,
        .imuVariance = 0.005f,
        .horizDependentVarianceProp = 0.01f,
        .horizConstantNoise = 0.03f,
        .distSyncProp = 0.10f,
        .thetaSyncProp = 0.001f,
        .updatePeriodMs = 25.0f,
        .confidenceThreshold = 10,
        .minSensorRange = 1.0f,
        .autoSync = true,
    },
    .rclConfig = {
        .sensorCount = 4,
        .maxSyncDist = 30.0f,
        .accumulationAlpha = 0.3f,
        .updatePeriodMs = 15.0f,
        .angleTolerance = 10.0f,
        .confidenceThreshold = 25,
        .autoSync = true,
        .useAccumulation = false,
    },
};

Chassis chassis(drivetrain, odomSensors, chassisConfig);

AutonSelector selector;



}  // namespace riku
