#pragma once

#include <array>
#include <vector>

#include "main.h"
#include "chad/api.hpp"
#include "chad/tracking/rcl_tracking.hpp"

namespace chad {
extern pros::Controller master;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Motor frontMotor;
extern pros::Motor leverMotor;

extern pros::MotorGroup liftMotor;

extern ScaledIMU imu;
extern pros::Optical frontOptic;

extern pros::adi::Pneumatics claw;

extern pros::adi::Pneumatics matchLoadGate;
extern pros::adi::Pneumatics lift;
extern pros::adi::Pneumatics leftDescoreArm;
extern pros::adi::Pneumatics trapDoor;
extern pros::adi::Pneumatics intakeLift;

extern Drivetrain drivetrain;
extern OdomSensors odomSensors;

extern pros::Distance front_dist;
extern pros::Distance left_dist;
extern pros::Distance back_dist;
extern pros::Distance right_dist;
extern pros::Distance claw_dist;
extern std::array<pros::Distance*, MAX_DISTANCE_SENSORS> distSensors;
extern std::array<Pose, MAX_DISTANCE_SENSORS> sensorMounts;

extern ChassisConfig chassisConfig;
extern FeedforwardGains ramseteFf;
extern Chassis chassis;

extern AutonSelector selector;

void registerFieldObstacles();

const std::vector<MclTracking::CircleObstacle>& fieldCircleObstacles();

void clearFieldObstacles();

void registerAutonDisableLine(float lifetimeMs = 16000.0f);
}
