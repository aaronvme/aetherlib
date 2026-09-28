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

/// Ignore distance returns from across the diagonal for the autonomous period.
/// Two opposing robots move around over there, so anything ranged across it is
/// measuring one of them rather than a wall -- and at that distance it is the
/// weakest reading on the robot even when it is honest.
///
/// Call it AFTER setPose: the half it keeps is the half the robot is standing
/// in when it runs, which is how it works for either alliance untold.
void registerAutonDisableLine(float lifetimeMs = 16000.0f);

void clearAutonDisableLine();
}
