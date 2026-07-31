#pragma once

#include <array>

#include "main.h"
#include "AaronLib/AaronLib_API.hpp"
#include "AaronLib/Tracking/RCL_Tracking.hpp"


namespace AARONLIB {

extern pros::Controller master;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Motor frontMotor;
extern pros::Motor leverMotor;
extern pros::Motor armMotor;

extern ScaledIMU imu;
extern pros::Optical frontOptic;

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
extern std::array<pros::Distance*, MAX_DISTANCE_SENSORS> distSensors;
extern std::array<Pose, MAX_DISTANCE_SENSORS> sensorMounts;

extern ChassisConfig chassisConfig;
extern Chassis chassis;

extern AutonSelector selector;

void registerFieldObstacles();

void registerAutonDisableLine(float lifetimeMs = 16000.0f);

}  // namespace AARONLIB

