#include "main.h"

#include "gamespecific/gamespecific_api.hpp"
#include "chad/api.hpp"
#include "gamespecific/autons.hpp"
#include "gamespecific/lift.hpp"
#include "pros/motors.h"

using namespace chad;

void initialize() {
    chassis.calibrate();
    registerFieldObstacles();
    startPoseTelemetry();
    liftMotor.tare_position();
    claw.set_value(true);
    startTempWarning(master, {&leftMotors, &rightMotors});
}

void disabled() {}

void competition_initialize() {}

void jiggle() {
    chassis.moveDistance(0.5, 1000);
    chassis.moveDistance(-0.5, 1000);
}

void autonomous() {
    autonBlue3Pin();
}

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    autonTuneRamsete();           // path follow b and zeta
    // autonCalibrateOdomBias();     // odometry along/cross bias
    // autonCalibrateLocalization();  // MCL/EKF trust, hand placed
    // autonCharacterize4Filters();  // sensor + pose noise
    // autonCalibrateMounts();   // distance sensor mount offsets
    // autonCalibrateLift();     // kG, friction, travel
    // autonTuneLiftPresets();   // the 7 macro heights

    driverControl();
}
