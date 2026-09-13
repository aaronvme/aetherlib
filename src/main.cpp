#include "main.h"

#include "gamespecific/gamespecific_api.hpp"
#include "chad/api.hpp"
#include "gamespecific/autons.hpp"
#include "gamespecific/lift.hpp"

using namespace chad;

void initialize() {
    chassis.calibrate();
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
    // autonCalibrateLift();      // kG, friction, travel
    // autonTuneLiftPresets();   // the 7 macro heights

    driverControl();
}
