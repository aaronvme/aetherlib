#include "main.h"

#include "gamespecific/gamespecific_api.hpp"
#include "chad/api.hpp"
#include "gamespecific/autons.hpp"

using namespace chad;

void initialize() {
    chassis.calibrate();
    startPoseTelemetry();

    startTempWarning(master, {&leftMotors, &rightMotors});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    autonBlue3Pin();
}

void opcontrol() {
    autonCalibrateLift();

    driverControl();
}
