#include "main.h"

#include "gamespecific/gamespecific_api.hpp"
#include "chad/api.hpp"

using namespace chad;

void initialize() {
    chassis.calibrate();
    startPoseTelemetry();

    startTempWarning(master, {&leftMotors, &rightMotors});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis.startTracking();
}

void opcontrol() {
    autonCalibrateLift();

    driverControl();
}
