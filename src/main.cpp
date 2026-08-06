#include "main.h"

#include "gamespecific/gamespecific_api.hpp"
#include "riku/api.hpp"

using namespace riku;

void initialize() {
    chassis.calibrate();

    selector.registerAuton("Example", autonExample);
    selector.registerAuton("Red Left", autonRedLeft);
    selector.registerAuton("Blue Left", autonBlueLeft);
    selector.registerAuton("Skills", autonSkills);
    selector.init();

    startTempWarning(master, {&leftMotors, &rightMotors});
    startMotorDisconnectWarning(master);
    startDistanceDisconnectWarning(master, {&front_dist, &left_dist, &back_dist, &right_dist});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    registerFieldObstacles();
    registerAutonDisableLine();
    chassis.startTracking();
    selector.runSelected();
}

void opcontrol() {
    startPoseTelemetry();
    driverControl();
}
