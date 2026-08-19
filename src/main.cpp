#include "main.h"

#include "gamespecific/gamespecific_api.hpp"
#include "riku/api.hpp"

using namespace riku;

void initialize() {
    chassis.calibrate();

    // selector.registerAuton("Example", autonExample);
    // selector.registerAuton("Red Left", autonRedLeft);
    // selector.registerAuton("Blue Left", autonBlueLeft);
    // selector.registerAuton("Skills", autonSkills);

    // Characterisation measures constants of this robot; PidAutoTuner and the
    // sim sweeps search for gains. Run these one per attempt in order, pasting
    // each printed block into config.cpp first: phase 3 drives on phase 2's
    // PID, and phase 4 measures the filter phase 3 configures. See API.md.
    //
    //   1 drive    trackWidth, kV/kS/kA   spins, then shuttles +-60 in
    //   2 pid      angular + lateral      needs phase 1
    //   3 odom     scale, drift           needs phase 2, localization OFF
    //   4 filters  sensor + pose noise    parked square to a wall
    selector.registerAuton("Char 1 drive", autonCharacterize1Drive);
    selector.registerAuton("Char 2 pid", autonCharacterize2Pid);
    selector.registerAuton("Char 3 odom", autonCharacterize3Odom);
    selector.registerAuton("Char 4 filters", autonCharacterize4Filters);
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
