#include "main.h"

#include "gamespecific/gamespecific_api.hpp"
#include "chad/api.hpp"

using namespace chad;

void initialize() {
    chassis.calibrate();
    startPoseTelemetry();

    // selector.registerAuton("Example", autonExample);
    // selector.registerAuton("Red Left", autonRedLeft);
    // selector.registerAuton("Blue Left", autonBlueLeft);
    // selector.registerAuton("Skills", autonSkills);

    // Characterisation measures constants of this robot; PidAutoTuner and the
    // sim sweeps search for gains. Run these one per attempt in order, pasting
    // each printed block into config.cpp first: phase 3 drives on phase 2's
    // PID, and phase 4 measures the filter phase 3 configures. Results print to
    // the brain screen and, tethered, to `pros terminal`. Each phase applies its
    // own values so the next one runs on them, but nothing survives a reboot.
    // Sized to fit two tiles. See API.md.
    //
    //   1 drive    trackWidth, kV/kS/kA   spins, plus ~4 in for ksStraight
    //   2 pid      angular + lateral      needs phase 1, 16 in lane
    //   3 odom     scale, drift           needs phase 2, +-16 in, filters OFF
    //   4 filters  sensor + pose noise    parked square to a wall
    // selector.registerAuton("Char 1 drive", autonCharacterize1Drive);
    // selector.registerAuton("Probe 2 motions", autonProbe2Motions);
    // selector.registerAuton("Char 2 pid", autonCharacterize2Pid);
    // selector.registerAuton("Verify pid", autonVerifyPid);
    // selector.registerAuton("Char 3 odom", autonCharacterize3Odom);
    // selector.registerAuton("Char 4 filters", autonCharacterize4Filters);
    // selector.init();

    // startTempWarning(master, {&leftMotors, &rightMotors});
    // startMotorDisconnectWarning(master);
    // startDistanceDisconnectWarning(master, {&front_dist, &left_dist, &back_dist, &right_dist});
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
    registerFieldObstacles();
    // registerAutonDisableLine();
    chassis.startTracking();
    // selector.runSelected();
    autonCharacterize1Drive();
    // autonProbe2Motions();
    // autonCharacterize2Pid();
    // autonVerifyPid();
    // autonTuneRamsete();
    // autonCharacterize3Odom();
    // autonCharacterize4Filters();
    // chassis.setPose({0,0,0});
    // chassis.turnToHeading(90, 2000);
    // chassis.moveToPoint(0, 30, 2000, {.forwards=false});
    // chassis.moveToPoint(0, 0, 2000);
    // chassis.turnToHeading(0, 2000);
    
    driverControl();
}
