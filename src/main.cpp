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

    // Tuning routines. One per attempt, in this order -- each measures
    // something the next depends on. Results go to the brain screen; paste them
    // into config.cpp, because nothing survives a reboot.
    //
    //   imu      IMU scale factor        parked, you spin it by hand
    //   1 drive  trackWidth, kV/kS/kA    spins, then shuttles ~30 in
    //   probe    are the motions sane    ~16 in
    //   2 pid    angular + lateral       relay wants +-24 in
    //   verify   steps on those gains    48 in lane
    //   mounts   sensor y offsets        turns in place, 20-40 in off a wall
    //   3 odom   wheel scale, drift      ~66 in ahead, filters OFF
    //   4 filter sensor + pose noise     corner, plus ~16 in behind
    //   ramsete  b and zeta              48 x 24 in, needs phase 1's ffGains
    // selector.registerAuton("Imu scale", autonCalibrateImu);
    // selector.registerAuton("Char 1 drive", autonCharacterize1Drive);
    // selector.registerAuton("Probe 2 motions", autonProbe2Motions);
    // selector.registerAuton("Char 2 pid", autonCharacterize2Pid);
    // selector.registerAuton("Verify pid", autonVerifyPid);
    // selector.registerAuton("Mounts", autonCalibrateMounts);
    // selector.registerAuton("Char 3 odom", autonCharacterize3Odom);
    // selector.registerAuton("Char 4 filters", autonCharacterize4Filters);
    // selector.registerAuton("Tune ramsete", autonTuneRamsete);
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
    autonCalibrateImu();
    // autonCharacterize/1Drive();
    // autonProbe2Motions();
    // autonCharacterize2Pid();
    // autonVerifyPid();
    // autonCalibrateMounts();
    // autonCharacterize3Odom();
    // autonCharacterize4Filters();
    // autonTuneRamsete();
    // chassis.setPose({0,0,0});
    // chassis.turnToHeading(90, 2000);
    // chassis.moveToPoint(0, 30, 2000, {.forwards=false});
    // chassis.moveToPoint(0, 0, 2000);
    // chassis.turnToHeading(0, 2000);
    
    driverControl();
}
