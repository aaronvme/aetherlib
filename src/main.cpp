#include "main.h"

#include "gamespecific/gamespecific_api.hpp"
#include "chad/api.hpp"
#include "gamespecific/autons.hpp"
#include "gamespecific/lift.hpp"
#include "pros/motors.h"

using namespace chad;

void initialize() {
    chassis.calibrate();
    liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    registerFieldObstacles();
    startPoseTelemetry();
    startClawWatch();
    liftMotor.tare_position();
    clawClose();
    startTempWarning(master, {&leftMotors, &rightMotors});

    

    selector.registerAuton("One Field Two Wall", awp);
    selector.registerAuton("None", [] {});
    selector.init();
}

// PROS deletes the previous mode's task without unwinding it, so a motion or
// lift move it started is still running, and its guards never ran.
static void stopLeftovers() {
    chassis.cancelMotion();
    liftCancel();
}

void disabled() {
    stopLeftovers();
}

void competition_initialize() {}

void autonomous() {
    stopLeftovers();
    chassis.startTracking();
    liftToPinAbove(0, 50, 500, 100, true);

    clawSetAutoGrab(false);
    chassis.setMirror(FieldMirror::None);
    // autonSkills();
    // autonRed3Pin();
    // autonTemp();
    selector.runSelected();
}

void opcontrol() {
    stopLeftovers();
    chassis.setMirror(FieldMirror::None);
    clearAutonDisableLine();

    chassis.setPose({-63.5, -7.5, vexToStdRad(0)});
    chassis.startTracking();
    clawSetAutoGrab(false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    // autonProbeGeometry();      // opposing-pair sums vs map
    // autonProbeMcl();              // drive with MCL on, check against beams
    // autonTuneRamsete();           // path follow b and zeta
    // autonCalibrateOdomBias();     // odometry along/cross bias
    // autonCalibrateLocalization();  // mcl/ekf trust
    // autonCharacterize4Filters();  // sensor/pose noise
    // autonCalibrateMounts();   // distance sensor mount offsets
    // autonCalibrateLift();     // kG, friction, rom
    // autonTuneLiftPresets();   // 7 macro heights

    driverControl();
}
