#include "main.h"

#include "gamespecific/gamespecific_api.hpp"
#include "chad/api.hpp"
#include "gamespecific/autons.hpp"
#include "gamespecific/lift.hpp"
#include "pros/motors.h"

using namespace chad;

void initialize() {
    chassis.calibrate();
    liftMotor.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    registerFieldObstacles();
    startPoseTelemetry();
    startClawWatch();
    startSideToggle();
    liftMotor.tare_position();
    clawClose();
    startTempWarning(master, {&leftMotors, &rightMotors});

    

    selector.registerAuton("AWP", awp);
    selector.registerAuton("AWP Farside", awpFar);
    selector.registerAuton("None", [] {});
    selector.init();
}

// PROS deletes the previous mode's task without unwinding it, so a motion or
// lift move it started is still running, and its guards never ran. The same
// goes for a side-toggle flip, or R2's manual spin if driver was holding it.
static void stopLeftovers() {
    chassis.cancelMotion();
    liftCancel();
    sideToggleCancel();
}

void disabled() {
    liftMotor.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
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
    liftMotor.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
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
