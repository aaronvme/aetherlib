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
    claw.set_value(true);
    startTempWarning(master, {&leftMotors, &rightMotors});

    
    selector.registerAuton("Red 3 Pin", autonRed3Pin);
    selector.registerAuton("None", [] {});
    selector.registerAuton("Blue 3 Pin", autonBlue3Pin);
    selector.registerAuton("Skills", autonSkills);
    selector.registerAuton("UNc", autonTemp);
    selector.init();

    pros::delay(250);

    liftToPinAbove(0, 50, 500);
}

void disabled() {

}

void competition_initialize() {}

void autonomous() {
    
    clawSetAutoGrab(false);
    chassis.setMirror(FieldMirror::None);
    // autonSkills();
    // autonRed3Pin();
    // autonTemp();
    selector.runSelected();
}

void opcontrol() {
    chassis.startTracking();
    // chassis.setPose({-55, 0, vexToStdRad(270)});
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
