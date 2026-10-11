#include "gamespecific/autons.hpp"
#include "gamespecific/lift.hpp"
// #include "gamespecific/autons/tuning/support.hpp"
#include "gamespecific/claw.hpp"
#include "chad/api.hpp"
#include "pros/rtos.hpp"


namespace chad {
void awp() {
//rn its with rcl on.

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose({-63.5f, -7.5f, vexToStdRad(0.0f)});
    //do toggle
    chassis.rcl().disableSensorFor(2, 2000);
    liftToPinAbove(0, 200,1000, 127, true);
    pros::delay(500);

    //pin1
    chassis.turnToPoint(-51, -20, 400, {.forwards=false, .maxSpeed = 100, .earlyExitRange = 15});
    chassis.moveToPoint(-50, -19, 500, {.forwards=false, .maxSpeed = 55});
    chassis.tank(-40, -40);
    pros::delay(100);
    liftToPin(0, 600, 60, true);
    pros::delay(100);
    clawOpen();
    // chassis.rcl().updateBotPoseFromBestSensor();

    //pin2
    pros::delay(350);
    chassis.moveToPoint(-60, -3, 900, {.maxSpeed = 70, .earlyExitRange = 2});
    chassis.swingToHeading(0, SwingSide::Right, 450, {.minSpeed = 100.0f});
    chassis.moveToPoint(-63, -13, 750, {.forwards=false, .maxSpeed = 60, .earlyExitRange = 2});
    chassis.swingToHeading(90, SwingSide::Left, 500, {}, false);
    chassis.tank(-30, -30);
    pros::delay(250);
    clawClose();
    chassis.rcl().updateBotPoseFromBestSensor();
    pros::delay(400);
    chassis.turnToHeading(20, 400, {}, true);
    liftToPinAbove(1, 120, 1500, 127, true);

    chassis.moveToPoint(-58, -6, 900, {.maxSpeed = 70, .earlyExitRange = 2});
    chassis.turnToPoint(-51, -20, 400, {.forwards=false, .maxSpeed = 100, .earlyExitRange = 15});
    chassis.moveToPoint(-46.8, -24, 850, {.forwards=false, .maxSpeed = 55});
    chassis.tank(-40, -40);
    pros::delay(100);
    liftToPin(0, 600, 60, true);
    pros::delay(100);
    clawOpen();
    chassis.tank(0,0);
    chassis.rcl().updateBotPoseFromBestSensor();
    //pin3

    chassis.moveToPoint(-64, 2, 900, {.maxSpeed = 70});
    chassis.turnToHeading(155, 400, {.earlyExitRange = 30});
    chassis.swingToHeading(180, SwingSide::Left, 250, {.minSpeed = 100});
    // chassis.moveDistance(-6, 600, {.maxSpeed = 60.0f});
    pros::delay(250);
    chassis.moveToPoint(-62, 16, 700, {.forwards=false, .maxSpeed = 60, .earlyExitRange = 2});
    // pros::delay(200);
    // chassis.tank(-20, -20);
    // pros::delay(200);
    chassis.swingToHeading(90, SwingSide::Right, 700, {.maxSpeed = 70.0f}, false);
    clawClose();
    pros::delay(400);
    chassis.rcl().updateBotPoseFromBestSensor();
    

    ///////////////////////////
    chassis.turnToHeading(160, 300, {}, true);
    liftToPinAbove(2, 60, 1500, 127, true);

    chassis.moveToPoint(-58, 6, 900, {.maxSpeed = 70, .earlyExitRange = 2});
    chassis.turnToPoint(-46.6, 23.6, 400, {.forwards=false, .maxSpeed = 100, .earlyExitRange = 15});
    chassis.moveToPoint(-46.6, 23.6, 750, {.forwards=false, .maxSpeed = 55});
    chassis.tank(-30, -30);
    pros::delay(300);
    liftToPin(0, 600, 60, true);
    pros::delay(100);
    clawOpen();
    chassis.tank(0,0);
    pros::delay(250);
    // chassis.rcl().updateBotPoseFromBestSensor();

    chassis.moveToPoint(-46.5, 0, 900, {.maxSpeed = 70, .earlyExitRange = 2});
    chassis.turnToPoint(-26.5, -20.5, 700, {.forwards=false, .maxSpeed = 100});
    chassis.moveToPoint(-26.5, -20.5, 1300, {.forwards=false, .maxSpeed = 75});
    clawClose();
}

}