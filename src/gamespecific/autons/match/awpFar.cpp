#include "gamespecific/autons.hpp"
#include "gamespecific/lift.hpp"
// #include "gamespecific/autons/tuning/support.hpp"
#include "gamespecific/claw.hpp"
#include "chad/api.hpp"
#include "pros/rtos.hpp"


namespace chad {
void awpFar() {
//rn its with rcl on.

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose({-7.5f, -63.5f, vexToStdRad(90.0f)});
    //do toggle
    chassis.rcl().disableSensorFor(2, 2000);
    liftToPinAbove(0, 200,1000, 127, true);
    pros::delay(500);

    //pin1
    chassis.turnToPoint(-20, -51, 400, {.forwards=false, .maxSpeed = 100, .earlyExitRange = 15});
    chassis.moveToPoint(-19, -50, 500, {.forwards=false, .maxSpeed = 55});
    chassis.tank(-40, -40);
    pros::delay(100);
    liftToPin(0, 600, 60, true);
    pros::delay(100);
    clawOpen();
    // chassis.rcl().updateBotPoseFromBestSensor();

    //pin2
    pros::delay(350);
    chassis.moveToPoint(-3, -60, 900, {.maxSpeed = 70, .earlyExitRange = 2});
    chassis.swingToHeading(90, SwingSide::Left, 450, {.minSpeed = 100.0f});
    chassis.moveToPoint(-13, -63, 750, {.forwards=false, .maxSpeed = 60, .earlyExitRange = 2});
    chassis.swingToHeading(0, SwingSide::Right, 500, {}, false);
    chassis.tank(-30, -30);
    pros::delay(250);
    clawClose();
    chassis.rcl().updateBotPoseFromBestSensor();
    pros::delay(400);
    chassis.turnToHeading(70, 400, {}, true);
    liftToPinAbove(1, 120, 1500, 127, true);

    chassis.moveToPoint(-6, -58, 900, {.maxSpeed = 70, .earlyExitRange = 2});
    chassis.turnToPoint(-20, -51, 400, {.forwards=false, .maxSpeed = 100, .earlyExitRange = 15});
    chassis.moveToPoint(-24, -46.8, 850, {.forwards=false, .maxSpeed = 55});
    chassis.tank(-40, -40);
    pros::delay(100);
    liftToPin(0, 600, 60, true);
    pros::delay(100);
    clawOpen();
    chassis.tank(0,0);
    chassis.rcl().updateBotPoseFromBestSensor();
    //pin3

    chassis.moveToPoint(2, -64, 900, {.maxSpeed = 70});
    chassis.turnToHeading(295, 400, {.earlyExitRange = 30});
    chassis.swingToHeading(270, SwingSide::Right, 250, {.minSpeed = 100});
    // chassis.moveDistance(-6, 600, {.maxSpeed = 60.0f});
    pros::delay(250);
    chassis.moveToPoint(16, -62, 700, {.forwards=false, .maxSpeed = 60, .earlyExitRange = 2});
    // pros::delay(200);
    // chassis.tank(-20, -20);
    // pros::delay(200);
    chassis.swingToHeading(0, SwingSide::Left, 700, {.maxSpeed = 70.0f}, false);
    clawClose();
    pros::delay(400);
    chassis.rcl().updateBotPoseFromBestSensor();
    

    ///////////////////////////
    chassis.turnToHeading(290, 300, {}, true);
    liftToPinAbove(2, 60, 1500, 127, true);

    chassis.moveToPoint(6, -58, 900, {.maxSpeed = 70, .earlyExitRange = 2});
    chassis.turnToPoint(23.6, -46.6, 400, {.forwards=false, .maxSpeed = 100, .earlyExitRange = 15});
    chassis.moveToPoint(23.6, -46.6, 750, {.forwards=false, .maxSpeed = 55});
    chassis.tank(-30, -30);
    pros::delay(300);
    liftToPin(0, 600, 60, true);
    pros::delay(100);
    clawOpen();
    chassis.tank(0,0);
    pros::delay(250);
    // chassis.rcl().updateBotPoseFromBestSensor();

    chassis.moveToPoint(0, -46.5, 900, {.maxSpeed = 70, .earlyExitRange = 2});
    chassis.turnToPoint(-20.5, -26.5, 700, {.forwards=false, .maxSpeed = 100});
    chassis.moveToPoint(-20.5, -26.5, 1300, {.forwards=false, .maxSpeed = 75});
    clawClose();
}

}