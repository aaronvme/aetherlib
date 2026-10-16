#include "chad/api.hpp"
#include "pros/rtos.hpp"
#include "gamespecific/gamespecific_api.hpp"
#include "gamespecific/autons/tuning/support.hpp"

namespace chad {
void awpFar() {
//rn its with rcl on.

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose({-7.5f, -63.5f, vexToStdRad(90.0f)});
    //do toggle
    sideToggleFlip(Side::Right, 2);
    chassis.rcl().disableSensorFor(2, 2000);


    liftToPinAbove(0, 200,1000, 127, true);
    pros::delay(500);

    //pin1
    chassis.turnToPoint(-32, -48, 400, {.forwards=false, .maxSpeed = 70, .earlyExitRange = 15});
    chassis.moveToPoint(-33, -47, 500, {.forwards=false, .maxSpeed = 55});
    chassis.tank(-40, -40);
    pros::delay(100);
    liftToPin(0, 800, 40, true);
    pros::delay(100);
    clawOpen();
    // chassis.rcl().updateBotPoseFromBestSensor();

    //pin2
    pros::delay(350);
    chassis.moveToPoint(-9, -56, 500, {.maxSpeed = 70});
    // chassis.swingToHeading(90, SwingSide::Left, 450, {.minSpeed = 100.0f});
    chassis.turnToPoint(-20, -59, 300, {.forwards=false, .maxSpeed = 70.0f});
    chassis.moveToPoint(-17, -56, 450, {.forwards=false, .maxSpeed = 70, .earlyExitRange = 1});
    pros::delay(100);
    chassis.swingToHeading(0, SwingSide::Right, 500, {}, false);
    autons::resetFromBeamsNear();
    // chassis.tank(-30, -70);
    pros::delay(50);
    clawClose();
    autons::resetFromBeamsNear(); 
    chassis.tank(-30,-30);
    chassis.rcl().updateBotPoseFromBestSensor();
    pros::delay(200);
    chassis.turnToHeading(110, 400, {}, true);
    pros::delay(300);
    liftToPinAbove(1, 140, 1500, 127, true);

    chassis.moveToPoint(-6, -58, 900, {.maxSpeed = 70, .earlyExitRange = 2});
    chassis.turnToPoint(-20, -51, 400, {.forwards=false, .maxSpeed = 70, .earlyExitRange = 15});
    chassis.moveToPoint(-24, -46.8, 850, {.forwards=false, .maxSpeed = 55});
    chassis.tank(-40, -40);
    pros::delay(100);
    liftToPin(0, 600, 60, true);
    pros::delay(100);
    autons::resetFromBeamsNear();
    clawOpen();
    chassis.tank(0,0);
    chassis.rcl().updateBotPoseFromBestSensor();
    //pin3

    chassis.moveToPoint(2, -61, 900, {.maxSpeed = 60});
    chassis.turnToHeading(265, 500, {.earlyExitRange = 30});
    // chassis.swingToHeading(270, SwingSide::Right, 250, {.minSpeed = 100});
    // chassis.moveDistance(-6, 600, {.maxSpeed = 60.0f});
    // pros::delay(250);
    chassis.moveToPoint(15.3, -61, 400, {.forwards=false, .maxSpeed = 65, .earlyExitRange = 2});
    // pros::delay(200);
    // chassis.tank(-20, -20);
    // pros::delay(200);
    chassis.swingToHeading(0, SwingSide::Left, 700, {.maxSpeed = 60.0f}, false);
     chassis.tank(-60, -60);
    autons::resetFromBeamsNear();
     pros::delay(100);
    clawClose();
    pros::delay(200);
    chassis.rcl().updateBotPoseFromBestSensor();


    // ///////////////////////////
    chassis.turnToHeading(250, 300, {}, true);
    liftToPinAbove(2, 60, 1500, 127, true);

    chassis.moveToPoint(6, -58, 900, {.maxSpeed = 70, .earlyExitRange = 2});
    // chassis.turnToPoint(23.6, -46.6, 400, {.forwards=false, .maxSpeed = 100, .earlyExitRange = 15});
    chassis.moveToPoint(23.6, -46.6, 750, {.forwards=false, .maxSpeed = 55});
    chassis.tank(-30, -30);
    pros::delay(300);
    liftToPin(0, 600, 60, true);
    pros::delay(100);
    clawOpen();
    chassis.tank(0,0);
    pros::delay(250);

    chassis.rcl().stopTracking();
    chassis.rcl().updateBotPoseFromBestSensor();
    // pin 4


    // chassis.moveToPoint(0, -46.5, 900, {.earlyExitRange = 2});
    // // chassis.turnToPoint(-20.5, -26.5, 600, {.forwards=false, .maxSpeed = 100, .earlyExitRange = 15});
    // chassis.moveToPoint(-19.5, -30.3, 1200, {.forwards=false, .maxSpeed = 75}, true);
    // pros::delay(1000);
    // clawClose();
    // pros::delay(250);
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // liftToPinAbove(2, 20, 1500, 127, true);
    // chassis.moveToPoint(-47.5f, -23.2f, 2000, {.forwards=false, .minSpeed = 90}, true);

}

}
