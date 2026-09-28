#include "gamespecific/autons.hpp"
#include "gamespecific/lift.hpp"
#include "gamespecific/autons/tuning/support.hpp"
#include "gamespecific/claw.hpp"
#include "chad/api.hpp"
#include "pros/rtos.hpp"

namespace chad {
void autonTemp() {
//toggle========================================
    claw.toggle();
    // chassis.startTracking();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.setPose({-0, -64.5, vexToStdRad(180)});
    registerAutonDisableLine();
    // chassis.moveDistance(1.5, 250, {.minSpeed = 120});
    chassis.tank(-50, -50);
    pros::delay(150);
    chassis.tank(0, 0);
    pros::delay(100);
    chassis.tank(60, 60);
    pros::delay(650);
    chassis.tank(-30, -30);
    pros::delay(250);
    chassis.tank(0, 0);
    pros::delay(200);
    chassis.tank(60, 60);
    pros::delay(500);
    chassis.tank(0,0);

    // pros::delay(750);
    chassis.setPose({0.0, -64.0, vexToStdRad(180)});
    // autons::resetFromBeams();
    // pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
    // pros::delay(250);
    
// //preload=========================================
    chassis.moveToPoint(-23.5, -52.0, 1100, {.forwards=false, .maxSpeed=55.0f}, true);
    liftToPinAbove(0, 200,1000, 127, true);
    chassis.waitUntilSettled();
    chassis.tank(-20, -20);
    pros::delay(500);
    chassis.tank(0,0);
    liftToPin(0, 500, 80, true);
    pros::delay(100);
    claw.toggle();
    pros::delay(500);

    autons::resetFromBeamsNear();
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");





    // // //pin1============================================ 
    chassis.moveToPoint(-0.0f, -58.5f, 500, {.minSpeed = 60.0f, .earlyExitRange=2}, true);
    liftToPin(0, 650);
    // clawGrabWhenClose(1500);
    chassis.turnToPoint(-17.8, -32.5f, 650, {.forwards=false});
    clawSetAutoGrab(true);
    chassis.moveToPoint(-22.2f, -30.3f, 1300, {.forwards=false, .maxSpeed = 55.0f});
    // chassis.waitUntilSettled();
    clawSetAutoGrab(false);
    clawClose();
    // liftToPinAbove(0, 50, 500);
    pros::delay(250);
    chassis.turnToPoint(-23.0, -38.0, 800, {.forwards=false}, true);
    liftToPinAbove(1, 120, 1500);
    chassis.moveToPoint(-24.0f, -43.0f, 1000, {.forwards=false, .maxSpeed = 55.0f}, false);
    chassis.tank(-40, -40);
    pros::delay(500);
    chassis.tank(0, 0);
    pros::delay(500);
    liftToPin(0, 1000, 80,true);
    pros::delay(100);
    claw.toggle();
    pros::delay(150);

    autons::resetFromBeamsNear();
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");

//     pros::delay(500);
// // // // //pin2====================================================
//     chassis.moveToPoint(28.5f, -30.7f, 1300, {.minSpeed = 60.0f, .earlyExitRange = 3}, true);
//     liftToPin(0, 750);
// // // chassis.turnToPoint(-38.0f, -42.0f, 800, {.forwards=false});
//     chassis.turnToPoint(43.0, -42.8f, 650, {.forwards=false});
//     clawSetAutoGrab(true);
//     chassis.moveToPoint(44.0f, -44.0f, 1000, {.forwards=false, .maxSpeed = 40.0f});
//     clawSetAutoGrab(false);
//     clawClose();
//     // liftToPinAbove(0, 50, 500);
//     pros::delay(100);
    

//     liftToPinAbove(2, 145, 1200, 127, true);
//     pros::delay(100);
//     chassis.moveToPoint(23.3f, -52.1f, 700, {.forwards=false, .maxSpeed = 55.0f}, false);
    
//     chassis.tank(-45, -45);
//     pros::delay(200);
//     chassis.tank(0,0);
//     // chassis.waitUntilSettled();
//     pros::delay(150);
//     liftToPin(0, 1000, 80, true);
//     pros::delay(100);
//     claw.toggle();
//     pros::delay(250);

//     autons::resetFromBeamsNear();
//     pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}
}