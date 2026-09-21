#include "gamespecific/autons.hpp"
#include "gamespecific/lift.hpp"
#include "gamespecific/autons/tuning/support.hpp"
#include "gamespecific/claw.hpp"
#include "chad/api.hpp"
#include "pros/rtos.hpp"

namespace chad {
void autonRed3Pin() {
    registerAutonDisableLine();
//toggle========================================
    claw.toggle();
    // chassis.startTracking();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.setPose({-61.5, 0, vexToStdRad(270)});
    // chassis.moveDistance(1.5, 250, {.minSpeed = 120});
    chassis.tank(50, 50);
    pros::delay(500);
    chassis.tank(-30, -30);
    pros::delay(250);
    chassis.tank(50, 50);
    pros::delay(500);
    chassis.tank(0,0);

    // pros::delay(750);
    chassis.setPose({-64.0, 0, vexToStdRad(270)});
    // autons::resetFromBeams();
    // pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
    // pros::delay(250);
    
// //preload=========================================
    chassis.moveToPoint(-52.0, -16.5, 1100, {.forwards=false, .maxSpeed=55.0f}, true);
    liftToPinAbove(0, 50,1000);
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
    chassis.moveToPoint(-52.2f, -0.0f, 500, {.minSpeed = 60.0f, .earlyExitRange=2}, true);
    liftToPin(0, 650);
    // clawGrabWhenClose(1500);
    chassis.turnToPoint(-31.5, -19.8f, 690, {.forwards=false});
    clawSetAutoGrab(true);
    chassis.moveToPoint(-30.3f, -21.2f, 1000, {.forwards=false, .maxSpeed = 65.0f});
    // chassis.waitUntilSettled();
    clawSetAutoGrab(false);
    clawClose();
    pros::delay(250);
    chassis.turnToPoint(-38.0, -25.0, 800, {.forwards=false}, true);
    liftToPinAbove(1, 120, 1500);
    chassis.moveToPoint(-43.0f, -26.0f, 1000, {.forwards=false, .maxSpeed = 55.0f}, false);
    liftToPin(0, 1000, 80,true);
    pros::delay(100);
    claw.toggle();
    pros::delay(150);

    autons::resetFromBeamsNear();
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");

    pros::delay(500);
// // // //pin2====================================================
    chassis.moveToPoint(-30.7f, -28.5f, 1300, {.minSpeed = 60.0f, .earlyExitRange = 3}, true);
    liftToPin(0, 750);
// // chassis.turnToPoint(-38.0f, -42.0f, 800, {.forwards=false});
    chassis.turnToPoint(-42.8, -43.0f, 750, {.forwards=false});
    clawSetAutoGrab(true);
    chassis.moveToPoint(-44.9f, -45.0f, 1000, {.forwards=false, .maxSpeed = 60.0f});
    clawSetAutoGrab(false);
    // Close either way. If the watcher never saw the ring the claw is still
    // open, and every claw.toggle() after this point would come out inverted.
    clawClose();
    pros::delay(250);
    

    liftToPinAbove(2, 145, 1200, 127, true);
    pros::delay(500);
    chassis.moveToPoint(-52.1f, -23.3f, 800, {.forwards=false, .maxSpeed = 55.0f}, false);
    
    chassis.tank(-25, -25);
    pros::delay(500);
    chassis.tank(0,0);
    // chassis.waitUntilSettled();
    pros::delay(250);
    liftToPin(0, 1000, 80, true);
    pros::delay(100);
    claw.toggle();
    pros::delay(250);

    autons::resetFromBeamsNear();
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");

}
}
