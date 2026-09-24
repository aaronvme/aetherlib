#include "gamespecific/autons.hpp"
#include "gamespecific/claw.hpp"
#include "gamespecific/lift.hpp"
#include "gamespecific/autons/tuning/support.hpp"
#include "chad/api.hpp"
#include "pros/misc.h"

namespace chad {
void autonSkills() {
//paths===============================================
    std::vector<Pose> go = {
        {-34.2f, -37.6f},
        {-44.8f, -51.6f},
        {-58.5f, -54.4f}
    };
    std::vector<Pose> go2 = {
        {-34.2f, -37.6f},
        {-44.8f, -48.1f},
        {-58.0f, -48.4f}
    };
    std::vector<Pose> come = {
        {-61.4f, -58.4f},
        {-46.5f, -45.7f},
        {-45.0f, -39.1f}
    };

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
    liftToPinAbove(0, 100,1000);
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
    chassis.turnToPoint(-30.5, -22.5f, 690, {.forwards=false});
    clawSetAutoGrab(true);
    chassis.moveToPoint(-30.3f, -22.7f, 1000, {.forwards=false, .maxSpeed = 65.0f});
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
    chassis.moveToPoint(-52.1f, -24.3f, 800, {.forwards=false, .maxSpeed = 55.0f}, false);
    
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

//matchloads (repeat2)====================================
    clawSetAutoGrab(true);
    chassis.moveToPoint(-40.5f, -40.7f, 800, {.minSpeed=60}, true);
    liftToPin(0, 1000);
    chassis.moveRAMSETE(go, 2400,
                        {.forwards=false, .maxSpeed = 65.0f, .minSpeed = 50.0f},
                        false);
    clawClose();
    pros::delay(250);
    autons::resetFromBeamsNear();
    clawSetAutoGrab(false);
    
    chassis.moveRAMSETE(come, 1000, {.minSpeed=50.0f}, true);
    pros::delay(500);
    liftToPinAbove(3, 145, 1500);
    pros::delay(500);
    chassis.moveToPoint(-49.0f, -18.9f, 1000, {.forwards=false, .maxSpeed = 60.0f},false);
    // pros::delay(750);
    chassis.tank(-30, -30);
    pros::delay(500);
    chassis.tank(0,0);
    liftToPin(0, 2000, 80, true);
    pros::delay(100);
    claw.toggle();
    pros::delay(250);

    autons::resetFromBeamsNear();
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
    clawSetAutoGrab(true);
    pros::delay(500);

    chassis.moveToPoint(-34.5f, -34.7f, 800, {.minSpeed=60}, false);
    chassis.moveRAMSETE(go2, 2400,
                        {.forwards=false, .maxSpeed = 65.0f, .minSpeed = 50.0f},
                        false);
    clawClose();
    pros::delay(250);
    autons::resetFromBeamsNear();
    chassis.moveRAMSETE(come, 1000, {.minSpeed=50.0f}, true);
    pros::delay(500);
    liftToPinAbove(4, 155, 1750);
    pros::delay(500);
    chassis.moveToPoint(-52.0f, -16.9f, 1000, {.forwards=false, .maxSpeed = 60.0f},false);
    // // pros::delay(750);
    chassis.tank(-30, -30);
    pros::delay(500);
    chassis.tank(0,0);
    liftToPin(0, 2000, 80, true);
    pros::delay(100);
    claw.toggle();
    pros::delay(250);
    clawSetAutoGrab(false);


    autons::resetFromBeamsNear();
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");

    pros::delay(500);
// //2ndgoofystud
    chassis.moveToPoint(-32.5f, -35.7f, 800, {.minSpeed=60}, true);
    clawClose();
    liftToPin(0);
    chassis.moveRAMSETE(go2, 2000,
                    {.forwards=false, .minSpeed = 50.0f},
                    false);








                    //     chassis.moveToPoint(-17.8f, -46.1f, 2500, {.forwards=false, .maxSpeed = 70.0f}, true);
//     liftToPinAbove(0, 50, 1000);
//     chassis.waitUntilSettled();
//     claw.toggle();
//     pros::delay(250);
//     chassis.moveToPoint(-42.2f, -38.1f, 1000, {.minSpeed = 70.0f, .earlyExitRange = 2}, true);
//     liftToPin(0);

}  

}
