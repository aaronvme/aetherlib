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
        {-44.8f, -56.6f},
        {-58.0f, -57.4f}
    };
    std::vector<Pose> go2 = {
        {-34.2f, -37.6f},
        {-44.8f, -54.1f},
        {-58.5f, -55.4f}
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
    chassis.tank(-25, -25);
    pros::delay(300);
    chassis.tank(0, 0);
    pros::delay(100);
    chassis.tank(60, 60);
    pros::delay(650);
    chassis.tank(-22, -22);
    pros::delay(250);
    chassis.tank(0, 0);
    pros::delay(200);
    chassis.tank(60, 60);
    pros::delay(500);
    chassis.tank(0,0);

    // pros::delay(750);
    chassis.setPose({-64.0, 0, vexToStdRad(270)});
    // autons::resetFromBeams();
    // pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
    // pros::delay(250);
    
// //preload=========================================
    chassis.moveToPoint(-52.0, -16.5, 1100, {.forwards=false, .maxSpeed=55.0f}, true);
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
    chassis.moveToPoint(-59.2f, -0.0f, 500, {.minSpeed = 60.0f, .earlyExitRange=2}, true);
    liftToPin(0, 650);
    // clawGrabWhenClose(1500);
    chassis.turnToPoint(-32.5, -17.8f, 650, {.forwards=false});
    clawSetAutoGrab(true);
    chassis.moveToPoint(-30.3f, -19.2f, 1700, {.forwards=false, .maxSpeed = 55.0f});
    // chassis.waitUntilSettled();
    clawSetAutoGrab(false);
    clawClose();
    // liftToPinAbove(0, 50, 500);
    pros::delay(250);
    chassis.turnToPoint(-38.0, -23.0, 800, {.forwards=false}, true);
    liftToPinAbove(1, 120, 1500);
    chassis.moveToPoint(-43.0f, -24.0f, 1000, {.forwards=false, .maxSpeed = 55.0f}, false);
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
    chassis.turnToPoint(-42.8, -43.0f, 650, {.forwards=false});
    clawSetAutoGrab(true);
    chassis.moveToPoint(-43.5f, -43.0f, 1000, {.forwards=false, .maxSpeed = 40.0f});
    clawSetAutoGrab(false);
    clawClose();
    // liftToPinAbove(0, 50, 500);
    pros::delay(100);
    

    liftToPinAbove(2, 145, 1200, 127, true);
    pros::delay(100);
    chassis.moveToPoint(-52.1f, -23.3f, 700, {.forwards=false, .maxSpeed = 55.0f}, false);
    
    chassis.tank(-45, -45);
    pros::delay(200);
    chassis.tank(0,0);
    // chassis.waitUntilSettled();
    pros::delay(150);
    liftToPin(0, 1000, 80, true);
    pros::delay(100);
    claw.toggle();
    pros::delay(250);

    autons::resetFromBeamsNear();
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");

// //matchloads (repeat2)====================================
    
    chassis.moveToPoint(-32.5f, -35.7f, 800, {.minSpeed=60}, true);
    liftToPin(0, 1000);
    clawSetAutoGrab(true);
    chassis.moveRAMSETE(go, 2400,
                        {.forwards=false, .maxSpeed = 65.0f, .minSpeed = 50.0f},
                        false);
    
    clawSetAutoGrab(false);
    clawClose();
    pros::delay(250);
    autons::resetFromBeamsNear();
    
    chassis.moveRAMSETE(come, 1000, {.minSpeed=50.0f}, true);
    pros::delay(500);
    liftToPinAbove(3, 165, 2000);
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
    
    pros::delay(500);
    chassis.moveToPoint(-34.5f, -34.7f, 800, {.minSpeed=60}, false);
    clawSetAutoGrab(true);
    chassis.moveRAMSETE(go2, 2400,
                        {.forwards=false, .maxSpeed = 65.0f, .minSpeed = 50.0f},
                        false);
    clawClose();
    
    clawSetAutoGrab(false);
    pros::delay(250);
    autons::resetFromBeamsNear();
    chassis.moveRAMSETE(come, 1000, {.minSpeed=50.0f}, true);
    pros::delay(500);
    liftToPinAbove(4, 165, 1750);
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


    //gay
    chassis.moveToPoint(-50, -30, 1000, {}, false);
    chassis.turnToPoint(0, 0, 1000, {});
    chassis.moveToPoint(0, 0,2000, {.maxSpeed = 50.0f});

//     autons::resetFromBeamsNear();
//     pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");

//     pros::delay(500);
// // //2ndgoofystud
//     chassis.moveToPoint(-32.5f, -35.7f, 800, {.minSpeed=60}, true);
//     clawClose();
//     liftToPin(0);
//     chassis.moveRAMSETE(go2, 2000,
//                     {.forwards=false, .minSpeed = 50.0f},
//                     false);








                    //     chassis.moveToPoint(-17.8f, -46.1f, 2500, {.forwards=false, .maxSpeed = 70.0f}, true);
//     liftToPinAbove(0, 50, 1000);
//     chassis.waitUntilSettled();
//     claw.toggle();
//     pros::delay(250);
//     chassis.moveToPoint(-42.2f, -38.1f, 1000, {.minSpeed = 70.0f, .earlyExitRange = 2}, true);
//     liftToPin(0);

}  

}
