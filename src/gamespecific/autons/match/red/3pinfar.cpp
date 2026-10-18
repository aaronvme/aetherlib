#include "chad/movement_control/turntopoint.hpp"
#include "gamespecific/autons.hpp"
#include "gamespecific/lift.hpp"
#include "gamespecific/autons/tuning/support.hpp"
#include "gamespecific/claw.hpp"
#include "chad/api.hpp"
#include "pros/rtos.hpp"

namespace chad{
void autonRed3PinFar(){
    //setup===========================================
    claw.toggle();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.startTracking();
    chassis.setPose({0, -61.5, vexToStdRad(180)});
    registerAutonDisableLine();

    //toggle===========================================
    chassis.tank(50, 50);
    pros::delay(500);
    chassis.tank(-30, -30);
    pros::delay(250);
    chassis.tank(50, 50);
    pros::delay(500);
    chassis.tank(0,0);


    //preload===========================================
    chassis.moveToPoint(-23.5, -46.6, 1100, {.forwards=false, .maxSpeed=55.0f}, true);
    liftToPinAbove(0, 15,1000);
    chassis.waitUntilSettled();
    pros::delay(200);
    claw.toggle();
    pros::delay(500);

    //1stpin============================================
    chassis.moveToPoint(0, -63, 500, {.maxSpeed= 100}, true);
    liftToPin(0, 500);
    chassis.turnToHeading(80, 500, {});
    chassis.moveToPoint(-13, -61.5, 800, {.forwards=false, .maxSpeed = 70.0f, .minSpeed = 30.0f});
    pros::delay(250);
    chassis.swingToHeading(0, SwingSide::Left, 1200, {.minSpeed = 100.0f});
    claw.toggle();
    pros::delay(250);
    chassis.turnToPoint(0, -55, 500, {}, true);
    liftToPinAbove(1, 120, 1000, 127, true);
    chassis.moveToPoint(0, -55, 500, {}, false);
    chassis.turnToPoint(-22, -48, 500, {.forwards=false}, true);
    chassis.moveToPoint(-22, -48, 900, {.forwards=false, .maxSpeed = 50.0f}, false);
    pros::delay(250);
    liftToPin(0, 1000, 127, true);
    pros::delay(200);
    claw.toggle();

    chassis.moveToPoint(2, -60, 700, {}, false);
    chassis.turnToPoint(12, -63, 800, {.forwards=false});


    // chassis.waitUntilSettled();

    
}
}