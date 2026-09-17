#include "chad/movement_control/movetopoint.hpp"
#include "gamespecific/autons.hpp"
#include "gamespecific/lift.hpp"

#include "chad/api.hpp"
#include "pros/rtos.hpp"

namespace chad {
void autonBlue3Pin() {
//toggle========================================
    // chassis.startTracking();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose({64.0f, 0.0f, 90.0f});
    chassis.moveToPoint(64, -1, 200, {.forwards=false, .minSpeed=127.0f});
    // chassis.turnToHeading(90.0f, 1000);
    chassis.moveToPoint(64, 9, 400,{ .minSpeed=120.0f});
    chassis.moveToPoint(64, 0, 400,{.forwards=false, .minSpeed=120.0f});
    chassis.moveToPoint(64, 9, 400,{ .minSpeed=120.0f});

//pin1=========================================
    chassis.moveToPoint(44, -10.0, 1250, {.forwards=false, .maxSpeed=60.0f}, true);
    liftToPinAbove(0, 17,1000);
    chassis.waitUntilSettled();
    pros::delay(270);
    claw.toggle();
    claw.toggle();
    pros::delay(35 );

//pin2=========================================
    chassis.moveToPoint(58,-1.5, 500, {}, false);
    chassis.moveToPoint(41.8, -28, 1150,{.forwards=false, .maxSpeed=60.0f}, true);
    liftToPin(0, 1000);
    chassis.waitUntilSettled();
    pros::delay(200);
    claw.toggle();
    pros::delay(280);
    chassis.turnToPoint(41,-20,800,{ .forwards=false}, true);
    liftToPinAbove(1, 105,1000);
    chassis.moveToPoint(40,-18.5, 850,{ .forwards=false, .maxSpeed=60.0f}, false);
    claw.toggle();
    pros::delay(500);

//pin3=========================================
    chassis.moveToPoint(45, -30.5, 1000, {}, true);
    liftToPin(0, 1000);
    chassis.turnToHeading(125, 800);
    chassis.moveToPoint(22.4,-16.6,1300, {.forwards=false, .maxSpeed=60.0f},false);
    claw.toggle();
    pros::delay(250);
    chassis.turnToPoint(28.9,-13.4, 800 ,{ .forwards=false}, true);
    liftToPinAbove(2, 105);
    chassis.moveDistance(-12.0f, 800, {.maxSpeed=50.0f}, false);
    pros::delay(500);
    claw.toggle();
//middle
    chassis.moveToPoint(26.7, -13.9, 1000, {}, true);
    pros::delay(50);

}
}
