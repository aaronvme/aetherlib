#include "gamespecific/autons.hpp"
#include "gamespecific/lift.hpp"

#include "chad/api.hpp"
#include "pros/rtos.hpp"

namespace chad {
void autonRed3Pin() {
//toggle========================================
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose({-65.0f, 0.0f, vexToStdRad(270.0f)});
    chassis.moveToPoint(-64, 0, 200, {.forwards=false, .minSpeed=127.0f});
    chassis.moveToPoint(-74, 0, 500,{ .minSpeed=120.0f});
    chassis.moveToPoint(-65, 0, 500,{.forwards=false, .minSpeed=120.0f});
    chassis.moveToPoint(-74, 0, 500,{ .minSpeed=120.0f});

//pin1=========================================
    chassis.moveToPoint(-55, -20.0, 1250, {.forwards=false, .maxSpeed=60.0f}, true);
    liftToPinAbove(0, 10,1000);
    chassis.waitUntilSettled();
    claw.toggle();
    claw.toggle();
    pros::delay(350);

//pin2=========================================
    chassis.moveToPoint(-63.5,-6, 500, {}, false);
    chassis.moveToPoint(-36.5, -21, 1150,{.forwards=false, .maxSpeed=60.0f}, true);
    liftToPin(0, 1000);
    chassis.waitUntilSettled();
    pros::delay(200);
    claw.toggle();
    pros::delay(280);
    chassis.turnToPoint(-87.5,-23.5,500,{ .forwards=false}, true);
    liftToPinAbove(1, 105,1000);
    chassis.moveToPoint(-87.5,-23.5, 850,{ .forwards=false, .maxSpeed=60.0f}, false);
    claw.toggle();
    pros::delay(500);

//pin3=========================================
    chassis.moveToPoint(-34.5, -19, 650, {}, true);
    liftToPin(0, 1000);
    chassis.turnToHeading(35, 500);
    chassis.moveToPoint(-50.6,-41.8,1300, {.forwards=false, .maxSpeed=60.0f},false);
    claw.toggle();
    pros::delay(250);
    chassis.turnToPoint(-51.6,-35.1, 500 ,{ .forwards=false}, true);
    liftToPinAbove(2, 105);
    chassis.moveDistance(-12.0f, 650, {.maxSpeed=50.0f}, false);
    pros::delay(500);
    claw.toggle();
}
}
