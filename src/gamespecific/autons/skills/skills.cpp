#include "gamespecific/autons.hpp"
#include "gamespecific/lift.hpp"
#include "chad/api.hpp"

namespace chad {
void autonSkills() {
//paths===============================================
    std::vector<Pose> go = {
        {-34.2f, -37.6f},
        {-44.8f, -55.6f},
        {-61.4f, -58.4f}
    };
    std::vector<Pose> come = {
        {-61.4f, -58.4f},
        {-46.5f, -45.7f},
        {-45.0f, -39.1f}
    };
//toggle========================================
    chassis.startTracking();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose({-60.0f, 0.0f, vexToStdRad(270.0f)});
    chassis.moveToPoint(-64, 0, 200, {.forwards=false, .minSpeed=127.0f});
    chassis.moveToPoint(-74, 0, 500,{ .minSpeed=120.0f});
    chassis.moveToPoint(-65, 0, 500,{.forwards=false, .minSpeed=120.0f});
    chassis.moveToPoint(-74, 0, 500,{ .minSpeed=120.0f});

//preload=========================================
    chassis.moveToPoint(-46.7, -23.5, 1250, {.forwards=false, .maxSpeed=60.0f}, true);
    liftToPinAbove(0, 15,1000);
    chassis.waitUntilSettled();
    pros::delay(200);
    claw.toggle();
    claw.toggle();
    pros::delay(350);
//pin1============================================ 
    chassis.moveToPoint(-52.2f, -4.4f, 650, {.minSpeed = 60.0f, .earlyExitRange=2}, true);
    liftToPin(0);
    chassis.moveToPoint(-28.3f, -19.8f, 1000, {.forwards=false, .maxSpeed = 60.0f});
    chassis.waitUntilSettled();
    claw.toggle();
    pros::delay(250);
    chassis.turnToPoint(-46.3, -23.3, 800, {.forwards=false}, true);
    liftToPinAbove(0, 105, 1000);
    chassis.moveToPoint(-49.4f, -24.3f, 800, {.forwards=false, .maxSpeed = 60.0f}, false);
    claw.toggle();
//pin2====================================================
    chassis.moveToPoint(-24.5f, -27.5f, 600, {.minSpeed = 60.0f, .earlyExitRange = 3}, true);
    liftToPin(0);
    chassis.moveToPoint(-43.7f, -43.9f, 1000, {.forwards=false, .maxSpeed = 60.0f});
    claw.toggle();
    pros::delay(250);
    chassis.moveToPoint(-47.3f, -20.2f, 800, {.forwards=false, .maxSpeed = 55.0f}, true);
    liftToPinAbove(1, 105, 1000);
    chassis.waitUntilSettled();
    claw.toggle();
    pros::delay(250);
//matchloads (repeat6)====================================
    for (int i = 0; i < 4; i++) {
        chassis.moveToPoint(-32.5f, -35.7f, 800, {.minSpeed=60}, true);
        liftToPin(0);
        chassis.moveRAMSETE(go, 2000,
                        {.forwards=false, .minSpeed = 50.0f},
                        false);
        claw.toggle();
        chassis.moveRAMSETE(come, 1500, {.minSpeed=50.0f}, true);
        liftToPinAbove(i+2, 105, 1500);
        chassis.moveToPoint(-47.0f, -18.9f, 800, {.forwards=false, .maxSpeed = 60.0f},false);
        claw.toggle();
        pros::delay(250);
    }

//2ndgoofystud
    chassis.moveToPoint(-32.5f, -35.7f, 800, {.minSpeed=60}, true);
    claw.toggle();
    liftToPin(0);
    chassis.moveRAMSETE(go, 2000,
                    {.forwards=false, .minSpeed = 50.0f},
                    false);

    chassis.moveToPoint(-17.8f, -46.1f, 2500, {.forwards=false, .maxSpeed = 70.0f}, true);
    liftToPinAbove(0, 50, 1000);
    chassis.waitUntilSettled();
    claw.toggle();
    pros::delay(250);
    chassis.moveToPoint(-42.2f, -38.1f, 1000, {.minSpeed = 70.0f, .earlyExitRange = 2}, true);
    liftToPin(0);

}  

}
