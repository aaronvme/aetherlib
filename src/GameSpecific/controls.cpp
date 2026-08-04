#include "GameSpecific/controls.hpp"
#include "GameSpecific/robot_config.hpp"

#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace riku {

// ============================================================================
// Pose telemetry — controller screen
// ============================================================================
// Runs in its own task so printing never stalls the drive loop, and keeps
// printing during autonomous. The V5 controller screen only accepts a write
// per line every ~50 ms, so poll at a comfortable 250 ms.

static pros::Task* poseTelemetryTask = nullptr;

void startPoseTelemetry() {
    if (poseTelemetryTask != nullptr) return;   // already running
    poseTelemetryTask = new pros::Task([] {
        while (true) {
            Pose p = chassis.getPose();
            master.print(0, 0, "X%.1f Y%.1f H%.0f ",
                         p.x, p.y, stdRadToVexDeg(p.theta));
            pros::delay(250);
        }
    }, "PoseTelemetry");
}

// ============================================================================
// Driver control
// ============================================================================

void driverControl() {
    chassis.stopTracking();

    while (true) {
        if (chassis.isSettled()) {
            int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            chassis.arcade(leftY, rightX);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            chassis.turnToHeading(stdRadToVexDeg(chassis.getPose().theta) + 90.0f, 2000, { .maxSpeed = 80.0f }, true);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            chassis.moveDistance(24.0f, 2000, {}, true);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            chassis.cancelMotion();
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            static bool hold = true;
            hold = !hold;
            chassis.setBrakeMode(hold ? pros::E_MOTOR_BRAKE_HOLD : pros::E_MOTOR_BRAKE_COAST);
            master.rumble(".");
        }

        pros::delay(20);
    }
}

}  // namespace riku
