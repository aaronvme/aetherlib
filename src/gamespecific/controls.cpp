#include "gamespecific/controls.hpp"
#include "gamespecific/config.hpp"
#include "gamespecific/lift.hpp"

#include <atomic>
#include <cmath>

#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace chad {
namespace {
constexpr double kLiftManualUp   = 100.0;
constexpr double kLiftManualDown = 80.0;    // gravity is already helping
constexpr double kLiftMacroSpeed = 100.0;
constexpr double kLiftMacroTimeoutMs = 2500.0;
}

static pros::Task* poseTelemetryTask = nullptr;
static std::atomic<bool> poseTelemetryEnabled{true};

void setPoseTelemetryEnabled(bool enabled) {
    poseTelemetryEnabled.store(enabled);
}

void startPoseTelemetry() {
    if (poseTelemetryTask != nullptr) return;
    poseTelemetryTask = new pros::Task([] {
        while (true) {
            if (!poseTelemetryEnabled.load()) {
                pros::delay(100);
                continue;
            }
            Pose p = chassis.getPose();
            master.print(0, 0, "X%.1f Y%.1f H%.1f ",
                         p.x, p.y, stdRadToVexDeg(p.theta));
            pros::delay(60);

            const double arm = liftPosition();
            const double height = liftHeight();
            const char* tag = liftBusy() ? "down" : "";
            if (!std::isfinite(arm)) {
                master.print(1, 0, "lift --         ");
            } else if (std::isfinite(height)) {
                master.print(1, 0, "lift %.1f\" %-4s ", height, tag);
            } else {
                master.print(1, 0, "lift %.0f %-4s  ", arm, tag);
            }
            pros::delay(250);
        }
    }, "PoseTelemetry");
}

void driverControl() {
    // Tracking is left however the caller set it. Stopping it here meant a
    // startTracking() in opcontrol was undone on the next line.
    liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    while (true) {
        if (chassis.isSettled()) {
            int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            chassis.arcade(leftY, rightX);
        }

        const bool upNow   = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        const bool downNow = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        if (upNow || downNow) {
            if (liftBusy()) liftCancel();
            liftMotor.move(static_cast<int>(upNow ? kLiftManualUp
                                                  : -kLiftManualDown));
        } else if (!liftBusy()) {
            liftMotor.move(static_cast<int>(std::lround(liftHoldPower())));
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            liftToPin(0, kLiftMacroTimeoutMs, kLiftMacroSpeed, true);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            chassis.cancelMotion();
            liftCancel();
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            claw.extend();
        } else {
            claw.retract();
        }

        pros::delay(20);
    }
}
}
