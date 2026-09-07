#include "gamespecific/controls.hpp"
#include "gamespecific/config.hpp"
#include "gamespecific/lift.hpp"

#include <cmath>

#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace chad {
namespace {
constexpr int kLiftPinCount = 6;
constexpr int kLiftPresetCount = kLiftPinCount + 1;

constexpr double kLiftBase   = 0.0;
constexpr double kLiftPerPin = 250.0;
constexpr double kLiftTrim[kLiftPresetCount] = {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
};

constexpr double liftPreset(int index) {
    return kLiftBase + kLiftPerPin * static_cast<double>(index) +
           kLiftTrim[index];
}

constexpr int kLiftCoarseStep = 3;

constexpr uint32_t kLiftTapMs = 140;
constexpr double kLiftNudgeDeg = 40.0;
constexpr double kLiftNudgeTimeoutMs = 800.0;

constexpr double kLiftManualUp    = 100.0;
constexpr double kLiftManualDown  =80.0;    // gravity is already helping
constexpr double kLiftMacroSpeed  = 100.0;
constexpr double kLiftMacroTimeoutMs = 2500.0;

int gLiftTarget = 0;

double gLiftCommanded = 0.0;
bool gLiftHasCommanded = false;

int nearestPreset(double motorDeg) {
    int best = 0;
    double bestGap = 1e18;
    for (int i = 0; i < kLiftPresetCount; ++i) {
        const double gap = std::fabs(liftPreset(i) - motorDeg);
        if (gap < bestGap) { bestGap = gap; best = i; }
    }
    return best;
}

void liftGoTo(int index) {
    if (index < 0) index = 0;
    if (index >= kLiftPresetCount) index = kLiftPresetCount - 1;
    gLiftTarget = index;
    gLiftCommanded = liftPreset(index);
    gLiftHasCommanded = true;
    liftMoveTo(gLiftCommanded, kLiftMacroSpeed, kLiftMacroTimeoutMs, true);
}

void liftNudge(double delta) {
    double from;
    if (liftBusy() && gLiftHasCommanded) {
        from = gLiftCommanded;
    } else {
        from = liftPosition();
        if (!std::isfinite(from)) return;
    }
    gLiftCommanded = from + delta;
    gLiftHasCommanded = true;
    liftMoveTo(gLiftCommanded, kLiftMacroSpeed, kLiftNudgeTimeoutMs, true);
}

void liftStep(int direction) {
    const double pos = liftPosition();
    if (!std::isfinite(pos)) return;
    liftGoTo(nearestPreset(pos) + direction);
}
}

static pros::Task* poseTelemetryTask = nullptr;

void startPoseTelemetry() {
    if (poseTelemetryTask != nullptr) return;
    poseTelemetryTask = new pros::Task([] {
        while (true) {
            Pose p = chassis.getPose();
            master.print(0, 0, "X%.1f Y%.1f H%.1f ",
                         p.x, p.y, stdRadToVexDeg(p.theta));
            pros::delay(60);

            const double arm = liftPosition();
            if (std::isfinite(arm)) {
                master.print(1, 0, "arm %.0f  p%d ", arm, gLiftTarget);
            } else {
                master.print(1, 0, "arm --      ");
            }
            pros::delay(250);
        }
    }, "PoseTelemetry");
}

void driverControl() {
    chassis.stopTracking();

    liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    bool upWas = false, downWas = false;
    uint32_t upSince = 0, downSince = 0;

    while (true) {
        if (chassis.isSettled()) {
            int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            chassis.arcade(leftY, rightX);
        }

        const uint32_t now = pros::millis();
        const bool upNow   = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        const bool downNow = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        if (upNow && !upWas) upSince = now;
        if (downNow && !downWas) downSince = now;

        const bool upHeld   = upNow && (now - upSince >= kLiftTapMs);
        const bool downHeld = downNow && (now - downSince >= kLiftTapMs);

        if (upHeld || downHeld) {
            if (liftBusy()) liftCancel();

            gLiftHasCommanded = false;
            liftMotor.move(static_cast<int>(upHeld ? kLiftManualUp
                                                   : -kLiftManualDown));
        } else if (!upNow && !downNow && !liftBusy()) {
            liftMotor.move(0);
        }

        if (!upNow && upWas && now - upSince < kLiftTapMs) {
            liftNudge(+kLiftNudgeDeg);
        }
        if (!downNow && downWas && now - downSince < kLiftTapMs) {
            liftNudge(-kLiftNudgeDeg);
        }
        upWas = upNow;
        downWas = downNow;

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            liftStep(+1);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            liftStep(-1);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            liftGoTo(kLiftPresetCount - 1);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            liftGoTo(0);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            liftStep(+kLiftCoarseStep);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            liftStep(-kLiftCoarseStep);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            chassis.turnToHeading(stdRadToVexDeg(chassis.getPose().theta) + 90.0f, 2000, { .maxSpeed = 80.0f }, true);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            chassis.moveDistance(24.0f, 2000, {}, true);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            chassis.cancelMotion();
            liftCancel();
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
}
