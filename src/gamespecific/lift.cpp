#include <atomic>
#include <cmath>

#include "gamespecific/lift.hpp"
#include "gamespecific/config.hpp"
#include "chad/util/exitcondition.hpp"
#include "chad/util/logger.hpp"
#include "chad/util/pid.hpp"
#include "pros/rtos.hpp"

using namespace chad;

static const double kP = 0.5;
static const double kI = 0.0;
static const double kD = 0.0;

static const bool   LIFT_IS_PIVOTED        = false;

static const double kG                     = 19.6;
static const double LIFT_KG_PER_MOTOR_DEG  = 0.0;

static const double MOTOR_DEG_PER_ARM_DEG  = 28.0;
static const double HORIZONTAL_MOTOR_DEG   = 1360.0;

static const double INTEGRAL_LIMIT = 50.0;
static const double SETTLE_ERROR   = 2.0;
static const double SETTLE_TIME    = 250;

static std::atomic<uint32_t> gLiftGen{0};
static std::atomic<bool> gLiftBusy{false};
static std::function<void(const LiftSample&)> gLiftTelemetry;

double liftPosition() {
    return liftMotor.get_position();
}

bool liftBusy() {
    return gLiftBusy.load();
}

void liftCancel() {
    ++gLiftGen;
    gLiftBusy = false;
    liftMotor.move(0);
}

void liftSetTelemetry(std::function<void(const LiftSample&)> callback) {
    gLiftTelemetry = std::move(callback);
}

static double gravityFeedforward(double motorPosition) {
    if (kG == 0.0 && LIFT_KG_PER_MOTOR_DEG == 0.0) return 0.0;
    if (!LIFT_IS_PIVOTED) {
        return kG + LIFT_KG_PER_MOTOR_DEG * motorPosition;
    }
    const double armRad =
        (motorPosition - HORIZONTAL_MOTOR_DEG) / MOTOR_DEG_PER_ARM_DEG * DEG_TO_RAD;
    return kG * std::cos(armRad);
}

static void liftLoop(double targetDegrees, double maxSpeed, double timeoutMs,
                     uint32_t gen) {
    PID pid(static_cast<float>(kP), static_cast<float>(kI),
            static_cast<float>(kD), static_cast<float>(INTEGRAL_LIMIT), false);

    pid.setDerivativeFilter(0.5f);
    pid.reset();

    ExitCondition settle(static_cast<float>(SETTLE_ERROR),
                         static_cast<int>(SETTLE_TIME));

    uint32_t startTime = pros::millis();

    while (gen == gLiftGen) {
        uint32_t now = pros::millis();
        if (now - startTime >= timeoutMs) break;

        double position = liftMotor.get_position();
        if (!std::isfinite(position)) {
            liftMotor.move(0);
            pros::delay(20);
            continue;
        }

        double error = targetDegrees - position;
        double pidOut = pid.update(static_cast<float>(error));
        double ff = gravityFeedforward(position);
        double power = pidOut + ff;

        if (power >  maxSpeed) power =  maxSpeed;
        if (power < -maxSpeed) power = -maxSpeed;
        liftMotor.move(static_cast<int>(std::lround(power)));

        if (gLiftTelemetry) {
            gLiftTelemetry({(float)targetDegrees, (float)position, (float)error,
                            (float)pidOut, (float)ff, (float)power});
        }

        if (settle.update(static_cast<float>(error))) break;

        pros::delay(20);
    }

    if (gen == gLiftGen) {
        double position = liftMotor.get_position();
        double hold = std::isfinite(position) ? gravityFeedforward(position) : 0.0;
        liftMotor.move(static_cast<int>(std::lround(hold)));
        gLiftBusy = false;
    }
}

void liftMoveTo(double targetDegrees, double maxSpeed, double timeoutMs, bool async) {
    uint32_t gen = ++gLiftGen;
    gLiftBusy = true;
    if (async) {
        pros::Task([=] { liftLoop(targetDegrees, maxSpeed, timeoutMs, gen); },
                   "LiftPID");
    } else {
        liftLoop(targetDegrees, maxSpeed, timeoutMs, gen);
    }
}
