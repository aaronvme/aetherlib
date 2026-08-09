#include <atomic>
#include <cmath>

#include "gamespecific/lift.hpp"
#include "gamespecific/robot_config.hpp"
#include "riku/util/exitcondition.hpp"
#include "riku/util/logger.hpp"
#include "riku/util/pid.hpp"
#include "pros/rtos.hpp"

using namespace riku;

// ── arm gains ────────────────────────────────────────────────
// tune values =============================================================================
static const double kP = 0.5;
static const double kI = 0.0;
static const double kD = 0.0;

// ── gravity feedforward ──────────────────────────────────────
// Holding power, in motor units, needed with the arm horizontal. Applied as
// kG * cos(angle from horizontal), so it tapers to 0 when the arm is vertical.
//
// kG = 0 disables it and reproduces the plain-PID behaviour exactly.
//
// To tune: hold the arm level, raise kG until it neither sags nor climbs; then
// set HORIZONTAL_MOTOR_DEG to the motor reading at that level position, and
// MOTOR_DEG_PER_ARM_DEG to the gear reduction between motor and arm.
static const double kG                     = 0.0;
static const double MOTOR_DEG_PER_ARM_DEG  = 1.0;
static const double HORIZONTAL_MOTOR_DEG   = 0.0;

// ── arm limits ───────────────────────────────────────────────
static const double INTEGRAL_LIMIT = 50.0;
static const double SETTLE_ERROR   = 2.0;
static const double SETTLE_TIME    = 250;

static std::atomic<uint32_t> gLiftGen{0};
static std::function<void(const LiftSample&)> gLiftTelemetry;

void liftSetTelemetry(std::function<void(const LiftSample&)> callback) {
    gLiftTelemetry = std::move(callback);
}

static double gravityFeedforward(double motorPosition) {
    if (kG == 0.0) return 0.0;
    const double armRad =
        (motorPosition - HORIZONTAL_MOTOR_DEG) / MOTOR_DEG_PER_ARM_DEG * DEG_TO_RAD;
    return kG * std::cos(armRad);
}

// ── loop────
// ==========================================
static void liftLoop(double targetDegrees, double maxSpeed, double timeoutMs,
                     uint32_t gen) {
    PID pid(static_cast<float>(kP), static_cast<float>(kI),
            static_cast<float>(kD), static_cast<float>(INTEGRAL_LIMIT), false);
    pid.reset();

    ExitCondition settle(static_cast<float>(SETTLE_ERROR),
                         static_cast<int>(SETTLE_TIME));


    uint32_t startTime = pros::millis();

    while (gen == gLiftGen) {
        uint32_t now = pros::millis();
        if (now - startTime >= timeoutMs) break;

        // A disconnected motor reports PROS_ERR_F (== INFINITY). Feeding that
        // to the PID makes power infinite, and casting infinity to int is UB.
        double position = armMotor.get_position();
        if (!std::isfinite(position)) {
            armMotor.move(0);
            pros::delay(20);
            continue;
        }

        double error = targetDegrees - position;
        double pidOut = pid.update(static_cast<float>(error));
        double ff = gravityFeedforward(position);
        double power = pidOut + ff;

        if (power >  maxSpeed) power =  maxSpeed;
        if (power < -maxSpeed) power = -maxSpeed;
        armMotor.move(static_cast<int>(power));

        if (gLiftTelemetry) {
            gLiftTelemetry({(float)targetDegrees, (float)position, (float)error,
                            (float)pidOut, (float)ff, (float)power});
        }

        if (settle.update(static_cast<float>(error))) break;

        pros::delay(20);
    }

    // Hold against gravity rather than dropping to 0. Identical to the old
    // move(0) while kG is 0.
    if (gen == gLiftGen) {
        double position = armMotor.get_position();
        double hold = std::isfinite(position) ? gravityFeedforward(position) : 0.0;
        armMotor.move(static_cast<int>(hold));
    }
}

void liftMoveTo(double targetDegrees, double maxSpeed, double timeoutMs, bool async) {
    uint32_t gen = ++gLiftGen;
    if (async) {
        pros::Task([=] { liftLoop(targetDegrees, maxSpeed, timeoutMs, gen); },
                   "LiftPID");
    } else {
        liftLoop(targetDegrees, maxSpeed, timeoutMs, gen);
    }
}
