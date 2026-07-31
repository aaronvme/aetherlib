#include <atomic>
#include <cmath>

#include "GameSpecific/lift.hpp"
#include "GameSpecific/robot_config.hpp"
#include "AaronLib/Util/exitcondition.hpp"
#include "AaronLib/Util/pid.hpp"
#include "pros/rtos.hpp"

using namespace AARONLIB;

// ── arm gains ────────────────────────────────────────────────
// tune values =============================================================================
static const double kP = 0.5;
static const double kI = 0.0;
static const double kD = 0.0;

// ── arm limits ───────────────────────────────────────────────
static const double INTEGRAL_LIMIT = 50.0;
static const double SETTLE_ERROR   = 2.0;
static const double SETTLE_TIME    = 250;

static std::atomic<uint32_t> gLiftGen{0};

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
        double power = pid.update(static_cast<float>(error));

        if (power >  maxSpeed) power =  maxSpeed;
        if (power < -maxSpeed) power = -maxSpeed;
        armMotor.move(static_cast<int>(power));

        if (settle.update(static_cast<float>(error))) break;

        pros::delay(20);
    }

   
    if (gen == gLiftGen) armMotor.move(0);
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
