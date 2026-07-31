#include <atomic>
#include <cmath>

#include "GameSpecific/lift.hpp"
#include "GameSpecific/robot_config.hpp"
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

    uint32_t startTime    = pros::millis();
    uint32_t settledSince = 0;
    bool     isSettled    = false;

    while (gen == gLiftGen) { 
        uint32_t now = pros::millis();
        if (now - startTime >= timeoutMs) break;

        double error = targetDegrees - armMotor.get_position();
        double power = pid.update(static_cast<float>(error));

        if (power >  maxSpeed) power =  maxSpeed;
        if (power < -maxSpeed) power = -maxSpeed;
        armMotor.move(static_cast<int>(power));

        if (std::fabs(error) <= SETTLE_ERROR) {
            if (!isSettled) { settledSince = now; isSettled = true; }
            else if (now - settledSince >= SETTLE_TIME) break;
        } else {
            isSettled = false;
        }

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
