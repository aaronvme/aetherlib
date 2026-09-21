#include <atomic>
#include <cmath>

#include "gamespecific/lift.hpp"
#include "gamespecific/config.hpp"
#include "chad/util/exitcondition.hpp"
#include "chad/util/logger.hpp"
#include "chad/util/pid.hpp"
#include "pros/rtos.hpp"

using namespace chad;

static const double kP = 3.0;//old 5.0
static const double kI = 0.0;//old 0.0
static const double kD = 0.02;//old 0.02

static const LiftKind LIFT_KIND            = LiftKind::Cascade;

static const double kG                     = 0.0; // 15.5
static const double LIFT_KG_PER_MOTOR_DEG  = 0.0;

static const double MOTOR_DEG_PER_ARM_DEG  = 28.0;
static const double HORIZONTAL_MOTOR_DEG   = 1360.0;

// cascade geometry -- height stays NaN until the spool is measured
static const double LIFT_GEAR_RATIO        = 1.0;   // motor rev per spool rev
static const double SPOOL_DIAMETER_IN      = 0.0;   // over the string
static const double LIFT_STAGES            = 1.0;   // carriage in per spool in
static const double LIFT_BOTTOM_MOTOR_DEG  = 0.0;
static const double LIFT_BOTTOM_HEIGHT_IN  = 0.0;

// pivoted geometry
static const double ARM_LENGTH_IN          = 0.0;   // pivot to the carry point
static const double PIVOT_HEIGHT_IN        = 0.0;

// pin heights in motor degrees -- from autonTuneLiftPresets
static const double LIFT_PIN_BASE    = 0.0;
static const double LIFT_PIN_PER_PIN = 156.0;
static const double LIFT_PIN_TRIM[kLiftPresetCount] = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

static const double INTEGRAL_LIMIT = 50.0;
static const double SETTLE_ERROR   = 2.0;
static const double SETTLE_TIME    = 250;

static std::atomic<uint32_t> gLiftGen{0};
static std::atomic<bool> gLiftBusy{false};
static std::function<void(const LiftSample&)> gLiftTelemetry;

double liftPosition() {
    return liftMotor.get_position();
}

double liftHeightAt(double motorDegrees) {
    if (!std::isfinite(motorDegrees)) return NAN;
    if (LIFT_KIND == LiftKind::Pivoted) {
        if (!(ARM_LENGTH_IN > 0.0) || MOTOR_DEG_PER_ARM_DEG == 0.0) return NAN;
        const double armRad = (motorDegrees - HORIZONTAL_MOTOR_DEG) /
                              MOTOR_DEG_PER_ARM_DEG * DEG_TO_RAD;
        return PIVOT_HEIGHT_IN + ARM_LENGTH_IN * std::sin(armRad);
    }
    if (!(SPOOL_DIAMETER_IN > 0.0) || !(LIFT_GEAR_RATIO > 0.0) ||
        !(LIFT_STAGES > 0.0)) return NAN;
    const double spoolRev =
        (motorDegrees - LIFT_BOTTOM_MOTOR_DEG) / 360.0 / LIFT_GEAR_RATIO;
    return LIFT_BOTTOM_HEIGHT_IN + spoolRev * PI * SPOOL_DIAMETER_IN * LIFT_STAGES;
}

double liftHeight() {
    return liftHeightAt(liftPosition());
}

double liftPinDegrees(int pin) {
    if (pin < 0) pin = 0;
    if (pin >= kLiftPresetCount) pin = kLiftPresetCount - 1;
    return LIFT_PIN_BASE + LIFT_PIN_PER_PIN * static_cast<double>(pin) +
           LIFT_PIN_TRIM[pin];
}

int liftNearestPin(double motorDegrees) {
    if (!std::isfinite(motorDegrees)) return 0;
    int best = 0;
    double bestGap = 1e18;
    for (int i = 0; i < kLiftPresetCount; ++i) {
        const double gap = std::fabs(liftPinDegrees(i) - motorDegrees);
        if (gap < bestGap) { bestGap = gap; best = i; }
    }
    return best;
}

bool liftBusy() {
    return gLiftBusy.load();
}

bool liftWaitUntilSettled(double timeoutMs) {
    const uint32_t start = pros::millis();
    const uint32_t limit = static_cast<uint32_t>(timeoutMs);
    while (gLiftBusy.load()) {
        if (pros::millis() - start >= limit) return false;
        pros::delay(10);
    }
    return true;
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
    if (LIFT_KIND != LiftKind::Pivoted) {
        return kG + LIFT_KG_PER_MOTOR_DEG * motorPosition;
    }
    if (MOTOR_DEG_PER_ARM_DEG == 0.0) return kG;
    const double armRad =
        (motorPosition - HORIZONTAL_MOTOR_DEG) / MOTOR_DEG_PER_ARM_DEG * DEG_TO_RAD;
    return kG * std::cos(armRad);
}

double liftHoldPower() {
    const double position = liftPosition();
    if (!std::isfinite(position)) return 0.0;
    const double hold = gravityFeedforward(position);
    return std::isfinite(hold) ? hold : 0.0;
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
        if (!std::isfinite(power)) power = 0.0;

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

void liftToPin(int pin, double timeoutMs, double maxSpeed, bool async) {
    liftMoveTo(liftPinDegrees(pin), maxSpeed, timeoutMs, async);
}

void liftToPinAbove(int pin, double extraDegrees, double timeoutMs,
                    double maxSpeed, bool async) {
    if (!(extraDegrees > 0.0)) extraDegrees = kLiftPinClearanceDeg;
    const double up = (LIFT_PIN_PER_PIN < 0.0) ? -1.0 : 1.0;
    liftMoveTo(liftPinDegrees(pin) + up * extraDegrees, maxSpeed, timeoutMs,
               async);
}
