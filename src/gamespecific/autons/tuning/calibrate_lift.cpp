#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include <cmath>

#include "chad/api.hpp"
#include "chad/util/lift_fit.hpp"

namespace chad {
using namespace autons;

namespace {
constexpr int kBins = 20;
constexpr double kSweepRpm = 25.0;

constexpr double kSweepRpm2 = 50.0;
constexpr uint32_t kSampleMs = 20;
constexpr uint32_t kSweepTimeoutMs = 12000;
constexpr uint32_t kTransientMs = 350;
constexpr float kMinSpanArmDeg = 25.0f;

constexpr float kKnownRatio = 0.0f;
constexpr float kKnownRatioMinSpanArmDeg = 15.0f;

float unitsFromMv(double mv) {
    return static_cast<float>(mv) / 12000.0f * 127.0f;
}

double liftPosition() {
    return liftMotor.get_position();
}

bool sweep(double velRpm, float sign, double lowLocal, double highLocal,
           double until, bool up, tuning::LiftSweepBin* bins, double base) {
    const double span = highLocal - lowLocal;
    if (span <= 0.0) return false;

    liftMotor.move_velocity(velRpm);
    const uint32_t t0 = pros::millis();
    bool reached = false;

    while (pros::millis() - t0 < kSweepTimeoutMs) {
        const double raw = liftPosition();
        if (!std::isfinite(raw)) break;
        const double local = sign * (raw - base);

        if (up ? (local >= until) : (local <= until)) { reached = true; break; }

        if (pros::millis() - t0 >= kTransientMs && local >= lowLocal &&
            local <= highLocal) {
            int idx = static_cast<int>((local - lowLocal) / span * kBins);
            if (idx < 0) idx = 0;
            if (idx >= kBins) idx = kBins - 1;
            const float power = sign * unitsFromMv(liftMotor.get_voltage());
            if (std::isfinite(power)) {
                if (up) { bins[idx].sumUp += power; bins[idx].nUp++; }
                else    { bins[idx].sumDown += power; bins[idx].nDown++; }
            }
        }
        pros::delay(kSampleMs);
    }

    liftMotor.move_velocity(0);
    return reached;
}

tuning::LiftGravityFit searchRatio(const tuning::LiftHoldSample* held, int n,
                                   double motorSpan, float& ratioOut,
                                   float& ratioLoOut, float& ratioHiOut) {
    tuning::LiftGravityFit best;
    ratioOut = 0.0f;
    ratioLoOut = 0.0f;
    ratioHiOut = 0.0f;

    const float rMax = static_cast<float>(motorSpan) / kMinSpanArmDeg;
    if (rMax < 1.0f) return best;

    for (float r = 1.0f; r <= rMax; r += 0.05f) {
        const auto f = tuning::fitLiftGravity(held, n, r, kMinSpanArmDeg);
        if (!f.valid || f.kG <= 0.5f) continue;
        if (!best.valid || f.rms < best.rms) {
            best = f;
            ratioOut = r;
        }
    }
    if (!best.valid) return best;

    const float band = best.rms * 2.0f + 0.25f;
    ratioLoOut = ratioOut;
    ratioHiOut = ratioOut;
    for (float r = 1.0f; r <= rMax; r += 0.05f) {
        const auto f = tuning::fitLiftGravity(held, n, r, kMinSpanArmDeg);
        if (!f.valid || f.kG <= 0.5f || f.rms > band) continue;
        if (r < ratioLoOut) ratioLoOut = r;
        if (r > ratioHiOut) ratioHiOut = r;
    }
    return best;
}

bool sweepPair(double rpm, float sign, double lowLocal, double highLocal,
               tuning::LiftSweepBin* bins, double base) {
    const double upRpm = sign * rpm;
    sweep(-upRpm, sign, lowLocal, highLocal, lowLocal, false, bins, base);
    pros::delay(300);
    const bool gotUp =
        sweep(upRpm, sign, lowLocal, highLocal, highLocal, true, bins, base);
    pros::delay(300);
    const bool gotDown =
        sweep(-upRpm, sign, lowLocal, highLocal, lowLocal, false, bins, base);
    liftMotor.move(0);
    return gotUp && gotDown;
}
}

void autonCalibrateLift() {
    Report out;
    out("lift: kG, horizontal, friction");

    if (!std::isfinite(liftPosition())) {
        out.result("lift motor not reporting");
        out.result("check ports 14/15/1/2");
        return;
    }

    liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    waitForA(out, "move arm to its LOW stop");
    const double loRaw = liftPosition();
    waitForA(out, "now to its HIGH stop");
    const double hiRaw = liftPosition();

    const double travel = std::fabs(hiRaw - loRaw);
    if (!std::isfinite(travel) || travel < 60.0) {
        out.result("only %.0f motor deg of travel", travel);
        out.result("need 60+. is it the arm?");
        return;
    }

    const float sign = (hiRaw >= loRaw) ? 1.0f : -1.0f;
    const double margin = travel * 0.1;
    const double lowLocal = margin;
    const double highLocal = travel - margin;
    const double span = highLocal - lowLocal;

    tuning::LiftSweepBin bins[kBins];
    tuning::LiftSweepBin bins2[kBins];
    for (int i = 0; i < kBins; ++i) {
        bins[i].motorDeg = static_cast<float>(
            lowLocal + span * (static_cast<double>(i) + 0.5) / kBins);
        bins2[i].motorDeg = bins[i].motorDeg;
    }

    if (!sweepPair(kSweepRpm, sign, lowLocal, highLocal, bins, loRaw)) {
        out.result("sweep stalled at %.0f rpm", kSweepRpm);
        out.result("clear it, or raise kSweepRpm");
        return;
    }
    if (!sweepPair(kSweepRpm2, sign, lowLocal, highLocal, bins2, loRaw)) {
        out.result("sweep stalled at %.0f rpm", kSweepRpm2);
        return;
    }

    tuning::LiftHoldSample held[kBins];
    tuning::LiftHoldSample held2[kBins];
    float gap1 = 0.0f, gap2 = 0.0f;
    const int n = tuning::combineLiftSweeps(bins, kBins, held, &gap1);
    const int n2 = tuning::combineLiftSweeps(bins2, kBins, held2, &gap2);
    if (n < 3 || n2 < 3) {
        out.result("only %d/%d usable bins", n, n2);
        out.result("both sweeps must cover it");
        return;
    }

    const auto flat2 = tuning::fitLiftConstant(held2, n2);
    const auto split = tuning::splitFrictionAndEmf(
        gap1, static_cast<float>(kSweepRpm), gap2,
        static_cast<float>(kSweepRpm2));
    const float friction = split.valid ? split.frictionUnits : gap1;

    const auto flat = tuning::fitLiftConstant(held, n);

    float ratio = kKnownRatio, ratioLo = 0.0f, ratioHi = 0.0f;
    tuning::LiftGravityFit pivoted;
    if (kKnownRatio > 0.0f) {
        pivoted = tuning::fitLiftGravity(held, n, kKnownRatio,
                                         kKnownRatioMinSpanArmDeg);
    } else {
        pivoted = searchRatio(held, n, span, ratio, ratioLo, ratioHi);
    }

    if (!flat.valid && !pivoted.valid) {
        out.result("no fit (%d bins)", n);
        return;
    }

    const bool sloped = flat.valid && flat.rmsConstant > 0.0f &&
                        flat.rms < 0.8f * flat.rmsConstant;
    const float flatRms = sloped ? flat.rms : flat.rmsConstant;
    const bool preferPivot = pivoted.valid && flat.valid &&
                             pivoted.rms < 0.8f * flatRms;

    out.result("copy into lift.cpp:");
    if (preferPivot) {
        const float horizontalRaw =
            static_cast<float>(loRaw) + sign * pivoted.horizontalMotorDeg;
        out.result("LIFT_IS_PIVOTED = true");
        out.result("kG = %.1f", pivoted.kG);
        out.result("MOTOR_DEG_PER_ARM_DEG=%.2f", sign * ratio);
        out.result("HORIZONTAL_MOTOR_DEG=%.0f", horizontalRaw);
        out.result("rms %.2f vs flat %.2f", pivoted.rms, flatRms);
        if (kKnownRatio <= 0.0f && ratioHi > ratioLo * 1.35f) {
            out.result("RATIO WEAK: %.1f-%.1f", ratioLo, ratioHi);
            out.result("set kKnownRatio and rerun");
        }
    } else {
        out.result("LIFT_IS_PIVOTED = false");
        if (sloped) {
            const float perRaw = sign * flat.perMotorDeg;
            const float baseRaw =
                flat.base - perRaw * static_cast<float>(loRaw);
            out.result("kG = %.1f", baseRaw);
            out.result("LIFT_KG_PER_MOTOR_DEG=%.5f", perRaw);
            out.result("sloped: rms %.2f vs %.2f", flat.rms,
                       flat.rmsConstant);
        } else {
            out.result("kG = %.1f", flat.meanPower);
            out.result("LIFT_KG_PER_MOTOR_DEG = 0.0");
            out.result("flat rms %.2f", flat.rmsConstant);
        }
        if (pivoted.valid) {
            out.result("(cosine fit was worse: %.2f)", pivoted.rms);
        }
    }
    if (split.valid) {
        out.result("friction %.1f (gap %.1f)", friction, gap1);
        out.result("emf %.2f /rpm  n=%d", split.emfPerRpm, n);
    } else {
        out.result("friction <= %.1f, emf unsplit", gap1);
        out.result("gaps did not rise with speed");
    }

    const float drift = std::fabs(flat.meanPower - flat2.meanPower);
    if (flat2.valid && drift > 0.15f * flat.meanPower) {
        out.result("GRAVITY DRIFTED %.1f vs %.1f", flat.meanPower,
                   flat2.meanPower);
        out.result("string reseating? rerun");
    }
    if (friction > flat.meanPower) {
        out.result("friction > hold: it self-holds");
    }
}
}
