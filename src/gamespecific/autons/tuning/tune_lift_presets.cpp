#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include <cmath>
#include <cstdio>

#include "chad/api.hpp"
#include "gamespecific/lift.hpp"

namespace chad {
using namespace autons;

namespace {
constexpr int kPresets = kLiftPresetCount;

constexpr int kJogUp = 60;
constexpr int kJogDown = 45;
constexpr uint32_t kCaptureTimeoutMs = 120000;
constexpr uint32_t kPrintMs = 120;

void holdOrJog() {
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        liftMotor.move(kJogUp);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        liftMotor.move(-kJogDown);
    } else {
        liftMotor.move(static_cast<int>(std::lround(liftHoldPower())));
    }
}

void waitRelease() {
    while (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        holdOrJog();
        pros::delay(20);
    }
}

bool capture(Report& out, int index, double& degOut) {
    out("preset %d: R1/R2 move, A sets", index);
    waitRelease();
    master.rumble("-");

    const uint32_t t0 = pros::millis();
    uint32_t lastPrint = 0;
    while (pros::millis() - t0 < kCaptureTimeoutMs) {
        holdOrJog();
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) break;

        const uint32_t now = pros::millis();
        if (now - lastPrint >= kPrintMs) {
            lastPrint = now;
            ctlLine(0, "p%d %.0f A=set", index, ::liftPosition());
        }
        pros::delay(20);
    }

    degOut = ::liftPosition();
    waitRelease();
    if (!std::isfinite(degOut)) return false;
    out("  p%d = %.0f", index, degOut);
    return true;
}
}

void autonTuneLiftPresets() {
    Report out;
    out("lift presets: set %d stops", kPresets);

    if (!std::isfinite(::liftPosition())) {
        out.result("lift motor not reporting");
        out.result("check the lift ports");
        return;
    }

    liftMotor.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

    double deg[kPresets];
    for (int i = 0; i < kPresets; ++i) {
        if (!capture(out, i, deg[i])) {
            liftMotor.move(0);
            out.result("lost the encoder at p%d", i);
            return;
        }
    }
    liftMotor.move(static_cast<int>(std::lround(liftHoldPower())));

    double sumI = 0.0, sumD = 0.0;
    for (int i = 0; i < kPresets; ++i) {
        sumI += static_cast<double>(i);
        sumD += deg[i];
    }
    const double meanI = sumI / kPresets;
    const double meanD = sumD / kPresets;

    double num = 0.0, den = 0.0;
    for (int i = 0; i < kPresets; ++i) {
        const double di = static_cast<double>(i) - meanI;
        num += di * (deg[i] - meanD);
        den += di * di;
    }
    const double perPin = (den > 0.0) ? num / den : 0.0;
    const double base = meanD - perPin * meanI;

    double trim[kPresets];
    double worst = 0.0;
    for (int i = 0; i < kPresets; ++i) {
        trim[i] = deg[i] - (base + perPin * static_cast<double>(i));
        if (std::fabs(trim[i]) > std::fabs(worst)) worst = trim[i];
    }

    bool ordered = true;
    for (int i = 1; i < kPresets; ++i) {
        if ((deg[i] - deg[i - 1]) * perPin <= 0.0) ordered = false;
    }

    out.result("copy into lift.cpp:");
    out.result("LIFT_PIN_BASE    = %.0f", base);
    out.result("LIFT_PIN_PER_PIN = %.0f", perPin);
    for (int i = 0; i < kPresets; i += 4) {
        const int last = (i + 3 < kPresets - 1) ? i + 3 : kPresets - 1;
        char line[80];
        int n = std::snprintf(line, sizeof(line), "trim %d-%d", i, last);
        for (int j = i; j <= last; ++j) {
            if (n < 0 || n >= static_cast<int>(sizeof(line))) break;
            const int w = std::snprintf(line + n, sizeof(line) - n, " %.0f",
                                        trim[j]);
            if (w < 0) break;
            n += w;
        }
        out.result("%s", line);
    }
    out.result("worst trim %.0f deg", worst);

    const double lowIn = ::liftHeightAt(deg[0]);
    const double highIn = ::liftHeightAt(deg[kPresets - 1]);
    if (std::isfinite(lowIn) && std::isfinite(highIn)) {
        out.result("p0-p%d span %.1f in", kPresets - 1,
                   std::fabs(highIn - lowIn));
    }
    if (!ordered) {
        out.result("STOPS OUT OF ORDER - rerun");
    }
}
}
