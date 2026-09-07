#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include "chad/api.hpp"
#include "chad/util/autotune.hpp"

namespace chad {
using namespace autons;

void autonVerifyPid() {
    Report out;
    out("verify: steps on config gains");
    if (!headingOk(out)) return;
    setFilters(false);

    tuning::AutoTuneConfig acfg;
    acfg.reps = 1;
    acfg.trialMs = 2500.0f;

    acfg.tiltDeg = tiltProbe();
    acfg.tiltAbortDeg = kTiltLimitDeg;
    tuning::PidAutoTuner t(chassis, acfg);

    for (const float deg : {15.0f, 45.0f, 90.0f, 180.0f}) {
        const StepMetrics m = t.measureAt(true, deg);
        if (t.lastTiltAborted()) {
            out.result("%.0fdeg TILT CUT %.1f", deg, t.lastPeakTilt());
            continue;
        }
        if (!m.valid) { out.result("%.0fdeg: no trace", deg); continue; }
        out.result("%.0fdeg o=%.0f%% s=%.0fms e=%.1f", deg, m.overshootPct,
                   m.settleMs, m.steadyState);
    }

    out("needs ~80 in of clear lane");
    for (const float in : {8.0f, 24.0f, 48.0f, 72.0f}) {
        const StepMetrics m = t.measureAt(false, in);
        if (t.lastTiltAborted()) {
            out.result("%.0fin TILT CUT %.1f", in, t.lastPeakTilt());
            continue;
        }
        if (!m.valid) { out.result("%.0fin: no trace", in); continue; }
        out.result("%.0fin o=%.0f%% s=%.0fms e=%.1f", in, m.overshootPct,
                   m.settleMs, m.steadyState);
    }
    out.result("o over 20% -> raise kD");
    out.result("e large and o low -> raise kP");
    out.result("o climbing with step size");
    out.result("-> kP too high for the lane");
}
}
