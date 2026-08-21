#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include "riku/api.hpp"
#include "riku/util/autotune.hpp"

namespace riku {

using namespace autons;

void autonVerifyPid() {
    Report out;
    out("verify: steps on config gains");
    if (!headingOk(out)) return;
    setFilters(false);

    tuning::AutoTuneConfig acfg;
    acfg.reps = 1;
    acfg.trialMs = 2500.0f;
    tuning::PidAutoTuner t(chassis, acfg);

    for (const float deg : {15.0f, 45.0f, 90.0f, 180.0f}) {
        const StepMetrics m = t.measureAt(true, deg);
        if (!m.valid) { out.result("%.0fdeg: no trace", deg); continue; }
        out.result("%.0fdeg o=%.0f%% s=%.0fms e=%.1f", deg, m.overshootPct,
                   m.settleMs, m.steadyState);
    }
    for (const float in : {8.0f, 24.0f, 48.0f}) {
        const StepMetrics m = t.measureAt(false, in);
        if (!m.valid) { out.result("%.0fin: no trace", in); continue; }
        out.result("%.0fin o=%.0f%% s=%.0fms e=%.1f", in, m.overshootPct,
                   m.settleMs, m.steadyState);
    }
    out.result("o over 20% -> raise kD");
    out.result("e large and o low -> raise kP");
}

}  // namespace riku
