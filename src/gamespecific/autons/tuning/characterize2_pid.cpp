#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include "chad/api.hpp"
#include "chad/util/autotune.hpp"
#include "chad/util/relay_tune.hpp"

namespace chad {

using namespace autons;

namespace {

void applyGains(bool angular, const PIDGains& g) {
    if (angular) chassis.setAngularGains(g);
    else chassis.setLateralGains(g);
}

PIDGains refineGains(tuning::PidAutoTuner& t, Report& out, bool angular,
                     PIDGains g, float amp, float ssTol) {
    const char* tag = angular ? "ang" : "lat";
    const float kp0 = g.kP;
    const float kd0 = g.kD > 0.0f ? g.kD : 0.05f * g.kP;

    for (int i = 0; i < 6; ++i) {
        applyGains(angular, g);
        const StepMetrics m = t.measureAt(angular, amp);
        if (!m.valid) {
            out("%s refine: no trace", tag);
            break;
        }
        const float tilt = t.lastPeakTilt();
        out("%s o=%.0f%% e=%.1f t=%.1f kP=%.0f", tag, m.overshootPct,
            m.steadyState, tilt, g.kP);

        if (!angular && tilt > kTiltLimitDeg) {
            if (g.kP > 0.4f * kp0) { g.kP *= 0.85f; continue; }
            out("lat: still tilting at the kP floor");
            break;
        }

        if (m.overshootPct > 15.0f) {
            if (g.kD < 5.0f * kd0) g.kD *= 1.35f;
            else break;
        } else if (m.steadyState > ssTol && m.overshootPct < 5.0f &&
                   g.kP < 2.0f * kp0) {
            g.kP *= 1.15f;
        } else {
            break;
        }
    }
    applyGains(angular, g);
    return g;
}

}  // namespace

void autonCharacterize2Pid() {
    Report out;
    out("phase 2: relay, then refine");
    if (!headingOk(out)) return;
    out("about a minute. no sweep");
    setFilters(false);

    tuning::RelayTuneConfig rcfg;
    rcfg.abortIn = 24.0f;
    Report* sink = &out;
    rcfg.onProgress = [sink](const char* stage, const char* detail) {
        (*sink)("[%s] %s", stage, detail);
    };
    tuning::RelayAutoTuner tuner(chassis, drivetrain, rcfg);

    const tuning::RelayFit ang = tuner.tuneAngular();
    const tuning::RelayFit lat = tuner.tuneLateral();

    if (!ang.valid || !lat.valid) {
        out.result("no limit cycle on %s",
                   !ang.valid ? "angular" : "lateral");
        out.result("raise amplitude, rerun");
        return;
    }

    out("relay done, refining");
    tuning::AutoTuneConfig acfg;
    acfg.reps = 1;
    acfg.trialMs = 2000.0f;
    acfg.tiltDeg = tiltProbe();
    tuning::PidAutoTuner refiner(chassis, acfg);

    const PIDGains angFinal =
        refineGains(refiner, out, true, {ang.kP, ang.kI, ang.kD}, 90.0f, 1.5f);
    const PIDGains latFinal =
        refineGains(refiner, out, false, {lat.kP, lat.kI, lat.kD}, 24.0f, 0.5f);

    out.result("APPLIED. copy into config.cpp:");
    out.result("angular {%.3ff,%.4ff,%.3ff}", angFinal.kP, angFinal.kI,
               angFinal.kD);
    out.result("lateral {%.3ff,%.4ff,%.3ff}", latFinal.kP, latFinal.kI,
               latFinal.kD);
    out.result("relay gave kP %.0f/%.0f", ang.kP, lat.kP);
}

}  // namespace chad
