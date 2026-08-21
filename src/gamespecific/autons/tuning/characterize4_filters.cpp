#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include "chad/api.hpp"

namespace chad {

using namespace autons;

void autonCharacterize4Filters() {
    Report out;
    out("phase 4: sensor + pose noise");
    if (!headingOk(out)) return;
    out("stays parked. do not touch it");

    setFilters(false);
    auto ch = makeCharacterizer(out);
    const auto beams = ch.measureRangeNoise(distSensors);

    for (int i = 0; i < MAX_DISTANCE_SENSORS; ++i) {
        if (beams.samples[i] > 8) {
            out.result("beam %d %.2fin s=%.3f", i, beams.meanIn[i],
                       beams.sigmaIn[i]);
        }
    }
    if (beams.worstSigmaIn <= 0.0f) {
        out.result("no beam saw a wall. park");
        out.result("within ~70 in of two walls");
        return;
    }

    auto mcl = chassis.mcl().config();
    mcl.rangeSigmaCloseIn = beams.worstSigmaIn;
    chassis.mcl().setConfig(mcl);

    auto ekf = chassis.ekf().config();
    ekf.rangeSigmaBase = beams.worstSigmaIn;
    chassis.ekf().setConfig(ekf);

    const float th = chassis.getPose().theta;
    float px = 0.0f, py = 0.0f;
    int pinned = 0;
    if (!solvePark(beams, th, px, py, pinned)) {
        out.result("only %d of 2 axes pinned --", pinned);
        out.result("park in a corner and rerun");
        out.result("mcl.rangeSigmaCloseIn = %.4ff", beams.worstSigmaIn);
        out.result("ekf.rangeSigmaBase = %.4ff", beams.worstSigmaIn);
        return;
    }
    chassis.setPose({px, py, th});
    out.result("at %.1f,%.1f h=%.0f", px, py, stdRadToVexDeg(th));

    setFilters(true);
    pros::delay(800);
    const auto noise = ch.localizationNoise();

    out.result("copy to config.cpp:");
    out.result("mcl.rangeSigmaCloseIn = %.4ff", beams.worstSigmaIn);
    out.result("ekf.rangeSigmaBase = %.4ff", beams.worstSigmaIn);
    out.result("targetTolerance >= %.2ff", noise.suggestTolerance);
    out.result("bearingHoldDist = %.2ff", noise.suggestBearingHold);
    out.result("pose sigma %.2fin pk %.2fin", noise.sigmaIn, noise.peakIn);
}

}  // namespace chad
