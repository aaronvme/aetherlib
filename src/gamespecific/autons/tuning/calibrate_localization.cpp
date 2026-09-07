#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "chad/api.hpp"
#include "chad/util/localization_fit.hpp"

namespace chad {
using namespace autons;

namespace {
struct ResampleCandidate {
    float distVar;
    float thetaVar;
};
constexpr ResampleCandidate kCandidates[] = {
    {0.75f, 0.005f},
    {1.50f, 0.010f},
    {3.00f, 0.020f},
};
constexpr int kSettings = 3;
constexpr int kTrialsPerSetting = 5;

constexpr int kSamplesPerPlacement = 40;
constexpr float kSampleMs = 15.0f;
constexpr float kConvergedIn = 2.0f;
constexpr float kHoldMs = 200.0f;
constexpr float kRecoveryTimeoutMs = 5000.0f;
constexpr float kBracketSigmas = 3.0f;

int waitForChoice(Report& out, const char* why, const char* bMeans) {
    out.result("%s", why);
    out.result("A = record, B = %s", bMeans);
    master.set_text(0, 0, "place it, press A ");
    pros::delay(60);
    master.rumble("-");
    int picked = 0;
    const uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < 180000u) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) { picked = 1; break; }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) { picked = 2; break; }
        pros::delay(20);
    }
    while (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) ||
           master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        pros::delay(20);
    }
    master.set_text(0, 0, "running-hands off ");
    return picked;
}
}

void autonCalibrateLocalization() {
    Report out;
    out("phase 5: localization");
    if (!headingOk(out)) return;
    out("hand placed, never drives");
    if (kFieldIsClear) {
        clearFieldObstacles();
        out("clear field: anywhere square-ish");
    } else {
        out("square-ish, 2 walls in view");
    }

    const float span = 2.0f * Field::FIELD_HALF_WALL;

    std::vector<float> resid;
    std::vector<float> repSigma, absErr;
    std::vector<tuning::RecoveryTrial> trials[kSettings];
    int placements = 0, pairsSeen = 0, noPair = 0, blocked = 0;

    const int total = kSettings * kTrialsPerSetting;
    for (int idx = 0; idx < total; ++idx) {
        const int setting = idx / kTrialsPerSetting;
        out("place %d/%d  set %d", idx + 1, total, setting + 1);
        if (waitForChoice(out, "put it somewhere new", "stop") != 1) break;

        setFilters(false);
        const float theta = chassis.getPose().theta;
        BeamRead beams[MAX_DISTANCE_SENSORS];
        readBeams(beams, theta);
        BeamPair pairs[2];
        const int nPairs = findBeamPairs(beams, pairs);

        if (nPairs == 0 && !kFieldIsClear) {
            noPair++;
            out("no opposing pair -- skipped");
            continue;
        }
        if (nPairs == 0) noPair++;
        pairsSeen += nPairs;

        for (int k = 0; k < kSamplesPerPlacement; ++k) {
            BeamRead now[MAX_DISTANCE_SENSORS];
            readBeams(now, theta);
            for (int q = 0; q < nPairs; ++q) {
                const BeamPair& p = pairs[q];
                if (!now[p.plus].valid || !now[p.minus].valid) continue;
                const float oPlus =
                    p.xAxis ? now[p.plus].alongX : now[p.plus].alongY;
                const float oMinus =
                    p.xAxis ? now[p.minus].alongX : now[p.minus].alongY;
                const float dPlus =
                    p.xAxis ? now[p.plus].bx : now[p.plus].by;
                const float dMinus =
                    p.xAxis ? now[p.minus].bx : now[p.minus].by;
                const float expect = tuning::opposingPairSum(span, oPlus, oMinus);

                const float got = now[p.plus].inches * dPlus -
                                  now[p.minus].inches * dMinus;
                const float sp = now[p.plus].sigmaIn * dPlus;
                const float sm = now[p.minus].sigmaIn * dMinus;
                const float sPair = std::sqrt(sp * sp + sm * sm);
                if (sPair <= 1e-4f) continue;

                resid.push_back((got - expect) / sPair);
            }
            pros::delay(static_cast<uint32_t>(kSampleMs));
        }

        float tx = 0.0f, ty = 0.0f, bracket = 0.0f;
        if (!solveTruth(beams, pairs, nPairs, kBracketSigmas, tx, ty,
                        bracket)) {
            out("blocked or 1 axis (%.1f in)", bracket);
            blocked++;
            placements++;
            continue;
        }
        placements++;

        auto cfg = chassis.mcl().config();
        cfg.distResampleVariance = kCandidates[setting].distVar;
        cfg.thetaResampleVariance = kCandidates[setting].thetaVar;
        chassis.mcl().setConfig(cfg);
        setFilters(true);

        tuning::RecoveryTrial trial;
        const uint32_t t0 = pros::millis();
        float heldMs = 0.0f;
        float lastErr = 0.0f;
        while (pros::millis() - t0 < static_cast<uint32_t>(kRecoveryTimeoutMs)) {
            const auto est = chassis.mcl().estimate();
            const float ex = est.pose.x - tx;
            const float ey = est.pose.y - ty;
            lastErr = std::sqrt(ex * ex + ey * ey);

            if (est.covXX > 0.0f) {
                repSigma.push_back(std::sqrt(est.covXX));
                absErr.push_back(std::fabs(ex));
            }
            if (est.covYY > 0.0f) {
                repSigma.push_back(std::sqrt(est.covYY));
                absErr.push_back(std::fabs(ey));
            }

            if (lastErr < kConvergedIn) {
                heldMs += kSampleMs;
                if (heldMs >= kHoldMs) {
                    trial.converged = true;
                    trial.settleMs =
                        static_cast<float>(pros::millis() - t0) - kHoldMs;
                    break;
                }
            } else {
                heldMs = 0.0f;
            }
            pros::delay(static_cast<uint32_t>(kSampleMs));
        }
        trial.finalErrorIn = lastErr;
        trials[setting].push_back(trial);
        out("  %s %.0fms err %.1f", trial.converged ? "ok" : "NO",
            trial.settleMs, trial.finalErrorIn);
        setFilters(false);
    }

    const auto fit = tuning::fitBeamMixture(resid.data(),
                                            static_cast<int>(resid.size()));
    const auto cal = tuning::calibrateMclSigma(repSigma.data(), absErr.data(),
                                               static_cast<int>(repSigma.size()));

    int best = -1;
    tuning::RecoveryScore bestScore;
    for (int s = 0; s < kSettings; ++s) {
        const auto sc = tuning::scoreRecovery(trials[s].data(),
                                             static_cast<int>(trials[s].size()));
        if (tuning::recoveryBetterThan(sc, bestScore)) { bestScore = sc; best = s; }
    }

    if (!fit.valid && !kFieldIsClear) {
        out.result("beam fit needs 12+ samples,");
        out.result("got %d over %d placements", static_cast<int>(resid.size()),
                   placements);
        if (noPair > 0) {
            out.result("%d placements saw no pair:", noPair);
            out.result("keep it square to 2 walls");
        }
        return;
    }

    auto mcl = chassis.mcl().config();
    if (!kFieldIsClear) {
        const auto perBeam = tuning::perBeamFromPair(fit.params);
        mcl.shortWeight = perBeam.shortWeight;
        mcl.shortDecaySigmas = perBeam.shortDecaySigmas;
        mcl.randWeight = perBeam.randWeight;
    }
    if (best >= 0) {
        mcl.distResampleVariance = kCandidates[best].distVar;
        mcl.thetaResampleVariance = kCandidates[best].thetaVar;
    }
    chassis.mcl().setConfig(mcl);

    if (cal.valid) {
        auto ekf = chassis.ekf().config();
        ekf.mclSigmaScale = cal.scale;
        ekf.mclSigmaFloor = cal.floorIn;
        chassis.ekf().setConfig(ekf);
    }

    out.result("copy to config.cpp:");
    if (!kFieldIsClear) {
        const auto perBeam = tuning::perBeamFromPair(fit.params);
        out.result("mcl.shortWeight = %.3ff", perBeam.shortWeight);
        out.result("mcl.shortDecaySigmas = %.2ff", perBeam.shortDecaySigmas);
        out.result("mcl.randWeight = %.5ff", perBeam.randWeight);
    } else {
        out.result("beam mixture NOT fitted:");
        out.result("clear field has no tail.");
        out.result("keep the shipped shortWeight");
    }
    if (cal.valid) {
        out.result("ekf.mclSigmaScale = %.2ff", cal.scale);
        out.result("ekf.mclSigmaFloor = %.2ff", cal.floorIn);
    } else {
        out.result("sigma cal: too few samples");
    }
    if (best >= 0) {
        out.result("mcl.distResampleVariance=%.2ff", kCandidates[best].distVar);
        out.result("mcl.thetaResampleVar=%.3ff", kCandidates[best].thetaVar);
        out.result("set %d: %.0fms %d/%d", best + 1, bestScore.medianMs,
                   bestScore.converged, bestScore.trials);
    } else {
        out.result("no setting ever recovered");
    }
    out.result("n=%d plc=%d tail=%.0f%%", fit.samples, placements,
               100.0f * fit.shortFraction);

    if (kFieldIsClear && fit.valid && fit.shortFraction > 0.05f) {
        out.result("TAIL on a clear field --");
        out.result("something is in the way");
    }
    if (blocked > 0) {
        out.result("%d placements had a beam", blocked);
        out.result("blocked: no sigma sample");
    }
}
}
