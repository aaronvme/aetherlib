#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "chad/api.hpp"

namespace chad {
using namespace autons;

namespace {
constexpr float kImuSigmaFloorRad = 0.01f * DEG_TO_RAD;

bool fitSigma(const std::vector<float>& x, const std::vector<float>& y,
              float& a, float& b, float& originSlope) {
    const int n = static_cast<int>(x.size());
    if (n < 3) return false;
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < n; ++i) {
        sx += x[i]; sy += y[i]; sxx += x[i] * x[i]; sxy += x[i] * y[i];
    }
    const double det = n * sxx - sx * sx;
    if (std::fabs(det) < 1e-9 || sxx < 1e-9) return false;
    b = static_cast<float>((n * sxy - sx * sy) / det);
    a = static_cast<float>((sy - b * sx) / n);
    originSlope = static_cast<float>(sxy / sxx);
    return true;
}
}

void autonCharacterize4Filters() {
    Report out;
    out("phase 4: sensor + pose noise");
    if (!headingOk(out)) return;
    out("stays parked. do not touch it");

    setFilters(false);
    auto ch = makeCharacterizer(out);
    const auto beams = ch.measureRangeNoise(distSensors);

    const auto imu = ch.localizationNoise();
    const float imuSigmaRad = std::max(imu.sigmaThetaDeg * DEG_TO_RAD,
                                       kImuSigmaFloorRad);
    {
        auto m = chassis.mcl().config();
        m.imuVariance = imuSigmaRad;
        chassis.mcl().setConfig(m);
        auto e = chassis.ekf().config();
        e.imuSigmaRad = imuSigmaRad;
        chassis.ekf().setConfig(e);
    }

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
    out.result("mcl.imuVariance = %.5ff", imuSigmaRad);
    out.result("ekf.imuSigmaRad = %.5ff", imuSigmaRad);
    out.result("bearingHoldDist = %.2ff", noise.suggestBearingHold);
    out.result("pose sigma %.2fin pk %.2fin", noise.sigmaIn, noise.peakIn);

    out.result("gate streak %d reloc %d",
               chassis.ekf().mclRejectStreak(), chassis.ekf().relocateCount());

    constexpr int kStops = 3;
    constexpr float kStopIn = 8.0f;
    std::vector<float> xs, ys;
    for (int i = 0; i < MAX_DISTANCE_SENSORS; ++i) {
        if (beams.samples[i] > 8) {
            xs.push_back(beams.meanIn[i]);
            ys.push_back(beams.sigmaIn[i]);
        }
    }
    for (int stop = 1; stop < kStops; ++stop) {
        out("range sweep %d/%d", stop, kStops - 1);
        chassis.moveDistance(-kStopIn, 4000);
        const auto b = ch.measureRangeNoise(distSensors);
        for (int i = 0; i < MAX_DISTANCE_SENSORS; ++i) {
            if (b.samples[i] > 8) {
                xs.push_back(b.meanIn[i]);
                ys.push_back(b.sigmaIn[i]);
            }
        }
    }
    chassis.moveDistance((kStops - 1) * kStopIn, 6000);

    float a = 0.0f, slope = 0.0f, originSlope = 0.0f;
    if (!fitSigma(xs, ys, a, slope, originSlope)) {
        out.result("range fit needs 3+ points,");
        out.result("got %d. keep the values above",
                   static_cast<int>(xs.size()));
        return;
    }
    auto ekf2 = chassis.ekf().config();
    ekf2.rangeSigmaBase = std::max(a, 0.0f);
    ekf2.rangeSigmaProp = std::max(slope, 0.0f);
    chassis.ekf().setConfig(ekf2);
    auto mcl2 = chassis.mcl().config();
    mcl2.rangeSigmaProp = std::max(originSlope, 0.0f);
    chassis.mcl().setConfig(mcl2);

    out.result("range fit supersedes:");
    out.result("ekf.rangeSigmaBase = %.4ff", std::max(a, 0.0f));
    out.result("ekf.rangeSigmaProp = %.4ff", std::max(slope, 0.0f));
    out.result("mcl.rangeSigmaProp = %.4ff", std::max(originSlope, 0.0f));
    out.result("from %d readings", static_cast<int>(xs.size()));
    if (slope < 0.0f) {
        out.result("slope came out negative:");
        out.result("noise did not grow with range");
    }
}
}
