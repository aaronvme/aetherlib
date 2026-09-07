#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include <cmath>
#include <vector>

#include "chad/api.hpp"
#include "chad/util/localization_fit.hpp"

namespace chad {
using namespace autons;

namespace {
constexpr float kLengths[] = {12.0f, 36.0f, 72.0f};
constexpr int kNumLengths = 3;
constexpr int kRepsPerLength = 4;
constexpr int kMaxAttempts = 3;
constexpr float kBracketSigmas = 3.0f;
constexpr float kSettleMs = 350.0f;
constexpr uint32_t kDriveTimeoutMs = 5000u;

struct Fix {
    float x{0.0f}, y{0.0f};
    float bracketIn{0.0f};
    bool ok{false};
};

Fix takeFix(float theta) {
    Fix f;
    BeamRead beams[MAX_DISTANCE_SENSORS];
    readBeams(beams, theta);
    BeamPair pairs[2];
    const int n = findBeamPairs(beams, pairs);
    if (n == 0 && !kFieldIsClear) return f;
    f.ok = solveTruth(beams, pairs, n, kBracketSigmas, f.x, f.y, f.bracketIn);
    return f;
}
}

void autonCalibrateOdomBias() {
    Report out;
    out("phase 6: odometry bias");
    if (!headingOk(out)) return;
    out("%.0f in one way, filters OFF", kLengths[kNumLengths - 1]);
    if (kFieldIsClear) {
        clearFieldObstacles();
        out("clear field: 1 beam per axis");
    } else {
        out("obstacles fine, it rejects");
    }

    setFilters(false);

    const float decorr = chassis.ekf().config().biasDecorrIn;
    if (!(decorr > 0.0f)) {
        out.result("biasDecorrIn must be > 0");
        return;
    }

    std::vector<tuning::BiasLeg> legs;
    int rejected = 0, noFix = 0;
    float dir = 1.0f;

    for (int li = 0; li < kNumLengths; ++li) {
        const float len = kLengths[li];
        for (int rep = 0; rep < kRepsPerLength; ++rep) {
            out("leg %.0f in  rep %d/%d  %s", len, rep + 1, kRepsPerLength,
                dir > 0.0f ? "fwd" : "rev");

            bool done = false;
            for (int attempt = 0; attempt < kMaxAttempts && !done; ++attempt) {
                pros::delay(static_cast<uint32_t>(kSettleMs));
                const Pose before = chassis.getPose();
                const Fix a = takeFix(before.theta);
                if (!a.ok) {
                    noFix++;
                    out("  no fix here (%.1f in)", a.bracketIn);

                    chassis.moveDistance(dir * 4.0f, kDriveTimeoutMs);
                    continue;
                }

                chassis.moveDistance(dir * len, kDriveTimeoutMs);
                pros::delay(static_cast<uint32_t>(kSettleMs));

                const Pose after = chassis.getPose();
                const Fix b = takeFix(after.theta);
                if (!b.ok) {
                    noFix++;
                    out("  no fix at the far end");
                    dir = -dir;
                    continue;
                }

                const float odomDx = after.x - before.x;
                const float odomDy = after.y - before.y;
                const float trueDx = b.x - a.x;
                const float trueDy = b.y - a.y;
                const float ex = odomDx - trueDx;
                const float ey = odomDy - trueDy;

                const float c = std::cos(before.theta);
                const float sn = std::sin(before.theta);
                tuning::BiasLeg leg;
                leg.lengthIn = len;
                leg.alongErrIn = ex * c + ey * sn;
                leg.crossErrIn = -ex * sn + ey * c;

                const float trueLen = std::sqrt(trueDx * trueDx + trueDy * trueDy);
                if (std::fabs(trueLen - len) > 0.5f * len) {
                    rejected++;
                    out("  moved %.1f, wanted %.0f", trueLen, len);
                } else {
                    legs.push_back(leg);
                    out("  along %+.2f cross %+.2f", leg.alongErrIn,
                        leg.crossErrIn);
                }
                done = true;

                dir = -dir;
            }
            if (!done) rejected++;
        }
    }
    drivetrain.setVoltage(0.0f, 0.0f);

    const auto fit = tuning::fitOdomBias(legs.data(),
                                        static_cast<int>(legs.size()), decorr);
    if (!fit.valid) {
        out.result("need 4+ good legs, got %d", static_cast<int>(legs.size()));
        out.result("no fix %d, rejected %d", noFix, rejected);
        if (kFieldIsClear) {
            out.result("keep 1 wall per axis in");
            out.result("view, and stay square-ish");
        } else {
            out.result("park where 2 walls are");
            out.result("visible over the whole run");
        }
        return;
    }

    auto ekf = chassis.ekf().config();
    ekf.vertBiasProp = fit.vertBiasProp;
    ekf.slipBiasProp = fit.slipBiasProp;
    chassis.ekf().setConfig(ekf);

    out.result("copy to config.cpp:");
    out.result("ekf.vertBiasProp = %.4ff", fit.vertBiasProp);
    out.result("ekf.slipBiasProp = %.4ff", fit.slipBiasProp);

    out.result("keep biasDecorrIn = %.0ff", decorr);
    out.result("legs %d  exp %.2f (want .5)", fit.legs, fit.growthExponent);
    if (fit.shapeSuspect) {
        out.result("SHAPE OFF: error grows as");
        out.result("L^%.2f, filter assumes L^.5", fit.growthExponent);
    }
    if (noFix + rejected > 0) {
        out.result("no fix %d, rejected %d", noFix, rejected);
    }
}
}
