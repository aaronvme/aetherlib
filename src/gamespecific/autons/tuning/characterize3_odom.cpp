#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include <algorithm>
#include <cmath>

#include "riku/api.hpp"

namespace riku {

using namespace autons;

void autonCharacterize3Odom() {
    Report out;
    out("phase 3: scale + odom drift");
    if (!headingOk(out)) return;
    out("filters off. needs ~66 in ahead");
    setFilters(false);
    auto ch = makeCharacterizer(out);

    // Your tape reading of the 48 in drive below; 0 skips the correction.
    constexpr float kTapeMeasuredIn = 48.2f;

    if (kTapeMeasuredIn > 0.0f) {
        const float was = drivetrain.wheelDiameter;
        drivetrain.wheelDiameter *= ch.measureScale(kTapeMeasuredIn, 48.0f);
        out.result("wheelDiameter = %.4ff (was %.4f)", drivetrain.wheelDiameter, was);
        chassis.moveDistance(-48.0f, 6000);
    } else {
        const float before = drivetrain.averagePositionDeg();
        chassis.moveDistance(48.0f, 6000);
        const float after = drivetrain.averagePositionDeg();
        const float reported = (after - before) / 360.0f *
                               drivetrain.externalGearRatio() * PI *
                               drivetrain.wheelDiameter;
        out.result("odom says %.2f in over 48", reported);
        waitForA(out, "measure the real distance now");
        chassis.moveDistance(-48.0f, 6000);
    }

    out("drift legs: 3x 48 in out and back");
    const auto drift = ch.measureOdomDrift(48.0f, 3);
    if (drift.alongPerIn > 0.5f) {
        out.result("drift %.3f/in implausible --", drift.alongPerIn);
        out.result("it did not return. check phase 2");
        return;
    }

    constexpr float kDriftFloor = 0.004f;
    const float along = std::max(drift.alongPerIn, kDriftFloor);
    const float lateral = std::max(drift.lateralPerIn, kDriftFloor);

    auto mcl = chassis.mcl().config();
    mcl.imeVariance = along;
    mcl.horizConstantNoise = lateral;
    chassis.mcl().setConfig(mcl);

    auto ekf = chassis.ekf().config();
    ekf.vertNoiseProp = along;
    ekf.slipNoiseProp = lateral;
    chassis.ekf().setConfig(ekf);

    out.result("copy to config.cpp:");
    out.result("mcl.imeVariance = %.4ff", along);
    out.result("mcl.horizConstantNoise = %.4ff", lateral);
    out.result("ekf.vertNoiseProp = %.4ff", along);
    out.result("ekf.slipNoiseProp = %.4ff", lateral);
    out.result("raw %.4f/%.4f floor %.3f", drift.alongPerIn,
               drift.lateralPerIn, kDriftFloor);
    if (kTapeMeasuredIn <= 0.0f) {
        out.result("then set kTapeMeasuredIn and");
        out.result("rerun for wheelDiameter");
    }
}

}  // namespace riku
