#include "chad/util/relay_tune.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

#include "chad/util/util.hpp"

namespace chad::tuning {

namespace {

float mean(const std::vector<float>& v) {
    if (v.empty()) return 0.0f;
    float s = 0.0f;
    for (float x : v) s += x;
    return s / static_cast<float>(v.size());
}

}  // namespace

RelayAutoTuner::RelayAutoTuner(Chassis& chassis, Drivetrain& drivetrain,
                               const RelayTuneConfig& config)
    : mChassis(chassis), mDrivetrain(drivetrain), mConfig(config) {}

void RelayAutoTuner::report(const char* stage, const char* detail) const {
    std::printf("[relay] %s: %s\n", stage, detail);
    if (mConfig.onProgress) mConfig.onProgress(stage, detail);
}

// Right side faster is CCW, matching the mix in Chassis. Backwards makes the
// relay positive feedback.
void RelayAutoTuner::drive(float out, bool angular) {
    mDrivetrain.setVoltage(angular ? -out : out, out);
}

RelayFit RelayAutoTuner::tuneAngular() {
    return run(true);
}

RelayFit RelayAutoTuner::tuneLateral() {
    return run(false);
}

RelayFit RelayAutoTuner::run(bool angular) {
    const char* axis = angular ? "angular" : "lateral";
    report(axis, "settling");
    mDrivetrain.setVoltage(0.0f, 0.0f);
    pros::delay(static_cast<uint32_t>(mConfig.settleMs));

    const Pose start = mChassis.getPose();
    const float fx = std::cos(start.theta);
    const float fy = std::sin(start.theta);

    RelayIdentifyConfig cfg;
    cfg.hysteresis =
        angular ? degToRad(mConfig.hysteresisDeg) : mConfig.hysteresisIn;
    cfg.amplitude =
        angular ? mConfig.amplitudeAngular : mConfig.amplitudeLateral;
    cfg.abort = angular ? degToRad(mConfig.abortDeg) : mConfig.abortIn;
    cfg.cycles = mConfig.cycles;
    cfg.minHalfPeriodMs = mConfig.minHalfPeriodMs;
    cfg.minSwingRatio = mConfig.minSwingRatio;
    cfg.dwellSlack = mConfig.dwellSlack;
    cfg.kpFactor =
        angular ? mConfig.kpFactorAngular : mConfig.kpFactorLateral;
    cfg.tdDivisor = mConfig.tdDivisor;
    cfg.tiFactor = mConfig.tiFactor;
    cfg.includeIntegral = mConfig.includeIntegral;

    report(axis, "oscillating");
    RelayIdentifier id(cfg);
    const uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < static_cast<uint32_t>(mConfig.timeoutMs)) {
        const Pose p = mChassis.getPose();
        // Displacement from the setpoint. Negated because angleDiffRad(from, to)
        // returns to - from, which is a controller error, the opposite sign.
        const float err = angular
                              ? -angleDiffRad(p.theta, start.theta)
                              : (p.x - start.x) * fx + (p.y - start.y) * fy;
        const float out =
            id.update(static_cast<float>(pros::millis() - t0), err);
        if (id.outcome() != RelayOutcome::Cycling) break;
        drive(out, angular);
        pros::delay(static_cast<uint32_t>(mConfig.sampleMs));
    }
    id.finish();
    mDrivetrain.setVoltage(0.0f, 0.0f);

    char msg[96];
    const RelayFit& fit = id.fit();
    switch (id.outcome()) {
        case RelayOutcome::RanAway:
            report(axis, "ran away, no fit");
            break;
        case RelayOutcome::NoCycle:
            report(axis, "no limit cycle, raise amplitude");
            break;
        case RelayOutcome::DwellFloor:
            std::snprintf(msg, sizeof(msg), "tu %.2fs pinned at the dwell floor",
                          fit.tu);
            report(axis, msg);
            break;
        case RelayOutcome::Chatter:
            std::snprintf(msg, sizeof(msg),
                          "swing %.3f under %.1f dead bands, raise amplitude",
                          fit.amplitude, static_cast<double>(cfg.minSwingRatio));
            report(axis, msg);
            break;
        case RelayOutcome::Fitted:
            std::snprintf(msg, sizeof(msg), "ku %.1f tu %.3fs a %.3f n=%d",
                          fit.ku, fit.tu, fit.amplitude, fit.cycles);
            report(axis, msg);
            break;
        case RelayOutcome::Cycling:
            report(axis, "timed out mid-cycle");
            break;
    }
    return fit;
}

}  // namespace chad::tuning
