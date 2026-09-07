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
}

RelayAutoTuner::RelayAutoTuner(Chassis& chassis, Drivetrain& drivetrain,
                               const RelayTuneConfig& config)
    : mChassis(chassis), mDrivetrain(drivetrain), mConfig(config) {}

void RelayAutoTuner::report(const char* stage, const char* detail) const {
    std::printf("[relay] %s: %s\n", stage, detail);
    if (mConfig.onProgress) mConfig.onProgress(stage, detail);
}

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
    RelayFit fit;
    report(angular ? "angular" : "lateral", "settling");
    mDrivetrain.setVoltage(0.0f, 0.0f);
    pros::delay(static_cast<uint32_t>(mConfig.settleMs));

    const Pose start = mChassis.getPose();
    const float hyst =
        angular ? degToRad(mConfig.hysteresisDeg) : mConfig.hysteresisIn;
    const float abort = angular ? degToRad(mConfig.abortDeg) : mConfig.abortIn;
    const float drive_d =
        angular ? mConfig.amplitudeAngular : mConfig.amplitudeLateral;
    const float fx = std::cos(start.theta);
    const float fy = std::sin(start.theta);

    report(angular ? "angular" : "lateral", "oscillating");

    std::vector<float> halfPeriods, peaks;
    float sign = 1.0f;
    float peak = 0.0f;
    float lastSwitchMs = -1.0f;
    int switches = 0;
    const uint32_t t0 = pros::millis();

    while (pros::millis() - t0 < static_cast<uint32_t>(mConfig.timeoutMs)) {
        const Pose p = mChassis.getPose();

        const float err = angular
                              ? -angleDiffRad(p.theta, start.theta)
                              : (p.x - start.x) * fx + (p.y - start.y) * fy;

        if (std::fabs(err) > abort) {
            mDrivetrain.setVoltage(0.0f, 0.0f);
            report(angular ? "angular" : "lateral", "ran away, no fit");
            return fit;
        }
        peak = std::max(peak, std::fabs(err));

        float next = sign;
        if (err > hyst) next = -1.0f;
        else if (err < -hyst) next = 1.0f;

        const float sinceSwitch = static_cast<float>(pros::millis() - t0) -
                                  (lastSwitchMs >= 0.0f ? lastSwitchMs : 0.0f);
        if (next != sign && sinceSwitch >= mConfig.minHalfPeriodMs) {
            sign = next;
            switches++;
            const float now = static_cast<float>(pros::millis() - t0);

            if (lastSwitchMs >= 0.0f && switches > 2) {
                halfPeriods.push_back(now - lastSwitchMs);
                peaks.push_back(peak);
            }
            lastSwitchMs = now;
            peak = 0.0f;
            if (static_cast<int>(halfPeriods.size()) >= mConfig.cycles * 2) break;
        }

        drive(sign * drive_d, angular);
        pros::delay(static_cast<uint32_t>(mConfig.sampleMs));
    }
    mDrivetrain.setVoltage(0.0f, 0.0f);

    char msg[96];
    if (halfPeriods.size() < 2) {
        report(angular ? "angular" : "lateral", "no limit cycle, raise amplitude");
        return fit;
    }

    const float halfMs = mean(halfPeriods);
    fit.tu = 2.0f * halfMs / 1000.0f;
    fit.amplitude = mean(peaks);
    fit.cycles = static_cast<int>(halfPeriods.size()) / 2;

    if (halfMs < mConfig.minHalfPeriodMs * 1.2f) {
        std::snprintf(msg, sizeof(msg), "tu %.2fs pinned at the dwell floor",
                      fit.tu);
        report(angular ? "angular" : "lateral", msg);
        return fit;
    }

    if (fit.tu <= 0.0f || fit.amplitude < mConfig.minSwingRatio * hyst) {
        std::snprintf(msg, sizeof(msg), "swing %.3f under %.1f dead bands, raise amplitude",
                      fit.amplitude,
                      static_cast<double>(mConfig.minSwingRatio));
        report(angular ? "angular" : "lateral", msg);
        return fit;
    }

    fit.ku = 4.0f * drive_d / (PI * fit.amplitude);
    fit.kP = (angular ? mConfig.kpFactorAngular
                      : mConfig.kpFactorLateral) * fit.ku;
    fit.kD = fit.kP * fit.tu / mConfig.tdDivisor;
    fit.kI = mConfig.includeIntegral
                 ? fit.kP / (mConfig.tiFactor * fit.tu)
                 : 0.0f;
    fit.valid = true;

    std::snprintf(msg, sizeof(msg), "ku %.1f tu %.3fs a %.3f n=%d", fit.ku,
                  fit.tu, fit.amplitude, fit.cycles);
    report(angular ? "angular" : "lateral", msg);
    return fit;
}
}
