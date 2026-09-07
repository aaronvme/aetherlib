#include "chad/util/autotune.hpp"

#include <cmath>
#include <cstdio>
#include <mutex>

#include "chad/util/util.hpp"

namespace chad::tuning {
PidAutoTuner::PidAutoTuner(Chassis& chassis, const AutoTuneConfig& config)
    : mChassis(chassis), mConfig(config) {}

PidAutoTuner::~PidAutoTuner() {
    mChassis.setTelemetry(nullptr);
}

PIDGains PidAutoTuner::tuneAngular() {
    return tuneAxis(true);
}

PIDGains PidAutoTuner::tuneLateral() {
    return tuneAxis(false);
}

StepMetrics PidAutoTuner::runTrial(bool angular, float delta) {
    {
        std::lock_guard<pros::Mutex> lock(mTraceMutex);
        mTrace.t.clear();
        mTrace.error.clear();
    }

    mStartPose = mChassis.getPose();
    mDelta = delta;
    mTarget = stdRadToVexDeg(mStartPose.theta) + delta;
    const uint32_t t0 = pros::millis();

    mChassis.setTelemetry([this, angular, t0](const ChassisSample& s) {
        float err;
        if (angular) {
            err = radToDeg(angleDiffRad(vexToStdRad(s.thetaDeg), vexToStdRad(mTarget)));
        } else {
            const float travelled =
                (s.x - mStartPose.x) * std::cos(mStartPose.theta) +
                (s.y - mStartPose.y) * std::sin(mStartPose.theta);
            err = mDelta - travelled;
        }
        if (mConfig.tiltDeg) {
            const float tilt = mConfig.tiltDeg();
            if (std::isfinite(tilt) && tilt > mPeakTilt) mPeakTilt = tilt;

            if (mConfig.tiltAbortDeg > 0.0f && std::isfinite(tilt) &&
                tilt > mConfig.tiltAbortDeg) {
                mTiltAborted = true;
                mChassis.abortMotion();
            }
        }
        std::lock_guard<pros::Mutex> lock(mTraceMutex);
        mTrace.t.push_back(static_cast<float>(pros::millis() - t0));
        mTrace.error.push_back(err);
    });

    if (angular) {
        mChassis.turnToHeading(mTarget, mConfig.trialMs, {.targetTolerance = 0.0f});
    } else {
        mChassis.moveDistance(mDelta, mConfig.trialMs, {.targetTolerance = 0.0f});
    }

    mChassis.setTelemetry(nullptr);

    Trace copy;
    {
        std::lock_guard<pros::Mutex> lock(mTraceMutex);
        copy = mTrace;
    }
    return analyseStep(copy.t, copy.error, std::fabs(delta), mConfig.weights);
}

StepMetrics PidAutoTuner::evaluate(const PIDGains& gains, bool angular, float step) {
    if (angular) {
        mChassis.setAngularGains(gains);
    } else {
        mChassis.setLateralGains(gains);
    }
    return runSteps(angular, step);
}

StepMetrics PidAutoTuner::measureAt(bool angular, float step) {
    return runSteps(angular, step);
}

StepMetrics PidAutoTuner::runSteps(bool angular, float step) {
    mPeakTilt = 0.0f;
    mTiltAborted = false;
    MetricsAverage avg;
    avg.reset();

    for (int r = 0; r < mConfig.reps && !mCancelled; ++r) {
        avg.add(runTrial(angular, step));

        if (mTiltAborted) break;
        avg.add(runTrial(angular, -step));
        if (mTiltAborted) break;
    }

    return avg.result();
}

void PidAutoTuner::report(const char* stage, const PIDGains& g, const StepMetrics& m) {
    std::printf("[autotune] %-8s kP=%.3f kI=%.4f kD=%.3f | e0=%.2f over=%.1f%% "
                "sse=%.2f settle=%.0fms osc=%d cost=%.1f\n",
                stage, g.kP, g.kI, g.kD, m.initial, m.overshootPct,
                m.steadyState, m.settleMs, m.oscillations, m.cost);
    if (mConfig.onProgress) mConfig.onProgress(stage, g, m);
}

namespace {
const char* phaseName(GainPhase p) {
    switch (p) {
        case GainPhase::Kp: return "kP";
        case GainPhase::Kd: return "kD";
        case GainPhase::Ki: return "kI";
        default: return "step";
    }
}
}

PIDGains PidAutoTuner::tuneAxis(bool angular) {
    mCancelled = false;

    const std::vector<float>& steps =
        angular ? mConfig.angleStepsDeg : mConfig.distanceStepsIn;

    GainSearchConfig gs;
    gs.kpGrowth = mConfig.kpGrowth;
    gs.overshootLimitPct = mConfig.overshootLimitPct;
    gs.oscillationLimit = mConfig.oscillationLimit;
    gs.kdSeedRatio = mConfig.kdSeedRatio;
    gs.kdRatioStep = mConfig.kdRatioStep;
    gs.kiRatioStep = mConfig.kiRatioStep;
    gs.includeIntegral = mConfig.includeIntegral;
    gs.steadyStateTol =
        angular ? mConfig.steadyStateTolDeg : mConfig.steadyStateTolIn;
    gs.maxIterations = mConfig.maxIterations;

    GainSearch search;
    search.reset(gs,
                 {angular ? mConfig.kpStart : mConfig.kpStartLateral, 0.0f, 0.0f});

    for (float step : steps) {
        if (mCancelled) break;
        search.beginStep();
        while (!search.stepDone() && !mCancelled) {
            const TrialRequest t = search.request();
            const StepMetrics m = evaluate(t.gains, angular, step);

            report(m.valid ? phaseName(t.phase) : "no trace", t.gains, m);
            search.submit(m);
        }
        mLast = search.bestMetrics();
        report("step", search.best(), mLast);
    }

    const PIDGains best = search.best();
    if (angular) {
        mChassis.setAngularGains(best);
    } else {
        mChassis.setLateralGains(best);
    }
    mChassis.setTelemetry(nullptr);
    return best;
}
}
