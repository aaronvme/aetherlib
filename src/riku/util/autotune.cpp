#include "riku/util/autotune.hpp"

#include <cmath>
#include <cstdio>
#include <mutex>

#include "riku/util/util.hpp"

namespace riku::tuning {

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

    // Built from the live heading: setPose cannot fake a start heading.
    mStartPose = mChassis.getPose();
    mDelta = delta;
    mTarget = stdRadToVexDeg(mStartPose.theta) + delta;
    const uint32_t t0 = pros::millis();

    mChassis.setTelemetry([this, angular, t0](const ChassisSample& s) {
        float err;
        if (angular) {
            err = radToDeg(angleDiffRad(vexToStdRad(s.thetaDeg), vexToStdRad(mTarget)));
        } else {
            // Signed along the start heading. Unsigned distance made the return
            // trial read as 100% overshoot.
            const float travelled =
                (s.x - mStartPose.x) * std::cos(mStartPose.theta) +
                (s.y - mStartPose.y) * std::sin(mStartPose.theta);
            err = mDelta - travelled;
        }
        if (mConfig.tiltDeg) {
            const float tilt = mConfig.tiltDeg();
            if (std::isfinite(tilt) && tilt > mPeakTilt) mPeakTilt = tilt;
        }
        std::lock_guard<pros::Mutex> lock(mTraceMutex);
        mTrace.t.push_back(static_cast<float>(pros::millis() - t0));
        mTrace.error.push_back(err);
    });

    // Tolerance 0 never satisfies |err| < tol, so the motion runs the full
    // window and the whole response is captured.
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
    StepMetrics total;
    int counted = 0;

    for (int r = 0; r < mConfig.reps && !mCancelled; ++r) {
        const StepMetrics out = runTrial(angular, step);
        const StepMetrics back = runTrial(angular, -step);

        for (const StepMetrics& m : {out, back}) {
            if (!m.valid) {
                // The opening error is the only clue to why it was rejected.
                if (total.initial == 0.0f) total.initial = m.initial;
                continue;
            }
            total.initial += m.initial;
            total.overshootPct += m.overshootPct;
            total.steadyState += m.steadyState;
            total.settleMs += m.settleMs;
            total.oscillations += m.oscillations;
            total.cost += m.cost;
            counted++;
        }
    }

    if (counted == 0) return {};

    const float n = static_cast<float>(counted);
    total.initial /= n;
    total.overshootPct /= n;
    total.steadyState /= n;
    total.settleMs /= n;
    total.oscillations = static_cast<int>(std::lround(total.oscillations / n));
    total.cost /= n;
    total.valid = true;
    return total;
}

void PidAutoTuner::report(const char* stage, const PIDGains& g, const StepMetrics& m) {
    std::printf("[autotune] %-8s kP=%.3f kI=%.4f kD=%.3f | e0=%.2f over=%.1f%% "
                "sse=%.2f settle=%.0fms osc=%d cost=%.1f\n",
                stage, g.kP, g.kI, g.kD, m.initial, m.overshootPct,
                m.steadyState, m.settleMs, m.oscillations, m.cost);
    if (mConfig.onProgress) mConfig.onProgress(stage, g, m);
}

PIDGains PidAutoTuner::tuneAxis(bool angular) {
    mCancelled = false;

    const std::vector<float>& steps =
        angular ? mConfig.angleStepsDeg : mConfig.distanceStepsIn;
    const float ssTol =
        angular ? mConfig.steadyStateTolDeg : mConfig.steadyStateTolIn;

    PIDGains best{angular ? mConfig.kpStart : mConfig.kpStartLateral, 0.0f, 0.0f};

    for (float step : steps) {
        if (mCancelled) break;

        // kP: grow until overshoot or oscillation says stop, keep the best cost.
        PIDGains candidate = best;
        candidate.kI = 0.0f;
        candidate.kD = 0.0f;
        float bestCost = 1e30f;
        float bestKp = candidate.kP;

        for (int i = 0; i < mConfig.maxIterations && !mCancelled; ++i) {
            const StepMetrics m = evaluate(candidate, angular, step);
            // Silence used to look identical to a finished search.
            if (!m.valid) { report("no trace", candidate, m); break; }
            report("kP", candidate, m);

            if (m.cost < bestCost) {
                bestCost = m.cost;
                bestKp = candidate.kP;
                mLast = m;
            }
            if (m.overshootPct > mConfig.overshootLimitPct ||
                m.oscillations >= mConfig.oscillationLimit) {
                break;
            }
            candidate.kP *= mConfig.kpGrowth;
        }

        best.kP = bestKp;
        best.kD = 0.0f;
        best.kI = 0.0f;

        // kD: raise while it keeps helping.
        for (int i = 1; i <= mConfig.maxIterations && !mCancelled; ++i) {
            PIDGains trial = best;
            trial.kD = best.kP * mConfig.kdRatioStep * static_cast<float>(i);

            const StepMetrics m = evaluate(trial, angular, step);
            if (!m.valid) { report("no trace", trial, m); break; }
            report("kD", trial, m);

            if (m.cost < bestCost) {
                bestCost = m.cost;
                best.kD = trial.kD;
                mLast = m;
            } else {
                break;
            }
        }

        // kI: only if there is steady-state error left to remove.
        if (mLast.valid && mLast.steadyState > ssTol) {
            for (int i = 1; i <= mConfig.maxIterations && !mCancelled; ++i) {
                PIDGains trial = best;
                trial.kI = mConfig.kiStep * static_cast<float>(i);

                const StepMetrics m = evaluate(trial, angular, step);
                if (!m.valid) { report("no trace", trial, m); break; }
                report("kI", trial, m);

                if (m.cost < bestCost) {
                    bestCost = m.cost;
                    best.kI = trial.kI;
                    mLast = m;
                }
                if (m.steadyState <= ssTol) break;
            }
        }

        report("step", best, mLast);
    }

    if (angular) {
        mChassis.setAngularGains(best);
    } else {
        mChassis.setLateralGains(best);
    }
    mChassis.setTelemetry(nullptr);
    return best;
}

}  // namespace riku::tuning
