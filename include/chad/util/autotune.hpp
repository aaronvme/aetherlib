#pragma once

#include <functional>
#include <vector>

#include "chad/chassis.hpp"
#include "chad/util/pid.hpp"
#include "chad/util/autotune_core.hpp"
#include "chad/util/step_response.hpp"

namespace chad::tuning {
struct AutoTuneConfig {
    std::vector<float> angleStepsDeg{90.0f, 180.0f};
    std::vector<float> distanceStepsIn{24.0f, 48.0f};

    float trialMs{1500.0f};

    int reps{2};

    float kpStart{20.0f};
    float kpStartLateral{4.0f};
    float kpGrowth{1.5f};

    float overshootLimitPct{20.0f};
    int oscillationLimit{3};

    float kdSeedRatio{0.075f};

    float kdRatioStep{0.05f};

    float steadyStateTolDeg{1.5f};
    float steadyStateTolIn{0.75f};

    float kiRatioStep{0.5f};

    bool includeIntegral{false};

    int maxIterations{6};

    StepWeights weights;

    std::function<float()> tiltDeg;

    float tiltAbortDeg{0.0f};

    std::function<void(const char* stage, PIDGains gains,
                       const StepMetrics& metrics)> onProgress;
};

class PidAutoTuner {
public:
    explicit PidAutoTuner(Chassis& chassis, const AutoTuneConfig& config = {});
    ~PidAutoTuner();

    PIDGains tuneAngular();

    PIDGains tuneLateral();

    void cancel() { mCancelled = true; }
    bool wasCancelled() const { return mCancelled; }

    const StepMetrics& lastMetrics() const { return mLast; }

    float lastPeakTilt() const { return mPeakTilt; }

    bool lastTiltAborted() const { return mTiltAborted; }

    StepMetrics measureAt(bool angular, float step);

private:
    struct Trace {
        std::vector<float> t;
        std::vector<float> error;
    };

    PIDGains tuneAxis(bool angular);
    StepMetrics evaluate(const PIDGains& gains, bool angular, float step);
    StepMetrics runSteps(bool angular, float step);

    StepMetrics runTrial(bool angular, float delta);
    void report(const char* stage, const PIDGains& gains, const StepMetrics& m);

    Chassis& mChassis;
    AutoTuneConfig mConfig;
    StepMetrics mLast;
    bool mCancelled{false};

    Trace mTrace;
    pros::Mutex mTraceMutex;
    float mPeakTilt{0.0f};
    bool mTiltAborted{false};
    float mTarget{0};
    float mDelta{0};
    Pose mStartPose;
};
}
