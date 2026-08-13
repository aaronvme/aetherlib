#pragma once

#include <functional>
#include <vector>

#include "riku/chassis.hpp"
#include "riku/util/pid.hpp"
#include "riku/util/step_response.hpp"

namespace riku::tuning {

struct AutoTuneConfig {
    /// Turn sizes to tune against, in order. Gains carry forward, so later
    /// (larger) steps refine what the earlier ones found.
    std::vector<float> angleStepsDeg{90.0f, 180.0f};
    std::vector<float> distanceStepsIn{24.0f, 48.0f};

    /// Each trial runs for exactly this long. Tolerance is forced to 0 so the
    /// motion cannot settle early - otherwise it exits the moment it reaches
    /// tolerance and the overshoot is never observed.
    float trialMs{1500.0f};

    /// Reps per candidate gain, averaged. Each rep is an out-and-back pair.
    int reps{2};

    float kpStart{20.0f};
    float kpStartLateral{4.0f};
    float kpGrowth{1.5f};

    /// Stop raising kP once a trial exceeds this overshoot, as a percentage of
    /// the step size, or this many oscillations.
    float overshootLimitPct{20.0f};
    int oscillationLimit{3};

    /// kD is searched as a multiple of the chosen kP.
    float kdRatioStep{0.05f};

    /// kI is only attempted if steady-state error stays above this.
    float steadyStateTolDeg{1.5f};
    float steadyStateTolIn{0.75f};
    float kiStep{0.01f};

    int maxIterations{6};

    StepWeights weights;

    /// Called after each trial so progress is visible on a long run.
    std::function<void(const char* stage, PIDGains gains, float cost)> onProgress;
};


// Drives the robot through repeated step responses and fits gains from the
// resulting traces. The robot moves on its own - give it clear space.
class PidAutoTuner {
public:
    explicit PidAutoTuner(Chassis& chassis, const AutoTuneConfig& config = {});
    ~PidAutoTuner();

    /// Turns 0 -> step -> 0, tuning kP then kD then kI. Restores the chassis
    /// telemetry hook and returns the gains it settled on.
    PIDGains tuneAngular();

    /// Same, driving forward and back.
    PIDGains tuneLateral();

    void cancel() { mCancelled = true; }
    bool wasCancelled() const { return mCancelled; }

    /// Metrics for the winning gains of the last completed stage.
    const StepMetrics& lastMetrics() const { return mLast; }

private:
    struct Trace {
        std::vector<float> t;
        std::vector<float> error;
    };

    PIDGains tuneAxis(bool angular);
    StepMetrics evaluate(const PIDGains& gains, bool angular, float step);
    StepMetrics runTrial(bool angular, float from, float to);
    void report(const char* stage, const PIDGains& gains, const StepMetrics& m);

    Chassis& mChassis;
    AutoTuneConfig mConfig;
    StepMetrics mLast;
    bool mCancelled{false};

    Trace mTrace;
    pros::Mutex mTraceMutex;
    float mTarget{0};
    Pose mStartPose;
};

}  // namespace riku::tuning
