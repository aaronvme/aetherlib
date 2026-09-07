#pragma once

#include <functional>

#include "chad/chassis.hpp"
#include "chad/drivetrain.hpp"
#include "chad/util/pid.hpp"

namespace chad::tuning {
struct RelayTuneConfig {
    float amplitudeAngular{40.0f};
    float amplitudeLateral{80.0f};

    float hysteresisDeg{0.5f};
    float hysteresisIn{0.5f};

    int cycles{4};

    float minHalfPeriodMs{60.0f};
    float settleMs{300.0f};
    float timeoutMs{8000.0f};
    float sampleMs{10.0f};

    float abortDeg{90.0f};
    float abortIn{12.0f};

    float kpFactorAngular{0.45f};
    float kpFactorLateral{0.45f};
    float tdDivisor{6.3f};
    float tiFactor{2.2f};

    bool includeIntegral{false};

    float minSwingRatio{3.0f};

    std::function<void(const char* stage, const char* detail)> onProgress;
};

struct RelayFit {
    float ku{0.0f};
    float tu{0.0f};
    float amplitude{0.0f};
    float kP{0.0f};
    float kI{0.0f};
    float kD{0.0f};
    int cycles{0};
    bool valid{false};
};

class RelayAutoTuner {
public:
    RelayAutoTuner(Chassis& chassis, Drivetrain& drivetrain,
                   const RelayTuneConfig& config = {});

    RelayFit tuneAngular();

    RelayFit tuneLateral();

private:
    RelayFit run(bool angular);
    void drive(float out, bool angular);
    void report(const char* stage, const char* detail) const;

    Chassis& mChassis;
    Drivetrain& mDrivetrain;
    RelayTuneConfig mConfig;
};
}
