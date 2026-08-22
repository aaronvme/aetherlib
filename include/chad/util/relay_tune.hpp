#pragma once

#include <functional>

#include "chad/chassis.hpp"
#include "chad/drivetrain.hpp"
#include "chad/util/pid.hpp"

namespace chad::tuning {

// Relay (Astrom-Hagglund) identification: switch a fixed output across the
// setpoint, then read ku and tu off the limit cycle. Seconds per axis at one
// output, where PidAutoTuner takes minutes and climbs kP to find the limit.
// PidAutoTuner is kept for axes that will not oscillate.

struct RelayTuneConfig {
    /// Relay output, motor units. Per axis: 40 gives a clean heading cycle and
    /// barely moves the drive axis off its dead band.
    float amplitudeAngular{40.0f};
    float amplitudeLateral{80.0f};

    /// Dead band. A lag feeding an integrator never reaches -180 of phase, so
    /// there is no natural frequency and this sets the excited one. Wider is
    /// slower and larger.
    float hysteresisDeg{0.5f};
    float hysteresisIn{0.5f};

    int cycles{4};                 // full cycles to average, after the transient

    /// Shortest half period allowed. A loaded drivetrain cannot reverse faster,
    /// so anything quicker is the sensor switching the relay, not the robot.
    float minHalfPeriodMs{60.0f};
    float settleMs{300.0f};
    float timeoutMs{8000.0f};
    float sampleMs{10.0f};

    /// Give up if the oscillation runs away instead of settling.
    float abortDeg{90.0f};
    float abortIn{12.0f};

    /// Tyreus-Luyben. Ziegler-Nichols (0.6, Tu/8, Tu/2) rings on a drivetrain.
    /// Per axis so the drive can be backed off, but equal by default -- for
    /// tipping use ChassisConfig::lateralSlewPerSec, which costs no speed.
    float kpFactorAngular{0.45f};
    float kpFactorLateral{0.45f};
    float tdDivisor{6.3f};
    float tiFactor{2.2f};

    /// Neither axis has a standing load, and stiction winds an integrator up
    /// while parked.
    bool includeIntegral{false};

    std::function<void(const char* stage, const char* detail)> onProgress;
};

struct RelayFit {
    float ku{0.0f};          // ultimate gain, output units per error unit
    float tu{0.0f};          // ultimate period, seconds
    float amplitude{0.0f};   // measured half-amplitude of the limit cycle
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

    /// Spins across the heading it starts on. Needs no floor.
    RelayFit tuneAngular();

    /// Drives across the point it starts on; needs about abortIn either side.
    RelayFit tuneLateral();

private:
    RelayFit run(bool angular);
    void drive(float out, bool angular);
    void report(const char* stage, const char* detail) const;

    Chassis& mChassis;
    Drivetrain& mDrivetrain;
    RelayTuneConfig mConfig;
};

}  // namespace chad::tuning
