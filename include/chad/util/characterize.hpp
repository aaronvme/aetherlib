#pragma once

#include <functional>

#include "chad/chassis.hpp"
#include "chad/drivetrain.hpp"
#include "chad/movement_control/feedforward.hpp"

namespace chad::tuning {

// Measures constants of this robot from what it can observe itself. Nothing here
// needs a known field position, except measureScale(). Run before PidAutoTuner.

struct CharacterizeConfig {
    // Two slopes, slow first. One slope leaves the acceleration column nearly
    // constant once the drivetrain settles, so kA is barely excited: with
    // realistic velocity quantisation a single-slope ramp fits kA to within
    // 100% of zero at every window length, while switching slope part way
    // recovers it to a few percent. Slow-first also pins kS, which fast-first
    // misses by 15-70%.
    float rampVoltsPerSec{1.0f};
    float rampVoltsPerSecFast{6.0f};
    float rampSwitchVolts{4.5f};
    float maxVolts{9.0f};

    float sampleMs{10.0f};

    // Ticks the acceleration difference spans, paired against the window mean.
    // Velocity is quantised, so a short span is mostly quantisation noise.
    int accelSpanTicks{40};

    // Below this the fit reads stiction as slope.
    float minSpeedInPerSec{3.0f};

    // Standard errors of kA needed before kA is believed. r2 cannot police it:
    // a ramp that leaves kA unidentified still fits r2 = 0.9999, because kS and
    // kV carry the voltage on their own.
    float kaMinT{25.0f};

    // Whole-fit gate. Every fit measured, sound or not, cleared 0.996.
    float minR2{0.9f};

    // The ramp shuttles, reversing past this. Samples around a reversal are
    // discarded, so a shorter span just wastes more of them.
    float maxTravelIn{30.0f};

    int spinRotations{6};
    float parkSeconds{6.0f};

    std::function<void(const char* stage, const char* detail)> onProgress;
};

// Units FeedforwardGains expects.
struct FeedforwardFit {
    float kV{0.0f};
    float kS{0.0f};
    float kA{0.0f};
    float r2{0.0f};
    float kaT{0.0f};        // |kA| in standard errors
    int samples{0};
    bool valid{false};      // kS and kV are usable
    bool kaValid{false};    // and so is kA
};

// Odometry error per inch: the robot ends where it began, so whatever odometry
// reports instead is its own.
struct OdomDriftFit {
    float alongPerIn{0.0f};    // residual along-track error per inch driven
    float lateralPerIn{0.0f};  // residual cross-track error per inch driven
    float thetaPerRad{0.0f};   // heading residual per radian turned
    float distanceIn{0.0f};    // total travelled
    int reps{0};
    bool valid{false};
};

// Per-sensor range noise, parked square to a wall. Index matches
// distanceSensors.
struct RangeNoiseFit {
    float sigmaIn[MAX_DISTANCE_SENSORS]{};
    float meanIn[MAX_DISTANCE_SENSORS]{};
    int samples[MAX_DISTANCE_SENSORS]{};
    float worstSigmaIn{0.0f};
};

// Scatter while parked, which is noise by definition. Accuracy needs truth.
struct NoiseFit {
    float sigmaIn{0.0f};        // position, inches
    float sigmaThetaDeg{0.0f};
    float peakIn{0.0f};         // worst excursion from the mean
    int samples{0};

    // targetTolerance below the peak excursion can never be met. bearingHold
    // covers the radius where atan2 to the target is mostly noise.
    float suggestTolerance{0.0f};
    float suggestBearingHold{0.0f};
};

// The robot drives itself. The ramp and step move forward; the rest spin.
class Characterizer {
public:
    Characterizer(Chassis& chassis, Drivetrain& drivetrain,
                  const CharacterizeConfig& config = {});

    /// One ramp, fitting kS, kV and kA together. spinInPlace fits turning
    /// friction and inertia instead of straight-line, and needs no room.
    FeedforwardFit feedforward(bool spinInPlace = false);

    /// Differential wheel travel over the heading the IMU turned through.
    /// Independent of wheel diameter.
    float trackWidth();

    /// Pass the measured distance, get the factor to multiply
    /// Drivetrain::wheelDiameter by.
    float measureScale(float trueDistanceIn, float commandDistanceIn = 24.0f);

    /// Volts at which the robot first moves, which is kS. Costs a couple of
    /// inches, for when there is no room to ramp.
    float breakawayVolts(bool spinInPlace = false);

    /// Parked scatter of Chassis::getPose(). Run with the localization you race.
    NoiseFit localizationNoise();

    /// Out and back `reps` times; whatever odometry reports as displacement is
    /// its drift. Run with localization off.
    OdomDriftFit measureOdomDrift(float legIn = 36.0f, int reps = 3);

    /// Parked square to a wall. Sigma per sensor over parkSeconds.
    RangeNoiseFit measureRangeNoise(
        const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors);

private:
    void report(const char* stage, const char* detail) const;
    void drive(float volts, bool spin);

    Chassis& mChassis;
    Drivetrain& mDrivetrain;
    CharacterizeConfig mConfig;
};

}  // namespace chad::tuning
