#pragma once

#include <functional>

#include "riku/chassis.hpp"
#include "riku/drivetrain.hpp"
#include "riku/movement_control/feedforward.hpp"

namespace riku::tuning {

// Measures constants of this robot from what it can observe itself. Nothing here
// needs a known field position, except measureScale(). Run before PidAutoTuner.

struct CharacterizeConfig {
    // Slow enough that acceleration is negligible.
    float rampVoltsPerSec{0.6f};
    float maxVolts{9.0f};

    float stepVolts{7.0f};   // separate faster step, for kA
    float stepMs{700.0f};

    float sampleMs{10.0f};

    // Below this the fit reads stiction as slope.
    float minSpeedInPerSec{3.0f};

    // The straight ramp needs roughly 36 ft of travel to reach maxVolts at
    // rampVoltsPerSec, and a VEX field is 12. It shuttles instead: on passing
    // this it reverses and carries on, which the fit does not mind because the
    // commanded voltage reverses with it.
    float maxTravelIn{60.0f};

    int spinRotations{6};
    float parkSeconds{6.0f};

    std::function<void(const char* stage, const char* detail)> onProgress;
};

// Units FeedforwardGains expects. r2 under about 0.9 means the ramp outran the
// drivetrain, the surface changed, or a motor is not pulling.
struct FeedforwardFit {
    float kV{0.0f};
    float kS{0.0f};
    float kA{0.0f};
    float r2{0.0f};
    int samples{0};
    bool valid{false};
};

// Odometry error per inch, from an out-and-back: the robot ends where it began,
// so whatever odometry reports instead is its own error.
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

    /// Ramp for kV/kS, then a step for kA. spinInPlace fits turning friction
    /// instead of straight-line, and needs no room.
    FeedforwardFit feedforward(bool spinInPlace = false);

    /// Differential wheel travel over the heading the IMU turned through.
    /// Independent of wheel diameter.
    float trackWidth();

    /// Pass the measured distance, get the factor to multiply
    /// Drivetrain::wheelDiameter by.
    float measureScale(float trueDistanceIn, float commandDistanceIn = 24.0f);

    /// Parked scatter of Chassis::getPose(). Run with the localization you race.
    NoiseFit localizationNoise();

    /// Drives out and back `reps` times. Odometry should report returning to
    /// where it started; whatever it reports instead is its drift. Localization
    /// off, so this measures odometry alone.
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

}  // namespace riku::tuning
