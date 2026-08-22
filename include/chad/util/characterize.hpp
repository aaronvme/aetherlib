#pragma once

#include <functional>

#include "chad/chassis.hpp"
#include "chad/drivetrain.hpp"
#include "chad/movement_control/feedforward.hpp"

namespace chad::tuning {

// Measures constants of this robot from what it can observe itself. Nothing here
// needs a known field position, except measureScale(). Run before PidAutoTuner.

struct CharacterizeConfig {
    // The fit carries an acceleration term, so the ramp need not be quasistatic.
    // That assumption used to cost 36 ft of travel and a separate step for kA.
    float rampVoltsPerSec{2.0f};
    float maxVolts{9.0f};

    float stepVolts{7.0f};   // unused now; kA comes out of the ramp fit
    float stepMs{700.0f};

    float sampleMs{10.0f};

    // Below this the fit reads stiction as slope.
    float minSpeedInPerSec{3.0f};

    // The ramp shuttles, reversing past this. Samples around a reversal are
    // discarded, so a shorter span just wastes more of them.
    float maxTravelIn{30.0f};

    int spinRotations{6};
    float parkSeconds{6.0f};

    std::function<void(const char* stage, const char* detail)> onProgress;
};

// Units FeedforwardGains expects. r2 under ~0.9 means the ramp outran the
// drivetrain, the surface changed, or a motor is not pulling.
struct FeedforwardFit {
    float kV{0.0f};
    float kS{0.0f};
    float kA{0.0f};
    float r2{0.0f};
    int samples{0};
    bool valid{false};
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
