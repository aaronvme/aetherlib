#pragma once

#include <functional>

#include "chad/chassis.hpp"
#include "chad/drivetrain.hpp"
#include "chad/movement_control/feedforward.hpp"
#include "chad/util/characterize_fit.hpp"

namespace chad::tuning {

// Measures constants of this robot from what it can observe itself. Nothing here
// needs a known field position, except measureScale(). Run before PidAutoTuner.

struct CharacterizeConfig {
    // kS and kV come from a quasistatic staircase: hold a voltage until the
    // speed stops changing, so the acceleration term is zero by construction
    // rather than fitted. The first level sits just above the measured
    // breakaway, which is what keeps kS from being a long extrapolation.
    int staircaseLevels{11};
    float maxVolts{9.0f};
    float breakawayVoltsPerSec{2.0f};   // creep rate, breakawayVolts()
    float dwellMs{900.0f};
    float holdMs{400.0f};   // averaged, at the tail of each dwell

    // kA comes from steps, where acceleration is large instead of constant.
    float stepVolts{7.0f};
    float stepMs{550.0f};
    float slopeWindowMs{200.0f};
    int stepReps{6};

    float sampleMs{10.0f};

    // Below this the fit reads stiction as slope.
    float minSpeedInPerSec{3.0f};

    // Room needed either side of the start. Levels that cannot fit their hold
    // window inside it are skipped, and direction alternates to stay near the
    // start rather than walking away from it.
    float maxTravelIn{60.0f};

    // Gates. r2 alone passes a fit whose kA is wrong by 100%, so it is not the
    // only one; see FeedforwardFit.
    float minR2{0.9f};
    float maxKaSpread{0.5f};
    float freeSpeedLo{0.55f};
    float freeSpeedHi{1.15f};

    int spinRotations{6};
    float parkSeconds{6.0f};

    std::function<void(const char* stage, const char* detail)> onProgress;
};

// Units FeedforwardGains expects. Three gates rather than one, because they
// fail independently: r2 covers the straightness of the staircase, kaSpread the
// agreement between step reps, and freeSpeedRatio the one fault the other two
// cannot see -- a drivetrain that is simply weaker than configured.
struct FeedforwardFit {
    float kV{0.0f};
    float kS{0.0f};
    float kA{0.0f};
    float r2{0.0f};
    float kaSpread{0.0f};        // (max - min) / kA over step reps
    float freeSpeedRatio{0.0f};  // fitted no-load speed / what rpm implies
    float speedSpanInPerSec{0.0f};
    int samples{0};              // staircase levels that held steady
    int stepReps{0};             // step reps that produced a kA
    bool valid{false};           // kS and kV usable
    bool kaValid{false};         // kA usable; kA is left as fitted either way
    bool drivetrainSuspect{false};
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

    /// Staircase for kS and kV, then steps for kA. spinInPlace fits turning
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
