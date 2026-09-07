#pragma once

#include <functional>

#include "chad/chassis.hpp"
#include "chad/drivetrain.hpp"
#include "chad/movement_control/feedforward.hpp"
#include "chad/util/characterize_fit.hpp"

namespace chad::tuning {
struct CharacterizeConfig {
    int staircaseLevels{11};
    float maxVolts{9.0f};
    float breakawayVoltsPerSec{2.0f};
    float dwellMs{900.0f};
    float holdMs{400.0f};

    float stepVolts{7.0f};
    float stepMs{550.0f};
    float slopeWindowMs{200.0f};
    int stepReps{6};

    float sampleMs{10.0f};

    float minSpeedInPerSec{3.0f};

    float maxTravelIn{60.0f};

    float minR2{0.9f};
    float maxKaSpread{0.5f};
    float freeSpeedLo{0.55f};
    float freeSpeedHi{1.15f};

    int spinRotations{6};
    float parkSeconds{6.0f};

    std::function<void(const char* stage, const char* detail)> onProgress;
};

struct FeedforwardFit {
    float kV{0.0f};
    float kS{0.0f};
    float kA{0.0f};
    float r2{0.0f};
    float kaSpread{0.0f};
    float freeSpeedRatio{0.0f};
    float speedSpanInPerSec{0.0f};
    int samples{0};
    int stepReps{0};
    bool valid{false};
    bool kaValid{false};
    bool drivetrainSuspect{false};
};

struct OdomDriftFit {
    float alongPerIn{0.0f};
    float lateralPerIn{0.0f};

    float thetaPerIn{0.0f};
    float distanceIn{0.0f};
    int reps{0};
    bool valid{false};
};

struct RangeNoiseFit {
    float sigmaIn[MAX_DISTANCE_SENSORS]{};
    float meanIn[MAX_DISTANCE_SENSORS]{};
    int samples[MAX_DISTANCE_SENSORS]{};
    float worstSigmaIn{0.0f};
};

struct NoiseFit {
    float sigmaIn{0.0f};
    float sigmaThetaDeg{0.0f};
    float peakIn{0.0f};
    int samples{0};

    float suggestTolerance{0.0f};
    float suggestBearingHold{0.0f};
};

class Characterizer {
public:
    Characterizer(Chassis& chassis, Drivetrain& drivetrain,
                  const CharacterizeConfig& config = {});

    FeedforwardFit feedforward(bool spinInPlace = false);

    float trackWidth();

    float measureScale(float trueDistanceIn, float commandDistanceIn = 24.0f);

    float breakawayVolts(bool spinInPlace = false);

    NoiseFit localizationNoise();

    OdomDriftFit measureOdomDrift(float legIn = 36.0f, int reps = 3);

    RangeNoiseFit measureRangeNoise(
        const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors);

private:
    void report(const char* stage, const char* detail) const;
    void drive(float volts, bool spin);

    Chassis& mChassis;
    Drivetrain& mDrivetrain;
    CharacterizeConfig mConfig;
};
}
