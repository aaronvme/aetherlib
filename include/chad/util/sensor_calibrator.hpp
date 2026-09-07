#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "pros/distance.hpp"

#include "chad/util/util.hpp"

namespace chad {
class Chassis;

struct CalibrationConfig {
    std::string name;
    pros::Distance* sensor{nullptr};
    float baseHeadingDeg{0.0f};
};

constexpr float CALIBRATION_FAILED_SCORE = 999.9f;

struct CalibrationResult {
    std::string name;
    float offsetIn{0.0f};

    float confidence{CALIBRATION_FAILED_SCORE};

    int samplesUsed{0};
    bool valid{false};
};

struct SensorCalibratorConfig {
    std::vector<float> sweepAnglesDeg{0.0f, 30.0f, 35.0f};

    float turnMaxSpeed{60.0f};
    float turnTimeoutMs{1000.0f};
    uint32_t settleDelayMs{500};
    uint32_t postTurnDelayMs{600};

    float maxValidReadingMm{9990.0f};

    bool verbose{true};
};

class SensorCalibrator {
public:

    using Config = SensorCalibratorConfig;

    explicit SensorCalibrator(Chassis& chassis, const Config& config = {});

    std::vector<CalibrationResult> calibrateAll(const std::vector<CalibrationConfig>& configs);

    CalibrationResult calibrate(const CalibrationConfig& config);

    const Config& config() const { return mConfig; }
    void setConfig(const Config& config) { mConfig = config; }

private:

    static CalibrationResult solveLeastSquares(const std::string& name,
                                               const std::vector<float>& thetaRad,
                                               const std::vector<float>& distIn);

    void turnTo(float headingDeg);

    float currentHeadingDeg() const;

    Chassis& mChassis;
    Config mConfig;
};

constexpr float WALL_ANGLE_INVALID = -1000.0f;

inline bool wallAngleValid(float wallAngleDeg) {
    return wallAngleDeg > -360.0f;
}

struct WallAngleConfig {
    float sensorBaselineIn{10.75f};

    float sensorBiasMm{10.0f};

    float maxValidReadingMm{1800.0f};
};

float estimateWallAngle(pros::Distance& sensorA, pros::Distance& sensorB,
                        float robotHeadingDeg, const WallAngleConfig& cfg = {});
}
