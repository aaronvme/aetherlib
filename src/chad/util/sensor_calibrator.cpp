#include "chad/util/sensor_calibrator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "pros/rtos.hpp"

#include "chad/chassis.hpp"
#include "chad/movement_control/turntopoint.hpp"

namespace chad {
namespace {
float signedHeadingDelta(float headingDeg, float referenceDeg) {
    float diff = wrapDeg(headingDeg - referenceDeg);
    if (diff > 180.0f) diff -= 360.0f;
    return diff;
}

CalibrationResult failedResult(const std::string& name, int samplesUsed) {
    CalibrationResult r;
    r.name = name;
    r.offsetIn = 0.0f;
    r.confidence = CALIBRATION_FAILED_SCORE;
    r.samplesUsed = samplesUsed;
    r.valid = false;
    return r;
}
}

SensorCalibrator::SensorCalibrator(Chassis& chassis, const Config& config)
    : mChassis(chassis), mConfig(config) {}

float SensorCalibrator::currentHeadingDeg() const {
    return stdRadToVexDeg(mChassis.getPose().theta);
}

void SensorCalibrator::turnTo(float headingDeg) {
    TurntoHeadingParams params;
    params.maxSpeed = mConfig.turnMaxSpeed;

    mChassis.turnToHeading(headingDeg, mConfig.turnTimeoutMs, params, false);
}

// A turn that settles inside its tolerance can still be rocking, and a
// reading taken mid-rock pairs a distance with the wrong heading.
void SensorCalibrator::waitUntilStill() const {
    constexpr uint32_t kStepMs = 100;
    const float maxStepDeg = mConfig.stillDegPerSec * (kStepMs / 1000.0f);
    const uint32_t t0 = pros::millis();
    float prev = currentHeadingDeg();
    while (pros::millis() - t0 < mConfig.stillTimeoutMs) {
        pros::delay(kStepMs);
        const float now = currentHeadingDeg();
        if (std::fabs(signedHeadingDelta(now, prev)) <= maxStepDeg) return;
        prev = now;
    }
}

CalibrationResult SensorCalibrator::solveLeastSquares(const std::string& name,
                                                       const std::vector<float>& thetaRad,
                                                       const std::vector<float>& distIn) {
    const std::size_t count = thetaRad.size();
    if (count < 2 || distIn.size() != count) {
        return failedResult(name, static_cast<int>(count));
    }

    double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumXX = 0.0;
    for (std::size_t i = 0; i < count; i++) {
        const double x = std::sin(static_cast<double>(thetaRad[i]));
        const double y = static_cast<double>(distIn[i]) * std::cos(static_cast<double>(thetaRad[i]));

        sumX  += x;
        sumY  += y;
        sumXY += x * y;
        sumXX += x * x;
    }

    const double n = static_cast<double>(count);
    const double denom = n * sumXX - sumX * sumX;

    if (std::fabs(denom) < 1e-9) {
        return failedResult(name, static_cast<int>(count));
    }

    const double slope = (n * sumXY - sumX * sumY) / denom;
    const double intercept = (sumY - slope * sumX) / n;

    const double offsetIn = -slope;

    double totalError = 0.0;
    for (std::size_t i = 0; i < count; i++) {
        const double x = std::sin(static_cast<double>(thetaRad[i]));
        const double yActual = static_cast<double>(distIn[i]) * std::cos(static_cast<double>(thetaRad[i]));
        const double yPred = slope * x + intercept;
        const double residual = yActual - yPred;
        totalError += residual * residual;
    }

    double sumCos = 0.0;
    for (std::size_t i = 0; i < count; i++) {
        sumCos += std::cos(static_cast<double>(thetaRad[i]));
    }

    CalibrationResult result;
    result.name = name;
    result.offsetIn = static_cast<float>(offsetIn);
    result.confidence = static_cast<float>(totalError / n);
    result.interceptIn = static_cast<float>(intercept);
    result.meanCosTheta = static_cast<float>(sumCos / n);
    result.samplesUsed = static_cast<int>(count);
    result.valid = std::isfinite(result.offsetIn) && std::isfinite(result.confidence);
    if (!result.valid) return failedResult(name, static_cast<int>(count));
    return result;
}

CalibrationResult SensorCalibrator::calibrate(const CalibrationConfig& config) {
    if (config.sensor == nullptr) {
        if (mConfig.verbose) {
            printf("  [FAIL] %s: null sensor pointer\n", config.name.c_str());
        }
        return failedResult(config.name, 0);
    }

    if (mConfig.verbose) {
        printf("Calibrating %s sensor (base heading %.1f deg)...\n",
               config.name.c_str(), static_cast<double>(config.baseHeadingDeg));
    }

    std::vector<float> recordedThetas;
    std::vector<float> recordedDists;
    recordedThetas.reserve(mConfig.sweepAnglesDeg.size());
    recordedDists.reserve(mConfig.sweepAnglesDeg.size());

    turnTo(wrapDeg(config.baseHeadingDeg));
    pros::delay(mConfig.settleDelayMs);

    for (float relAngleDeg : mConfig.sweepAnglesDeg) {
        turnTo(wrapDeg(config.baseHeadingDeg + relAngleDeg));

        pros::delay(mConfig.postTurnDelayMs);
        waitUntilStill();

        // The heading is read with each distance, so the pair always belongs
        // to the same instant even if the robot creeps between them.
        const int samples = std::max(1, mConfig.samplesPerAngle);
        std::vector<float> dists;
        dists.reserve(samples);
        double sumDiffDeg = 0.0;
        for (int k = 0; k < samples; ++k) {
            if (k > 0) pros::delay(mConfig.sampleSpacingMs);
            const float diffDeg =
                signedHeadingDelta(currentHeadingDeg(), config.baseHeadingDeg);
            const std::int32_t readingMm = config.sensor->get();
            if (readingMm <= 0 ||
                static_cast<float>(readingMm) > mConfig.maxValidReadingMm) {
                continue;
            }
            dists.push_back(mmToInch(static_cast<float>(readingMm)));
            sumDiffDeg += diffDeg;
        }

        // Fewer than half usable means the beam is not reliably on the wall.
        if (dists.empty() || static_cast<int>(dists.size()) * 2 < samples) {
            if (mConfig.verbose) {
                printf("  [ERR] %s: %u/%d good readings at %+.0f deg — skipped\n",
                       config.name.c_str(), static_cast<unsigned>(dists.size()),
                       samples, static_cast<double>(relAngleDeg));
            }
            continue;
        }

        const float measuredDiffDeg =
            static_cast<float>(sumDiffDeg / static_cast<double>(dists.size()));
        // The median, so one stray return cannot drag the point.
        std::sort(dists.begin(), dists.end());
        const std::size_t mid = dists.size() / 2;
        const float distIn = (dists.size() % 2 == 1)
                                 ? dists[mid]
                                 : 0.5f * (dists[mid - 1] + dists[mid]);
        recordedThetas.push_back(degToRad(measuredDiffDeg));
        recordedDists.push_back(distIn);

        if (mConfig.verbose) {
            printf("  angle %6.2f deg | dist %7.3f in\n",
                   static_cast<double>(measuredDiffDeg), static_cast<double>(distIn));
        }
    }

    if (recordedDists.size() < 2) {
        if (mConfig.verbose) {
            printf("  [FAIL] %s: only %u valid point(s), need 2+\n",
                   config.name.c_str(), static_cast<unsigned>(recordedDists.size()));
        }
        return failedResult(config.name, static_cast<int>(recordedDists.size()));
    }

    CalibrationResult result = solveLeastSquares(config.name, recordedThetas, recordedDists);
    if (mConfig.verbose) {
        if (result.valid) {
            printf("  -> %s offset %+.4f in (left+), residual %.5f in^2 over %d pts%s\n",
                   result.name.c_str(), static_cast<double>(result.offsetIn),
                   static_cast<double>(result.confidence), result.samplesUsed,
                   (result.samplesUsed < 3) ? "  [residual meaningless at 2 pts]" : "");
        } else {
            printf("  [FAIL] %s: singular fit (sweep angles not distinct?)\n",
                   result.name.c_str());
        }
    }
    return result;
}

std::vector<CalibrationResult> SensorCalibrator::calibrateAll(
        const std::vector<CalibrationConfig>& configs) {
    std::vector<CalibrationResult> results;
    results.reserve(configs.size());

    const float startHeadingDeg = currentHeadingDeg();

    if (mConfig.verbose) {
        printf("--- CHAD SENSOR CALIBRATION (start heading %.1f deg) ---\n",
               static_cast<double>(startHeadingDeg));
    }

    for (const auto& config : configs) {
        results.push_back(calibrate(config));
    }

    turnTo(startHeadingDeg);

    if (mConfig.verbose) {
        printf("--- CALIBRATION COMPLETE (%u sensor(s)) ---\n",
               static_cast<unsigned>(results.size()));
        for (const auto& r : results) {
            printf("  %-10s offset %+8.4f in  residual %8.5f  %s\n",
                   r.name.c_str(), static_cast<double>(r.offsetIn),
                   static_cast<double>(r.confidence), r.valid ? "OK" : "FAILED");
        }
    }

    return results;
}

float estimateWallAngle(pros::Distance& sensorA, pros::Distance& sensorB,
                        float robotHeadingDeg, const WallAngleConfig& cfg) {
    const std::int32_t rawA = sensorA.get();
    const std::int32_t rawB = sensorB.get();

    if (rawA <= 0 || rawB <= 0) return WALL_ANGLE_INVALID;

    const float dA = static_cast<float>(rawA);
    const float dB = static_cast<float>(rawB) + cfg.sensorBiasMm;

    if (dA > cfg.maxValidReadingMm || dB > cfg.maxValidReadingMm) {
        return WALL_ANGLE_INVALID;
    }

    if (cfg.sensorBaselineIn <= 0.0f) return WALL_ANGLE_INVALID;

    const float deltaIn = mmToInch(dA - dB);
    const float deviationDeg = radToDeg(std::atan2(deltaIn, cfg.sensorBaselineIn));

    const float heading = wrapDeg(robotHeadingDeg);
    float cardinal;
    if (heading >= 315.0f || heading < 45.0f) {
        cardinal = 0.0f;
    } else if (heading < 135.0f) {
        cardinal = 90.0f;
    } else if (heading < 225.0f) {
        cardinal = 180.0f;
    } else {
        cardinal = 270.0f;
    }

    return wrapDeg(cardinal + deviationDeg);
}
}
