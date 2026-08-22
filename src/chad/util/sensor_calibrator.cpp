// Ported from FBLib - https://github.com/jiazegao/FBLib
#include "chad/util/sensor_calibrator.hpp"

#include <cmath>
#include <cstdio>

#include "pros/rtos.hpp"

#include "chad/chassis.hpp"
#include "chad/movement_control/turntopoint.hpp"

namespace chad {

namespace {

/// Shortest signed difference `heading - reference`, folded to (-180, 180].
/// Used instead of a raw subtraction so a sweep that straddles the 0/360 seam
/// (e.g. baseHeadingDeg = 350) still produces small thetas for the fit.
float signedHeadingDelta(float headingDeg, float referenceDeg) {
    float diff = wrapDeg(headingDeg - referenceDeg);
    if (diff > 180.0f) diff -= 360.0f;
    return diff;
}

/// A result marked unusable. offsetIn stays 0 so a caller that ignores `valid`
/// at least applies no correction rather than a garbage one.
CalibrationResult failedResult(const std::string& name, int samplesUsed) {
    CalibrationResult r;
    r.name = name;
    r.offsetIn = 0.0f;
    r.confidence = CALIBRATION_FAILED_SCORE;
    r.samplesUsed = samplesUsed;
    r.valid = false;
    return r;
}

}  // namespace

// ============================================================================
// Construction
// ============================================================================

SensorCalibrator::SensorCalibrator(Chassis& chassis, const Config& config)
    : mChassis(chassis), mConfig(config) {}

// ============================================================================
// Internal helpers
// ============================================================================

float SensorCalibrator::currentHeadingDeg() const {
    // Pose::theta is standard math radians (CCW+). The sweep math, and every
    // heading the Chassis turn API takes, are VEX degrees (CW+).
    return stdRadToVexDeg(mChassis.getPose().theta);
}

void SensorCalibrator::turnTo(float headingDeg) {
    TurntoHeadingParams params;
    params.maxSpeed = mConfig.turnMaxSpeed;
    // async = false — every turn must finish before the next reading is taken.
    mChassis.turnToHeading(headingDeg, mConfig.turnTimeoutMs, params, false);
}

// ============================================================================
// solveLeastSquares()
// ============================================================================

CalibrationResult SensorCalibrator::solveLeastSquares(const std::string& name,
                                                       const std::vector<float>& thetaRad,
                                                       const std::vector<float>& distIn) {
    const std::size_t count = thetaRad.size();
    if (count < 2 || distIn.size() != count) {
        return failedResult(name, static_cast<int>(count));
    }

    // Accumulate in double: the normal-equation determinant is a difference of
    // similar-magnitude products and loses most of its significant digits in
    // float when the sweep angles are close together (the {30, 35} pair here).
    double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumXX = 0.0;
    for (std::size_t i = 0; i < count; i++) {
        // Y = d*cos(theta), X = sin(theta) — see the model derivation in the header.
        const double x = std::sin(static_cast<double>(thetaRad[i]));
        const double y = static_cast<double>(distIn[i]) * std::cos(static_cast<double>(thetaRad[i]));

        sumX  += x;
        sumY  += y;
        sumXY += x * y;
        sumXX += x * x;
    }

    const double n = static_cast<double>(count);
    const double denom = n * sumXX - sumX * sumX;

    // Singular fit: every sample landed on (effectively) the same angle, so the
    // sweep never produced the lever arm the slope is read from. The donor
    // divided by this unconditionally and returned inf/NaN as a real offset.
    if (std::fabs(denom) < 1e-9) {
        return failedResult(name, static_cast<int>(count));
    }

    const double slope = (n * sumXY - sumX * sumY) / denom;
    const double intercept = (sumY - slope * sumX) / n;

    // Model is Y = D - x*X, so the fitted slope is -x. The intercept D folds
    // together the wall distance and the forward mount offset and is dropped.
    const double offsetIn = -slope;

    // Mean squared residual — the reported "confidence" (lower is better).
    double totalError = 0.0;
    for (std::size_t i = 0; i < count; i++) {
        const double x = std::sin(static_cast<double>(thetaRad[i]));
        const double yActual = static_cast<double>(distIn[i]) * std::cos(static_cast<double>(thetaRad[i]));
        const double yPred = slope * x + intercept;
        const double residual = yActual - yPred;
        totalError += residual * residual;
    }

    CalibrationResult result;
    result.name = name;
    result.offsetIn = static_cast<float>(offsetIn);
    result.confidence = static_cast<float>(totalError / n);
    result.samplesUsed = static_cast<int>(count);
    result.valid = std::isfinite(result.offsetIn) && std::isfinite(result.confidence);
    if (!result.valid) return failedResult(name, static_cast<int>(count));
    return result;
}

// ============================================================================
// calibrate() — one sensor
// ============================================================================

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

    // 1. Square up on the wall this sensor faces.
    turnTo(wrapDeg(config.baseHeadingDeg));
    pros::delay(mConfig.settleDelayMs);

    // 2. Sweep.
    for (float relAngleDeg : mConfig.sweepAnglesDeg) {
        turnTo(wrapDeg(config.baseHeadingDeg + relAngleDeg));

        // Let the chassis stop ringing. Without this the reading is taken while
        // the robot is still rocking and the noise swamps the offset.
        pros::delay(mConfig.postTurnDelayMs);

        // 3. Sample. Use the MEASURED heading, not the commanded one — the turn
        // settles inside targetTolerance, not exactly on target, and feeding the
        // commanded angle into the fit would bake that error into the slope.
        const float measuredDiffDeg = signedHeadingDelta(currentHeadingDeg(), config.baseHeadingDeg);

        const std::int32_t readingMm = config.sensor->get();
        // Rejects the 9999 "nothing in range" sentinel and PROS_ERR (INT32_MAX)
        // in the same comparison; <= 0 catches a lens against the wall.
        if (readingMm <= 0 || static_cast<float>(readingMm) > mConfig.maxValidReadingMm) {
            if (mConfig.verbose) {
                printf("  [ERR] %s: bad reading (%ld mm) at %.1f deg — skipped\n",
                       config.name.c_str(), static_cast<long>(readingMm),
                       static_cast<double>(measuredDiffDeg));
            }
            continue;
        }

        const float distIn = mmToInch(static_cast<float>(readingMm));
        recordedThetas.push_back(degToRad(measuredDiffDeg));
        recordedDists.push_back(distIn);

        if (mConfig.verbose) {
            printf("  angle %6.2f deg | dist %7.3f in\n",
                   static_cast<double>(measuredDiffDeg), static_cast<double>(distIn));
        }
    }

    // 4. Solve.
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

// ============================================================================
// calibrateAll()
// ============================================================================

std::vector<CalibrationResult> SensorCalibrator::calibrateAll(
        const std::vector<CalibrationConfig>& configs) {
    std::vector<CalibrationResult> results;
    results.reserve(configs.size());

    // Record where we started. The donor hardcoded a return to heading 0, which
    // silently relocates the robot's notion of "forward" whenever the routine is
    // launched from anything else; returning to the recorded start heading is
    // both correct and idempotent.
    const float startHeadingDeg = currentHeadingDeg();

    if (mConfig.verbose) {
        printf("--- CHAD SENSOR CALIBRATION (start heading %.1f deg) ---\n",
               static_cast<double>(startHeadingDeg));
    }

    for (const auto& config : configs) {
        results.push_back(calibrate(config));
    }

    // Restore the starting heading.
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

// ============================================================================
// estimateWallAngle()
// ============================================================================

float estimateWallAngle(pros::Distance& sensorA, pros::Distance& sensorB,
                        float robotHeadingDeg, const WallAngleConfig& cfg) {
    // sensorA = right, sensorB = left (+ mounting-depth bias). See the header
    // for the sign derivation and the swap-if-inverted note.
    const std::int32_t rawA = sensorA.get();
    const std::int32_t rawB = sensorB.get();

    // Reject before adding the bias: PROS_ERR is INT32_MAX and adding to it
    // overflows. Also rejects the 9999 no-target sentinel and dead lenses.
    if (rawA <= 0 || rawB <= 0) return WALL_ANGLE_INVALID;

    const float dA = static_cast<float>(rawA);
    const float dB = static_cast<float>(rawB) + cfg.sensorBiasMm;

    if (dA > cfg.maxValidReadingMm || dB > cfg.maxValidReadingMm) {
        return WALL_ANGLE_INVALID;
    }

    // A zero baseline would make atan2 report +/-90 deg for any imbalance.
    if (cfg.sensorBaselineIn <= 0.0f) return WALL_ANGLE_INVALID;

    // deviation = atan2(dRight - dLeft, baseline), clockwise positive.
    const float deltaIn = mmToInch(dA - dB);
    const float deviationDeg = radToDeg(std::atan2(deltaIn, cfg.sensorBaselineIn));

    // Quadrant snap: add the deviation to whichever cardinal the robot is
    // nearest. Assumes a rectangular field perimeter aligned to the field axes
    // (see header) — the deviation is measured against *a* wall, and only the
    // heading estimate tells us which one.
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

    // Wrap into [0, 360). The donor returned the raw sum, so a slightly
    // counter-clockwise robot near heading 0 came back as e.g. -3.0 — both an
    // invalid VEX heading and indistinguishable from its -1 error sentinel.
    return wrapDeg(cardinal + deviationDeg);
}

}  // namespace chad
