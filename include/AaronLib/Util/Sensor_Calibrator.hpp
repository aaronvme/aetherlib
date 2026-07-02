// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "pros/distance.hpp"

#include "AaronLib/Util/Util.hpp"

namespace AARONLIB {

// Forward declaration — the calibrator only needs a reference. Chassis.hpp is
// included by Sensor_Calibrator.cpp instead, which keeps this header usable
// from anywhere in the include graph (Util/ is the lowest layer; Chassis is
// the highest) without creating a cycle.
class Chassis;

// ============================================================================
// SensorCalibrator — least-squares distance-sensor mounting-offset solver
// ============================================================================
//
// Purpose
// -------
// RclTracking needs each sensor's mount pose in the robot frame
// (RclTracking::RclSensor::mountOffset — x = forward, y = left, theta =
// pointing angle).  Measuring the lateral (y) component with a ruler is
// error-prone: a 0.25 in error there becomes a 0.25 in position bias in every
// RCL fix.  This class measures it instead.
//
// Method
// ------
// Park the robot facing a flat wall, then rotate through a few small headings
// and record (heading, distance) pairs.  For a forward-pointing sensor mounted
// at robot-frame (a = forward, b = left), with the robot rotated by theta
// (VEX convention, clockwise positive) away from the wall normal:
//
//     d * cos(theta) = (W - c) - a * cos(theta) - b * sin(theta)
//
// where d is the raw reading, W the wall coordinate and c the robot centre
// coordinate.  Over a small sweep, cos(theta) ~ 1, so the whole first group is
// effectively a constant.  That leaves the straight line the solver fits:
//
//     Y = D - x * X        with  Y = d * cos(theta),  X = sin(theta)
//
// Ordinary least squares gives slope and intercept; the NEGATED SLOPE is the
// lateral mounting offset:
//
//     offsetIn == b == RclSensor::mountOffset.y   (left positive, inches)
//
// so the result drops straight into ChassisConfig::sensorMounts[i].y.
//
// The intercept D conflates the wall distance with the FORWARD mount offset a
// (they are not separately identifiable from this sweep), so it is discarded —
// measure the forward offset with a ruler, it is far less sensitive.
//
// Limitations worth knowing before trusting a number
// --------------------------------------------------
//   * The wall must be flat, non-reflective and square to the field axes.
//     35 deg is already a steep incidence angle for the V5 laser; readings at
//     that angle are the first ones to drop out.
//   * The `cos(theta) ~ 1` approximation on the forward-offset term is the
//     dominant modelling error and shows up as residual (see confidence).
//   * The sweep must contain at least two DISTINCT angles or the fit is
//     singular; the solver detects this and returns valid == false.
// ============================================================================

/// One sensor to calibrate.
struct CalibrationConfig {
    std::string name;                  ///< human-readable label, e.g. "Front"
    pros::Distance* sensor{nullptr};   ///< the sensor to characterise
    float baseHeadingDeg{0.0f};        ///< VEX heading (0-360, CW+) at which
                                       ///< this sensor faces the wall head-on
};

/// Confidence score reported when a sensor could not be solved at all.
/// Chosen to be far worse than any physically plausible residual.
constexpr float CALIBRATION_FAILED_SCORE = 999.9f;

/// Result of calibrating one sensor.
struct CalibrationResult {
    std::string name;          ///< echoed from the CalibrationConfig
    float offsetIn{0.0f};      ///< solved lateral mount offset, inches, LEFT POSITIVE.
                               ///< Assign to ChassisConfig::sensorMounts[i].y.

    /// Mean squared residual of the fit, in square inches.
    ///
    /// LOWER IS BETTER. The donor library called this a "confidence score",
    /// which reads backwards — a *high* value here means a *bad* fit. It is an
    /// error metric, not a confidence: 0 is a perfect fit and
    /// CALIBRATION_FAILED_SCORE means no fit was possible.
    ///
    /// Also note: with exactly 2 samples the line passes through both points,
    /// so the residual is identically 0 and the score is MEANINGLESS. Always
    /// check samplesUsed >= 3 before reading anything into a low score.
    float confidence{CALIBRATION_FAILED_SCORE};

    int samplesUsed{0};        ///< readings that survived the range filter
    bool valid{false};         ///< false => offsetIn is not usable
};

struct SensorCalibratorConfig {
    /// Headings to sample, RELATIVE to each sensor's baseHeadingDeg
    /// (VEX degrees, clockwise positive). Needs >= 2 distinct entries;
    /// 3+ makes the residual score meaningful.
    std::vector<float> sweepAnglesDeg{0.0f, 30.0f, 35.0f};

    float turnMaxSpeed{60.0f};       ///< TurntoHeadingParams::maxSpeed for every turn
    float turnTimeoutMs{1000.0f};    ///< per-turn timeout handed to Chassis::turnToHeading
    uint32_t settleDelayMs{500};     ///< pause after the initial turn onto baseHeadingDeg
    uint32_t postTurnDelayMs{600};   ///< pause after each sweep turn, before sampling.
                                     ///< Load-bearing: a still-rocking robot smears the
                                     ///< reading far more than the offset being measured.
    float maxValidReadingMm{9990.0f};///< readings at/above this are the V5 "nothing in
                                     ///< range" sentinel (and PROS_ERR) — discarded
    bool verbose{true};              ///< printf progress to the PROS terminal
};

class SensorCalibrator {
public:
    // ========================================================================
    // Configuration — everything the donor routine hardcoded
    // ========================================================================

    using Config = SensorCalibratorConfig;

    // ========================================================================
    // Construction
    // ========================================================================

    /// @param chassis  drives the turns and supplies the fused heading. Taken by
    ///                 reference — AaronLib has no lemlib, and the donor's
    ///                 separate pros::Imu* is unnecessary here because
    ///                 Chassis::getPose() already exposes the fused heading.
    explicit SensorCalibrator(Chassis& chassis, const Config& config = {});

    // ========================================================================
    // Calibration
    // ========================================================================

    /// Calibrate every listed sensor in turn, then drive back to the heading the
    /// robot held when this call started.
    ///
    /// BLOCKING, and it moves the robot — expect roughly
    /// (settleDelayMs + sweepAnglesDeg.size() * (turnTimeoutMs + postTurnDelayMs))
    /// milliseconds per sensor. Run it from a dedicated tuning routine with the
    /// robot squared up against a wall, never from a match auton.
    ///
    /// @return one result per input config, in the same order. Entries with
    ///         valid == false failed and must not be applied.
    std::vector<CalibrationResult> calibrateAll(const std::vector<CalibrationConfig>& configs);

    /// Calibrate a single sensor. Does NOT restore the starting heading — the
    /// robot is left at the last swept angle. calibrateAll() wraps this.
    CalibrationResult calibrate(const CalibrationConfig& config);

    // ========================================================================
    // Configuration access
    // ========================================================================

    const Config& config() const { return mConfig; }
    void setConfig(const Config& config) { mConfig = config; }

private:
    // ========================================================================
    // Internal helpers
    // ========================================================================

    /// Ordinary least squares on Y = slope*X + intercept, X = sin(theta),
    /// Y = d*cos(theta). Returns offsetIn = -slope and the mean squared
    /// residual. Detects a singular normal equation (fewer than 2 distinct
    /// angles) and reports valid == false instead of dividing by zero.
    static CalibrationResult solveLeastSquares(const std::string& name,
                                               const std::vector<float>& thetaRad,
                                               const std::vector<float>& distIn);

    /// Blocking turn to an absolute VEX heading at the configured speed.
    void turnTo(float headingDeg);

    /// Current fused heading in VEX degrees (0-360, clockwise positive).
    float currentHeadingDeg() const;

    Chassis& mChassis;
    Config mConfig;
};

// ============================================================================
// Wall-normal estimator — two forward sensors vs. a flat wall
// ============================================================================
//
// Two forward-pointing distance sensors separated laterally by a known
// baseline see different distances when the robot is not square to the wall.
// The difference gives the robot's angular deviation from the wall normal:
//
//     deviation = atan2(dRight - dLeft, baseline)
//
// Snapping that deviation onto the nearest cardinal (0/90/180/270) turns it
// into an absolute field heading, which is a drift-free heading fix — the IMU
// cannot supply one because its error is unbounded in time.
//
// Provenance: this is a revival, not a straight port. In the donor repo the
// equivalent function was DEAD CODE — it referenced a global `frontDistance2`
// that globals.h declared but no translation unit ever defined, so the routine
// could never have linked, let alone run. It has therefore never been validated
// on hardware. Both sensors are explicit parameters here so the caller is
// forced to supply two that actually exist. Verify the sign on your robot
// before trusting it (see sensorA/sensorB below).
// ============================================================================

/// Returned by estimateWallAngle() when the reading is rejected.
///
/// The donor returned -1, which is ambiguous: -1 is also a perfectly legal
/// wall angle (1 degree counter-clockwise of the cardinal), so a caller could
/// not tell a rejection from a valid near-square reading. This sentinel is far
/// outside the [0, 360) range of any valid result. Test it with
/// wallAngleValid() rather than comparing floats directly.
constexpr float WALL_ANGLE_INVALID = -1000.0f;

/// True when estimateWallAngle() returned a usable heading.
inline bool wallAngleValid(float wallAngleDeg) {
    return wallAngleDeg > -360.0f;
}

struct WallAngleConfig {
    /// Lateral separation between the two sensors, inches. This is the
    /// denominator of the angle estimate, so an error here scales the whole
    /// result — measure it between the sensor lenses, not the housings.
    float sensorBaselineIn{10.75f};

    /// Additive bias applied to sensorB's raw reading, millimetres. Corrects a
    /// known mounting-depth mismatch between the two sensors (one recessed
    /// deeper than the other). Zero if both sit at the same depth.
    float sensorBiasMm{10.0f};

    /// Readings at/above this are rejected, millimetres. Deliberately far below
    /// the sensor's 9999 "nothing in range" sentinel: beyond ~1.8 m the V5
    /// laser's noise is comparable to the difference being measured, so the
    /// angle it yields is meaningless even though the reading looks valid.
    float maxValidReadingMm{1800.0f};
};

/// Estimate the robot's absolute field heading from two forward sensors facing
/// a perimeter wall.
///
/// @param sensorA         forward sensor on the robot's RIGHT
/// @param sensorB         forward sensor on the robot's LEFT; cfg.sensorBiasMm
///                        is added to this one's reading
/// @param robotHeadingDeg current heading estimate, VEX degrees (0-360, CW+).
///                        Used ONLY to pick the quadrant; its precision does not
///                        affect the returned angle, it just has to be within
///                        45 degrees of the right cardinal.
/// @param cfg             geometry and rejection thresholds
///
/// @return absolute VEX heading in [0, 360), or WALL_ANGLE_INVALID if either
///         reading was out of range / errored.
///
/// Sign convention: with sensorA on the right, rotating the robot clockwise
/// away from a wall it faces pushes the right sensor further from that wall, so
/// dRight - dLeft goes positive and the reported heading increases — matching
/// the VEX clockwise-positive convention. IF THE CORRECTION PUSHES YOUR HEADING
/// THE WRONG WAY, SWAP THE TWO SENSOR ARGUMENTS. (The donor's naming gave no
/// hint which of its two sensors was which, and being dead code it was never
/// checked against hardware.)
///
/// Quadrant snapping assumes A RECTANGULAR FIELD PERIMETER whose walls lie on
/// the field axes, and that the robot is roughly square to one of them: the
/// deviation is added to whichever of 0/90/180/270 robotHeadingDeg is nearest.
/// It is therefore only meaningful when the robot is within ~45 degrees of a
/// wall normal and the sensors see that perimeter wall — not a goal, a stack of
/// game objects, or a robot. Aim it at a known-clear wall segment; the
/// maxValidReadingMm filter is the only defence against a bogus target, and it
/// will not catch a nearby obstacle.
///
/// Feeding the result back into tracking — this is the whole point of the
/// function. Chassis::setHeading() is the correct entry point because it fans
/// the corrected heading out to odometry, MCL and RclTracking together
/// (Chassis::setHeading -> mOdom.setPose + mMcl.setPose + mRcl.setRclPose),
/// leaving no estimator holding a stale angle:
///
///     float wall = estimateWallAngle(rightDist, leftDist,
///                                    stdRadToVexDeg(chassis.getPose().theta));
///     if (wallAngleValid(wall)) chassis.setHeading(wall);
///
/// RclTracking benefits immediately and indirectly: computePose() rotates every
/// sensor's mountOffset by the odometry heading, so a heading error leaks into
/// both the X and Y fixes. Correcting the heading first, then letting the next
/// RCL update() run, gives a materially better position fix than correcting
/// position alone. Do this while stationary and square to a wall — mid-motion
/// readings are smeared and will inject error rather than remove it.
float estimateWallAngle(pros::Distance& sensorA, pros::Distance& sensorB,
                        float robotHeadingDeg, const WallAngleConfig& cfg = {});

}  // namespace AARONLIB
