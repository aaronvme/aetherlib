// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include <cmath>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace chad {

// ============================================================================
// Mathematical constants
// ============================================================================

constexpr float PI = M_PI;
constexpr float HALF_PI = M_PI / 2.0f;
constexpr float QUARTER_PI = M_PI / 4.0f;
constexpr float TWO_PI = 2.0f * M_PI;
constexpr float INV_PI = 1.0f / M_PI;
constexpr float RAD_TO_DEG = 180.0f * INV_PI;
constexpr float DEG_TO_RAD = M_PI / 180.0f;

// ============================================================================
// Hardware limits
// ============================================================================

/// Maximum number of V5 distance sensors supported by the library.
/// Changing this one constant resizes all sensor arrays system-wide
/// (RCL, MCL, ChassisConfig). All translation units must be recompiled
/// after a change — `pros make clean && pros make`.
constexpr int MAX_DISTANCE_SENSORS = 8;

// ============================================================================
// Pose — 2D position + heading
// ============================================================================

struct Pose {
    float x{0.0f};       // field X (inches)
    float y{0.0f};       // field Y (inches)
    float theta{0.0f};   // heading (radians), standard math convention

    // Vector addition / subtraction
    Pose operator+(const Pose& other) const {
        return {x + other.x, y + other.y, theta + other.theta};
    }
    Pose operator-(const Pose& other) const {
        return {x - other.x, y - other.y, theta - other.theta};
    }

    // Scalar multiplication
    Pose operator*(float scalar) const {
        return {x * scalar, y * scalar, theta * scalar};
    }

    // Equality with configurable tolerance
    bool equals(const Pose& other, float linTol = 0.001f, float angTol = 0.0001f) const {
        return (std::fabs(x - other.x) < linTol) &&
               (std::fabs(y - other.y) < linTol) &&
               (std::fabs(theta - other.theta) < angTol);
    }
};

// ============================================================================
// Angle utilities
// ============================================================================

/// Wrap angle to [-PI, PI]
///
/// Reduced in one step, not by repeated subtraction. Past about 2^27 the gap
/// between adjacent floats exceeds 2π, so `rad -= TWO_PI` rounds straight back
/// to `rad` and the loop never ends -- a hard freeze, not a wrong answer. The
/// non-finite guard alone does not cover it: 1e9 is perfectly finite. Above
/// that magnitude a float no longer resolves the angle anyway, so the reduction
/// is only meaningful in the range where it is also exact.
inline float wrapRad(float rad) {
    // PROS_ERR_F is INFINITY, so a dead IMU lands here.
    if (!std::isfinite(rad)) return 0.0f;
    if (rad >= -PI && rad <= PI) return rad;   // the usual case
    rad = std::fmod(rad, TWO_PI);
    if (rad > PI) rad -= TWO_PI;
    else if (rad < -PI) rad += TWO_PI;
    return rad;
}

/// Wrap angle to [0, TWO_PI). One step, for the reason wrapRad gives.
inline float wrapRadPositive(float rad) {
    if (!std::isfinite(rad)) return 0.0f;
    if (rad >= 0.0f && rad < TWO_PI) return rad;
    rad = std::fmod(rad, TWO_PI);
    if (rad < 0.0f) rad += TWO_PI;
    // fmod of a tiny negative can round to TWO_PI exactly; the range is half open.
    return rad < TWO_PI ? rad : 0.0f;
}

/// Wrap degrees to [0, 360). One step, for the reason wrapRad gives.
inline float wrapDeg(float deg) {
    if (!std::isfinite(deg)) return 0.0f;
    if (deg >= 0.0f && deg < 360.0f) return deg;
    deg = std::fmod(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg < 360.0f ? deg : 0.0f;
}

/// Shortest signed angular difference from `from` to `to` (radians)
inline float angleDiffRad(float from, float to) {
    return wrapRad(to - from);
}

/// Shortest absolute angular distance (radians)
inline float angleDistRad(float from, float to) {
    return std::fabs(angleDiffRad(from, to));
}

// ============================================================================
// Unit conversions
// ============================================================================

inline constexpr float degToRad(float deg) { return deg * DEG_TO_RAD; }
inline constexpr float radToDeg(float rad) { return rad * RAD_TO_DEG; }

constexpr float MM_TO_INCH = 0.039370078740157f;
constexpr float INCH_TO_MM = 25.4f;
constexpr float INCH_TO_METER = 0.0254f;
constexpr float METER_TO_INCH = 39.370078740157f;

inline constexpr float mmToInch(float mm) { return mm * MM_TO_INCH; }
inline constexpr float inchToMm(float in) { return in * INCH_TO_MM; }
inline constexpr float inchToMeter(float in) { return in * INCH_TO_METER; }
inline constexpr float meterToInch(float m)  { return m * METER_TO_INCH; }

// ============================================================================
// General math utilities
// ============================================================================

/// Clamp value to [min, max]
template<typename T>
inline T clamp(T val, T min, T max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/// Sign of a value: +1, 0, or -1
template<typename T>
inline T sign(T val) {
    return (val > T(0)) ? T(1) : ((val < T(0)) ? T(-1) : T(0));
}

/// Approximate floating point equality
inline bool nearlyEqual(float a, float b, float epsilon = 1e-5f) {
    return std::fabs(a - b) < epsilon;
}

/// Linear interpolation
inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

/// Square of a value
template<typename T>
inline T sq(T val) { return val * val; }

/// Euclidean distance between two points
inline float distance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

/// Distance from a Pose to a point
inline float distanceToPoint(const Pose& pose, float x, float y) {
    return distance(pose.x, pose.y, x, y);
}

/// Bearing (absolute angle) from `from` to `to` in standard math radians
inline float bearingToPoint(const Pose& from, float toX, float toY) {
    return std::atan2(toY - from.y, toX - from.x);
}

// ============================================================================
// Odometry delta — decomposed motion for MCL per-axis noise
// ============================================================================

/// Decomposed odometry delta from a single update() tick.
/// dVert and dHoriz are independent sensor measurements (tracking wheels or
/// motor encoders), not trigonometrically decomposed from a fused (X,Y) pose.
/// This allows MCL to apply different noise models to each axis.
struct OdomDelta {
    float dVert{0.0f};    // pure vertical distance change (inches, forward+)
    float dHoriz{0.0f};   // pure horizontal distance change (inches, left+)
    float dTheta{0.0f};   // heading change (radians, CCW+)
};

// ============================================================================
// VEX / PROS coordinate convention conversions
// ============================================================================

/// Convert VEX heading (0 = forward, CW positive) to standard math radians
/// (0 = +X, CCW positive)
inline float vexToStdRad(float vexDeg) {
    float rad = degToRad(90.0f - vexDeg);
    return wrapRad(rad);
}

/// Convert standard math radians to VEX heading (degrees, 0 = forward, CW positive)
inline float stdRadToVexDeg(float stdRad) {
    float deg = 90.0f - radToDeg(stdRad);
    return wrapDeg(deg);
}

/// Convert robot-centric bearing to trig angle for field-frame math
/// Robot 0 = forward, 90 = left. Trig 0 = +X, +PI/2 = +Y.
inline float botHeadingToTrig(float botDeg) {
    // This one had no finite guard at all, so PROS_ERR_F froze it outright:
    // 90 - inf is -inf, and -inf + 360 is -inf.
    return wrapDeg(90.0f - botDeg);
}

// ============================================================================
// Field geometry constants (VRC 2025-26 "High Stakes" field)
// ============================================================================

namespace Field {
    // Field boundaries
    constexpr float FIELD_LENGTH = 144.0f;       // full field length (inches)
    constexpr float FIELD_WIDTH  = 144.0f;       // full field width (inches)
    constexpr float FIELD_HALF   = 72.0f;        // half field
    constexpr float FIELD_WALL_TO_WALL = 140.4f; // inner wall-to-wall
    constexpr float FIELD_HALF_WALL   = 70.2f;   // half of wall-to-wall

    // Field tile dimensions
    constexpr float TILE_SIZE = 24.0f;           // one foam tile (inches)

    // Obstacle lifetime (for dynamic obstacles in RCL)
    constexpr float MAX_OBSTACLE_DURATION_MS = 1e12f;

    // Scale field coordinate to screen pixel (for path preview)
    // Screen is typically 480x240 on V5 Brain
    inline float fieldToScreenX(float fieldX, float screenWidth = 480.0f) {
        return (fieldX + FIELD_HALF) * (screenWidth / FIELD_LENGTH);
    }
    inline float fieldToScreenY(float fieldY, float screenHeight = 240.0f) {
        return (FIELD_HALF - fieldY) * (screenHeight / FIELD_WIDTH);
    }
}  // namespace Field

}  // namespace chad
