// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include <cmath>
#include <cstdint>

#include "chad/util/geometry.hpp"

namespace chad {

/// Resizes every distance-sensor array in the library (RCL, MCL, ChassisConfig).
/// Changing it requires a full rebuild.
constexpr int MAX_DISTANCE_SENSORS = 8;

constexpr float MM_TO_INCH = 0.039370078740157f;
constexpr float INCH_TO_MM = 25.4f;
constexpr float INCH_TO_METER = 0.0254f;
constexpr float METER_TO_INCH = 39.370078740157f;

inline constexpr float mmToInch(float mm) { return mm * MM_TO_INCH; }
inline constexpr float inchToMm(float in) { return in * INCH_TO_MM; }
inline constexpr float inchToMeter(float in) { return in * INCH_TO_METER; }
inline constexpr float meterToInch(float m) { return m * METER_TO_INCH; }

template <typename T>
inline T clamp(T val, T min, T max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

template <typename T>
inline T sign(T val) {
    return (val > T(0)) ? T(1) : ((val < T(0)) ? T(-1) : T(0));
}

inline bool nearlyEqual(float a, float b, float epsilon = 1e-5f) {
    return std::fabs(a - b) < epsilon;
}

inline float lerp(float a, float b, float t) { return a + t * (b - a); }

template <typename T>
inline T sq(T val) {
    return val * val;
}

/// One odometry tick, decomposed per axis. dVert and dHoriz are independent
/// sensor measurements rather than projections of a fused (x, y), which is what
/// lets MCL apply a different noise model to each.
struct OdomDelta {
    float dVert{0.0f};    // inches, forward positive
    float dHoriz{0.0f};   // inches, left positive
    float dTheta{0.0f};   // radians, counter-clockwise positive
};

/// Robot-centric bearing (0 = forward, 90 = left) to a field-frame trig angle.
inline float botHeadingToTrig(float botDeg) {
    // Had no non-finite guard and a subtractive loop: PROS_ERR_F is INFINITY,
    // and 90 - INFINITY never climbs back above zero however much you add.
    return wrapDeg(90.0f - botDeg);
}

namespace Field {
constexpr float FIELD_LENGTH = 144.0f;
constexpr float FIELD_WIDTH = 144.0f;
constexpr float FIELD_HALF = 72.0f;
constexpr float FIELD_WALL_TO_WALL = 140.4f;
constexpr float FIELD_HALF_WALL = 70.2f;
constexpr float TILE_SIZE = 24.0f;

constexpr float MAX_OBSTACLE_DURATION_MS = 1e12f;

inline float fieldToScreenX(float fieldX, float screenWidth = 480.0f) {
    return (fieldX + FIELD_HALF) * (screenWidth / FIELD_LENGTH);
}
inline float fieldToScreenY(float fieldY, float screenHeight = 240.0f) {
    return (FIELD_HALF - fieldY) * (screenHeight / FIELD_WIDTH);
}
}  // namespace Field

}  // namespace chad
