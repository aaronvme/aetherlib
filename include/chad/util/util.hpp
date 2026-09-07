#pragma once

#include <cmath>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace chad {
constexpr float PI = M_PI;
constexpr float HALF_PI = M_PI / 2.0f;
constexpr float QUARTER_PI = M_PI / 4.0f;
constexpr float TWO_PI = 2.0f * M_PI;
constexpr float INV_PI = 1.0f / M_PI;
constexpr float RAD_TO_DEG = 180.0f * INV_PI;
constexpr float DEG_TO_RAD = M_PI / 180.0f;

constexpr int MAX_DISTANCE_SENSORS = 8;

struct Pose {
    float x{0.0f};
    float y{0.0f};
    float theta{0.0f};

    Pose operator+(const Pose& other) const {
        return {x + other.x, y + other.y, theta + other.theta};
    }
    Pose operator-(const Pose& other) const {
        return {x - other.x, y - other.y, theta - other.theta};
    }

    Pose operator*(float scalar) const {
        return {x * scalar, y * scalar, theta * scalar};
    }

    bool equals(const Pose& other, float linTol = 0.001f, float angTol = 0.0001f) const {
        return (std::fabs(x - other.x) < linTol) &&
               (std::fabs(y - other.y) < linTol) &&
               (std::fabs(theta - other.theta) < angTol);
    }
};

inline float wrapRad(float rad) {
    if (!std::isfinite(rad)) return 0.0f;
    if (rad >= -PI && rad <= PI) return rad;
    rad = std::fmod(rad, TWO_PI);
    if (rad > PI) rad -= TWO_PI;
    else if (rad < -PI) rad += TWO_PI;
    return rad;
}

inline float wrapRadPositive(float rad) {
    if (!std::isfinite(rad)) return 0.0f;
    if (rad >= 0.0f && rad < TWO_PI) return rad;
    rad = std::fmod(rad, TWO_PI);
    if (rad < 0.0f) rad += TWO_PI;

    return rad < TWO_PI ? rad : 0.0f;
}

inline float wrapDeg(float deg) {
    if (!std::isfinite(deg)) return 0.0f;
    if (deg >= 0.0f && deg < 360.0f) return deg;
    deg = std::fmod(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg < 360.0f ? deg : 0.0f;
}

inline float angleDiffRad(float from, float to) {
    return wrapRad(to - from);
}

inline float angleDistRad(float from, float to) {
    return std::fabs(angleDiffRad(from, to));
}

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

template<typename T>
inline T clamp(T val, T min, T max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

template<typename T>
inline T sign(T val) {
    return (val > T(0)) ? T(1) : ((val < T(0)) ? T(-1) : T(0));
}

inline bool nearlyEqual(float a, float b, float epsilon = 1e-5f) {
    return std::fabs(a - b) < epsilon;
}

inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

template<typename T>
inline T sq(T val) { return val * val; }

inline float distance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

inline float distanceToPoint(const Pose& pose, float x, float y) {
    return distance(pose.x, pose.y, x, y);
}

inline float bearingToPoint(const Pose& from, float toX, float toY) {
    return std::atan2(toY - from.y, toX - from.x);
}

struct OdomDelta {
    float dVert{0.0f};
    float dHoriz{0.0f};
    float dTheta{0.0f};
};

inline float vexToStdRad(float vexDeg) {
    float rad = degToRad(90.0f - vexDeg);
    return wrapRad(rad);
}

inline float stdRadToVexDeg(float stdRad) {
    float deg = 90.0f - radToDeg(stdRad);
    return wrapDeg(deg);
}

inline float botHeadingToTrig(float botDeg) {
    return wrapDeg(90.0f - botDeg);
}

namespace Field {
    constexpr float FIELD_LENGTH = 144.0f;
    constexpr float FIELD_WIDTH  = 144.0f;
    constexpr float FIELD_HALF   = 72.0f;
    constexpr float FIELD_WALL_TO_WALL = 140.4f;
    constexpr float FIELD_HALF_WALL   = 70.2f;

    constexpr float TILE_SIZE = 24.0f;

    constexpr float MAX_OBSTACLE_DURATION_MS = 1e12f;

    inline float fieldToScreenX(float fieldX, float screenWidth = 480.0f) {
        return (fieldX + FIELD_HALF) * (screenWidth / FIELD_LENGTH);
    }
    inline float fieldToScreenY(float fieldY, float screenHeight = 240.0f) {
        return (FIELD_HALF - fieldY) * (screenHeight / FIELD_WIDTH);
    }
}
}
