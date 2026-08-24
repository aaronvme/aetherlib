#pragma once

#include <cmath>

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

inline constexpr float degToRad(float deg) { return deg * DEG_TO_RAD; }
inline constexpr float radToDeg(float rad) { return rad * RAD_TO_DEG; }

/// Wrap to [-PI, PI]. Non-finite input returns 0, because dead PROS sensors
/// report PROS_ERR_F == INFINITY.
///
/// Reduced in one step rather than by repeated subtraction. Past 2^27 rad the
/// float32 ULP exceeds 4*PI, so `rad -= TWO_PI` rounds straight back to `rad`
/// and a subtractive loop never terminates at all -- on a brain with no
/// watchdog that is a hang. Below that it merely takes up to a million turns.
inline float wrapRad(float rad) {
    if (!std::isfinite(rad)) return 0.0f;
    return std::remainder(rad, TWO_PI);
}

/// Wrap to [0, TWO_PI).
inline float wrapRadPositive(float rad) {
    if (!std::isfinite(rad)) return 0.0f;
    float r = std::fmod(rad, TWO_PI);
    if (r < 0.0f) r += TWO_PI;
    // A tiny negative r can round up to exactly TWO_PI on the add.
    return (r < TWO_PI) ? r : 0.0f;
}

/// Wrap to [0, 360).
inline float wrapDeg(float deg) {
    if (!std::isfinite(deg)) return 0.0f;
    float d = std::fmod(deg, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return (d < 360.0f) ? d : 0.0f;
}

/// Shortest signed difference from `from` to `to`, radians.
inline float angleDiffRad(float from, float to) { return wrapRad(to - from); }

inline float angleDistRad(float from, float to) {
    return std::fabs(angleDiffRad(from, to));
}

/// VEX heading (0 = forward, clockwise positive) to standard math radians
/// (0 = +X, counter-clockwise positive).
inline float vexToStdRad(float vexDeg) { return wrapRad(degToRad(90.0f - vexDeg)); }

inline float stdRadToVexDeg(float stdRad) { return wrapDeg(90.0f - radToDeg(stdRad)); }

/// sin(h)/h, with the series taken over where the quotient loses precision.
inline float sinc(float h) {
    const float h2 = h * h;
    if (h2 < 1e-6f) return 1.0f - h2 / 6.0f;
    return std::sin(h) / h;
}

/// h/tan(h), likewise.
inline float hCotH(float h) {
    const float h2 = h * h;
    if (h2 < 1e-6f) return 1.0f - h2 / 3.0f;
    return h / std::tan(h);
}

// ============================================================================

/// A heading held as its own cosine and sine, so composing rotations is exact
/// and never needs wrapping.
struct Rotation2d {
    float cos{1.0f};
    float sin{0.0f};

    static Rotation2d fromRadians(float rad) {
        if (!std::isfinite(rad)) return {};
        return {std::cos(rad), std::sin(rad)};
    }
    static Rotation2d fromDegrees(float deg) { return fromRadians(degToRad(deg)); }
    static Rotation2d fromVexDegrees(float deg) { return fromRadians(vexToStdRad(deg)); }

    /// Direction of (x, y), normalised. A zero vector has no direction, so it
    /// yields identity rather than a NaN.
    static Rotation2d fromVector(float x, float y) {
        const float n = std::sqrt(x * x + y * y);
        if (!std::isfinite(n) || n < 1e-12f) return {};
        return {x / n, y / n};
    }

    float radians() const { return std::atan2(sin, cos); }
    float degrees() const { return radToDeg(radians()); }
    float vexDegrees() const { return stdRadToVexDeg(radians()); }

    Rotation2d operator+(const Rotation2d& o) const {
        return {cos * o.cos - sin * o.sin, cos * o.sin + sin * o.cos};
    }
    Rotation2d operator-(const Rotation2d& o) const { return *this + (-o); }
    Rotation2d operator-() const { return {cos, -sin}; }
    Rotation2d operator*(float scalar) const { return fromRadians(radians() * scalar); }

    Rotation2d& operator+=(const Rotation2d& o) { return *this = *this + o; }

    /// Signed shortest rotation from this to `o`, radians.
    float distanceTo(const Rotation2d& o) const { return (o - *this).radians(); }

    Rotation2d interpolate(const Rotation2d& o, float t) const {
        return fromRadians(radians() + distanceTo(o) * t);
    }

    bool nearly(const Rotation2d& o, float tolRad = 1e-4f) const {
        return std::fabs(distanceTo(o)) <= tolRad;
    }
};

struct Translation2d {
    float x{0.0f};
    float y{0.0f};

    static Translation2d fromPolar(float radius, const Rotation2d& angle) {
        return {radius * angle.cos, radius * angle.sin};
    }

    float norm() const { return std::sqrt(x * x + y * y); }
    float radians() const { return std::atan2(y, x); }
    Rotation2d angle() const { return Rotation2d::fromVector(x, y); }

    Translation2d rotateBy(const Rotation2d& r) const {
        return {x * r.cos - y * r.sin, x * r.sin + y * r.cos};
    }

    Translation2d operator+(const Translation2d& o) const { return {x + o.x, y + o.y}; }
    Translation2d operator-(const Translation2d& o) const { return {x - o.x, y - o.y}; }
    Translation2d operator-() const { return {-x, -y}; }
    Translation2d operator*(float s) const { return {x * s, y * s}; }
    Translation2d operator/(float s) const { return {x / s, y / s}; }

    float dot(const Translation2d& o) const { return x * o.x + y * o.y; }
    float cross(const Translation2d& o) const { return x * o.y - y * o.x; }
    float distanceTo(const Translation2d& o) const { return (o - *this).norm(); }

    Translation2d interpolate(const Translation2d& o, float t) const {
        return {x + (o.x - x) * t, y + (o.y - y) * t};
    }
};

/// A movement measured in the frame it starts in: drive `dx` forward and `dy`
/// left while turning `dtheta`, all at once along a constant-curvature arc.
struct Twist2d {
    float dx{0.0f};
    float dy{0.0f};
    float dtheta{0.0f};

    Twist2d operator*(float s) const { return {dx * s, dy * s, dtheta * s}; }
    Twist2d operator+(const Twist2d& o) const {
        return {dx + o.dx, dy + o.dy, dtheta + o.dtheta};
    }
    Twist2d operator-() const { return {-dx, -dy, -dtheta}; }

    bool isZero(float linTol = 1e-9f, float angTol = 1e-9f) const {
        return std::fabs(dx) < linTol && std::fabs(dy) < linTol &&
               std::fabs(dtheta) < angTol;
    }
};

struct Pose;

/// Rigid transform: rotate, then translate, in the frame it is applied to.
struct Transform2d {
    Translation2d translation{};
    Rotation2d rotation{};

    Transform2d inverse() const {
        return {(-translation).rotateBy(-rotation), -rotation};
    }

    /// Apply `o` after this one.
    Transform2d operator+(const Transform2d& o) const {
        return {translation + o.translation.rotateBy(rotation), rotation + o.rotation};
    }

    Transform2d operator*(float s) const { return {translation * s, rotation * s}; }
};

// ============================================================================

/// Field pose. Position in inches, heading in standard math radians.
struct Pose {
    float x{0.0f};
    float y{0.0f};
    float theta{0.0f};

    static Pose fromVex(float x, float y, float thetaVexDeg) {
        return {x, y, vexToStdRad(thetaVexDeg)};
    }

    Translation2d translation() const { return {x, y}; }
    Rotation2d rotation() const { return Rotation2d::fromRadians(theta); }
    float thetaVexDeg() const { return stdRadToVexDeg(theta); }

    Pose operator+(const Pose& o) const { return {x + o.x, y + o.y, theta + o.theta}; }
    Pose operator-(const Pose& o) const { return {x - o.x, y - o.y, theta - o.theta}; }
    Pose operator*(float scalar) const {
        return {x * scalar, y * scalar, theta * scalar};
    }

    bool equals(const Pose& o, float linTol = 0.001f, float angTol = 0.0001f) const {
        return std::fabs(x - o.x) < linTol && std::fabs(y - o.y) < linTol &&
               std::fabs(angleDiffRad(theta, o.theta)) < angTol;
    }

    Pose transformBy(const Transform2d& t) const {
        const Rotation2d r = rotation();
        const Translation2d p = translation() + t.translation.rotateBy(r);
        return {p.x, p.y, (r + t.rotation).radians()};
    }

    /// This pose expressed in `origin`'s frame.
    Pose relativeTo(const Pose& origin) const {
        const Transform2d t = origin.transformTo(*this);
        return {t.translation.x, t.translation.y, t.rotation.radians()};
    }

    /// The transform that takes this pose to `end`.
    Transform2d transformTo(const Pose& end) const {
        const Rotation2d r = rotation();
        return {(end.translation() - translation()).rotateBy(-r), end.rotation() - r};
    }

    /// Integrate a robot-frame twist exactly along its constant-curvature arc.
    /// Straight-line integration of the same twist is what leaves odometry
    /// short on the inside of every turn.
    Pose exp(const Twist2d& t) const {
        const float h = 0.5f * t.dtheta;
        const float sc = sinc(h);
        const float s = std::cos(h) * sc;   // sin(dtheta)/dtheta
        const float c = std::sin(h) * sc;   // (1 - cos(dtheta))/dtheta
        const Transform2d step{{t.dx * s - t.dy * c, t.dx * c + t.dy * s},
                               Rotation2d::fromRadians(t.dtheta)};
        return transformBy(step);
    }

    /// The twist that `exp` would integrate to reach `end` from here.
    Twist2d log(const Pose& end) const {
        const Transform2d t = transformTo(end);
        const float dtheta = t.rotation.radians();
        const float h = 0.5f * dtheta;
        const float k = hCotH(h);
        const Translation2d d =
            t.translation.rotateBy(Rotation2d::fromVector(k, -h)) *
            std::sqrt(k * k + h * h);
        return {d.x, d.y, dtheta};
    }

    /// Constant-twist interpolation, so intermediate poses stay on one arc.
    Pose interpolate(const Pose& end, float t) const {
        if (t <= 0.0f) return *this;
        if (t >= 1.0f) return end;
        return exp(log(end) * t);
    }

    float distanceTo(const Pose& o) const { return translation().distanceTo(o.translation()); }
    float distanceTo(float px, float py) const {
        const float dx = px - x, dy = py - y;
        return std::sqrt(dx * dx + dy * dy);
    }

    /// Absolute field angle from here to a point, standard math radians.
    float bearingTo(float px, float py) const { return std::atan2(py - y, px - x); }
};

inline float distance(float x1, float y1, float x2, float y2) {
    const float dx = x2 - x1, dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

inline float distanceToPoint(const Pose& pose, float x, float y) {
    return pose.distanceTo(x, y);
}

inline float bearingToPoint(const Pose& from, float toX, float toY) {
    return from.bearingTo(toX, toY);
}

}  // namespace chad
