#pragma once

#include <cmath>
#include <limits>

#include "chad/util/geometry.hpp"

namespace chad {

/// What the wheels do when the chassis is pushed sideways. It decides how much
/// lateral acceleration a turn may ask for before the robot slides instead of
/// tracking, so it belongs in the robot config and not in a controller default.
enum class WheelType {
    /// Rollers offer almost no sideways grip: the base scrubs out of tight
    /// corners and carries a lateral slip velocity no differential model sees.
    Omni,
    /// Sideways grip comparable to forward grip. Holds a corner, resists being
    /// pushed, and pays for it in scrub torque when turning.
    Traction,
    /// Traction on one axle, omni on the other.
    Mixed,
};

/// Lateral acceleration a base can hold before it slides, inches/s^2.
/// Measured as the speed a circle of known radius can be driven at:
/// a = v^2 / r. Conservative on purpose — this bounds tracking authority.
inline constexpr float defaultGripLimit(WheelType wheels) {
    switch (wheels) {
        case WheelType::Traction: return 200.0f;
        case WheelType::Mixed: return 110.0f;
        case WheelType::Omni: break;
    }
    return 60.0f;
}

/// Robot-frame chassis velocity. `vy` is zero for anything a differential drive
/// can command; a nonzero value is the sideways slip an omni base actually
/// carries through a corner, which is a disturbance rather than an input.
struct ChassisSpeeds {
    float vx{0.0f};      // inches/s, forward positive
    float vy{0.0f};      // inches/s, left positive
    float omega{0.0f};   // rad/s, counter-clockwise positive

    static ChassisSpeeds fromFieldRelative(float fieldVx, float fieldVy, float omega,
                                           const Rotation2d& heading) {
        const Translation2d v = Translation2d{fieldVx, fieldVy}.rotateBy(-heading);
        return {v.x, v.y, omega};
    }

    Translation2d toFieldRelative(const Rotation2d& heading) const {
        return Translation2d{vx, vy}.rotateBy(heading);
    }

    /// Correct for holding this velocity over a whole tick. Commanding vx and
    /// omega together traces an arc, not the straight line a naive Euler step
    /// assumes, and the difference shows up as a curvature-dependent drift at
    /// the end of every motion.
    ChassisSpeeds discretize(float dt) const {
        if (!(dt > 0.0f)) return *this;
        const Pose target{vx * dt, vy * dt, omega * dt};
        const Twist2d twist = Pose{}.log(target);
        return {twist.dx / dt, twist.dy / dt, twist.dtheta / dt};
    }

    float norm() const { return std::sqrt(vx * vx + vy * vy); }

    ChassisSpeeds operator*(float s) const { return {vx * s, vy * s, omega * s}; }
};

/// Per-side wheel speeds, inches/s.
struct WheelSpeeds {
    float left{0.0f};
    float right{0.0f};

    /// Scale both sides by one factor so neither exceeds `maxSpeed`. Clipping
    /// the sides independently would change the ratio between them, which is
    /// the commanded curvature. Returns the factor applied.
    float desaturate(float maxSpeed) {
        const float peak = std::fmax(std::fabs(left), std::fabs(right));
        if (!(maxSpeed > 0.0f) || !std::isfinite(peak) || peak <= maxSpeed) return 1.0f;
        const float scale = maxSpeed / peak;
        left *= scale;
        right *= scale;
        return scale;
    }

    WheelSpeeds operator-() const { return {-left, -right}; }
};

/// Differential drive velocity kinematics. Track width in inches.
class DifferentialKinematics {
public:
    explicit DifferentialKinematics(float trackWidth) : mTrackWidth(trackWidth) {}

    float trackWidth() const { return mTrackWidth; }
    void setTrackWidth(float trackWidth) { mTrackWidth = trackWidth; }

    WheelSpeeds toWheelSpeeds(const ChassisSpeeds& speeds) const {
        const float half = 0.5f * mTrackWidth * speeds.omega;
        return {speeds.vx - half, speeds.vx + half};
    }

    ChassisSpeeds toChassisSpeeds(const WheelSpeeds& wheels) const {
        if (!(mTrackWidth > 0.0f)) return {0.5f * (wheels.left + wheels.right), 0.0f, 0.0f};
        return {0.5f * (wheels.left + wheels.right), 0.0f,
                (wheels.right - wheels.left) / mTrackWidth};
    }

    /// Wheel distance deltas over one tick to the robot-frame twist they trace.
    /// Feeding this to Pose::exp is exact for a constant-curvature tick.
    Twist2d toTwist(float leftDelta, float rightDelta) const {
        if (!(mTrackWidth > 0.0f)) return {0.5f * (leftDelta + rightDelta), 0.0f, 0.0f};
        return {0.5f * (leftDelta + rightDelta), 0.0f,
                (rightDelta - leftDelta) / mTrackWidth};
    }

private:
    float mTrackWidth;
};

/// The physical envelope of a drive base: what it can deliver, and what the
/// wheels will hold. Everything is inches, seconds and radians.
struct DriveModel {
    float trackWidth{12.0f};
    float wheelDiameter{3.25f};
    /// Free wheel speed, inches/s. Not a limit the controller may exceed.
    float maxWheelSpeed{60.0f};
    WheelType wheels{WheelType::Omni};
    /// Override for the lateral grip limit, inches/s^2. Zero derives it from
    /// `wheels`; measure it and set it once you have a number.
    float gripLimitOverride{0.0f};

    float gripLimit() const {
        return gripLimitOverride > 0.0f ? gripLimitOverride : defaultGripLimit(wheels);
    }

    /// Fastest speed that holds a path of signed curvature `k` (1/inches).
    float maxSpeedForCurvature(float k) const {
        const float mag = std::fabs(k);
        if (!std::isfinite(mag) || mag < 1e-9f) return maxWheelSpeed;
        const float grip = std::sqrt(gripLimit() / mag);
        // A turn also has to fit the wheel-speed budget: the outer wheel runs
        // at v(1 + |k|b/2), so grip alone is not the binding limit up close.
        const float budget = maxWheelSpeed / (1.0f + 0.5f * mag * trackWidth);
        return std::fmin(grip, budget);
    }

    float maxCurvatureAtSpeed(float v) const {
        const float speed = std::fabs(v);
        if (speed < 1e-6f) return std::numeric_limits<float>::infinity();
        return gripLimit() / (speed * speed);
    }

    /// True if both wheels can deliver this twist: |v| + |omega|b/2 <= vmax.
    bool isFeasible(const ChassisSpeeds& speeds) const {
        return std::fabs(speeds.vx) + std::fabs(speeds.omega) * 0.5f * trackWidth <=
               maxWheelSpeed + 1e-4f;
    }

    /// Turn rate still available at forward speed `v`, rad/s.
    float omegaBudgetAt(float v) const {
        if (!(trackWidth > 0.0f)) return 0.0f;
        const float slack = maxWheelSpeed - std::fabs(v);
        return slack > 0.0f ? 2.0f * slack / trackWidth : 0.0f;
    }
};

}  // namespace chad
