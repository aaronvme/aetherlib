#pragma once

#include <cmath>
#include <limits>

#include "chad/util/geometry.hpp"

namespace chad {
enum class WheelType {
    Omni,

    Traction,

    Mixed,
};

inline constexpr float defaultGripLimit(WheelType wheels) {
    switch (wheels) {
        case WheelType::Traction: return 200.0f;
        case WheelType::Mixed: return 110.0f;
        case WheelType::Omni: break;
    }
    return 60.0f;
}

struct ChassisSpeeds {
    float vx{0.0f};
    float vy{0.0f};
    float omega{0.0f};

    static ChassisSpeeds fromFieldRelative(float fieldVx, float fieldVy, float omega,
                                           const Rotation2d& heading) {
        const Translation2d v = Translation2d{fieldVx, fieldVy}.rotateBy(-heading);
        return {v.x, v.y, omega};
    }

    Translation2d toFieldRelative(const Rotation2d& heading) const {
        return Translation2d{vx, vy}.rotateBy(heading);
    }

    ChassisSpeeds discretize(float dt) const {
        if (!(dt > 0.0f)) return *this;
        const Pose target{vx * dt, vy * dt, omega * dt};
        const Twist2d twist = Pose{}.log(target);
        return {twist.dx / dt, twist.dy / dt, twist.dtheta / dt};
    }

    float norm() const { return std::sqrt(vx * vx + vy * vy); }

    ChassisSpeeds operator*(float s) const { return {vx * s, vy * s, omega * s}; }
};

struct WheelSpeeds {
    float left{0.0f};
    float right{0.0f};

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

    Twist2d toTwist(float leftDelta, float rightDelta) const {
        if (!(mTrackWidth > 0.0f)) return {0.5f * (leftDelta + rightDelta), 0.0f, 0.0f};
        return {0.5f * (leftDelta + rightDelta), 0.0f,
                (rightDelta - leftDelta) / mTrackWidth};
    }

private:
    float mTrackWidth;
};

struct DriveModel {
    float trackWidth{12.0f};
    float wheelDiameter{3.25f};

    float maxWheelSpeed{60.0f};
    WheelType wheels{WheelType::Omni};

    float gripLimitOverride{0.0f};

    float gripLimit() const {
        return gripLimitOverride > 0.0f ? gripLimitOverride : defaultGripLimit(wheels);
    }

    float maxSpeedForCurvature(float k) const {
        const float mag = std::fabs(k);
        if (!std::isfinite(mag) || mag < 1e-9f) return maxWheelSpeed;
        const float grip = std::sqrt(gripLimit() / mag);

        const float budget = maxWheelSpeed / (1.0f + 0.5f * mag * trackWidth);
        return std::fmin(grip, budget);
    }

    float maxCurvatureAtSpeed(float v) const {
        const float speed = std::fabs(v);
        if (speed < 1e-6f) return std::numeric_limits<float>::infinity();
        return gripLimit() / (speed * speed);
    }

    bool isFeasible(const ChassisSpeeds& speeds) const {
        return std::fabs(speeds.vx) + std::fabs(speeds.omega) * 0.5f * trackWidth <=
               maxWheelSpeed + 1e-4f;
    }

    float omegaBudgetAt(float v) const {
        if (!(trackWidth > 0.0f)) return 0.0f;
        const float slack = maxWheelSpeed - std::fabs(v);
        return slack > 0.0f ? 2.0f * slack / trackWidth : 0.0f;
    }
};
}
