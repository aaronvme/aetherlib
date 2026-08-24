// Design derived from WPILib's frc::TrajectoryConstraint family
// (BSD-3-Clause). See ATTRIBUTION.md.
#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include "chad/util/geometry.hpp"
#include "chad/util/kinematics.hpp"
#include "chad/util/util.hpp"

namespace chad {

/// Signed acceleration window along the path, inches/s^2. `min` is the hardest
/// braking allowed and `max` the hardest acceleration. `min > max` means the
/// constraint cannot be met at all, which is a generation failure, not a clamp.
struct AccelerationLimits {
    float min{-std::numeric_limits<float>::infinity()};
    float max{std::numeric_limits<float>::infinity()};

    bool feasible() const { return min <= max; }
};

/// Speed and acceleration limits as a function of where on the path the robot
/// is. Inches, seconds, radians throughout: `curvature` is signed 1/inches and
/// `speed` is signed along the path.
///
/// `curvatureRate` is dk/ds in 1/inches^2, differentiated along the path in the
/// direction the points are ordered. It has no default: a wheel accelerates at
/// `a*c + v^2*(b/2)*dk/ds`, and assuming that term away is how this file used to
/// emit states demanding 14.8 V from a 12 V battery.
class TrajectoryConstraint {
public:
    virtual ~TrajectoryConstraint() = default;

    /// Non-negative ceiling on |speed|. Infinity means "no opinion".
    virtual float maxVelocity(const Pose& pose, float curvature, float speed,
                              float curvatureRate) const = 0;

    virtual AccelerationLimits accelerationLimits(const Pose& pose, float curvature,
                                                  float speed,
                                                  float curvatureRate) const = 0;

    /// False if this constraint cannot bound the thing it was added to bound --
    /// a configuration error, not a permissive constraint. Generation reports it
    /// rather than running unprotected.
    virtual bool isUsable() const { return true; }
};

class MaxVelocityConstraint : public TrajectoryConstraint {
public:
    explicit MaxVelocityConstraint(float limit) : mLimit(limit) {}

    float maxVelocity(const Pose&, float, float, float) const override { return mLimit; }
    AccelerationLimits accelerationLimits(const Pose&, float, float, float) const override {
        return {};
    }

private:
    float mLimit;
};

/// Keeps the lateral acceleration a corner demands inside what the wheels hold:
/// v <= sqrt(aLat / |k|). Pair it with `DifferentialKinematicsConstraint` to
/// reproduce `DriveModel::maxSpeedForCurvature`, which takes the smaller.
class CentripetalConstraint : public TrajectoryConstraint {
public:
    explicit CentripetalConstraint(float maxLateralAccel)
        : mMaxLateralAccel(maxLateralAccel) {}
    explicit CentripetalConstraint(const DriveModel& model)
        : mMaxLateralAccel(model.gripLimit()) {}

    float maxVelocity(const Pose&, float curvature, float, float) const override {
        const float k = std::fabs(curvature);
        if (!std::isfinite(k)) return 0.0f;
        if (k < 1e-9f) return std::numeric_limits<float>::infinity();
        if (!(mMaxLateralAccel > 0.0f)) return 0.0f;
        return std::sqrt(mMaxLateralAccel / k);
    }

    AccelerationLimits accelerationLimits(const Pose&, float, float, float) const override {
        return {};
    }

private:
    float mMaxLateralAccel;
};

/// Keeps both wheels inside their speed budget. The outer wheel of a turn runs
/// at v(1 + |k|b/2), so a tight corner caps v even where grip would not.
class DifferentialKinematicsConstraint : public TrajectoryConstraint {
public:
    DifferentialKinematicsConstraint(const DifferentialKinematics& kinematics,
                                     float maxWheelSpeed)
        : mKinematics(kinematics), mMaxWheelSpeed(maxWheelSpeed) {}

    float maxVelocity(const Pose&, float curvature, float, float) const override {
        if (!(mMaxWheelSpeed > 0.0f)) return 0.0f;
        const float omega = mMaxWheelSpeed * curvature;
        if (!std::isfinite(omega)) return 0.0f;
        // Probing at the wheel budget makes desaturate return exactly the
        // chassis speed at which the outer wheel saturates, so the answer is a
        // pure function of curvature instead of of the candidate speed.
        WheelSpeeds wheels = mKinematics.toWheelSpeeds({mMaxWheelSpeed, 0.0f, omega});
        wheels.desaturate(mMaxWheelSpeed);
        return std::fabs(mKinematics.toChassisSpeeds(wheels).vx);
    }

    AccelerationLimits accelerationLimits(const Pose&, float, float, float) const override {
        return {};
    }

private:
    DifferentialKinematics mKinematics;
    float mMaxWheelSpeed;
};

/// Per-wheel voltage model V = kS*sign(v) + kV*v + kA*a, with v in inches/s and
/// a in inches/s^2.
struct WheelVoltageModel {
    float kS{0.0f};
    float kV{0.0f};
    float kA{0.0f};
    float maxVoltage{12.0f};

    /// `FeedforwardGains` is fitted in metres and lives behind a PROS-dependent
    /// header, so gains arrive as scalars. kV and kA carry a per-length unit and
    /// scale by metres-per-inch; kS is volts either way.
    static WheelVoltageModel fromMetreGains(float kS, float kVPerMps, float kAPerMps2,
                                            float maxVoltage = 12.0f) {
        return {kS, kVPerMps * INCH_TO_METER, kAPerMps2 * INCH_TO_METER, maxVoltage};
    }

    /// Duck-typed so `FeedforwardGains` can be handed over without this header
    /// having to include the PROS-dependent one it lives in.
    template <typename Gains>
    static WheelVoltageModel fromFeedforwardGains(const Gains& gains,
                                                  float maxVoltage = 12.0f) {
        return fromMetreGains(gains.ksStraight, gains.kV, gains.kaStraight, maxVoltage);
    }

    float maxWheelAccelAt(float v) const {
        if (!(kA > 0.0f)) return std::numeric_limits<float>::infinity();
        return (maxVoltage - kS * sign(v) - kV * v) / kA;
    }

    float minWheelAccelAt(float v) const {
        if (!(kA > 0.0f)) return -std::numeric_limits<float>::infinity();
        return (-maxVoltage - kS * sign(v) - kV * v) / kA;
    }
};

/// Turns the per-wheel voltage budget into a chassis speed and acceleration
/// window.
///
/// A wheel at ratio c = 1 +/- k*b/2 runs at v*c, and its acceleration is
/// d/dt(v*c) = a*c + v*|v|*(+/-b/2)*dk/ds -- NOT a*c. The second term is what a
/// wheel does while the path bends further underneath it; it is zero almost
/// nowhere on a spline, and it reached 178 in/s^2 (2.8 V at kA = 0.8) on an
/// ordinary arc. It carries v*|v| rather than v^2 because ds/dt is |v|.
class WheelVoltageConstraint : public TrajectoryConstraint {
public:
    WheelVoltageConstraint(const WheelVoltageModel& model,
                           const DifferentialKinematics& kinematics)
        : mModel(model), mKinematics(kinematics) {}

    /// Fastest |speed| whose CONSTANT-speed demand both wheels can still pay
    /// for. Without this the dk/ds term alone can eat the whole budget, and
    /// `accelerationLimits` would have to report an empty window -- a hard
    /// generation failure for a path whose only problem is the speed it is being
    /// driven at. Keeping a = 0 affordable also guarantees the window below is
    /// non-empty at every speed this allows.
    float maxVelocity(const Pose&, float curvature, float,
                      float curvatureRate) const override {
        if (!std::isfinite(curvature) || !std::isfinite(curvatureRate)) return 0.0f;
        if (std::fabs(mModel.kS) > mModel.maxVoltage) return 0.0f;

        const float half = 0.5f * mKinematics.trackWidth();
        float cap = std::numeric_limits<float>::infinity();
        for (const float side : {-1.0f, 1.0f}) {
            // At |speed| = u the wheel demands kS*sign(c) + kV*c*u + kA*d*u^2,
            // the same magnitude whichever way the path is driven.
            const float ratio = 1.0f + side * half * curvature;
            const float quad = mModel.kA * side * half * curvatureRate;
            const float lin = mModel.kV * ratio;
            const float con = mModel.kS * sign(ratio);
            cap = std::fmin(cap, smallestPositiveRoot(quad, lin, con - mModel.maxVoltage));
            cap = std::fmin(cap, smallestPositiveRoot(quad, lin, con + mModel.maxVoltage));
        }
        return cap;
    }

    /// Each wheel's budget bounds the chassis by  (B - t)/c <= a <= (A - t)/c,
    /// where t is that wheel's dk/ds term -- with the inequality REVERSED when c
    /// is negative, which is what turning about a point inside the wheelbase
    /// does. Both wheels bind in both directions, so the window is the
    /// intersection of the two.
    AccelerationLimits accelerationLimits(const Pose&, float curvature, float speed,
                                          float curvatureRate) const override {
        if (!std::isfinite(curvature) || !std::isfinite(speed) ||
            !std::isfinite(curvatureRate)) {
            return {};
        }

        const float half = 0.5f * mKinematics.trackWidth();
        AccelerationLimits out;
        for (const float side : {-1.0f, 1.0f}) {
            const float ratio = 1.0f + side * half * curvature;
            const float term = speed * std::fabs(speed) * side * half * curvatureRate;
            narrowFor(speed * ratio, ratio, term, out);
        }
        return out;
    }

    /// kA is what ties voltage to acceleration. Without it this constraint
    /// bounds acceleration by nothing at all, and on a base whose free speed
    /// already fits inside the battery it bounds speed by nothing either -- so
    /// it is reported rather than left looking like protection.
    bool isUsable() const override { return mModel.kA > 0.0f; }

private:
    /// Smallest u > 0 solving a*u^2 + b*u + c == 0, or infinity if there is
    /// none. Paired so the small root comes out of c/q rather than a difference
    /// of two nearly equal numbers, which is the ordinary case here.
    static float smallestPositiveRoot(float a, float b, float c) {
        const float none = std::numeric_limits<float>::infinity();
        if (std::fabs(a) < 1e-12f) {
            if (std::fabs(b) < 1e-12f) return none;
            const float u = -c / b;
            return u > 0.0f ? u : none;
        }
        const float disc = b * b - 4.0f * a * c;
        if (!(disc >= 0.0f)) return none;
        const float root = std::sqrt(disc);
        const float q = -0.5f * (b + (b >= 0.0f ? root : -root));
        float best = none;
        const float u1 = q / a;
        if (u1 > 0.0f) best = std::fmin(best, u1);
        if (std::fabs(q) > 1e-30f) {
            const float u2 = c / q;
            if (u2 > 0.0f) best = std::fmin(best, u2);
        }
        return best;
    }

    void narrowFor(float wheelSpeed, float ratio, float term,
                   AccelerationLimits& out) const {
        const float hi = mModel.maxWheelAccelAt(wheelSpeed) - term;
        const float lo = mModel.minWheelAccelAt(wheelSpeed) - term;
        if (std::fabs(ratio) < 1e-6f) {
            // The wheel is standing still -- turn radius exactly half the track
            // width -- so its whole demand is `term` and no chassis acceleration
            // changes it. That bounds nothing unless it is already over budget,
            // in which case the state is genuinely undeliverable.
            if (lo > 0.0f || hi < 0.0f) {
                out.min = 1.0f;
                out.max = -1.0f;
            }
            return;
        }
        out.min = std::fmax(out.min, (ratio > 0.0f) ? lo / ratio : hi / ratio);
        out.max = std::fmin(out.max, (ratio > 0.0f) ? hi / ratio : lo / ratio);
    }

    WheelVoltageModel mModel;
    DifferentialKinematics mKinematics;
};

/// Applies `inner` only inside an axis-aligned box. A null `inner`, or being
/// outside the box, means no limit. The pointer is not owned: the wrapped
/// constraint has to outlive generation.
class RectangularRegionConstraint : public TrajectoryConstraint {
public:
    RectangularRegionConstraint(const Translation2d& cornerA, const Translation2d& cornerB,
                                const TrajectoryConstraint* inner)
        : mMin{std::fmin(cornerA.x, cornerB.x), std::fmin(cornerA.y, cornerB.y)},
          mMax{std::fmax(cornerA.x, cornerB.x), std::fmax(cornerA.y, cornerB.y)},
          mInner(inner) {}

    bool contains(const Translation2d& p) const {
        return p.x >= mMin.x && p.x <= mMax.x && p.y >= mMin.y && p.y <= mMax.y;
    }

    float maxVelocity(const Pose& pose, float curvature, float speed,
                      float curvatureRate) const override {
        if (mInner == nullptr || !contains(pose.translation())) {
            return std::numeric_limits<float>::infinity();
        }
        return mInner->maxVelocity(pose, curvature, speed, curvatureRate);
    }

    AccelerationLimits accelerationLimits(const Pose& pose, float curvature, float speed,
                                          float curvatureRate) const override {
        if (mInner == nullptr || !contains(pose.translation())) return {};
        return mInner->accelerationLimits(pose, curvature, speed, curvatureRate);
    }

    /// A region that wraps nothing is inert on purpose, not misconfigured.
    bool isUsable() const override { return mInner == nullptr || mInner->isUsable(); }

private:
    Translation2d mMin;
    Translation2d mMax;
    const TrajectoryConstraint* mInner;
};

/// Applies `inner` only inside a (possibly rotated) ellipse. Semi-axes are in
/// inches, measured along the ellipse's own axes.
class EllipticalRegionConstraint : public TrajectoryConstraint {
public:
    EllipticalRegionConstraint(const Translation2d& center, float semiX, float semiY,
                               const Rotation2d& rotation, const TrajectoryConstraint* inner)
        : mCenter(center), mSemiX(semiX), mSemiY(semiY), mRotation(rotation), mInner(inner) {}

    bool contains(const Translation2d& p) const {
        if (!(mSemiX > 0.0f) || !(mSemiY > 0.0f)) return false;
        const Translation2d local = (p - mCenter).rotateBy(-mRotation);
        const float u = local.x / mSemiX;
        const float v = local.y / mSemiY;
        return u * u + v * v <= 1.0f;
    }

    float maxVelocity(const Pose& pose, float curvature, float speed,
                      float curvatureRate) const override {
        if (mInner == nullptr || !contains(pose.translation())) {
            return std::numeric_limits<float>::infinity();
        }
        return mInner->maxVelocity(pose, curvature, speed, curvatureRate);
    }

    AccelerationLimits accelerationLimits(const Pose& pose, float curvature, float speed,
                                          float curvatureRate) const override {
        if (mInner == nullptr || !contains(pose.translation())) return {};
        return mInner->accelerationLimits(pose, curvature, speed, curvatureRate);
    }

    /// A region that wraps nothing is inert on purpose, not misconfigured.
    bool isUsable() const override { return mInner == nullptr || mInner->isUsable(); }

private:
    Translation2d mCenter;
    float mSemiX;
    float mSemiY;
    Rotation2d mRotation;
    const TrajectoryConstraint* mInner;
};

/// The two constraints every path on this robot needs: the lateral grip its
/// wheels can hold, and the per-side wheel-speed budget. Owns them, so it has
/// to outlive the generate call it is handed to.
class DrivetrainConstraints {
public:
    explicit DrivetrainConstraints(const DriveModel& model)
        : mCentripetal(model),
          mKinematics(DifferentialKinematics(model.trackWidth), model.maxWheelSpeed) {}

    DrivetrainConstraints(const DrivetrainConstraints&) = delete;
    DrivetrainConstraints& operator=(const DrivetrainConstraints&) = delete;

    std::vector<const TrajectoryConstraint*> list() const {
        return {&mCentripetal, &mKinematics};
    }

    const CentripetalConstraint& centripetal() const { return mCentripetal; }
    const DifferentialKinematicsConstraint& kinematics() const { return mKinematics; }

private:
    CentripetalConstraint mCentripetal;
    DifferentialKinematicsConstraint mKinematics;
};

}  // namespace chad
