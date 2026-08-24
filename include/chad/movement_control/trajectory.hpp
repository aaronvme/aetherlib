// Design derived from WPILib's frc::Trajectory (BSD-3-Clause).
// See ATTRIBUTION.md.
#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

#include "chad/util/geometry.hpp"

namespace chad {

/// Length of the constant-twist path from `a` to `b`: the arc, not the chord.
/// Equal to the chord for a straight segment, never below it, and at most pi/2
/// times it. `Pose::log` wraps the heading change to (-pi, pi], so this only
/// measures the arc a differential drive traces while |dtheta| < pi -- past
/// that it under-measures, and a caller sampling a path coarsely enough to turn
/// half a revolution between points will get a short answer.
inline float arcLengthBetween(const Pose& a, const Pose& b) {
    const Twist2d twist = a.log(b);
    return std::sqrt(twist.dx * twist.dx + twist.dy * twist.dy);
}

/// One point of a time-parameterized path. Inches, inches/s, inches/s^2,
/// radians, 1/inches. `velocity` is signed along the path: negative means the
/// robot drives backwards through a pose that still faces forwards.
struct TrajectoryState {
    float t{0.0f};
    float velocity{0.0f};
    float acceleration{0.0f};
    Pose pose{};
    float curvature{0.0f};

    /// Interpolate towards `end`. `frac` is a fraction of the *time* between the
    /// two states; the pose is then placed by arc length along the connecting
    /// arc, so a mid-segment sample lands on the path rather than on its chord.
    TrajectoryState interpolate(const TrajectoryState& end, float frac) const;
};

/// Which line a mirrored trajectory is reflected across. The field origin is the
/// field centre (see `Field` in util.hpp), so neither axis needs a dimension.
///
/// Reflection is only the alliance swap on a mirror-symmetric field. A VEX
/// field is normally 180-degree rotationally symmetric instead, so reach for
/// `Trajectory::rotatedHalfTurn()` unless you know otherwise.
enum class MirrorAxis {
    /// Reflect across y = 0.
    X,
    /// Reflect across x = 0.
    Y,
};

class Trajectory {
public:
    Trajectory() = default;
    explicit Trajectory(std::vector<TrajectoryState> states);

    bool isEmpty() const { return mStates.empty(); }
    float totalTime() const { return mTotalTime; }
    const std::vector<TrajectoryState>& states() const { return mStates; }

    /// Zero pose when empty; there is nothing to throw on an embedded target.
    Pose initialPose() const;

    /// State at `t` seconds from the start, clamped at both ends. O(log n) and
    /// allocation-free, so it is safe to call from a control loop.
    TrajectoryState sample(float t) const;

    /// Move the whole path so its first pose is `transform` away from where it
    /// was, keeping the rest rigid relative to it. Robot-relative to
    /// field-relative.
    Trajectory transformBy(const Transform2d& transform) const;

    /// Re-express every pose in `origin`'s frame. Field-relative to
    /// robot-relative.
    Trajectory relativeTo(const Pose& origin) const;

    /// Reflected path for the other alliance. A reflection reverses handedness,
    /// so curvature changes sign while its magnitude, arc length and the whole
    /// velocity profile are untouched.
    Trajectory mirrored(MirrorAxis axis = MirrorAxis::X) const;

    /// The other alliance's path on a 180-degree rotationally symmetric field,
    /// which is what a VEX field normally is — reach for this before
    /// `mirrored`. Handedness survives a rotation, so curvature keeps its sign.
    Trajectory rotatedHalfTurn() const;

    /// Append `other`, shifting its timestamps to start at this trajectory's
    /// end. Matching the two end poses is the caller's problem. `other`'s first
    /// state is dropped so no two states share a timestamp.
    Trajectory concatenate(const Trajectory& other) const;

private:
    std::vector<TrajectoryState> mStates;
    float mTotalTime{0.0f};
};

}  // namespace chad
