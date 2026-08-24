#include "chad/movement_control/trajectory.hpp"

#include <cmath>
#include <utility>

namespace chad {

TrajectoryState TrajectoryState::interpolate(const TrajectoryState& end, float frac) const {
    // Clamping first also removes the only way the flip below could recurse.
    if (!(frac > 0.0f)) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;

    const float newT = t + (end.t - t) * frac;
    const float dt = newT - t;
    if (dt < 0.0f) return end.interpolate(*this, 1.0f - frac);

    const bool reversing =
        velocity < 0.0f || (std::fabs(velocity) < 1e-9f && acceleration < 0.0f);
    const float ds =
        (velocity * dt + 0.5f * acceleration * dt * dt) * (reversing ? -1.0f : 1.0f);

    // The span is the arc the pair actually spans, matching what the time
    // parameterization measured, so `frac == 1` lands exactly on `end`. A
    // zero-length segment has no arc, so fall back to the time fraction.
    const Twist2d geodesic = pose.log(end.pose);
    const float span = std::sqrt(geodesic.dx * geodesic.dx + geodesic.dy * geodesic.dy);
    float arcFrac = span > 1e-9f ? ds / span : frac;
    if (!std::isfinite(arcFrac)) arcFrac = frac;
    if (arcFrac < 0.0f) arcFrac = 0.0f;
    if (arcFrac > 1.0f) arcFrac = 1.0f;

    TrajectoryState out{};
    out.t = newT;
    out.velocity = velocity + acceleration * dt;
    out.acceleration = acceleration;
    out.pose = arcFrac <= 0.0f   ? pose
               : arcFrac >= 1.0f ? end.pose
                                 : pose.exp(geodesic * arcFrac);
    out.curvature = curvature + (end.curvature - curvature) * arcFrac;
    return out;
}

Trajectory::Trajectory(std::vector<TrajectoryState> states) : mStates(std::move(states)) {
    mTotalTime = mStates.empty() ? 0.0f : mStates.back().t;
}

Pose Trajectory::initialPose() const {
    return mStates.empty() ? Pose{} : mStates.front().pose;
}

TrajectoryState Trajectory::sample(float t) const {
    if (mStates.empty()) return {};
    if (std::isnan(t)) return mStates.front();
    if (t <= mStates.front().t) return mStates.front();
    if (t >= mTotalTime) return mStates.back();

    size_t low = 1;
    size_t high = mStates.size() - 1;
    while (low != high) {
        const size_t mid = low + (high - low) / 2;
        if (mStates[mid].t < t) {
            low = mid + 1;
        } else {
            high = mid;
        }
    }

    const TrajectoryState& after = mStates[low];
    const TrajectoryState& before = mStates[low - 1];
    const float span = after.t - before.t;
    if (span < 1e-9f) return after;

    // Landing exactly on a state returns it verbatim. Interpolating from the
    // state behind would carry that segment's acceleration, and a state's
    // acceleration describes the segment it LEAVES, so the two differ by the
    // whole acceleration step at every boundary.
    if (t <= before.t) return before;
    if (t >= after.t) return after;

    return before.interpolate(after, (t - before.t) / span);
}

Trajectory Trajectory::transformBy(const Transform2d& transform) const {
    if (mStates.empty()) return {};

    const Pose first = mStates.front().pose;
    const Pose newFirst = first.transformBy(transform);

    std::vector<TrajectoryState> out = mStates;
    out.front().pose = newFirst;
    for (size_t i = 1; i < out.size(); i++) {
        out[i].pose = newFirst.transformBy(first.transformTo(mStates[i].pose));
    }
    return Trajectory(std::move(out));
}

Trajectory Trajectory::relativeTo(const Pose& origin) const {
    std::vector<TrajectoryState> out = mStates;
    for (TrajectoryState& state : out) state.pose = state.pose.relativeTo(origin);
    return Trajectory(std::move(out));
}

Trajectory Trajectory::rotatedHalfTurn() const { return relativeTo({0.0f, 0.0f, PI}); }

Trajectory Trajectory::mirrored(MirrorAxis axis) const {
    std::vector<TrajectoryState> out = mStates;
    for (TrajectoryState& state : out) {
        if (axis == MirrorAxis::X) {
            state.pose.y = -state.pose.y;
            state.pose.theta = wrapRad(-state.pose.theta);
        } else {
            state.pose.x = -state.pose.x;
            state.pose.theta = wrapRad(PI - state.pose.theta);
        }
        state.curvature = -state.curvature;
    }
    return Trajectory(std::move(out));
}

Trajectory Trajectory::concatenate(const Trajectory& other) const {
    if (mStates.empty()) return other;
    if (other.mStates.size() < 2) return *this;

    std::vector<TrajectoryState> out = mStates;
    out.reserve(mStates.size() + other.mStates.size() - 1);

    // Each state's acceleration describes the segment LEAVING it. The seam state
    // is this trajectory's last, whose stored value is the one that brought it to
    // a stop, so it has to take over the acceleration of the segment it now
    // leads into. Left alone, a sample just past a stop-to-stop join reads a
    // braking acceleration and a negative velocity on a forward path.
    out.back().acceleration = other.mStates[0].acceleration;

    for (size_t i = 1; i < other.mStates.size(); i++) {
        TrajectoryState state = other.mStates[i];
        state.t += mTotalTime;
        out.push_back(state);
    }
    return Trajectory(std::move(out));
}

}  // namespace chad
