#include "chad/movement_control/trajectory_generator.hpp"

#include <cmath>
#include <utility>

namespace chad {
namespace {

/// Below this the two points are the same point, and the v^2 = v0^2 + 2 a ds
/// relation carries no information. Inches.
constexpr float DS_EPS = 1e-6f;
/// Convergence slack on the velocity/acceleration fixed point. WPILib uses 1e-6
/// in double; at float32 precision an acceleration of order 100 in/s^2 resolves
/// to about 1e-5, so a 1e-6 test can never be met and the loop spins forever.
constexpr float ACCEL_CONVERGE_EPS = 1e-3f;
/// Slack on "is this acceleration window empty". Same magnitude as the
/// convergence slack, and deliberately the same on both sides of the test, so a
/// window empty by less than the arithmetic can resolve is not a hard failure.
constexpr float ACCEL_FEASIBLE_EPS = 1e-3f;
constexpr float VEL_EPS = 1e-6f;
/// An interval entered at rest with an acceleration below this is not going
/// anywhere on any timescale a match has, so it is reported rather than emitted
/// with a duration in the hundreds of thousands of seconds.
constexpr float STALL_ACCEL_EPS = 1e-3f;
constexpr int MAX_INNER_ITERATIONS = 32;

struct ConstrainedState {
    PoseWithCurvature point{};
    float distance{0.0f};
    /// dk/ds at this point, 1/inches^2. A wheel's acceleration depends on it, so
    /// a constraint that never sees it cannot bound the wheel.
    float curvatureRate{0.0f};
    float maxVelocity{0.0f};
    float minAccel{0.0f};
    float maxAccel{0.0f};
};

/// Constant acceleration carrying |v| from `v0` to `v1` over arc length `ds`.
/// Factored rather than (v1^2 - v0^2)/(2 ds): squaring first cancels away all
/// but a few bits where the profile is nearly flat, while Sterbenz makes the
/// difference of two nearby floats exact.
float alongAccel(float v0, float v1, float ds) {
    if (!(std::fabs(ds) > DS_EPS)) return 0.0f;
    return (v1 - v0) * (v1 + v0) / (2.0f * ds);
}

bool finitePose(const Pose& pose) {
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta);
}

bool validConfig(const TrajectoryConfig& config) {
    return std::isfinite(config.maxVelocity) && config.maxVelocity > 0.0f &&
           std::isfinite(config.maxAcceleration) && config.maxAcceleration > 0.0f &&
           std::isfinite(config.startVelocity) && std::isfinite(config.endVelocity);
}

TrajectoryError mapSplineStatus(SplineStatus status) {
    switch (status) {
        case SplineStatus::Ok: return TrajectoryError::None;
        case SplineStatus::TooManyWaypoints: return TrajectoryError::TooManyWaypoints;
        case SplineStatus::NotEnoughWaypoints: return TrajectoryError::NoWaypoints;
        case SplineStatus::DegenerateWaypoints: return TrajectoryError::DegenerateWaypoints;
        case SplineStatus::TooManyPoints: return TrajectoryError::PathTooDense;
        default: break;
    }
    return TrajectoryError::MalformedSpline;
}

Pose flipHeading(const Pose& pose) { return {pose.x, pose.y, wrapRad(pose.theta + PI)}; }

TrajectoryError enforceAccelLimits(const TrajectoryConfig& config, ConstrainedState& state) {
    const float factor = config.reversed ? -1.0f : 1.0f;
    for (const TrajectoryConstraint* constraint : config.constraints) {
        if (constraint == nullptr) continue;
        const AccelerationLimits limits = constraint->accelerationLimits(
            state.point.pose, state.point.curvature, state.maxVelocity * factor,
            state.curvatureRate);
        if (limits.min > limits.max + ACCEL_FEASIBLE_EPS) {
            return TrajectoryError::InfeasibleConstraint;
        }
        state.minAccel = std::fmax(state.minAccel, config.reversed ? -limits.max : limits.min);
        state.maxAccel = std::fmin(state.maxAccel, config.reversed ? -limits.min : limits.max);
    }
    // Two constraints can each be satisfiable and still leave no overlap. WPILib
    // only checks them one at a time and would run on with min > max.
    if (state.minAccel > state.maxAccel + ACCEL_FEASIBLE_EPS) {
        return TrajectoryError::InfeasibleConstraint;
    }
    return TrajectoryError::None;
}

/// Narrow `out` by every constraint's window at one interval endpoint, in the
/// along-path sign convention the profile uses. An empty window is not an error
/// here as it is pointwise: the dk/ds term scales with v^2, so an interval no
/// acceleration can pay for is one to drive slower, and narrowing is what sends
/// the fixed point looking for that speed.
void endpointAccelLimits(const TrajectoryConfig& config, const PoseWithCurvature& point,
                         float speed, float curvatureRate, AccelerationLimits& out) {
    const float factor = config.reversed ? -1.0f : 1.0f;
    for (const TrajectoryConstraint* constraint : config.constraints) {
        if (constraint == nullptr) continue;
        const AccelerationLimits limits = constraint->accelerationLimits(
            point.pose, point.curvature, speed * factor, curvatureRate);
        out.min = std::fmax(out.min, config.reversed ? -limits.max : limits.min);
        out.max = std::fmin(out.max, config.reversed ? -limits.min : limits.max);
    }
}

/// A constraint sees the dk/ds term as speed^2*(b/2)*rate, so at an endpoint
/// that is stopped no rate expresses the ratio change the interval still makes.
/// Pricing such an end a hair above zero recovers the term exactly and costs
/// kV*1e-3 volts on the wheel speed; leaving it out cost 0.62 V on the final
/// braking interval of an 80 in path.
constexpr float ENDPOINT_SPEED_FLOOR = 1e-3f;

/// Narrow `out` by the window at one end of an interval, priced so that the
/// wheel-acceleration term is the change a wheel's speed ACTUALLY makes across
/// the interval: c_far*(v1 - v0) + v_near*(b/2)*dk. That term belongs to the
/// interval, and a central difference at either node reproduces it only where
/// dk/ds is itself constant -- everywhere else the node value understates it,
/// which is the whole of the voltage the rail used to be overshot by. `dk` is
/// far minus near.
void intervalEndpoint(const TrajectoryConfig& config, const PoseWithCurvature& point,
                      float vHere, float vOther, float dk, float ds,
                      AccelerationLimits& out) {
    const float used = std::fmax(std::fabs(vHere), ENDPOINT_SPEED_FLOOR);
    const float span = std::fabs(vHere) + std::fabs(vOther);
    const float rate =
        (ds > DS_EPS) ? std::fabs(vOther) * span * dk / (2.0f * ds * used * used) : 0.0f;
    endpointAccelLimits(config, point, used, rate, out);
}

/// Window on the constant along-path acceleration of one interval, narrowed at
/// both of its ends. v(t) is linear on an interval of constant acceleration, so
/// the two ends bracket every demand inside it.
AccelerationLimits intervalAccelLimits(const TrajectoryConfig& config,
                                       const ConstrainedState& nearEnd,
                                       const ConstrainedState& farEnd, float ds) {
    const float dk = farEnd.point.curvature - nearEnd.point.curvature;
    const float vN = nearEnd.maxVelocity;
    const float vF = farEnd.maxVelocity;
    AccelerationLimits out{};
    intervalEndpoint(config, farEnd.point, vF, vN, dk, ds, out);
    intervalEndpoint(config, nearEnd.point, vN, vF, dk, ds, out);
    return out;
}

TrajectoryResult singleStateTrajectory(const PoseWithCurvature& point) {
    TrajectoryResult result{};
    std::vector<TrajectoryState> states(1);
    states[0].pose = point.pose;
    states[0].curvature = point.curvature;
    result.trajectory = Trajectory(std::move(states));
    return result;
}

TrajectoryResult parameterizeSampledPath(const std::vector<PoseWithCurvature>& points,
                                         const TrajectoryConfig& config) {
    TrajectoryResult result{};
    if (points.empty()) {
        result.error = TrajectoryError::NoWaypoints;
        return result;
    }
    if (!validConfig(config)) {
        result.error = TrajectoryError::BadConfig;
        return result;
    }
    for (const TrajectoryConstraint* constraint : config.constraints) {
        if (constraint != nullptr && !constraint->isUsable()) {
            result.error = TrajectoryError::UnusableConstraint;
            return result;
        }
    }
    if (config.maxCurvature > 0.0f) {
        for (const PoseWithCurvature& point : points) {
            if (!(std::fabs(point.curvature) <= config.maxCurvature)) {
                result.error = TrajectoryError::CuspedPath;
                return result;
            }
        }
    }
    if (points.size() == 1) return singleStateTrajectory(points[0]);

    const float vMax = config.maxVelocity;
    const float aMax = config.maxAcceleration;
    const float vStart = clamp(std::fabs(config.startVelocity), 0.0f, vMax);
    const float vEnd = clamp(std::fabs(config.endVelocity), 0.0f, vMax);

    // Sized up front so the pointers below stay valid across the whole sweep.
    std::vector<ConstrainedState> states(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        states[i].point = points[i];
        // Arc length, not chord: the chord systematically understates how far
        // the robot has to drive, and TrajectoryState::interpolate measures the
        // same arc, so the two agree exactly at every state boundary.
        states[i].distance =
            (i == 0) ? 0.0f
                     : states[i - 1].distance +
                           arcLengthBetween(points[i - 1].pose, points[i].pose);
    }
    // dk/ds by central difference. The spline could differentiate exactly, but
    // timeParameterize() takes points from a caller who may not have one, so
    // both entry points get the same estimate -- and the same estimate a
    // follower differencing the emitted profile would make.
    for (size_t i = 0; i < points.size(); i++) {
        const size_t lo = (i > 0) ? i - 1 : i;
        const size_t hi = (i + 1 < points.size()) ? i + 1 : i;
        const float span = states[hi].distance - states[lo].distance;
        states[i].curvatureRate =
            (span > DS_EPS) ? (points[hi].curvature - points[lo].curvature) / span : 0.0f;
    }

    ConstrainedState boundary{};
    boundary.point = points.front();
    boundary.maxVelocity = vStart;
    boundary.minAccel = -aMax;
    boundary.maxAccel = aMax;

    ConstrainedState* pred = &boundary;
    for (size_t i = 0; i < points.size(); i++) {
        ConstrainedState& state = states[i];
        const float ds = state.distance - pred->distance;

        // An acceleration limit may itself depend on speed, so the pair has to
        // be iterated to a fixed point rather than solved once.
        for (int iteration = 0;; iteration++) {
            const float reachableSq =
                pred->maxVelocity * pred->maxVelocity + 2.0f * pred->maxAccel * ds;
            state.maxVelocity = std::fmin(vMax, std::sqrt(std::fmax(0.0f, reachableSq)));
            state.minAccel = -aMax;
            state.maxAccel = aMax;

            for (const TrajectoryConstraint* constraint : config.constraints) {
                if (constraint == nullptr) continue;
                const float limit =
                    constraint->maxVelocity(state.point.pose, state.point.curvature,
                                            state.maxVelocity, state.curvatureRate);
                // fmax against zero also turns a NaN limit into a hard stop
                // rather than letting fmin ignore it.
                state.maxVelocity = std::fmin(state.maxVelocity, std::fmax(0.0f, limit));
            }

            const TrajectoryError error = enforceAccelLimits(config, state);
            if (error != TrajectoryError::None) {
                result.error = error;
                return result;
            }

            if (ds < DS_EPS) break;

            const float cap =
                std::fmin(state.maxAccel, intervalAccelLimits(config, *pred, state, ds).max);

            const float actual = alongAccel(pred->maxVelocity, state.maxVelocity, ds);
            if (cap < actual - ACCEL_CONVERGE_EPS) {
                // The only lever this loop has is lowering the predecessor's
                // ceiling. Where that is already as low as the successor's,
                // `actual` can only be over it by float noise -- (v1^2 - v0^2)
                // cancels to about eps*v^2/(2 ds), which passes 1e-3 at 53 in/s
                // over a 0.19 in step -- and iterating changes nothing. Without
                // this a 48 in STRAIGHT LINE burned all 32 iterations and came
                // back ProfileDidNotConverge as soon as the sampling was
                // refined to maxDx = 0.25 in.
                if (!(cap < pred->maxAccel)) break;
                pred->maxAccel = cap;
            } else {
                // Anything below the predecessor's floor is the backward pass's
                // problem, not this one's.
                if (actual > pred->minAccel) pred->maxAccel = actual;
                break;
            }

            if (iteration + 1 >= MAX_INNER_ITERATIONS) {
                result.error = TrajectoryError::ProfileDidNotConverge;
                return result;
            }
        }
        pred = &state;
    }

    boundary.point = points.back();
    boundary.distance = states.back().distance;
    boundary.maxVelocity = vEnd;
    boundary.minAccel = -aMax;
    boundary.maxAccel = aMax;

    ConstrainedState* succ = &boundary;
    for (size_t i = points.size(); i-- > 0;) {
        ConstrainedState& state = states[i];
        const float ds = state.distance - succ->distance;  // <= 0

        for (int iteration = 0;; iteration++) {
            const float reachableSq =
                succ->maxVelocity * succ->maxVelocity + 2.0f * succ->minAccel * ds;
            const float reachable = std::sqrt(std::fmax(0.0f, reachableSq));
            const bool braked = reachable < state.maxVelocity;
            if (braked) {
                state.maxVelocity = reachable;

                // The window left behind by the forward pass was evaluated at
                // the higher speed reachable from the start. Narrowing that
                // stale pair against this slower speed can leave min above max
                // and report an infeasible constraint for a profile that is
                // perfectly feasible.
                state.minAccel = -aMax;
                state.maxAccel = aMax;

                const TrajectoryError error = enforceAccelLimits(config, state);
                if (error != TrajectoryError::None) {
                    result.error = error;
                    return result;
                }
            }

            if (ds > -DS_EPS) break;

            const float floorAccel = std::fmax(
                state.minAccel, intervalAccelLimits(config, state, *succ, -ds).min);

            const float actual = alongAccel(succ->maxVelocity, state.maxVelocity, ds);
            if (floorAccel > actual + ACCEL_CONVERGE_EPS) {
                if (!(floorAccel > succ->minAccel)) break;
                succ->minAccel = floorAccel;
            } else if (!braked) {
                break;
            } else {
                succ->minAccel = actual;
                break;
            }

            if (iteration + 1 >= MAX_INNER_ITERATIONS) {
                result.error = TrajectoryError::ProfileDidNotConverge;
                return result;
            }
        }
        succ = &state;
    }

    std::vector<TrajectoryState> out;
    out.reserve(states.size());
    float time = 0.0f;
    float distance = 0.0f;
    float velocity = 0.0f;

    for (size_t i = 0; i < states.size(); i++) {
        const ConstrainedState& state = states[i];
        const float ds = state.distance - distance;

        const float sum = velocity + state.maxVelocity;
        float accel = 0.0f;
        float dt = 0.0f;
        if (ds > DS_EPS) accel = alongAccel(velocity, state.maxVelocity, ds);
        if (i > 0) {
            out[i - 1].acceleration = config.reversed ? -accel : accel;
            const bool pinned =
                velocity <= VEL_EPS && std::fabs(accel) <= STALL_ACCEL_EPS;
            if (ds > DS_EPS && (pinned || !(sum > VEL_EPS))) {
                // Stopped, not allowed to accelerate, and still short of the
                // end: the rest of the path takes forever.
                result.error = TrajectoryError::StalledProfile;
                return result;
            }
            // ds*2/sum, not (v1 - v0)/accel: dividing by an acceleration that is
            // itself a cancelled difference amplifies its noise by v/(v1 - v0),
            // which reached 0.2% of dt on a nearly flat stretch and put the
            // reconstructed wheel demand back over the rail.
            if (sum > VEL_EPS) dt = 2.0f * ds / sum;
        }

        velocity = state.maxVelocity;
        distance = state.distance;
        time += dt;

        TrajectoryState emitted{};
        emitted.t = time;
        emitted.velocity = config.reversed ? -velocity : velocity;
        emitted.acceleration = config.reversed ? -accel : accel;
        emitted.pose = state.point.pose;
        emitted.curvature = state.point.curvature;
        out.push_back(emitted);
    }

    result.trajectory = Trajectory(std::move(out));
    return result;
}

}  // namespace

TrajectoryResult timeParameterize(const std::vector<PoseWithCurvature>& points,
                                  const TrajectoryConfig& config) {
    return parameterizeSampledPath(points, config);
}

TrajectoryResult generateTrajectory(const PathWaypoint* waypoints, size_t count,
                                    const TrajectoryConfig& config) {
    TrajectoryResult result{};
    if (waypoints == nullptr || count == 0) {
        result.error = TrajectoryError::NoWaypoints;
        return result;
    }
    if (!validConfig(config)) {
        result.error = TrajectoryError::BadConfig;
        return result;
    }
    for (size_t i = 0; i < count; i++) {
        if (!finitePose(waypoints[i].pose) || !std::isfinite(waypoints[i].tangentMagnitude)) {
            result.error = TrajectoryError::NonFiniteWaypoint;
            return result;
        }
    }
    if (count > MAX_PATH_WAYPOINTS) {
        result.error = TrajectoryError::TooManyWaypoints;
        return result;
    }

    PathWaypoint kept[MAX_PATH_WAYPOINTS];
    size_t n = 0;
    kept[n++] = waypoints[0];
    for (size_t i = 1; i < count; i++) {
        const Pose& previous = kept[n - 1].pose;
        const Pose& next = waypoints[i].pose;
        if (previous.translation().distanceTo(next.translation()) < WAYPOINT_COINCIDENT_EPS) {
            if (std::fabs(angleDiffRad(previous.theta, next.theta)) > 1e-3f) {
                result.error = TrajectoryError::DegenerateWaypoints;
                return result;
            }
            continue;
        }
        kept[n++] = waypoints[i];
    }

    if (n == 1) return singleStateTrajectory({kept[0].pose, 0.0f});

    if (config.reversed) {
        for (size_t i = 0; i < n; i++) kept[i].pose = flipHeading(kept[i].pose);
    }

    std::vector<QuinticHermiteSpline> splines;
    const SplineStatus built = quinticSplinesFromWaypoints(kept, n, splines);
    if (built != SplineStatus::Ok) {
        result.error = mapSplineStatus(built);
        return result;
    }
    optimizeQuinticCurvature(splines);

    std::vector<PoseWithCurvature> points;
    const SplineStatus sampled = parameterizeChain(splines, points, config.tolerance);
    if (sampled != SplineStatus::Ok) {
        result.error = mapSplineStatus(sampled);
        return result;
    }

    if (config.reversed) {
        for (PoseWithCurvature& point : points) {
            point.pose = flipHeading(point.pose);
            point.curvature = -point.curvature;
        }
    }
    return parameterizeSampledPath(points, config);
}

TrajectoryResult generateTrajectory(const std::vector<PathWaypoint>& waypoints,
                                    const TrajectoryConfig& config) {
    return generateTrajectory(waypoints.data(), waypoints.size(), config);
}

TrajectoryResult generateTrajectory(const std::vector<Pose>& waypoints,
                                    const TrajectoryConfig& config) {
    TrajectoryResult result{};
    if (waypoints.size() > MAX_PATH_WAYPOINTS) {
        result.error = TrajectoryError::TooManyWaypoints;
        return result;
    }
    PathWaypoint converted[MAX_PATH_WAYPOINTS];
    for (size_t i = 0; i < waypoints.size(); i++) converted[i] = {waypoints[i], 0.0f};
    return generateTrajectory(converted, waypoints.size(), config);
}

TrajectoryResult generateCubicTrajectory(const Pose& start,
                                         const std::vector<Translation2d>& interior,
                                         const Pose& end, const TrajectoryConfig& config,
                                         float startTangent, float endTangent) {
    TrajectoryResult result{};
    if (!validConfig(config)) {
        result.error = TrajectoryError::BadConfig;
        return result;
    }
    if (!finitePose(start) || !finitePose(end)) {
        result.error = TrajectoryError::NonFiniteWaypoint;
        return result;
    }
    for (const Translation2d& point : interior) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            result.error = TrajectoryError::NonFiniteWaypoint;
            return result;
        }
    }
    if (interior.size() + 2 > MAX_PATH_WAYPOINTS) {
        result.error = TrajectoryError::TooManyWaypoints;
        return result;
    }

    const Pose splineStart = config.reversed ? flipHeading(start) : start;
    const Pose splineEnd = config.reversed ? flipHeading(end) : end;

    std::vector<CubicHermiteSpline> splines;
    const SplineStatus built =
        cubicSplinesFromWaypoints(splineStart, interior.data(), interior.size(), splineEnd,
                                  splines, startTangent, endTangent);
    if (built != SplineStatus::Ok) {
        result.error = mapSplineStatus(built);
        return result;
    }

    std::vector<PoseWithCurvature> points;
    const SplineStatus sampled = parameterizeChain(splines, points, config.tolerance);
    if (sampled != SplineStatus::Ok) {
        result.error = mapSplineStatus(sampled);
        return result;
    }

    if (config.reversed) {
        for (PoseWithCurvature& point : points) {
            point.pose = flipHeading(point.pose);
            point.curvature = -point.curvature;
        }
    }
    return parameterizeSampledPath(points, config);
}

}  // namespace chad
