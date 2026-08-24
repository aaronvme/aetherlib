// Design derived from WPILib's frc::TrajectoryGenerator and
// TrajectoryParameterizer (BSD-3-Clause), which in turn follow Sprunk 2008.
// See ATTRIBUTION.md.
#pragma once

#include <cstddef>
#include <vector>

#include "chad/movement_control/spline.hpp"
#include "chad/movement_control/trajectory.hpp"
#include "chad/movement_control/trajectory_constraint.hpp"

namespace chad {

struct TrajectoryConfig {
    float maxVelocity{0.0f};      ///< inches/s, must be finite and > 0
    float maxAcceleration{0.0f};  ///< inches/s^2, must be finite and > 0
    /// Speed at the first and last state, inches/s. Magnitudes: `reversed`
    /// decides the sign. Clamped to `maxVelocity`.
    float startVelocity{0.0f};
    float endVelocity{0.0f};
    /// Drive the path backwards. Waypoint headings still say which way the
    /// robot faces, so the resulting velocities come out negative.
    bool reversed{false};
    /// Ceiling on |curvature|, 1/inches; <= 0 (the default) disables it.
    ///
    /// Off by default because no value is safe to pick for you. Quintic auto
    /// tangents cusp on waypoint headings nobody would flag -- four waypoints at
    /// 45/0/90/0 degrees give |k| = 0.516, a 1.9 in radius on a 11.9 in robot --
    /// but a deliberately short tangent magnitude reaches 0.69 and a reversed
    /// clamped cubic 0.61 on paths that are exactly what was asked for. The
    /// cusp is not the tightest path here, so a magnitude ceiling cannot tell
    /// the two apart. Set it to whatever the tightest corner YOUR paths contain,
    /// and generation reports `CuspedPath` instead of crawling round a loop you
    /// did not draw.
    float maxCurvature{0.0f};
    /// Not owned. Every constraint must outlive the generate call.
    std::vector<const TrajectoryConstraint*> constraints{};
    ParameterizeTolerance tolerance{};
};

enum class TrajectoryError {
    None,
    NoWaypoints,
    /// Non-positive or non-finite max velocity or acceleration.
    BadConfig,
    TooManyWaypoints,
    /// Consecutive waypoints sharing a position but not a heading. Arc-length
    /// parameterization cannot express a turn in place.
    DegenerateWaypoints,
    /// The spline could not be sampled: a vanishing tangent, or subdivision
    /// that never came inside tolerance.
    MalformedSpline,
    /// Subdivision wanted more points than it is allowed to allocate. An
    /// overscaled tangent magnitude is the usual cause.
    PathTooDense,
    /// A waypoint pose was not finite. Guarded because a non-finite heading
    /// otherwise yields an identity rotation and a plausible-looking path.
    NonFiniteWaypoint,
    /// A constraint's acceleration window is empty, alone or combined with
    /// another's.
    InfeasibleConstraint,
    /// The profile is pinned at zero speed part-way along, so the rest of the
    /// path would take forever. A constraint that allows no speed does this.
    StalledProfile,
    /// The velocity/acceleration fixed point did not settle. A constraint whose
    /// acceleration limit depends on speed can do this; so can one that answers
    /// differently every time it is asked.
    ProfileDidNotConverge,
    /// A constraint cannot bound the thing it was added to bound, which is a
    /// configuration error rather than a permissive constraint. A
    /// `WheelVoltageConstraint` built on an unmeasured kA is the case this
    /// exists for: it would otherwise silently protect nothing.
    UnusableConstraint,
    /// Sampled |curvature| exceeded `TrajectoryConfig::maxCurvature`. Quintic
    /// auto-tangents cusp on waypoint headings that look perfectly reasonable,
    /// and a path that doubles back inside its own wheelbase is one the robot
    /// cannot drive, not one it has to drive slowly.
    CuspedPath,
};

/// On failure the trajectory is empty — never a truncated prefix, which would
/// look like a valid short path to a follower.
struct TrajectoryResult {
    Trajectory trajectory{};
    TrajectoryError error{TrajectoryError::None};

    bool ok() const { return error == TrajectoryError::None; }
};

/// Quintic chain through every waypoint pose, C2 at the knots. A single
/// waypoint (or several that collapse to one) yields a one-state trajectory of
/// zero duration rather than an error: there is nothing to drive.
TrajectoryResult generateTrajectory(const PathWaypoint* waypoints, size_t count,
                                    const TrajectoryConfig& config);
TrajectoryResult generateTrajectory(const std::vector<PathWaypoint>& waypoints,
                                    const TrajectoryConfig& config);
TrajectoryResult generateTrajectory(const std::vector<Pose>& waypoints,
                                    const TrajectoryConfig& config);

/// Clamped-cubic form: only the two end poses carry a heading, and the interior
/// tangents are solved for. Tangent magnitudes <= 0 are chosen automatically.
TrajectoryResult generateCubicTrajectory(const Pose& start,
                                         const std::vector<Translation2d>& interior,
                                         const Pose& end, const TrajectoryConfig& config,
                                         float startTangent = 0.0f, float endTangent = 0.0f);

/// The two-pass time parameterization on its own, for a path that is already
/// sampled. `points` must be in path order.
TrajectoryResult timeParameterize(const std::vector<PoseWithCurvature>& points,
                                  const TrajectoryConfig& config);

/// Velocity and acceleration ceilings taken from the drive base. Constraints are
/// left empty on purpose: they are non-owning, so the caller has to own them
/// (see DrivetrainConstraints).
inline TrajectoryConfig trajectoryLimits(const DriveModel& model,
                                        float speedFraction = 1.0f,
                                        float maxAccelInPerSec2 = 60.0f) {
    TrajectoryConfig config;
    config.maxVelocity = model.maxWheelSpeed * clamp(speedFraction, 0.0f, 1.0f);
    config.maxAcceleration = maxAccelInPerSec2;
    return config;
}

}  // namespace chad
