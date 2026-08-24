#include "chad/movement_control/trajectory.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include "chad/movement_control/trajectory_constraint.hpp"
#include "chad/movement_control/trajectory_generator.hpp"
#include "chad_test.hpp"

using namespace chad;

namespace {

constexpr float INF = std::numeric_limits<float>::infinity();

TrajectoryConfig baseConfig(float vMax = 40.0f, float aMax = 50.0f) {
    TrajectoryConfig config{};
    config.maxVelocity = vMax;
    config.maxAcceleration = aMax;
    config.tolerance = {0.5f, 0.05f, 0.0872f, 20};
    return config;
}

std::vector<Pose> curvyPath() {
    return {{0.0f, 0.0f, 0.0f},
            {30.0f, 24.0f, HALF_PI},
            {6.0f, 54.0f, PI},
            {-36.0f, 30.0f, -HALF_PI}};
}

double totalArcLength(const Trajectory& traj) {
    const std::vector<TrajectoryState>& states = traj.states();
    double sum = 0.0;
    for (size_t i = 0; i + 1 < states.size(); i++) {
        sum += arcLengthBetween(states[i].pose, states[i + 1].pose);
    }
    return sum;
}

/// Re-derive the achieved profile from the emitted states and hold it against
/// every constraint. This is the real assertion about the two-pass profile.
/// dk/ds at state `i`, by the same central difference the generator makes.
float curvatureRateAt(const std::vector<TrajectoryState>& states, size_t i) {
    const size_t lo = (i > 0) ? i - 1 : i;
    const size_t hi = (i + 1 < states.size()) ? i + 1 : i;
    float span = 0.0f;
    for (size_t j = lo; j < hi; j++) {
        span += arcLengthBetween(states[j].pose, states[j + 1].pose);
    }
    if (!(span > 1e-6f)) return 0.0f;
    return (states[hi].curvature - states[lo].curvature) / span;
}

void recheck(const Trajectory& traj, const TrajectoryConfig& config) {
    const std::vector<TrajectoryState>& states = traj.states();
    CHECK(!states.empty());
    if (states.empty()) return;

    for (size_t i = 0; i < states.size(); i++) {
        const TrajectoryState& state = states[i];
        CHECK_FINITE(state.t);
        CHECK_FINITE(state.velocity);
        CHECK_FINITE(state.acceleration);
        CHECK_FINITE(state.pose.x);
        CHECK_FINITE(state.pose.y);
        CHECK_FINITE(state.pose.theta);
        CHECK_FINITE(state.curvature);

        const float speed = std::fabs(state.velocity);
        CHECK(speed <= config.maxVelocity + 1e-3f);
        if (config.reversed) {
            CHECK(state.velocity <= 1e-3f);
        } else {
            CHECK(state.velocity >= -1e-3f);
        }
        for (const TrajectoryConstraint* constraint : config.constraints) {
            const float limit = constraint->maxVelocity(
                state.pose, state.curvature, speed, curvatureRateAt(states, i));
            CHECK(speed <= limit + 1e-3f + 1e-3f * std::fabs(limit));
        }
    }

    const float slack = 0.05f + 1e-3f * config.maxAcceleration;
    for (size_t i = 0; i + 1 < states.size(); i++) {
        CHECK(states[i + 1].t >= states[i].t - 1e-6f);

        const float ds = arcLengthBetween(states[i].pose, states[i + 1].pose);
        if (ds < 1e-3f) continue;
        const float v0 = std::fabs(states[i].velocity);
        const float v1 = std::fabs(states[i + 1].velocity);
        const float along = (v1 * v1 - v0 * v0) / (2.0f * ds);
        CHECK(std::fabs(along) <= config.maxAcceleration + slack);

        // What the two passes actually guarantee: the forward sweep bounds the
        // segment above by the window at its START, the backward sweep bounds it
        // below by the window at its END. Taking the union of the two endpoints
        // instead -- fmin of the mins, fmax of the maxes -- is never tighter
        // than either one, so it cannot fail and asserts nothing.
        for (const TrajectoryConstraint* constraint : config.constraints) {
            const AccelerationLimits a = constraint->accelerationLimits(
                states[i].pose, states[i].curvature, states[i].velocity,
                curvatureRateAt(states, i));
            const AccelerationLimits b = constraint->accelerationLimits(
                states[i + 1].pose, states[i + 1].curvature, states[i + 1].velocity,
                curvatureRateAt(states, i + 1));
            const float hi = config.reversed ? -a.min : a.max;
            const float lo = config.reversed ? -b.max : b.min;
            CHECK(along <= hi + slack);
            CHECK(along >= lo - slack);
        }
    }
}

TrajectoryState bruteSample(const Trajectory& traj, float t) {
    const std::vector<TrajectoryState>& states = traj.states();
    if (states.empty()) return {};
    if (t <= states.front().t) return states.front();
    if (t >= traj.totalTime()) return states.back();
    size_t i = 1;
    while (i + 1 < states.size() && states[i].t < t) i++;
    const float span = states[i].t - states[i - 1].t;
    if (span < 1e-9f) return states[i];
    return states[i - 1].interpolate(states[i], (t - states[i - 1].t) / span);
}

class ImpossibleAccel : public TrajectoryConstraint {
public:
    float maxVelocity(const Pose&, float, float, float) const override { return INF; }
    AccelerationLimits accelerationLimits(const Pose&, float, float, float) const override {
        return {10.0f, -10.0f};
    }
};

class AccelFloor : public TrajectoryConstraint {
public:
    float maxVelocity(const Pose&, float, float, float) const override { return INF; }
    AccelerationLimits accelerationLimits(const Pose&, float, float, float) const override {
        return {5.0f, INF};
    }
};

class AccelCeiling : public TrajectoryConstraint {
public:
    float maxVelocity(const Pose&, float, float, float) const override { return INF; }
    AccelerationLimits accelerationLimits(const Pose&, float, float, float) const override {
        return {-INF, 1.0f};
    }
};

/// Two poses on a circle of radius R, `sweep` radians apart, driven at a
/// constant speed. Exact, so it isolates the sampler from the spline.
Trajectory circleTrajectory(float radius, float sweep, float speed) {
    std::vector<TrajectoryState> states(2);
    states[0] = {0.0f, speed, 0.0f, {radius, 0.0f, HALF_PI}, 1.0f / radius};
    states[1] = {radius * sweep / speed,
                 speed,
                 0.0f,
                 {radius * std::cos(sweep), radius * std::sin(sweep), HALF_PI + sweep},
                 1.0f / radius};
    return Trajectory(states);
}

}  // namespace

CHAD_TEST(trajectory, empty_trajectory_is_inert) {
    const Trajectory empty;
    CHECK(empty.isEmpty());
    CHECK_NEAR(empty.totalTime(), 0.0, 1e-9);
    CHECK(empty.states().empty());
    CHECK(empty.initialPose().equals({0.0f, 0.0f, 0.0f}));

    const TrajectoryState sampled = empty.sample(1.0f);
    CHECK_NEAR(sampled.t, 0.0, 1e-9);
    CHECK_NEAR(sampled.velocity, 0.0, 1e-9);

    CHECK(empty.transformBy({}).isEmpty());
    CHECK(empty.mirrored().isEmpty());
    CHECK(empty.relativeTo({}).isEmpty());
}

CHAD_TEST(trajectory, single_waypoint_yields_one_zero_length_state) {
    const std::vector<Pose> path = {{12.0f, -8.0f, 0.5f}};
    const TrajectoryResult result = generateTrajectory(path, baseConfig());
    CHECK(result.ok());
    if (!result.ok()) return;
    CHECK(!result.trajectory.isEmpty());
    CHECK_EQ(result.trajectory.states().size(), static_cast<size_t>(1));
    CHECK_NEAR(result.trajectory.totalTime(), 0.0, 1e-9);
    CHECK(result.trajectory.initialPose().equals({12.0f, -8.0f, 0.5f}));
    CHECK_NEAR(result.trajectory.sample(5.0f).velocity, 0.0, 1e-9);
}

CHAD_TEST(trajectory, no_waypoints_is_an_error) {
    const std::vector<Pose> none;
    const TrajectoryResult result = generateTrajectory(none, baseConfig());
    CHECK(result.error == TrajectoryError::NoWaypoints);
    CHECK(result.trajectory.isEmpty());
}

CHAD_TEST(trajectory, coincident_waypoints_collapse) {
    const std::vector<Pose> path = {
        {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {50.0f, 0.0f, 0.0f}, {50.0f, 0.0f, 0.0f}};
    const TrajectoryResult result = generateTrajectory(path, baseConfig());
    CHECK(result.ok());
    if (!result.ok()) return;
    CHECK(result.trajectory.states().size() > 2);
    CHECK_NEAR(totalArcLength(result.trajectory), 50.0, 0.05);
}

CHAD_TEST(trajectory, coincident_waypoints_that_disagree_on_heading_are_an_error) {
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, HALF_PI}};
    const TrajectoryResult result = generateTrajectory(path, baseConfig());
    CHECK(result.error == TrajectoryError::DegenerateWaypoints);
    CHECK(result.trajectory.isEmpty());
}

CHAD_TEST(trajectory, all_waypoints_coincident_collapses_to_one_state) {
    const std::vector<Pose> path = {{5.0f, 5.0f, 1.0f}, {5.0f, 5.0f, 1.0f}};
    const TrajectoryResult result = generateTrajectory(path, baseConfig());
    CHECK(result.ok());
    if (!result.ok()) return;
    CHECK_EQ(result.trajectory.states().size(), static_cast<size_t>(1));
}

CHAD_TEST(trajectory, bad_config_is_rejected_before_anything_else) {
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {50.0f, 0.0f, 0.0f}};

    TrajectoryConfig zeroVelocity = baseConfig();
    zeroVelocity.maxVelocity = 0.0f;
    CHECK(generateTrajectory(path, zeroVelocity).error == TrajectoryError::BadConfig);

    TrajectoryConfig zeroAccel = baseConfig();
    zeroAccel.maxAcceleration = 0.0f;
    CHECK(generateTrajectory(path, zeroAccel).error == TrajectoryError::BadConfig);

    TrajectoryConfig negative = baseConfig();
    negative.maxVelocity = -10.0f;
    CHECK(generateTrajectory(path, negative).error == TrajectoryError::BadConfig);

    TrajectoryConfig nan = baseConfig();
    nan.maxAcceleration = std::numeric_limits<float>::quiet_NaN();
    CHECK(generateTrajectory(path, nan).error == TrajectoryError::BadConfig);

    TrajectoryConfig infinite = baseConfig();
    infinite.maxVelocity = INF;
    CHECK(generateTrajectory(path, infinite).error == TrajectoryError::BadConfig);

    CHECK(generateTrajectory(path, zeroVelocity).trajectory.isEmpty());
}

CHAD_TEST(trajectory, too_many_waypoints_is_an_error) {
    std::vector<Pose> path;
    for (size_t i = 0; i <= MAX_PATH_WAYPOINTS; i++) {
        path.push_back({static_cast<float>(i) * 5.0f, 0.0f, 0.0f});
    }
    CHECK(generateTrajectory(path, baseConfig()).error == TrajectoryError::TooManyWaypoints);
}

CHAD_TEST(trajectory, straight_line_total_time_matches_a_trapezoid) {
    const TrajectoryConfig config = baseConfig(40.0f, 50.0f);
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {100.0f, 0.0f, 0.0f}};
    const TrajectoryResult result = generateTrajectory(path, config);
    CHECK(result.ok());
    if (!result.ok()) return;

    const double v = 40.0;
    const double a = 50.0;
    const double distance = 100.0;
    const double expected = 2.0 * v / a + (distance - v * v / a) / v;
    CHECK_NEAR(result.trajectory.totalTime(), expected, 1e-3);
    CHECK_NEAR(totalArcLength(result.trajectory), distance, 1e-3);

    float peak = 0.0f;
    for (const TrajectoryState& state : result.trajectory.states()) {
        peak = std::fmax(peak, std::fabs(state.velocity));
    }
    CHECK_NEAR(peak, v, 1e-3);
    recheck(result.trajectory, config);
}

CHAD_TEST(trajectory, short_straight_line_total_time_matches_a_triangle) {
    TrajectoryConfig config = baseConfig(40.0f, 50.0f);
    config.tolerance = {0.1f, 0.05f, 0.0872f, 20};
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {10.0f, 0.0f, 0.0f}};
    const TrajectoryResult result = generateTrajectory(path, config);
    CHECK(result.ok());
    if (!result.ok()) return;

    CHECK_NEAR(result.trajectory.totalTime(), 2.0 * std::sqrt(10.0 / 50.0), 1e-3);
    float peak = 0.0f;
    for (const TrajectoryState& state : result.trajectory.states()) {
        peak = std::fmax(peak, std::fabs(state.velocity));
    }
    CHECK_NEAR(peak, std::sqrt(50.0 * 10.0), 0.2);
    recheck(result.trajectory, config);
}

CHAD_TEST(trajectory, start_and_end_velocities_are_honoured) {
    TrajectoryConfig config = baseConfig();
    config.startVelocity = 15.0f;
    config.endVelocity = 8.0f;
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {80.0f, 0.0f, 0.0f}};
    const TrajectoryResult result = generateTrajectory(path, config);
    CHECK(result.ok());
    if (!result.ok()) return;
    CHECK_NEAR(result.trajectory.states().front().velocity, 15.0, 1e-3);
    CHECK_NEAR(result.trajectory.states().back().velocity, 8.0, 1e-3);
    recheck(result.trajectory, config);
}

CHAD_TEST(trajectory, start_velocity_above_the_cap_is_clamped_not_obeyed) {
    TrajectoryConfig config = baseConfig(40.0f, 50.0f);
    config.startVelocity = 500.0f;
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {80.0f, 0.0f, 0.0f}};
    const TrajectoryResult result = generateTrajectory(path, config);
    CHECK(result.ok());
    if (!result.ok()) return;
    CHECK_NEAR(result.trajectory.states().front().velocity, 40.0, 1e-3);
    recheck(result.trajectory, config);
}

CHAD_TEST(trajectory, profile_never_exceeds_any_constraint) {
    const DriveModel model{12.0f, 3.25f, 60.0f, WheelType::Omni, 0.0f};
    const DifferentialKinematics kinematics(model.trackWidth);
    const CentripetalConstraint grip(model);
    const DifferentialKinematicsConstraint wheels(kinematics, model.maxWheelSpeed);
    const MaxVelocityConstraint cap(45.0f);
    const WheelVoltageConstraint voltage({0.3f, 0.2f, 0.195f, 12.0f}, kinematics);

    TrajectoryConfig config = baseConfig(50.0f, 120.0f);
    config.constraints = {&grip, &wheels, &cap, &voltage};

    const TrajectoryResult result = generateTrajectory(curvyPath(), config);
    CHECK(result.ok());
    if (!result.ok()) return;
    CHECK(result.trajectory.states().size() > 50);
    recheck(result.trajectory, config);

    // The grip limit has to actually bite somewhere on a path this curvy,
    // otherwise the assertions above are vacuous.
    bool gripBinds = false;
    for (const TrajectoryState& state : result.trajectory.states()) {
        const float limit = grip.maxVelocity(state.pose, state.curvature, 0.0f, 0.0f);
        if (limit < 45.0f) gripBinds = true;
    }
    CHECK(gripBinds);
}

CHAD_TEST(trajectory, reversed_profile_never_exceeds_any_constraint) {
    const DriveModel model{12.0f, 3.25f, 60.0f, WheelType::Traction, 0.0f};
    const DifferentialKinematics kinematics(model.trackWidth);
    const CentripetalConstraint grip(model);
    const DifferentialKinematicsConstraint wheels(kinematics, model.maxWheelSpeed);

    TrajectoryConfig config = baseConfig(45.0f, 100.0f);
    config.reversed = true;
    config.constraints = {&grip, &wheels};

    const TrajectoryResult result = generateTrajectory(curvyPath(), config);
    CHECK(result.ok());
    if (!result.ok()) return;
    recheck(result.trajectory, config);
}

CHAD_TEST(trajectory, reversed_drives_backwards_through_forward_facing_poses) {
    TrajectoryConfig config = baseConfig();
    config.reversed = true;
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {-60.0f, 0.0f, 0.0f}};
    const TrajectoryResult result = generateTrajectory(path, config);
    CHECK(result.ok());
    if (!result.ok()) return;

    const std::vector<TrajectoryState>& states = result.trajectory.states();
    for (const TrajectoryState& state : states) {
        CHECK_NEAR(state.pose.theta, 0.0, 1e-3);
        CHECK(state.velocity <= 1e-3f);
    }
    CHECK_NEAR(states.front().pose.x, 0.0, 1e-3);
    CHECK_NEAR(states.back().pose.x, -60.0, 0.05);
    for (size_t i = 0; i + 1 < states.size(); i++) {
        CHECK(states[i + 1].pose.x <= states[i].pose.x + 1e-4f);
    }
    CHECK_NEAR(totalArcLength(result.trajectory), 60.0, 0.05);
    recheck(result.trajectory, config);
}

CHAD_TEST(trajectory, a_constraint_that_allows_no_speed_stalls_rather_than_truncating) {
    const MaxVelocityConstraint stop(0.0f);
    TrajectoryConfig config = baseConfig();
    config.constraints = {&stop};
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {50.0f, 0.0f, 0.0f}};

    const TrajectoryResult result = generateTrajectory(path, config);
    CHECK(result.error == TrajectoryError::StalledProfile);
    CHECK(result.trajectory.isEmpty());
}

CHAD_TEST(trajectory, an_empty_acceleration_window_is_reported) {
    const ImpossibleAccel impossible;
    TrajectoryConfig config = baseConfig();
    config.constraints = {&impossible};
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {50.0f, 0.0f, 0.0f}};

    const TrajectoryResult result = generateTrajectory(path, config);
    CHECK(result.error == TrajectoryError::InfeasibleConstraint);
    CHECK(result.trajectory.isEmpty());
}

CHAD_TEST(trajectory, two_individually_feasible_constraints_can_still_conflict) {
    const AccelFloor floorOnly;
    const AccelCeiling ceilingOnly;
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {50.0f, 0.0f, 0.0f}};

    TrajectoryConfig ceilingAlone = baseConfig();
    ceilingAlone.constraints = {&ceilingOnly};
    CHECK(generateTrajectory(path, ceilingAlone).error !=
          TrajectoryError::InfeasibleConstraint);

    TrajectoryConfig floorAlone = baseConfig();
    floorAlone.constraints = {&floorOnly};
    CHECK(generateTrajectory(path, floorAlone).error != TrajectoryError::InfeasibleConstraint);

    TrajectoryConfig both = baseConfig();
    both.constraints = {&floorOnly, &ceilingOnly};
    const TrajectoryResult result = generateTrajectory(path, both);
    CHECK(result.error == TrajectoryError::InfeasibleConstraint);
    CHECK(result.trajectory.isEmpty());
}

CHAD_TEST(trajectory, null_constraints_are_skipped) {
    TrajectoryConfig config = baseConfig();
    config.constraints = {nullptr};
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {50.0f, 0.0f, 0.0f}};
    CHECK(generateTrajectory(path, config).ok());
}

CHAD_TEST(trajectory, voltage_constraint_caps_the_reachable_speed) {
    const DifferentialKinematics kinematics(12.0f);
    // Free speed is (12 - 0.3)/0.2 = 58.5 in/s, well under the configured cap.
    const WheelVoltageConstraint voltage({0.3f, 0.2f, 0.195f, 12.0f}, kinematics);

    TrajectoryConfig config = baseConfig(100.0f, 400.0f);
    config.constraints = {&voltage};
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {130.0f, 0.0f, 0.0f}};

    const TrajectoryResult result = generateTrajectory(path, config);
    CHECK(result.ok());
    if (!result.ok()) return;

    float peak = 0.0f;
    for (const TrajectoryState& state : result.trajectory.states()) {
        peak = std::fmax(peak, std::fabs(state.velocity));
    }
    CHECK(peak < 58.6f);
    CHECK(peak > 50.0f);
    recheck(result.trajectory, config);
}

CHAD_TEST(trajectory, feedforward_gains_can_be_handed_over_without_the_pros_header) {
    struct GainsLike {
        float kV;
        float kaStraight;
        float ksStraight;
    };
    const GainsLike gains{5.0f, 0.4f, 0.5f};
    const WheelVoltageModel model = WheelVoltageModel::fromFeedforwardGains(gains, 11.0f);
    CHECK_NEAR(model.kV, 5.0 * 0.0254, 1e-7);
    CHECK_NEAR(model.kA, 0.4 * 0.0254, 1e-7);
    CHECK_NEAR(model.kS, 0.5, 1e-6);
    CHECK_NEAR(model.maxVoltage, 11.0, 1e-6);
}

CHAD_TEST(trajectory, metre_fitted_gains_convert_to_inches) {
    const WheelVoltageModel model = WheelVoltageModel::fromMetreGains(0.5f, 5.0f, 0.4f, 10.0f);
    CHECK_NEAR(model.kS, 0.5, 1e-6);
    CHECK_NEAR(model.kV, 5.0 * 0.0254, 1e-7);
    CHECK_NEAR(model.kA, 0.4 * 0.0254, 1e-7);
    CHECK_NEAR(model.maxVoltage, 10.0, 1e-6);
    // A wheel at rest can use everything but the static term.
    CHECK_NEAR(model.maxWheelAccelAt(0.0f), 10.0 / (0.4 * 0.0254), 1e-2);
}

CHAD_TEST(trajectory, rectangular_region_constraint_only_binds_inside) {
    const MaxVelocityConstraint slow(10.0f);
    const RectangularRegionConstraint region({40.0f, -5.0f}, {60.0f, 5.0f}, &slow);
    CHECK(region.contains({50.0f, 0.0f}));
    CHECK(!region.contains({50.0f, 6.0f}));
    CHECK(!region.contains({39.0f, 0.0f}));

    TrajectoryConfig config = baseConfig();
    config.constraints = {&region};
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {100.0f, 0.0f, 0.0f}};
    const TrajectoryResult result = generateTrajectory(path, config);
    CHECK(result.ok());
    if (!result.ok()) return;
    recheck(result.trajectory, config);

    bool sawFast = false;
    for (const TrajectoryState& state : result.trajectory.states()) {
        if (region.contains(state.pose.translation())) {
            CHECK(std::fabs(state.velocity) <= 10.0f + 1e-2f);
        } else if (state.pose.x < 20.0f) {
            if (std::fabs(state.velocity) > 39.0f) sawFast = true;
        }
    }
    CHECK(sawFast);
}

CHAD_TEST(trajectory, elliptical_region_constraint_only_binds_inside) {
    const MaxVelocityConstraint slow(12.0f);
    const EllipticalRegionConstraint region({50.0f, 0.0f}, 10.0f, 4.0f,
                                            Rotation2d::fromRadians(0.0f), &slow);
    CHECK(region.contains({50.0f, 0.0f}));
    CHECK(region.contains({59.9f, 0.0f}));
    CHECK(!region.contains({60.1f, 0.0f}));
    CHECK(!region.contains({50.0f, 4.1f}));

    const EllipticalRegionConstraint rotated({0.0f, 0.0f}, 10.0f, 4.0f,
                                             Rotation2d::fromRadians(HALF_PI), &slow);
    CHECK(rotated.contains({0.0f, 9.9f}));
    CHECK(!rotated.contains({9.9f, 0.0f}));

    const EllipticalRegionConstraint degenerate({0.0f, 0.0f}, 0.0f, 4.0f, {}, &slow);
    CHECK(!degenerate.contains({0.0f, 0.0f}));

    TrajectoryConfig config = baseConfig();
    config.constraints = {&region};
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {100.0f, 0.0f, 0.0f}};
    const TrajectoryResult result = generateTrajectory(path, config);
    CHECK(result.ok());
    if (!result.ok()) return;
    recheck(result.trajectory, config);
    for (const TrajectoryState& state : result.trajectory.states()) {
        if (region.contains(state.pose.translation())) {
            CHECK(std::fabs(state.velocity) <= 12.0f + 1e-2f);
        }
    }
}

CHAD_TEST(trajectory, region_constraint_with_no_inner_constraint_is_inert) {
    const RectangularRegionConstraint rect({0.0f, 0.0f}, {10.0f, 10.0f}, nullptr);
    CHECK(rect.maxVelocity({5.0f, 5.0f, 0.0f}, 0.0f, 0.0f, 0.0f) == INF);
    CHECK(rect.accelerationLimits({5.0f, 5.0f, 0.0f}, 0.0f, 0.0f, 0.0f).feasible());
}

CHAD_TEST(trajectory, centripetal_and_kinematics_limits_are_closed_form) {
    const DriveModel model{12.0f, 3.25f, 60.0f, WheelType::Traction, 0.0f};
    const CentripetalConstraint grip(model);
    const DifferentialKinematicsConstraint wheels(DifferentialKinematics(model.trackWidth),
                                                  model.maxWheelSpeed);

    const float k = 0.05f;
    CHECK_NEAR(grip.maxVelocity({}, k, 0.0f, 0.0f), std::sqrt(model.gripLimit() / k), 1e-3);
    CHECK_NEAR(wheels.maxVelocity({}, k, 0.0f, 0.0f),
               model.maxWheelSpeed / (1.0f + 0.5f * k * model.trackWidth), 1e-3);
    // Together they reproduce the drive model's own answer.
    CHECK_NEAR(std::fmin(grip.maxVelocity({}, k, 0.0f, 0.0f),
                         wheels.maxVelocity({}, k, 0.0f, 0.0f)),
               model.maxSpeedForCurvature(k), 1e-3);

    CHECK(grip.maxVelocity({}, 0.0f, 0.0f, 0.0f) == INF);
    CHECK_NEAR(grip.maxVelocity({}, INF, 0.0f, 0.0f), 0.0, 1e-9);
    CHECK_NEAR(wheels.maxVelocity({}, INF, 0.0f, 0.0f), 0.0, 1e-9);
}

CHAD_TEST(trajectory, sample_clamps_outside_the_time_range) {
    const TrajectoryResult result =
        generateTrajectory(std::vector<Pose>{{0.0f, 0.0f, 0.0f}, {60.0f, 0.0f, 0.0f}},
                           baseConfig());
    CHECK(result.ok());
    if (!result.ok()) return;
    const Trajectory& traj = result.trajectory;
    const std::vector<TrajectoryState>& states = traj.states();

    CHECK(traj.sample(0.0f).pose.equals(states.front().pose));
    CHECK(traj.sample(-5.0f).pose.equals(states.front().pose));
    CHECK(traj.sample(traj.totalTime()).pose.equals(states.back().pose));
    CHECK(traj.sample(traj.totalTime() + 5.0f).pose.equals(states.back().pose));
    CHECK(traj.sample(INF).pose.equals(states.back().pose));
    CHECK(traj.sample(-INF).pose.equals(states.front().pose));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    CHECK(traj.sample(nan).pose.equals(states.front().pose));
    CHECK(traj.initialPose().equals(states.front().pose));
}

CHAD_TEST(trajectory, sample_on_a_state_boundary_returns_that_state) {
    const TrajectoryConfig config = baseConfig();
    const TrajectoryResult result = generateTrajectory(curvyPath(), config);
    CHECK(result.ok());
    if (!result.ok()) return;
    const std::vector<TrajectoryState>& states = result.trajectory.states();

    for (size_t i = 0; i < states.size(); i++) {
        const TrajectoryState sampled = result.trajectory.sample(states[i].t);
        CHECK_NEAR(sampled.t, states[i].t, 1e-5);
        CHECK_NEAR(sampled.pose.x, states[i].pose.x, 2e-3);
        CHECK_NEAR(sampled.pose.y, states[i].pose.y, 2e-3);
        CHECK_NEAR(angleDiffRad(sampled.pose.theta, states[i].pose.theta), 0.0, 1e-3);
        CHECK_NEAR(sampled.velocity, states[i].velocity, 5e-3);
    }
}

CHAD_TEST(trajectory, sample_matches_a_brute_force_scan) {
    const TrajectoryResult result = generateTrajectory(curvyPath(), baseConfig());
    CHECK(result.ok());
    if (!result.ok()) return;
    const Trajectory& traj = result.trajectory;

    for (int i = -10; i <= 510; i++) {
        const float t = traj.totalTime() * static_cast<float>(i) / 500.0f;
        const TrajectoryState fast = traj.sample(t);
        const TrajectoryState slow = bruteSample(traj, t);
        CHECK_NEAR(fast.t, slow.t, 1e-6);
        CHECK_NEAR(fast.velocity, slow.velocity, 1e-4);
        CHECK_NEAR(fast.acceleration, slow.acceleration, 1e-4);
        CHECK_NEAR(fast.pose.x, slow.pose.x, 1e-4);
        CHECK_NEAR(fast.pose.y, slow.pose.y, 1e-4);
        CHECK_NEAR(fast.curvature, slow.curvature, 1e-5);
    }
}

CHAD_TEST(trajectory, sampling_is_monotonic_in_time_and_position) {
    const TrajectoryResult result = generateTrajectory(curvyPath(), baseConfig());
    CHECK(result.ok());
    if (!result.ok()) return;
    const Trajectory& traj = result.trajectory;

    TrajectoryState previous = traj.sample(0.0f);
    for (int i = 1; i <= 400; i++) {
        const float t = traj.totalTime() * static_cast<float>(i) / 400.0f;
        const TrajectoryState now = traj.sample(t);
        CHECK(now.t >= previous.t - 1e-6f);

        const Translation2d step = now.pose.translation() - previous.pose.translation();
        const Rotation2d heading = previous.pose.rotation();
        // Forward travel: the step never points backwards along the heading.
        CHECK(step.x * heading.cos + step.y * heading.sin >= -1e-3f);
        previous = now;
    }
}

CHAD_TEST(trajectory, mid_segment_samples_land_on_the_arc_not_the_chord) {
    constexpr float R = 10.0f;
    const Trajectory traj = circleTrajectory(R, PI / 3.0f, 10.0f);
    const TrajectoryState mid = traj.sample(0.5f * traj.totalTime());

    // Chord interpolation would land at radius R*cos(30 deg) = 8.66.
    CHECK_NEAR(mid.pose.translation().norm(), R, 2e-3);
    CHECK_NEAR(mid.pose.x, R * std::cos(PI / 6.0f), 2e-3);
    CHECK_NEAR(mid.pose.y, R * std::sin(PI / 6.0f), 2e-3);
    CHECK_NEAR(angleDiffRad(mid.pose.theta, HALF_PI + PI / 6.0f), 0.0, 1e-4);
    CHECK_NEAR(mid.velocity, 10.0, 1e-4);
}

CHAD_TEST(trajectory, interpolate_handles_reversed_order_and_degenerate_input) {
    const TrajectoryState a{1.0f, 20.0f, 5.0f, {0.0f, 0.0f, 0.0f}, 0.0f};
    const TrajectoryState b{2.0f, 25.0f, 5.0f, {22.5f, 0.0f, 0.0f}, 0.0f};

    const TrajectoryState forward = a.interpolate(b, 0.5f);
    const TrajectoryState backward = b.interpolate(a, 0.5f);
    CHECK_NEAR(forward.t, 1.5, 1e-6);
    CHECK_NEAR(backward.t, 1.5, 1e-6);
    CHECK_NEAR(forward.pose.x, backward.pose.x, 1e-3);

    // Out-of-range fractions clamp instead of extrapolating or recursing.
    CHECK_NEAR(a.interpolate(b, -3.0f).t, 1.0, 1e-6);
    CHECK_NEAR(a.interpolate(b, 7.0f).t, 2.0, 1e-6);

    // Coincident poses have no arc; the curvature still has to interpolate.
    const TrajectoryState c{0.0f, 0.0f, 0.0f, {3.0f, 4.0f, 0.0f}, 0.1f};
    const TrajectoryState d{1.0f, 0.0f, 0.0f, {3.0f, 4.0f, 0.0f}, 0.3f};
    const TrajectoryState degenerate = c.interpolate(d, 0.5f);
    CHECK_NEAR(degenerate.curvature, 0.2, 1e-6);
    CHECK_FINITE(degenerate.pose.x);
}

CHAD_TEST(trajectory, mirrored_preserves_arc_length_and_curvature_magnitude) {
    const TrajectoryResult result = generateTrajectory(curvyPath(), baseConfig());
    CHECK(result.ok());
    if (!result.ok()) return;
    const Trajectory& base = result.trajectory;

    for (const MirrorAxis axis : {MirrorAxis::X, MirrorAxis::Y}) {
        const Trajectory flipped = base.mirrored(axis);
        CHECK_EQ(flipped.states().size(), base.states().size());
        CHECK_NEAR(flipped.totalTime(), base.totalTime(), 1e-6);
        CHECK_NEAR(totalArcLength(flipped), totalArcLength(base), 1e-2);

        for (size_t i = 0; i < base.states().size(); i++) {
            const TrajectoryState& original = base.states()[i];
            const TrajectoryState& mirrored = flipped.states()[i];
            CHECK_NEAR(mirrored.t, original.t, 1e-6);
            CHECK_NEAR(mirrored.velocity, original.velocity, 1e-6);
            CHECK_NEAR(mirrored.acceleration, original.acceleration, 1e-6);
            CHECK_NEAR(std::fabs(mirrored.curvature), std::fabs(original.curvature), 1e-6);
            CHECK(mirrored.curvature * original.curvature <= 1e-9f);
            if (axis == MirrorAxis::X) {
                CHECK_NEAR(mirrored.pose.x, original.pose.x, 1e-6);
                CHECK_NEAR(mirrored.pose.y, -original.pose.y, 1e-6);
            } else {
                CHECK_NEAR(mirrored.pose.x, -original.pose.x, 1e-6);
                CHECK_NEAR(mirrored.pose.y, original.pose.y, 1e-6);
            }
        }

        // The reflected headings still have to agree with the direction of
        // travel, which is what a sign slip on theta would break.
        for (size_t i = 0; i + 1 < flipped.states().size(); i++) {
            const Translation2d step = flipped.states()[i + 1].pose.translation() -
                                       flipped.states()[i].pose.translation();
            if (step.norm() < 1e-3f) continue;
            const Rotation2d heading = flipped.states()[i].pose.rotation();
            CHECK(step.x * heading.cos + step.y * heading.sin > 0.0f);
        }
    }

    // Two mirrors about the same axis are the identity.
    const Trajectory twice = base.mirrored().mirrored();
    for (size_t i = 0; i < base.states().size(); i++) {
        CHECK(twice.states()[i].pose.equals(base.states()[i].pose));
        CHECK_NEAR(twice.states()[i].curvature, base.states()[i].curvature, 1e-9);
    }
}

CHAD_TEST(trajectory, half_turn_is_the_swap_on_a_rotationally_symmetric_field) {
    const TrajectoryResult result = generateTrajectory(curvyPath(), baseConfig());
    CHECK(result.ok());
    if (!result.ok()) return;
    const Trajectory& base = result.trajectory;

    // The documented alternative to mirrored() for a 180-degree symmetric field.
    const Trajectory turned = base.relativeTo({0.0f, 0.0f, PI});
    CHECK_NEAR(totalArcLength(turned), totalArcLength(base), 1e-2);
    for (size_t i = 0; i < base.states().size(); i++) {
        CHECK_NEAR(turned.states()[i].pose.x, -base.states()[i].pose.x, 1e-3);
        CHECK_NEAR(turned.states()[i].pose.y, -base.states()[i].pose.y, 1e-3);
        CHECK_NEAR(angleDiffRad(turned.states()[i].pose.theta,
                                base.states()[i].pose.theta + PI),
                   0.0, 1e-4);
        // A rotation keeps handedness, so curvature keeps its sign.
        CHECK_NEAR(turned.states()[i].curvature, base.states()[i].curvature, 1e-9);
    }
}

CHAD_TEST(trajectory, transform_by_is_rigid) {
    const TrajectoryResult result = generateTrajectory(curvyPath(), baseConfig());
    CHECK(result.ok());
    if (!result.ok()) return;
    const Trajectory& base = result.trajectory;

    const Transform2d shift{{40.0f, -15.0f}, Rotation2d::fromRadians(0.7f)};
    const Trajectory moved = base.transformBy(shift);

    CHECK_NEAR(totalArcLength(moved), totalArcLength(base), 1e-2);
    CHECK(moved.states().front().pose.equals(base.states().front().pose.transformBy(shift),
                                             1e-3f, 1e-4f));
    for (size_t i = 0; i < base.states().size(); i++) {
        CHECK_NEAR(moved.states()[i].curvature, base.states()[i].curvature, 1e-9);
        CHECK_NEAR(moved.states()[i].velocity, base.states()[i].velocity, 1e-9);
        CHECK_NEAR(moved.states()[i].t, base.states()[i].t, 1e-9);
    }
    for (size_t i = 0; i + 1 < base.states().size(); i++) {
        CHECK_NEAR(moved.states()[i].pose.distanceTo(moved.states()[i + 1].pose),
                   base.states()[i].pose.distanceTo(base.states()[i + 1].pose), 1e-3);
    }
}

CHAD_TEST(trajectory, relative_to_puts_the_origin_where_asked) {
    const TrajectoryResult result = generateTrajectory(curvyPath(), baseConfig());
    CHECK(result.ok());
    if (!result.ok()) return;
    const Trajectory& base = result.trajectory;

    const Trajectory local = base.relativeTo(base.initialPose());
    CHECK(local.states().front().pose.equals({0.0f, 0.0f, 0.0f}, 1e-3f, 1e-4f));
    CHECK_NEAR(totalArcLength(local), totalArcLength(base), 1e-2);
    for (size_t i = 0; i < base.states().size(); i++) {
        CHECK_NEAR(local.states()[i].curvature, base.states()[i].curvature, 1e-9);
    }

    // relativeTo then transformBy the same pose is the identity.
    const Pose origin = base.initialPose();
    const Trajectory restored =
        local.transformBy({origin.translation(), origin.rotation()});
    for (size_t i = 0; i < base.states().size(); i++) {
        CHECK(restored.states()[i].pose.equals(base.states()[i].pose, 0.02f, 1e-3f));
    }
}

CHAD_TEST(trajectory, concatenate_shifts_time_and_drops_the_duplicate_state) {
    const TrajectoryConfig config = baseConfig();
    const TrajectoryResult first =
        generateTrajectory(std::vector<Pose>{{0.0f, 0.0f, 0.0f}, {40.0f, 0.0f, 0.0f}}, config);
    const TrajectoryResult second =
        generateTrajectory(std::vector<Pose>{{40.0f, 0.0f, 0.0f}, {40.0f, 40.0f, HALF_PI}},
                           config);
    CHECK(first.ok());
    if (!first.ok()) return;
    CHECK(second.ok());
    if (!second.ok()) return;

    const Trajectory joined = first.trajectory.concatenate(second.trajectory);
    CHECK_EQ(joined.states().size(),
             first.trajectory.states().size() + second.trajectory.states().size() - 1);
    CHECK_NEAR(joined.totalTime(),
               first.trajectory.totalTime() + second.trajectory.totalTime(), 1e-4);
    for (size_t i = 0; i + 1 < joined.states().size(); i++) {
        CHECK(joined.states()[i + 1].t >= joined.states()[i].t - 1e-6f);
    }
    const size_t seam = first.trajectory.states().size();
    CHECK_NEAR(joined.states()[seam].t,
               first.trajectory.totalTime() + second.trajectory.states()[1].t, 1e-4);

    const Trajectory empty;
    CHECK_EQ(empty.concatenate(first.trajectory).states().size(),
             first.trajectory.states().size());
    CHECK_EQ(first.trajectory.concatenate(empty).states().size(),
             first.trajectory.states().size());
}

CHAD_TEST(trajectory, an_inline_180_degree_cusp_fails_cleanly) {
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {24.0f, 0.0f, PI}};
    const TrajectoryResult result = generateTrajectory(path, baseConfig());
    CHECK(result.error == TrajectoryError::MalformedSpline);
    CHECK(result.trajectory.isEmpty());
}

CHAD_TEST(trajectory, an_offset_180_degree_cusp_stays_finite) {
    const DriveModel model{12.0f, 3.25f, 60.0f, WheelType::Omni, 0.0f};
    const CentripetalConstraint grip(model);
    const DifferentialKinematicsConstraint wheels(DifferentialKinematics(model.trackWidth),
                                                  model.maxWheelSpeed);
    TrajectoryConfig config = baseConfig(50.0f, 100.0f);
    config.constraints = {&grip, &wheels};

    // Off the line the tangent never vanishes, so the path loops instead: a
    // real trajectory with a very tight corner in the middle of it.
    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {24.0f, 12.0f, PI}};
    const TrajectoryResult result = generateTrajectory(path, config);
    CHECK(result.ok());
    if (!result.ok()) return;
    recheck(result.trajectory, config);
    CHECK_FINITE(result.trajectory.totalTime());
    CHECK(result.trajectory.totalTime() > 0.0f);

    float peakCurvature = 0.0f;
    float speedAtPeak = 0.0f;
    for (const TrajectoryState& state : result.trajectory.states()) {
        if (std::fabs(state.curvature) > peakCurvature) {
            peakCurvature = std::fabs(state.curvature);
            speedAtPeak = std::fabs(state.velocity);
        }
    }
    CHECK(peakCurvature > 1.0f);
    CHECK(speedAtPeak <= std::sqrt(model.gripLimit() / peakCurvature) + 1e-2f);
}

CHAD_TEST(trajectory, a_near_zero_speed_pinch_stays_finite) {
    // Velocity hitting (almost) zero part-way along is where the v^2 relation
    // and the dt derivation both go singular.
    const MaxVelocityConstraint crawl(0.05f);
    const RectangularRegionConstraint pinch({48.0f, -5.0f}, {52.0f, 5.0f}, &crawl);
    TrajectoryConfig config = baseConfig();
    config.constraints = {&pinch};

    const std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {100.0f, 0.0f, 0.0f}};
    const TrajectoryResult result = generateTrajectory(path, config);
    CHECK(result.ok());
    if (!result.ok()) return;
    recheck(result.trajectory, config);

    CHECK_FINITE(result.trajectory.totalTime());
    CHECK(result.trajectory.totalTime() > 80.0f);

    bool crawled = false;
    const std::vector<TrajectoryState>& states = result.trajectory.states();
    for (size_t i = 1; i + 1 < states.size(); i++) {
        if (std::fabs(states[i].velocity) <= 0.05f + 1e-4f) crawled = true;
        CHECK(states[i].t >= states[i - 1].t);
        CHECK_FINITE(states[i].t);
        CHECK_FINITE(states[i].acceleration);
    }
    CHECK(crawled);
    CHECK_NEAR(totalArcLength(result.trajectory), 100.0, 0.05);
}

CHAD_TEST(trajectory, time_parameterize_accepts_a_prebuilt_point_list) {
    constexpr float R = 24.0f;
    std::vector<PoseWithCurvature> points;
    for (int i = 0; i <= 90; i++) {
        const float angle = degToRad(static_cast<float>(i));
        points.push_back({{R * std::cos(angle), R * std::sin(angle), angle + HALF_PI},
                          1.0f / R});
    }

    const DriveModel model{12.0f, 3.25f, 60.0f, WheelType::Traction, 0.0f};
    const CentripetalConstraint grip(model);
    TrajectoryConfig config = baseConfig(60.0f, 80.0f);
    config.constraints = {&grip};

    const TrajectoryResult result = timeParameterize(points, config);
    CHECK(result.ok());
    if (!result.ok()) return;
    CHECK_EQ(result.trajectory.states().size(), points.size());
    // Exact quarter arc: the parameterizer measures arcs, not chords.
    CHECK_NEAR(totalArcLength(result.trajectory), R * HALF_PI, 1e-2);
    recheck(result.trajectory, config);

    const float gripCap = std::sqrt(model.gripLimit() * R);
    float peak = 0.0f;
    for (const TrajectoryState& state : result.trajectory.states()) {
        peak = std::fmax(peak, std::fabs(state.velocity));
    }
    CHECK(peak <= gripCap + 1e-2f);

    CHECK(timeParameterize({}, config).error == TrajectoryError::NoWaypoints);
    CHECK(timeParameterize({points[0]}, config).ok());
    CHECK_EQ(timeParameterize({points[0]}, config).trajectory.states().size(),
             static_cast<size_t>(1));
}

CHAD_TEST(trajectory, cubic_generation_matches_its_waypoints) {
    const TrajectoryConfig config = baseConfig();
    const std::vector<Translation2d> interior = {{30.0f, 20.0f}, {60.0f, -20.0f}};
    const TrajectoryResult result =
        generateCubicTrajectory({0.0f, 0.0f, 0.0f}, interior, {90.0f, 0.0f, 0.0f}, config);
    CHECK(result.ok());
    if (!result.ok()) return;
    recheck(result.trajectory, config);

    CHECK(result.trajectory.states().front().pose.equals({0.0f, 0.0f, 0.0f}, 1e-2f, 1e-2f));
    const TrajectoryState& last = result.trajectory.states().back();
    CHECK_NEAR(last.pose.x, 90.0, 0.05);
    CHECK_NEAR(last.pose.y, 0.0, 0.05);

    // The path really does pass near the interior translations.
    for (const Translation2d& waypoint : interior) {
        float closest = 1e9f;
        for (const TrajectoryState& state : result.trajectory.states()) {
            closest = std::fmin(closest, state.pose.translation().distanceTo(waypoint));
        }
        CHECK(closest < 0.2f);
    }

    TrajectoryConfig reversed = config;
    reversed.reversed = true;
    const TrajectoryResult back =
        generateCubicTrajectory({0.0f, 0.0f, 0.0f}, interior, {90.0f, 0.0f, 0.0f}, reversed);
    CHECK(back.ok());
    if (!back.ok()) return;
    recheck(back.trajectory, reversed);
    CHECK(back.trajectory.states().back().velocity <= 1e-3f);

    const TrajectoryResult degenerate =
        generateCubicTrajectory({0.0f, 0.0f, 0.0f}, {}, {0.0f, 0.0f, 0.0f}, config);
    CHECK(degenerate.error == TrajectoryError::DegenerateWaypoints);
}

CHAD_TEST(trajectory, explicit_tangent_magnitudes_change_the_path_not_its_validity) {
    const TrajectoryConfig config = baseConfig();
    const std::vector<PathWaypoint> tight = {{Pose{0.0f, 0.0f, 0.0f}, 8.0f},
                                             {Pose{40.0f, 40.0f, HALF_PI}, 8.0f}};
    const std::vector<PathWaypoint> loose = {{Pose{0.0f, 0.0f, 0.0f}, 70.0f},
                                             {Pose{40.0f, 40.0f, HALF_PI}, 70.0f}};

    const TrajectoryResult a = generateTrajectory(tight, config);
    const TrajectoryResult b = generateTrajectory(loose, config);
    CHECK(a.ok());
    if (!a.ok()) return;
    CHECK(b.ok());
    if (!b.ok()) return;
    recheck(a.trajectory, config);
    recheck(b.trajectory, config);
    CHECK(totalArcLength(b.trajectory) > totalArcLength(a.trajectory) + 1.0);
}

CHAD_TEST(trajectory, rotatedHalfTurnIsTheAllianceSwapOnThisField) {
    std::vector<TrajectoryState> states;
    states.push_back({0.0f, 0.0f, 10.0f, {24.0f, 12.0f, 0.0f}, 0.05f});
    states.push_back({1.0f, 20.0f, 0.0f, {48.0f, 24.0f, 0.5f}, 0.05f});
    const Trajectory original(std::move(states));

    const Trajectory swapped = original.rotatedHalfTurn();
    CHECK_NEAR(swapped.states()[0].pose.x, -24.0f, 1e-3);
    CHECK_NEAR(swapped.states()[0].pose.y, -12.0f, 1e-3);
    CHECK_NEAR(swapped.states()[1].pose.x, -48.0f, 1e-3);
    CHECK_NEAR(swapped.states()[1].pose.y, -24.0f, 1e-3);

    // A rotation preserves handedness; a reflection does not.
    CHECK_NEAR(swapped.states()[0].curvature, original.states()[0].curvature, 1e-6);
    CHECK_NEAR(original.mirrored().states()[0].curvature,
               -original.states()[0].curvature, 1e-6);

    CHECK_NEAR(swapped.totalTime(), original.totalTime(), 1e-6);
    CHECK_NEAR(swapped.states()[1].velocity, original.states()[1].velocity, 1e-6);
}

CHAD_TEST(trajectory, drivetrainConstraintsShapeThePathTheyDescribe) {
    DriveModel traction;
    traction.trackWidth = 12.0f;
    traction.maxWheelSpeed = 60.0f;
    traction.wheels = WheelType::Traction;

    DriveModel omni = traction;
    omni.wheels = WheelType::Omni;

    const std::vector<Pose> path = {
        {0.0f, 0.0f, 0.0f}, {24.0f, 24.0f, HALF_PI}, {0.0f, 48.0f, PI}};

    auto timeFor = [&path](const DriveModel& model) {
        DrivetrainConstraints constraints(model);
        TrajectoryConfig config = trajectoryLimits(model, 1.0f, 60.0f);
        config.constraints = constraints.list();
        const TrajectoryResult result = generateTrajectory(path, config);
        CHECK(result.ok());
        return result.trajectory.totalTime();
    };

    const float tractionTime = timeFor(traction);
    const float omniTime = timeFor(omni);
    CHECK(tractionTime > 0.0f);
    // Traction holds a corner, so the same geometry takes less time.
    CHECK(tractionTime < omniTime);
}

CHAD_TEST(trajectory, generatedStatesRespectTheWheelBudget) {
    DriveModel model;
    model.trackWidth = 12.0f;
    model.maxWheelSpeed = 60.0f;
    model.wheels = WheelType::Traction;

    DrivetrainConstraints constraints(model);
    TrajectoryConfig config = trajectoryLimits(model, 1.0f, 60.0f);
    config.constraints = constraints.list();

    const std::vector<Pose> path = {
        {0.0f, 0.0f, 0.0f}, {30.0f, 18.0f, 0.6f}, {60.0f, 0.0f, -0.6f}};
    const TrajectoryResult result = generateTrajectory(path, config);
    CHECK(result.ok());

    const DifferentialKinematics kin(model.trackWidth);
    for (const TrajectoryState& state : result.trajectory.states()) {
        const float omega = state.velocity * state.curvature;
        const WheelSpeeds wheels = kin.toWheelSpeeds({state.velocity, 0.0f, omega});
        CHECK(std::fabs(wheels.left) <= model.maxWheelSpeed + 1e-2f);
        CHECK(std::fabs(wheels.right) <= model.maxWheelSpeed + 1e-2f);
    }
}

namespace {

/// Worst per-wheel voltage a profile demands, with the wheel acceleration
/// DIFFERENTIATED from the profile instead of assumed to be
/// `chassis accel * ratio`. d/dt(v*c) carries a v*|v|*(b/2)*dk/ds term that the
/// rigid-ratio model drops, so this is a statement about the battery rather than
/// about the constraint's own arithmetic.
///
/// Both ends of every interval are priced. The wheel acceleration is constant
/// there and the wheel speed linear, so the demand is linear in time and its
/// peak is one of the two ends -- taking only the near end (which is what this
/// used to do) misses the whole overshoot, because the profile accelerates INTO
/// the corner and the far end is where it is dearest.
float worstTrueWheelVoltage(const WheelVoltageModel& model, float trackWidth,
                            const std::vector<TrajectoryState>& states, int& overBudget,
                            float slack = 0.01f) {
    float worst = 0.0f;
    overBudget = 0;
    for (size_t i = 0; i + 1 < states.size(); i++) {
        const float dt = states[i + 1].t - states[i].t;
        if (!(dt > 1e-6f)) continue;
        for (float side : {-1.0f, 1.0f}) {
            const float half = side * 0.5f * trackWidth;
            const float w0 = states[i].velocity * (1.0f + half * states[i].curvature);
            const float w1 = states[i + 1].velocity * (1.0f + half * states[i + 1].curvature);
            const float accel = (w1 - w0) / dt;
            // kS is discontinuous at a standstill, so a wheel that reaches zero
            // is priced on the side it approaches from. sign(0) = 0 instead reads
            // 0.3 V of static term off a stop, which is not a rail overshoot.
            const float dir = (w0 != 0.0f) ? sign(w0) : sign(w1);
            for (float w : {w0, w1}) {
                const float volts = model.kS * dir + model.kV * w + model.kA * accel;
                if (std::fabs(volts) > model.maxVoltage + slack) overBudget++;
                worst = std::fmax(worst, std::fabs(volts));
            }
        }
    }
    return worst;
}

/// The two wheel voltages a state demands when the wheel acceleration really is
/// `chassis accel * ratio` -- true only where dk/ds is zero, so this is for
/// hand-built single states and never for a sampled path.
void wheelVoltages(const WheelVoltageModel& model, const DifferentialKinematics& kin,
                   const TrajectoryState& state, float& left, float& right) {
    const float half = 0.5f * kin.trackWidth() * state.curvature;
    const float vL = state.velocity * (1.0f - half);
    const float vR = state.velocity * (1.0f + half);
    const float aL = state.acceleration * (1.0f - half);
    const float aR = state.acceleration * (1.0f + half);
    left = model.kS * sign(vL) + model.kV * vL + model.kA * aL;
    right = model.kS * sign(vR) + model.kV * vR + model.kA * aR;
}

}  // namespace

CHAD_TEST(trajectory, everyStateStaysInsideTheVoltageBudget) {
    const WheelVoltageModel model{0.3f, 0.2f, 0.195f, 12.0f};
    const DifferentialKinematics kin(12.0f);
    const WheelVoltageConstraint constraint(model, kin);

    const std::vector<Pose> path = {
        {0.0f, 0.0f, 0.0f}, {40.0f, 20.0f, 0.0f}, {80.0f, 0.0f, 0.0f}};

    // Swept over sampling density, because the demand is priced per interval:
    // a coarse step is where an interval term evaluated at a node, or dropped
    // at a stopped end, shows up. A 2 in step used to demand 12.62 V.
    for (float maxDx : {2.0f, 1.0f, 0.5f, 0.25f}) {
        TrajectoryConfig config = baseConfig(60.0f, 200.0f);
        config.tolerance = {maxDx, 0.05f, 0.0872f, 20};
        config.constraints = {&constraint};

        const TrajectoryResult result = generateTrajectory(path, config);
        CHECK(result.ok());
        if (!result.ok()) continue;

        int over = 0;
        const float worst =
            worstTrueWheelVoltage(model, kin.trackWidth(), result.trajectory.states(), over);
        std::printf("[budget] maxDx=%.2f  %zu states, worst %.4f V, %d ends over\n", maxDx,
                    result.trajectory.states().size(), worst, over);
        // Bounding the window by the inner wheel alone demanded 22 V here.
        CHECK(worst <= model.maxVoltage + 0.02f);
        CHECK(over == 0);
    }
}

CHAD_TEST(trajectory, voltageWindowHoldsBothWheelsAtBothEnds) {
    const WheelVoltageModel model{0.3f, 0.2f, 0.195f, 12.0f};
    const DifferentialKinematics kin(12.0f);
    const WheelVoltageConstraint constraint(model, kin);

    for (float k : {0.0f, 0.01f, 0.05f, 0.1f, -0.05f}) {
        for (float v : {5.0f, 20.0f, -5.0f}) {
            const AccelerationLimits limits =
                constraint.accelerationLimits(Pose{}, k, v, 0.0f);
            CHECK(limits.min <= limits.max);
            for (float a : {limits.min, limits.max}) {
                if (!std::isfinite(a)) continue;
                TrajectoryState state{};
                state.velocity = v;
                state.acceleration = a;
                state.curvature = k;
                float left = 0.0f, right = 0.0f;
                wheelVoltages(model, kin, state, left, right);
                CHECK(std::fabs(left) <= model.maxVoltage + 1e-2f);
                CHECK(std::fabs(right) <= model.maxVoltage + 1e-2f);
            }
        }
    }
}

CHAD_TEST(trajectory, pinnedWheelConstrainsNothingRatherThanEverything) {
    const WheelVoltageModel model{0.3f, 0.2f, 0.195f, 12.0f};
    const DifferentialKinematics kin(12.0f);
    const WheelVoltageConstraint constraint(model, kin);

    // Turn radius exactly half the track width: one wheel stands still, and a
    // clamped divisor used to turn that into an empty window and a hard failure.
    const float k = 2.0f / kin.trackWidth();
    for (float v : {5.0f, -5.0f, 20.0f, -20.0f}) {
        CHECK(constraint.accelerationLimits(Pose{}, k, v, 0.0f).feasible());
        CHECK(constraint.accelerationLimits(Pose{}, -k, v, 0.0f).feasible());
    }
}

CHAD_TEST(trajectory, drivingTheSameBendBackwardsNegatesTheWindow) {
    const WheelVoltageModel model{0.3f, 0.2f, 0.195f, 12.0f};
    const DifferentialKinematics kin(11.918f);
    const WheelVoltageConstraint constraint(model, kin);

    // `reversed` progresses along the SAME ordered path, so ds/dt stays |v| and
    // dk/dt keeps its sign while the wheel speeds flip. The dk/ds term therefore
    // flips with the speed and the whole window negates. Spelling the term v*v
    // leaves it alone instead, and nothing else in this file can tell the two
    // apart: the reversed profile test carries only grip and wheel-speed
    // constraints, and neither reads the sign of speed.
    int checks = 0;
    for (float k : {0.0f, 0.02f, -0.05f, 0.15f}) {
        for (float rate : {0.0f, 0.004f, -0.01f}) {
            for (float v : {6.0f, 24.0f, 48.0f}) {
                const AccelerationLimits ahead =
                    constraint.accelerationLimits(Pose{}, k, v, rate);
                const AccelerationLimits back =
                    constraint.accelerationLimits(Pose{}, k, -v, rate);
                if (!std::isfinite(ahead.min) || !std::isfinite(ahead.max)) continue;
                CHECK_NEAR(back.max, -ahead.min, 1e-2);
                CHECK_NEAR(back.min, -ahead.max, 1e-2);
                checks++;
            }
        }
    }
    CHECK(checks >= 30);
}

CHAD_TEST(trajectory, theDkDsTermIsPricedOverTheIntervalNotAtTheNode) {
    // One interval, driven at the window's own upper edge, with the wheel
    // voltages reconstructed from the two endpoint states by finite difference
    // -- no model of the wheel acceleration at all. The edge of the window is
    // meant to BE the rail, so this is asserted from both sides: a term priced
    // at the node instead of over the interval lands off it.
    const WheelVoltageModel model{0.3f, 0.2f, 0.195f, 12.0f};
    const float track = 11.918f;
    const DifferentialKinematics kin(track);
    const WheelVoltageConstraint constraint(model, kin);

    const float ds = 1.0f;
    const float k0 = 0.020f;
    const float k1 = 0.022f;
    const float v0 = 20.0f;

    // Same fixed point the generator runs: the effective rate depends on the
    // speed the interval reaches, which depends on the acceleration it allows.
    float v1 = v0;
    float a = 0.0f;
    for (int i = 0; i < 20; i++) {
        const float rate = v0 * (v0 + v1) * (k1 - k0) / (2.0f * ds * v1 * v1);
        const AccelerationLimits limits =
            constraint.accelerationLimits(Pose{}, k1, v1, rate);
        CHECK(limits.feasible());
        a = limits.max;
        CHECK_FINITE(a);
        v1 = std::sqrt(std::fmax(0.0f, v0 * v0 + 2.0f * a * ds));
    }

    const float dt = 2.0f * ds / (v0 + v1);
    float worst = 0.0f;
    for (float side : {-1.0f, 1.0f}) {
        const float half = side * 0.5f * track;
        const float w0 = v0 * (1.0f + half * k0);
        const float w1 = v1 * (1.0f + half * k1);
        const float accel = (w1 - w0) / dt;
        for (float w : {w0, w1}) {
            worst = std::fmax(worst,
                              std::fabs(model.kS * sign(w) + model.kV * w + model.kA * accel));
        }
    }
    std::printf("[interval] a=%.4f in/s^2 -> v1=%.4f, worst reconstructed %.4f V\n", a, v1,
                worst);
    CHECK_NEAR(worst, model.maxVoltage, 0.02);
}

namespace {

/// Brakes hard only above 30 in/s, so a profile that never gets there is
/// unaffected. A stale acceleration window made it fail anyway.
class BrakeBandConstraint : public TrajectoryConstraint {
public:
    float maxVelocity(const Pose&, float, float, float) const override {
        return std::numeric_limits<float>::infinity();
    }
    AccelerationLimits accelerationLimits(const Pose&, float, float speed,
                                          float) const override {
        if (std::fabs(speed) >= 30.0f) return {-400.0f, -50.0f};
        return {-10.0f, 400.0f};
    }
};

}  // namespace

CHAD_TEST(trajectory, aBandTheProfileNeverEntersIsNotInfeasible) {
    const BrakeBandConstraint band;
    TrajectoryConfig config = baseConfig(60.0f, 300.0f);
    config.constraints = {&band};

    const std::vector<Pose> straight = {{0.0f, 0.0f, 0.0f}, {120.0f, 0.0f, 0.0f}};
    const TrajectoryResult result = generateTrajectory(straight, config);
    CHECK(result.error == TrajectoryError::None);
    CHECK(!result.trajectory.isEmpty());
    for (const TrajectoryState& state : result.trajectory.states()) {
        CHECK(std::fabs(state.velocity) < 30.0f + 1e-2f);
    }
}

CHAD_TEST(trajectory, seamOfAConcatenationNeverSamplesBackwards) {
    TrajectoryConfig config = baseConfig(40.0f, 50.0f);
    const std::vector<Pose> legOne = {{0.0f, 0.0f, 0.0f}, {40.0f, 0.0f, 0.0f}};
    const std::vector<Pose> legTwo = {{40.0f, 0.0f, 0.0f}, {40.0f, 40.0f, HALF_PI}};
    const TrajectoryResult first = generateTrajectory(legOne, config);
    const TrajectoryResult second = generateTrajectory(legTwo, config);
    CHECK(first.ok());
    CHECK(second.ok());

    const Trajectory joined = first.trajectory.concatenate(second.trajectory);
    CHECK_NEAR(joined.totalTime(),
               first.trajectory.totalTime() + second.trajectory.totalTime(), 1e-4);

    // Progress is measured along the robot's own heading: the geodesic back to
    // the start is not a path length once the path turns.
    Pose previous = joined.states().front().pose;
    for (int i = 0; i <= 400; i++) {
        const float t = joined.totalTime() * static_cast<float>(i) / 400.0f;
        const TrajectoryState state = joined.sample(t);
        CHECK(state.velocity >= -1e-3f);

        const Pose step = state.pose.relativeTo(previous);
        CHECK(step.x >= -1e-2f);
        previous = state.pose;
    }
}

CHAD_TEST(trajectory, boundarySampleReturnsThatStateExactly) {
    TrajectoryConfig config = baseConfig();
    const TrajectoryResult result = generateTrajectory(curvyPath(), config);
    CHECK(result.ok());

    const std::vector<TrajectoryState>& states = result.trajectory.states();
    float worstAccel = 0.0f;
    for (size_t i = 0; i < states.size(); i++) {
        const TrajectoryState sampled = result.trajectory.sample(states[i].t);
        CHECK_NEAR(sampled.velocity, states[i].velocity, 1e-3);
        CHECK_NEAR(sampled.pose.x, states[i].pose.x, 1e-3);
        CHECK_NEAR(sampled.pose.y, states[i].pose.y, 1e-3);
        worstAccel =
            std::fmax(worstAccel, std::fabs(sampled.acceleration - states[i].acceleration));
    }
    CHECK_NEAR(worstAccel, 0.0f, 1e-3);
}

CHAD_TEST(trajectory, nonFiniteWaypointIsRejected) {
    const float bad = std::numeric_limits<float>::quiet_NaN();
    TrajectoryConfig config = baseConfig();

    const std::vector<Pose> badHeading = {{0.0f, 0.0f, bad}, {40.0f, 0.0f, 0.0f}};
    const std::vector<Pose> badPosition = {{0.0f, bad, 0.0f}, {40.0f, 0.0f, 0.0f}};
    CHECK(generateTrajectory(badHeading, config).error ==
          TrajectoryError::NonFiniteWaypoint);
    CHECK(generateTrajectory(badPosition, config).error ==
          TrajectoryError::NonFiniteWaypoint);

    const TrajectoryResult cubic =
        generateCubicTrajectory({0.0f, 0.0f, bad}, {}, {40.0f, 0.0f, 0.0f}, config);
    CHECK(cubic.error == TrajectoryError::NonFiniteWaypoint);
}

CHAD_TEST(trajectory, anOverscaledTangentCannotRunAwayWithTheHeap) {
    TrajectoryConfig config = baseConfig();
    config.tolerance = {5.0f, 0.05f, 0.0872f, 20};

    std::vector<PathWaypoint> waypoints(3);
    waypoints[0] = {{0.0f, 0.0f, 0.0f}, 2000.0f};
    waypoints[1] = {{1.0f, 0.2f, 0.5f}, 2000.0f};
    waypoints[2] = {{2.0f, 0.0f, 0.0f}, 2000.0f};

    const TrajectoryResult result = generateTrajectory(waypoints, config);
    if (result.ok()) {
        CHECK(result.trajectory.states().size() <= PARAMETERIZE_MAX_POINTS);
    } else {
        CHECK(result.error == TrajectoryError::PathTooDense ||
              result.error == TrajectoryError::MalformedSpline);
    }
}

namespace {

/// Answers differently every time it is asked, so the velocity/acceleration
/// fixed point can never settle. No real constraint behaves like this -- the
/// point is to prove the iteration cap fires and the generator returns, rather
/// than spinning forever on a brain with no watchdog.
class FlipFloppingConstraint : public TrajectoryConstraint {
public:
    float maxVelocity(const Pose&, float, float, float) const override {
        return std::numeric_limits<float>::infinity();
    }
    AccelerationLimits accelerationLimits(const Pose&, float, float, float) const override {
        // Tighter by a RATIO than the answer it gave last time. A fixed
        // decrement with a floor settles as soon as it reaches the floor, so it
        // stopped proving anything the moment the generator started asking more
        // than 33 times: it converged instead of hitting the iteration cap.
        mLimit *= 0.9f;
        return {-400.0f, mLimit};
    }

private:
    mutable float mLimit{400.0f};
};

}  // namespace

CHAD_TEST(trajectory, aConstraintThatNeverSettlesIsGivenUpOn) {
    const FlipFloppingConstraint flip;
    TrajectoryConfig config = baseConfig(60.0f, 400.0f);
    config.constraints = {&flip};

    const std::vector<Pose> straight = {{0.0f, 0.0f, 0.0f}, {120.0f, 0.0f, 0.0f}};
    const TrajectoryResult result = generateTrajectory(straight, config);

    CHECK(result.error == TrajectoryError::ProfileDidNotConverge);
    CHECK(result.trajectory.isEmpty());
}

namespace {

// The robot config.cpp describes, so the numbers below are the ones it sees.
constexpr float ROBOT_TRACK = 11.918f;
constexpr float ROBOT_WHEEL_SPEED = 62.999f;

DriveModel robotModel() {
    return {ROBOT_TRACK, 2.6738f, ROBOT_WHEEL_SPEED, WheelType::Traction, 0.0f};
}

/// A path with no unreasonable waypoint on it: an arc over the top, entered and
/// left at 45 degrees. Peak |curvature| is about 0.15 1/in, a 6.7 in radius.
std::vector<Pose> wideArc() {
    return {{-50.0f, -50.0f, QUARTER_PI},
            {-10.0f, -20.0f, QUARTER_PI},
            {30.0f, -50.0f, -QUARTER_PI}};
}

}  // namespace

CHAD_TEST(trajectory, voltageConstraintHoldsTheWheelDemandItsOwnModelUnderstates) {
    const DriveModel drive = robotModel();
    const DifferentialKinematics kin(ROBOT_TRACK);
    const CentripetalConstraint grip(drive);
    const DifferentialKinematicsConstraint budget(kin, drive.maxWheelSpeed);

    for (float kAMetre : {0.4f, 0.8f, 2.0f}) {
        // config.cpp's fitted kS and kV, with the kA its phase 1 is measuring.
        const WheelVoltageModel model =
            WheelVoltageModel::fromMetreGains(0.7020f, 6.1287f, kAMetre, 12.0f);
        const WheelVoltageConstraint volts(model, kin);

        TrajectoryConfig config;
        config.maxVelocity = drive.maxWheelSpeed;
        config.maxAcceleration = 120.0f;
        config.tolerance = {0.5f, 0.005f, 0.02f, 20};
        config.constraints = {&grip, &budget, &volts};

        const TrajectoryResult result = generateTrajectory(wideArc(), config);
        CHECK(result.ok());
        if (!result.ok()) continue;

        int over = 0;
        const float worst =
            worstTrueWheelVoltage(model, ROBOT_TRACK, result.trajectory.states(), over);
        std::printf("[volts] kA=%.1f  worst true %.4f V, %d wheel-ends over budget of %zu\n",
                    kAMetre, worst, over, 4 * (result.trajectory.states().size() - 1));
        CHECK(worst <= model.maxVoltage + 0.02f);
        CHECK(over == 0);

        // Reversed, the only case that can tell v*|v| from v*v in the dk/ds
        // term. Nothing else here did: the reversed-path test carries only grip
        // and kinematics, and neither reads the sign of speed. Spelling it v*v
        // -- which reads as the obvious form -- demands 19.17 V of a 12 V rail
        // on this path while leaving every forward path untouched.
        TrajectoryConfig back = config;
        back.reversed = true;
        const TrajectoryResult rev = generateTrajectory(wideArc(), back);
        CHECK(rev.ok());
        if (!rev.ok()) continue;
        int revOver = 0;
        const float revWorst =
            worstTrueWheelVoltage(model, ROBOT_TRACK, rev.trajectory.states(), revOver);
        std::printf("[volts] kA=%.1f  reversed worst true %.4f V, %d over of %zu\n",
                    kAMetre, revWorst, revOver, 4 * (rev.trajectory.states().size() - 1));
        // 0.02 V is float32 noise on a 12 V rail, not headroom: v*v puts this at
        // 19.17 V, and evaluating the interval term at the node instead of over
        // the interval put it at 12.56 V with 67 ends over.
        CHECK(revWorst <= model.maxVoltage + 0.02f);
        CHECK(revOver == 0);
    }
}

CHAD_TEST(trajectory, refiningTheSamplingCannotMakeAStraightLineInfeasible) {
    for (float maxDx : {1.0f, 0.5f, 0.25f, 0.125f}) {
        TrajectoryConfig config;
        config.maxVelocity = ROBOT_WHEEL_SPEED;
        config.maxAcceleration = 60.0f;
        config.tolerance = {maxDx, 0.05f, 0.0872f, 20};

        const std::vector<Pose> line = {{0.0f, 0.0f, HALF_PI}, {0.0f, 48.0f, HALF_PI}};
        const TrajectoryResult result = generateTrajectory(line, config);
        std::printf("[straight] maxDx=%.3f -> error %d, %zu states\n", maxDx,
                    static_cast<int>(result.error), result.trajectory.states().size());
        CHECK(result.ok());
    }
}

CHAD_TEST(trajectory, aVoltageModelWithNoAccelerationTermIsReportedNotIgnored) {
    const DifferentialKinematics kin(ROBOT_TRACK);
    // config.cpp ships kaStraight = 0, so this is what a caller adding the
    // constraint today actually builds.
    const WheelVoltageModel model = WheelVoltageModel::fromMetreGains(0.7020f, 6.1287f, 0.0f);
    const WheelVoltageConstraint volts(model, kin);

    TrajectoryConfig config;
    config.maxVelocity = ROBOT_WHEEL_SPEED;
    config.maxAcceleration = 60.0f;
    config.tolerance = {0.5f, 0.05f, 0.0872f, 20};
    config.constraints = {&volts};

    const TrajectoryResult result = generateTrajectory(wideArc(), config);
    std::printf("[kA=0] error %d, %zu states\n", static_cast<int>(result.error),
                result.trajectory.states().size());
    CHECK(result.error == TrajectoryError::UnusableConstraint);
    CHECK(result.trajectory.isEmpty());
}

CHAD_TEST(trajectory, quinticAutoTangentsCuspOnReasonableHeadings) {
    // Not one unreasonable waypoint here: the auto tangent magnitudes overshoot
    // and the path doubles back inside the robot's own wheelbase.
    const std::vector<Pose> path = {{0.0f, 0.0f, QUARTER_PI},
                                    {24.0f, 0.0f, 0.0f},
                                    {48.0f, 24.0f, HALF_PI},
                                    {72.0f, 24.0f, 0.0f}};
    TrajectoryConfig config;
    config.maxVelocity = ROBOT_WHEEL_SPEED;
    config.maxAcceleration = 60.0f;
    config.tolerance = {0.5f, 0.05f, 0.0872f, 20};

    const TrajectoryResult loop = generateTrajectory(path, config);
    CHECK(loop.ok());
    float peak = 0.0f;
    for (const TrajectoryState& state : loop.trajectory.states()) {
        peak = std::fmax(peak, std::fabs(state.curvature));
    }
    std::printf("[cusp] no ceiling: peak |k| %.4f 1/in (%.3f in radius)\n", peak,
                peak > 0.0f ? 1.0f / peak : 0.0f);
    // A radius under a fifth of the track width, from headings a human would
    // sign off on. Nothing in the stack rejects it, so a caller who cares has to
    // say what "too tight" means for their paths.
    CHECK(peak > 4.0f / ROBOT_TRACK);

    config.maxCurvature = 0.25f;
    const TrajectoryResult capped = generateTrajectory(path, config);
    CHECK(capped.error == TrajectoryError::CuspedPath);
    CHECK(capped.trajectory.isEmpty());

    // The ceiling has to leave the same path alone once it is loose enough, or
    // it is rejecting something other than the loop.
    config.maxCurvature = 2.0f * peak;
    CHECK(generateTrajectory(path, config).ok());
}

CHAD_TEST(trajectory, emittedTimesMatchTheArcLengthAndTheSpeeds) {
    // v is linear on an interval, so its duration is 2*ds/(v0 + v1) exactly.
    // Deriving it as (v1 - v0)/a instead divides one cancelled difference by
    // another; on the nearly flat stretch a voltage-limited profile spends most
    // of its time in, that was 0.2% of dt, and 0.2% of dt is 0.2% of the wheel
    // acceleration the battery is being asked for.
    const DriveModel drive = robotModel();
    const DifferentialKinematics kin(ROBOT_TRACK);
    const CentripetalConstraint grip(drive);
    const DifferentialKinematicsConstraint budget(kin, drive.maxWheelSpeed);
    const WheelVoltageModel model =
        WheelVoltageModel::fromMetreGains(0.7020f, 6.1287f, 2.0f, 12.0f);
    const WheelVoltageConstraint volts(model, kin);

    TrajectoryConfig config;
    config.maxVelocity = drive.maxWheelSpeed;
    config.maxAcceleration = 120.0f;
    config.tolerance = {0.5f, 0.005f, 0.02f, 20};
    config.constraints = {&grip, &budget, &volts};

    const TrajectoryResult result = generateTrajectory(wideArc(), config);
    CHECK(result.ok());
    if (!result.ok()) return;

    const std::vector<TrajectoryState>& states = result.trajectory.states();
    float worstRel = 0.0f;
    int checked = 0;
    for (size_t i = 0; i + 1 < states.size(); i++) {
        const float dt = states[i + 1].t - states[i].t;
        const float ds = arcLengthBetween(states[i].pose, states[i + 1].pose);
        const float sum = std::fabs(states[i].velocity) + std::fabs(states[i + 1].velocity);
        if (!(dt > 1e-5f) || !(ds > 1e-3f) || !(sum > 1e-3f)) continue;
        worstRel = std::fmax(worstRel, std::fabs(dt - 2.0f * ds / sum) / dt);
        checked++;
    }
    std::printf("[timebase] %d intervals, worst |dt - 2ds/(v0+v1)|/dt = %.3e\n", checked,
                worstRel);
    CHECK(checked > 300);
    // 6.5e-5 measured, and that is the accumulated float32 timestamp, not the
    // formula: deriving dt from the acceleration puts it at 2.0e-3.
    CHECK(worstRel < 4e-4f);
}
