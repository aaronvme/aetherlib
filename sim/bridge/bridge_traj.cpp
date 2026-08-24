// chad-sim — the constrained-trajectory surface.
//
// include/chad/movement_control/{spline,trajectory,trajectory_constraint,
// trajectory_generator}.hpp had never run outside host unit tests. This exposes
// the generator and Chassis::followTrajectory to the browser and to
// sim/test/traj.mjs.
//
// Deliberately thin. The trajectory is handed out as a raw float buffer rather
// than as a summary, because the interesting checks -- does every state fit in
// 12 V, does dtheta/dt equal v*curvature -- have to be recomputed from physics
// on the other side. A C++ audit that called the constraint classes would only
// prove the formula was typed twice.
#include <emscripten/emscripten.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "../platform/sim_sched.hpp"

#include "chad/movement_control/trajectory_generator.hpp"

#include "gamespecific/config.hpp"

#include "bridge_internal.hpp"

using namespace simbridge;

namespace {

// ----------------------------------------------------------------------------
// Generation
// ----------------------------------------------------------------------------

struct TrajCfg {
    float maxVel{0.0f};        ///< 0 = the drive model's free wheel speed
    float maxAccel{60.0f};
    float startVel{0.0f};
    float endVel{0.0f};
    bool reversed{false};

    /// CentripetalConstraint + DifferentialKinematicsConstraint from
    /// chad::drivetrain.model(), i.e. what chassis.hpp tells callers to use.
    bool driveConstraints{true};
    /// 0 = whatever WheelType the robot config declares. Overriding it is how
    /// the test shows the grip term is live but never the binding one here.
    float gripLimit{0.0f};
    float wheelSpeedLimit{0.0f};  ///< 0 = model.maxWheelSpeed

    bool voltageConstraint{false};
    /// Metre-fitted, like FeedforwardGains: V per m/s and V per m/s^2.
    float kS{0.7020f};
    float kV{6.1287f};
    float kA{0.0f};
    float maxVoltage{12.0f};

    float maxDx{5.0f};
    float maxDy{0.05f};
    float maxDtheta{0.0872f};

    /// Clamped cubic through the interior points instead of a quintic through
    /// every waypoint pose.
    bool cubic{false};
    float startTangent{0.0f};
    float endTangent{0.0f};
};

TrajCfg gCfg;
std::vector<chad::PathWaypoint> gWaypoints;
chad::TrajectoryResult gResult;
std::string gJson;

constexpr int kStateStride = 7;  // t, v, a, x, y, theta, curvature
std::vector<float> gStates;
std::vector<float> gSamples;

const char* errorName(chad::TrajectoryError e) {
    switch (e) {
        case chad::TrajectoryError::None:                 return "none";
        case chad::TrajectoryError::NoWaypoints:          return "noWaypoints";
        case chad::TrajectoryError::BadConfig:            return "badConfig";
        case chad::TrajectoryError::TooManyWaypoints:     return "tooManyWaypoints";
        case chad::TrajectoryError::DegenerateWaypoints:  return "degenerateWaypoints";
        case chad::TrajectoryError::MalformedSpline:      return "malformedSpline";
        case chad::TrajectoryError::PathTooDense:         return "pathTooDense";
        case chad::TrajectoryError::NonFiniteWaypoint:    return "nonFiniteWaypoint";
        case chad::TrajectoryError::InfeasibleConstraint: return "infeasibleConstraint";
        case chad::TrajectoryError::StalledProfile:       return "stalledProfile";
        case chad::TrajectoryError::ProfileDidNotConverge: return "didNotConverge";
        case chad::TrajectoryError::UnusableConstraint:   return "unusableConstraint";
        case chad::TrajectoryError::CuspedPath:           return "cuspedPath";
    }
    return "unknown";
}

const char* wheelTypeName(chad::WheelType w) {
    switch (w) {
        case chad::WheelType::Omni:     return "omni";
        case chad::WheelType::Traction: return "traction";
        case chad::WheelType::Mixed:    return "mixed";
    }
    return "unknown";
}

/// The model generation runs against: the robot's own, with only the explicit
/// test overrides applied.
chad::DriveModel effectiveModel() {
    chad::DriveModel m = chad::drivetrain.model();
    if (gCfg.gripLimit > 0.0f) m.gripLimitOverride = gCfg.gripLimit;
    if (gCfg.wheelSpeedLimit > 0.0f) m.maxWheelSpeed = gCfg.wheelSpeedLimit;
    return m;
}

void fillStates(std::vector<float>& out, const std::vector<chad::TrajectoryState>& states) {
    out.clear();
    out.reserve(states.size() * kStateStride);
    for (const chad::TrajectoryState& s : states) {
        out.push_back(s.t);
        out.push_back(s.velocity);
        out.push_back(s.acceleration);
        out.push_back(s.pose.x);
        out.push_back(s.pose.y);
        out.push_back(s.pose.theta);
        out.push_back(s.curvature);
    }
}

// ----------------------------------------------------------------------------
// Following
// ----------------------------------------------------------------------------

chad::TrajectoryFollowParams gFollow;
sim::TaskId gTrajTask = sim::kInvalidTask;

}  // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE
void sim_traj_clear() { gWaypoints.clear(); }

/// Append a waypoint. Heading is a VEX bearing in degrees, so the browser and
/// the route DSL agree; `tangent` <= 0 lets the spline pick its own magnitude.
EMSCRIPTEN_KEEPALIVE
void sim_traj_waypoint(float x, float y, float headingDeg, float tangent) {
    gWaypoints.push_back({{x, y, chad::vexToStdRad(headingDeg)}, tangent});
}

EMSCRIPTEN_KEEPALIVE
int sim_traj_waypoint_count() { return static_cast<int>(gWaypoints.size()); }

EMSCRIPTEN_KEEPALIVE
void sim_traj_defaults() {
    gCfg = TrajCfg{};
    gFollow = chad::TrajectoryFollowParams{};
}

EMSCRIPTEN_KEEPALIVE
void sim_traj_set(const char* name, float value) {
    const std::string k(name);
    if      (k == "maxVel")       gCfg.maxVel = value;
    else if (k == "maxAccel")     gCfg.maxAccel = value;
    else if (k == "startVel")     gCfg.startVel = value;
    else if (k == "endVel")       gCfg.endVel = value;
    else if (k == "reversed")     gCfg.reversed = (value != 0.0f);
    else if (k == "driveConstraints") gCfg.driveConstraints = (value != 0.0f);
    else if (k == "gripLimit")    gCfg.gripLimit = value;
    else if (k == "wheelSpeedLimit") gCfg.wheelSpeedLimit = value;
    else if (k == "voltageConstraint") gCfg.voltageConstraint = (value != 0.0f);
    else if (k == "kS")           gCfg.kS = value;
    else if (k == "kV")           gCfg.kV = value;
    else if (k == "kA")           gCfg.kA = value;
    else if (k == "maxVoltage")   gCfg.maxVoltage = value;
    else if (k == "maxDx")        gCfg.maxDx = value;
    else if (k == "maxDy")        gCfg.maxDy = value;
    else if (k == "maxDtheta")    gCfg.maxDtheta = value;
    else if (k == "cubic")        gCfg.cubic = (value != 0.0f);
    else if (k == "startTangent") gCfg.startTangent = value;
    else if (k == "endTangent")   gCfg.endTangent = value;
}

EMSCRIPTEN_KEEPALIVE
void sim_traj_follow_set(const char* name, float value) {
    const std::string k(name);
    if      (k == "b")            gFollow.b = value;
    else if (k == "zeta")         gFollow.zeta = value;
    else if (k == "maxSpeed")     gFollow.maxSpeed = value;
    else if (k == "targetTol")    gFollow.targetTolerance = value;
    else if (k == "headingTol")   gFollow.headingTolerance = value;
    else if (k == "stallSpeed")   gFollow.stallSpeed = value;
    else if (k == "velCtrl")      gFollow.useVelocityController = (value != 0.0f);
    else if (k == "timeScale")    gFollow.timeScale = value;
    else if (k == "holdAtEnd")    gFollow.holdAtEnd = (value != 0.0f);
    else if (k == "settleMs")     gFollow.settleTimeMs = static_cast<std::uint32_t>(value);
    else if (k == "ffKv")         gFollow.ffGains.kV = value;
    else if (k == "ffKs")         gFollow.ffGains.ksStraight = value;
    else if (k == "ffKa")         gFollow.ffGains.kaStraight = value;
    else if (k == "ffKp")         gFollow.ffGains.kP = value;
    else if (k == "ffKi")         gFollow.ffGains.kI = value;
}

/// Generate, and return the TrajectoryError as an int (0 == None).
EMSCRIPTEN_KEEPALIVE
int sim_traj_generate() {
    const chad::DriveModel model = effectiveModel();

    chad::TrajectoryConfig config;
    config.maxVelocity = (gCfg.maxVel > 0.0f) ? gCfg.maxVel : model.maxWheelSpeed;
    config.maxAcceleration = gCfg.maxAccel;
    config.startVelocity = gCfg.startVel;
    config.endVelocity = gCfg.endVel;
    config.reversed = gCfg.reversed;
    config.tolerance = {gCfg.maxDx, gCfg.maxDy, gCfg.maxDtheta, 20};

    // Stack-owned, because TrajectoryConfig::constraints is non-owning and only
    // has to outlive the generate call below.
    chad::DrivetrainConstraints drive(model);
    const chad::WheelVoltageModel vm = chad::WheelVoltageModel::fromMetreGains(
        gCfg.kS, gCfg.kV, gCfg.kA, gCfg.maxVoltage);
    chad::WheelVoltageConstraint voltage(vm, chad::DifferentialKinematics(model.trackWidth));

    if (gCfg.driveConstraints) config.constraints = drive.list();
    if (gCfg.voltageConstraint) config.constraints.push_back(&voltage);

    if (gCfg.cubic) {
        if (gWaypoints.size() < 2) {
            gResult = {};
            gResult.error = chad::TrajectoryError::NoWaypoints;
        } else {
            std::vector<chad::Translation2d> interior;
            for (std::size_t i = 1; i + 1 < gWaypoints.size(); i++) {
                interior.push_back(gWaypoints[i].pose.translation());
            }
            gResult = chad::generateCubicTrajectory(
                gWaypoints.front().pose, interior, gWaypoints.back().pose, config,
                gCfg.startTangent, gCfg.endTangent);
        }
    } else {
        gResult = chad::generateTrajectory(gWaypoints, config);
    }

    fillStates(gStates, gResult.trajectory.states());
    gSamples.clear();
    return static_cast<int>(gResult.error);
}

EMSCRIPTEN_KEEPALIVE
int sim_traj_count() { return static_cast<int>(gResult.trajectory.states().size()); }

EMSCRIPTEN_KEEPALIVE
int sim_traj_stride() { return kStateStride; }

EMSCRIPTEN_KEEPALIVE
float* sim_traj_states_ptr() { return gStates.data(); }

/// Re-read the trajectory through Trajectory::sample() at `dt` seconds, into a
/// second buffer with the same stride. This is the follower's actual view of
/// the path -- followTrajectory samples by time, never by state index.
EMSCRIPTEN_KEEPALIVE
int sim_traj_resample(float dt) {
    gSamples.clear();
    if (gResult.trajectory.isEmpty() || !(dt > 0.0f)) return 0;
    std::vector<chad::TrajectoryState> out;
    const float total = gResult.trajectory.totalTime();
    for (float t = 0.0f; t < total + dt; t += dt) {
        out.push_back(gResult.trajectory.sample(t > total ? total : t));
    }
    fillStates(gSamples, out);
    return static_cast<int>(out.size());
}

EMSCRIPTEN_KEEPALIVE
float* sim_traj_samples_ptr() { return gSamples.data(); }

EMSCRIPTEN_KEEPALIVE
const char* sim_traj_result_json() {
    const auto& states = gResult.trajectory.states();
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "{\"error\":\"%s\",\"errorCode\":%d,\"count\":%d,\"totalTime\":%.6f,"
        "\"reversed\":%s,\"maxVelocity\":%.6f,\"maxAcceleration\":%.6f}",
        errorName(gResult.error), static_cast<int>(gResult.error),
        static_cast<int>(states.size()), gResult.trajectory.totalTime(),
        gCfg.reversed ? "true" : "false",
        (gCfg.maxVel > 0.0f) ? gCfg.maxVel : effectiveModel().maxWheelSpeed,
        gCfg.maxAccel);
    gJson.assign(buf);
    return gJson.c_str();
}

/// The drive base as the LIBRARY reports it, straight off
/// chad::drivetrain.model(). `gripLimit` is what WheelType resolves to, which is
/// the traction path the generator reasons about.
EMSCRIPTEN_KEEPALIVE
const char* sim_traj_model_json() {
    const chad::DriveModel raw = chad::drivetrain.model();
    const chad::DriveModel eff = effectiveModel();
    char buf[640];
    std::snprintf(buf, sizeof(buf),
        "{\"trackWidth\":%.6f,\"wheelDiameter\":%.6f,\"maxWheelSpeed\":%.6f,"
        "\"wheels\":%d,\"wheelsName\":\"%s\",\"gripLimit\":%.6f,"
        "\"gripLimitOverride\":%.6f,\"rpm\":%.4f,"
        "\"effGripLimit\":%.6f,\"effMaxWheelSpeed\":%.6f,"
        "\"kS\":%.6f,\"kV\":%.6f,\"kA\":%.6f,\"maxVoltage\":%.4f,"
        "\"voltageConstraint\":%s,\"driveConstraints\":%s}",
        raw.trackWidth, raw.wheelDiameter, raw.maxWheelSpeed,
        static_cast<int>(raw.wheels), wheelTypeName(raw.wheels), raw.gripLimit(),
        raw.gripLimitOverride, chad::drivetrain.rpm,
        eff.gripLimit(), eff.maxWheelSpeed,
        gCfg.kS, gCfg.kV, gCfg.kA, gCfg.maxVoltage,
        gCfg.voltageConstraint ? "true" : "false",
        gCfg.driveConstraints ? "true" : "false");
    gJson.assign(buf);
    return gJson.c_str();
}

/// What each constraint claims at one curvature, so the test can name the term
/// that binds. Compared against an independent closed form on the JS side --
/// they are two derivations of the same physics, and a disagreement is a bug in
/// one of them.
EMSCRIPTEN_KEEPALIVE
const char* sim_traj_probe_json(float curvature) {
    const chad::DriveModel model = effectiveModel();
    const chad::DrivetrainConstraints drive(model);
    const chad::Pose origin{};

    // "no opinion" is infinity, which snprintf renders as `inf` -- not JSON, and
    // JSON.parse throws on it. 1e30 survives the round trip and is still
    // unmistakably "no limit".
    const auto finite = [](float v) { return std::isfinite(v) ? v : 1e30f; };
    const float cent = finite(drive.centripetal().maxVelocity(origin, curvature, 0.0f, 0.0f));
    const float kin = finite(drive.kinematics().maxVelocity(origin, curvature, 0.0f, 0.0f));

    char buf[320];
    std::snprintf(buf, sizeof(buf),
        "{\"curvature\":%.8f,\"centripetal\":%.6g,\"kinematics\":%.6g,"
        "\"modelCombined\":%.6g}",
        curvature, cent, kin, finite(model.maxSpeedForCurvature(curvature)));
    gJson.assign(buf);
    return gJson.c_str();
}

// ----------------------------------------------------------------------------
// Following
// ----------------------------------------------------------------------------

/// Follow the generated trajectory on its own fiber, so the browser keeps
/// driving the clock -- the same reason sim_lqr_follow spawns one.
EMSCRIPTEN_KEEPALIVE
void sim_traj_run(float timeoutMs) {
    if (gResult.trajectory.isEmpty()) return;
    if (gTrajTask != sim::kInvalidTask) sim::sched().kill(gTrajTask);
    gManualActive = false;
    // Snapshot rather than reading the global inside the fiber: the fiber does
    // not run until the next scheduler tick, so a regenerate in between would
    // otherwise swap the trajectory out from under it.
    auto snapshot = std::make_shared<chad::Trajectory>(gResult.trajectory);
    const chad::TrajectoryFollowParams params = gFollow;
    gTrajTask = sim::sched().spawn([snapshot, params, timeoutMs]() {
        chad::chassis.followTrajectory(*snapshot, timeoutMs, params, false);
    }, "traj");
}

EMSCRIPTEN_KEEPALIVE
int sim_traj_running() {
    return (gTrajTask != sim::kInvalidTask &&
            sim::sched().stateOf(gTrajTask) != sim::TaskState::Finished) ? 1 : 0;
}

EMSCRIPTEN_KEEPALIVE
void sim_traj_cancel() {
    chad::chassis.cancelMotion();
    if (gTrajTask != sim::kInvalidTask) {
        sim::sched().kill(gTrajTask);
        gTrajTask = sim::kInvalidTask;
    }
}

EMSCRIPTEN_KEEPALIVE
const char* sim_traj_follow_json() {
    char buf[320];
    std::snprintf(buf, sizeof(buf),
        "{\"outcome\":\"%s\",\"arrived\":%s,\"running\":%s,\"settled\":%s,"
        "\"timeScale\":%.4f,\"velCtrl\":%s,\"b\":%.4f}",
        chad::outcomeName(chad::chassis.lastOutcome()),
        chad::chassis.lastMotionArrived() ? "true" : "false",
        sim_traj_running() ? "true" : "false",
        chad::chassis.isSettled() ? "true" : "false",
        gFollow.timeScale, gFollow.useVelocityController ? "true" : "false",
        gFollow.b);
    gJson.assign(buf);
    return gJson.c_str();
}

}  // extern "C"
