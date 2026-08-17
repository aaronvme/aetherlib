// riku-sim — C API surface consumed by the browser.
//
// Everything here is a thin adapter. The algorithms under test are the real
// riku objects declared in gamespecific/config.cpp; this file only starts
// them, advances virtual time, and marshals state out to JavaScript.
#include <emscripten/emscripten.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../platform/sim_runtime.hpp"
#include "../platform/sim_sched.hpp"
#include "../world/field.hpp"
#include "../world/world.hpp"

#include "riku/movement_control/ltv.hpp"

#include "gamespecific/autons.hpp"
#include "gamespecific/config.hpp"

namespace {

// --- localization mode bits -------------------------------------------------
constexpr int kLocMcl = 1;
constexpr int kLocRcl = 2;

int  gLocMode = 0;
bool gStarted = false;

// Pinned RNG seed. Held here so sim_reset() can restore the exact same
// starting conditions rather than continuing from a drifted generator.
std::uint32_t gSeed = 0xC0FFEEu;

// Manual drive command in volts, applied when no routine is running.
float gManualLeft = 0.0f;
float gManualRight = 0.0f;
bool  gManualActive = false;

sim::TaskId gRouteTask = sim::kInvalidTask;

std::string gStateJson;
std::vector<float> gParticleBuf(riku::MclTracking::PARTICLE_MAX * 4);
int gParticleCount = 0;

// Ground-truth and estimate history for the path overlays.
struct Trail { std::vector<float> xy; };
Trail gTrueTrail, gOdomTrail;
constexpr std::size_t kMaxTrail = 4000;

void pushTrail(Trail& t, float x, float y) {
    if (t.xy.size() >= kMaxTrail * 2) {
        t.xy.erase(t.xy.begin(), t.xy.begin() + 2);
    }
    t.xy.push_back(x);
    t.xy.push_back(y);
}

struct AutonEntry { const char* name; void (*fn)(); };
const AutonEntry kAutons[] = {
    {"Example",   riku::autonExample},
    {"Red Left",  riku::autonRedLeft},
    {"Blue Left", riku::autonBlueLeft},
    {"Skills",    riku::autonSkills},
};
constexpr int kAutonCount = static_cast<int>(sizeof(kAutons) / sizeof(kAutons[0]));

void applyManualDrive() {
    if (!gManualActive) return;
    riku::drivetrain.setVoltage(gManualLeft / 12.0f * 127.0f,
                                gManualRight / 12.0f * 127.0f);
}

// --- run recorder -----------------------------------------------------------
//
// A flat float ring the browser reads directly out of the heap and plots. One
// row per sample; keep kRecordStride in step with the column list below.

constexpr int kRecordStride = 14;
constexpr int kRecordMax = 20000;          // ~6.5 min at 20 ms
constexpr double kRecordPeriodMs = 20.0;

std::vector<float> gRecord;
double gRecordNextMs = 0.0;
bool   gRecording = true;

void recordSample() {
    if (!gRecording) return;
    const double now = sim::preciseMillis();
    if (now < gRecordNextMs) return;
    gRecordNextMs = now + kRecordPeriodMs;

    if (gRecord.size() >= static_cast<std::size_t>(kRecordMax * kRecordStride)) {
        gRecord.erase(gRecord.begin(), gRecord.begin() + kRecordStride);
    }

    const auto& t = sim::world().truth();
    const riku::Pose odom = riku::chassis.getPose();
    const riku::Pose mcl = riku::chassis.mcl().getRawEstimate();

    const float ex = odom.x - t.x;
    const float ey = odom.y - t.y;

    // Columns: t, trueX, trueY, trueH, odomX, odomY, odomH, mclX, mclY, mclH,
    //          errDist, errHeading, vLin, vAng
    gRecord.push_back(static_cast<float>(now / 1000.0));
    gRecord.push_back(t.x);
    gRecord.push_back(t.y);
    gRecord.push_back(riku::stdRadToVexDeg(t.theta));
    gRecord.push_back(odom.x);
    gRecord.push_back(odom.y);
    gRecord.push_back(riku::stdRadToVexDeg(odom.theta));
    gRecord.push_back(mcl.x);
    gRecord.push_back(mcl.y);
    gRecord.push_back(riku::stdRadToVexDeg(mcl.theta));
    gRecord.push_back(std::sqrt(ex * ex + ey * ey));
    gRecord.push_back(riku::angleDiffRad(t.theta, odom.theta) * riku::RAD_TO_DEG);
    gRecord.push_back(t.vLinIn);
    gRecord.push_back(t.vAngRad * riku::RAD_TO_DEG);
}

}  // namespace

extern "C" {

// ============================================================================
// Lifecycle
// ============================================================================

EMSCRIPTEN_KEEPALIVE
void sim_init() {
    if (gStarted) return;
    gStarted = true;

    sim::seedRng(gSeed);
    sim::world().init();

    // Same registration order as src/main.cpp so indices line up with the
    // brain's auton selector.
    for (const auto& a : kAutons) riku::selector.registerAuton(a.name, a.fn);
    riku::selector.init();

    riku::registerFieldObstacles();

    // Odometry only for now; MCL/RCL are opt-in from the UI.
    riku::chassis.odom().startBackgroundOdom();
}

EMSCRIPTEN_KEEPALIVE void sim_set_localization(int mode);

EMSCRIPTEN_KEEPALIVE
void sim_reset(float x, float y, float thetaDeg) {
    const riku::Pose p{x, y, riku::vexToStdRad(thetaDeg)};
    const int wasLoc = gLocMode;

    // Tear the run down through the library's own lifecycle calls so its
    // pros::Task pointers are released properly, rather than killing the
    // fibers out from under it.
    riku::chassis.cancelMotion();
    if (gRouteTask != sim::kInvalidTask) {
        sim::sched().kill(gRouteTask);
        gRouteTask = sim::kInvalidTask;
    }
    riku::chassis.mcl().stopTracking();
    riku::chassis.rcl().stopTracking();
    gLocMode = 0;

    // Rewind the clock and rephase every background loop so they all become due
    // at t=0. That is what makes a reset reproducible: odometry, MCL and the
    // motion queue interleave the same way on every run.
    //
    // Deliberately NOT sched().reset() -- Chassis owns a long-lived motion task
    // and holds the pros::Task pointer itself, so destroying fibers here leaves
    // it notifying a task that no longer exists and motions stop running.
    // Odometry keeps its task for the same reason.
    sim::resetClock();
    sim::sched().resyncSleepers();

    sim::seedRng(gSeed);
    riku::chassis.mcl().setSeed(gSeed);

    sim::world().resetTo(p);

    // Re-baseline odometry against the freshly reset hardware BEFORE setting
    // the pose. setPose() deliberately leaves mPrevHeadingRad alone (so an MCL
    // sync cannot fake a rotation), which means the first tick after a reset
    // would otherwise measure dTheta against the previous run's final heading.
    // reset() is the call that re-reads the IMU and drops the stale delta.
    riku::chassis.odom().reset();
    riku::chassis.setPose(p);
    riku::chassis.mcl().setPose(p);

    gTrueTrail.xy.clear();
    gOdomTrail.xy.clear();
    gRecord.clear();
    gRecordNextMs = 0.0;
    gManualLeft = gManualRight = 0.0f;

    // Odometry's task was never stopped, so this only re-arms the enabled flag.
    riku::chassis.odom().startBackgroundOdom();
    if (wasLoc != 0) sim_set_localization(wasLoc);
}

EMSCRIPTEN_KEEPALIVE
void sim_teleport(float x, float y, float thetaDeg) {
    // Kidnapped-robot test: ground truth moves, the estimator is not told.
    sim::world().teleport({x, y, riku::vexToStdRad(thetaDeg)});
}

// ============================================================================
// Time
// ============================================================================

EMSCRIPTEN_KEEPALIVE
void sim_step(float ms) {
    applyManualDrive();
    sim::sched().runFor(ms);

    const auto& t = sim::world().truth();
    pushTrail(gTrueTrail, t.x, t.y);
    const riku::Pose op = riku::chassis.getPose();
    pushTrail(gOdomTrail, op.x, op.y);

    recordSample();
}

// ============================================================================
// Control
// ============================================================================

EMSCRIPTEN_KEEPALIVE
void sim_set_drive(float leftVolts, float rightVolts) {
    gManualLeft = leftVolts;
    gManualRight = rightVolts;
    gManualActive = true;
}

EMSCRIPTEN_KEEPALIVE
void sim_set_localization(int mode) {
    if ((mode & kLocMcl) && !(gLocMode & kLocMcl)) {
        riku::chassis.mcl().setPose(riku::chassis.getPose());
        riku::chassis.mcl().startTracking();
    } else if (!(mode & kLocMcl) && (gLocMode & kLocMcl)) {
        riku::chassis.mcl().stopTracking();
    }

    if ((mode & kLocRcl) && !(gLocMode & kLocRcl)) {
        riku::chassis.rcl().startTracking();
    } else if (!(mode & kLocRcl) && (gLocMode & kLocRcl)) {
        riku::chassis.rcl().stopTracking();
    }
    gLocMode = mode;
}

EMSCRIPTEN_KEEPALIVE
int sim_auton_count() { return kAutonCount; }

EMSCRIPTEN_KEEPALIVE
const char* sim_auton_name(int i) {
    if (i < 0 || i >= kAutonCount) return "";
    return kAutons[i].name;
}

EMSCRIPTEN_KEEPALIVE
void sim_run_auton(int index) {
    if (index < 0 || index >= kAutonCount) return;
    if (gRouteTask != sim::kInvalidTask) sim::sched().kill(gRouteTask);

    gManualActive = false;
    auto fn = kAutons[index].fn;
    // Runs on its own fiber, so blocking calls like waitUntilSettled() behave
    // exactly as they do on the brain.
    gRouteTask = sim::sched().spawn([fn]() { fn(); }, "route");
}

EMSCRIPTEN_KEEPALIVE
void sim_stop_auton() {
    if (gRouteTask != sim::kInvalidTask) {
        sim::sched().kill(gRouteTask);
        gRouteTask = sim::kInvalidTask;
    }
    riku::chassis.cancelMotion();
    gManualActive = true;
    gManualLeft = gManualRight = 0.0f;
}

EMSCRIPTEN_KEEPALIVE
int sim_route_running() {
    return (gRouteTask != sim::kInvalidTask &&
            sim::sched().stateOf(gRouteTask) != sim::TaskState::Finished) ? 1 : 0;
}

// ============================================================================
// Route DSL
// ============================================================================
//
// The browser parses route text into a flat opcode list and pushes it here.
// Execution happens on its own fiber calling the real Chassis API, so a route
// written in the editor exercises exactly the same code path as a compiled
// auton -- including the motion queue and waitUntilSettled().

namespace {

enum RouteOp {
    OP_SET_POSE = 0,
    OP_MOVE_TO,
    OP_TURN_TO_HEADING,
    OP_TURN_TO_POINT,
    OP_DRIVE,            // signed distance; negative drives backward
    OP_BOOMERANG,
    OP_ARC,
    OP_SWING,
    OP_DELAY,
    OP_WAIT_SETTLED,
    OP_LATERAL_GAINS,
    OP_ANGULAR_GAINS,
    OP_MAX_SPEED,
    OP_FOLLOW_PATH,
};

struct RouteCmd {
    int op;
    float a, b, c, d;
};

std::vector<RouteCmd> gRoute;
int gRouteIndex = -1;
float gRouteMaxSpeed = 127.0f;

// RAMSETE gains, tunable from the browser via sim_set_param.
riku::RAMSETEParams gRamsete;

// Sampled trajectory points, pushed by the browser before a run. An
// OP_FOLLOW_PATH command names a [start, count) slice of this buffer, so one
// route can follow several paths.
std::vector<riku::Pose> gPath;

// --- LTV / LQR follower -----------------------------------------------------
//
// Nothing in the project constructs an LTVPathFollower, so it ships untested.
// The sim builds one on demand so the controller can actually be exercised.
// Its trajectory arrives as the same "P = {...} / V = {...}" text blob
// prepare_trajectory() parses on a robot -- positions in METRES, velocities in
// m/s and rad/s, one sample per 10 ms tick.

riku::LTVPathFollower* gLtv = nullptr;
std::string gLtvPath;

// Feedforward gains are settable so they can be tuned in the sim rather than
// guessed. kV is volts per m/s: at 12 V this drivetrain runs about 1.95 m/s,
// so the physically correct value is near 6.2, not the token default.
// kV defaults to the value implied by the drivetrain itself: 12 V produces
// 450 rpm on a 3.25 in wheel, i.e. ~1.95 m/s, so kV ~= 12 / 1.95 = 6.2 V per
// m/s. Guessing low here shows up as a systematic along-track deficit, not as
// a controller fault -- 0.9 leaves the path 9 in short, 6.2 lands within 0.5.
riku::FeedforwardGains gLtvGains{
    .kV = 6.2f,
    .kaStraight = 0.0f,
    .kaTurn = 0.0f,
    .ksStraight = 0.3f,
    .ksTurn = 0.0f,
    .kP = 2.0f,
};

riku::LTVPathFollower& ltv() {
    if (gLtv == nullptr) {
        gLtv = new riku::LTVPathFollower(riku::chassis, riku::drivetrain, gLtvGains);
    }
    return *gLtv;
}

/// Rebuild the follower so new gains take effect (they are captured by value).
void rebuildLtv() {
    delete gLtv;
    gLtv = nullptr;
}

void execRoute() {
    for (std::size_t i = 0; i < gRoute.size(); i++) {
        gRouteIndex = static_cast<int>(i);
        const RouteCmd& c = gRoute[i];
        const float timeout = (c.d > 0.0f) ? c.d : 0.0f;

        switch (c.op) {
            case OP_SET_POSE: {
                const riku::Pose p{c.a, c.b, riku::vexToStdRad(c.c)};
                sim::world().resetTo(p);
                riku::chassis.setPose(p);
                riku::chassis.mcl().setPose(p);
                break;
            }
            case OP_MOVE_TO: {
                riku::MoveToPointParams params;
                params.forwards = (c.c >= 0.0f);
                params.maxSpeed = gRouteMaxSpeed;
                riku::chassis.moveToPoint(c.a, c.b, timeout, params);
                break;
            }
            case OP_TURN_TO_HEADING: {
                riku::TurntoHeadingParams params;
                params.maxSpeed = gRouteMaxSpeed;
                riku::chassis.turnToHeading(c.a, timeout, params);
                break;
            }
            case OP_TURN_TO_POINT: {
                riku::TurnToPointParams params;
                params.maxSpeed = gRouteMaxSpeed;
                riku::chassis.turnToPoint(c.a, c.b, timeout, params);
                break;
            }
            case OP_DRIVE: {
                riku::MoveDistanceParams params;
                params.forwards = (c.a >= 0.0f);
                params.maxSpeed = gRouteMaxSpeed;
                riku::chassis.moveDistance(std::fabs(c.a), timeout, params);
                break;
            }
            case OP_BOOMERANG: {
                riku::BoomerangParams params;
                params.maxSpeed = gRouteMaxSpeed;
                riku::chassis.moveBoomerang(c.a, c.b, c.c, timeout, params);
                break;
            }
            case OP_ARC: {
                riku::ArcParams params;
                params.maxSpeed = gRouteMaxSpeed;
                riku::chassis.moveArc(c.a, c.b, c.c, timeout, params);
                break;
            }
            case OP_SWING: {
                riku::SwingParams params;
                params.maxSpeed = gRouteMaxSpeed;
                const auto side = (c.b >= 0.5f) ? riku::SwingSide::Right
                                                : riku::SwingSide::Left;
                riku::chassis.swingToHeading(c.a, side, timeout, params);
                break;
            }
            case OP_DELAY:
                pros::delay(static_cast<std::uint32_t>(c.a));
                break;
            case OP_WAIT_SETTLED:
                riku::chassis.waitUntilSettled();
                break;
            case OP_LATERAL_GAINS:
                riku::chassis.setLateralGains({c.a, c.b, c.c});
                break;
            case OP_ANGULAR_GAINS:
                riku::chassis.setAngularGains({c.a, c.b, c.c});
                break;
            case OP_MAX_SPEED:
                gRouteMaxSpeed = c.a;
                break;
            case OP_FOLLOW_PATH: {
                const int start = static_cast<int>(c.a);
                const int count = static_cast<int>(c.b);
                if (start < 0 || count <= 0 ||
                    start + count > static_cast<int>(gPath.size())) break;
                std::vector<riku::Pose> path(gPath.begin() + start,
                                             gPath.begin() + start + count);
                riku::RAMSETEParams params = gRamsete;
                params.maxSpeed = gRouteMaxSpeed;
                riku::chassis.moveRAMSETE(path, timeout, params);
                break;
            }
            default:
                break;
        }
    }
    gRouteIndex = -1;
}

}  // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE
void sim_route_clear() {
    gRoute.clear();
    gRouteIndex = -1;
    gRouteMaxSpeed = 127.0f;
}

EMSCRIPTEN_KEEPALIVE
void sim_path_clear() { gPath.clear(); }

/// Append one sampled trajectory point. Heading is a VEX bearing in degrees.
EMSCRIPTEN_KEEPALIVE
void sim_path_push(float x, float y, float headingDeg) {
    gPath.push_back({x, y, riku::vexToStdRad(headingDeg)});
}

EMSCRIPTEN_KEEPALIVE
int sim_path_count() { return static_cast<int>(gPath.size()); }

// ============================================================================
// LTV / LQR follower
// ============================================================================

/// Load the trajectory blob (P/V tuple text, metres and m/s).
EMSCRIPTEN_KEEPALIVE
void sim_ltv_set_path(const char* data) { gLtvPath = (data != nullptr) ? data : ""; }

/// Report how long the loaded trajectory is, in inches. Also a cheap check that
/// the blob parsed at all -- 0 means prepare_trajectory() rejected it.
EMSCRIPTEN_KEEPALIVE
float sim_ltv_path_length() {
    return static_cast<float>(ltv().getPathLength(gLtvPath));
}

EMSCRIPTEN_KEEPALIVE
void sim_ltv_run(int backwards, int endCorrection) {
    if (gLtvPath.empty()) return;
    gManualActive = false;
    riku::LTVPathFollower::ltvConfig cfg;
    cfg.backwards = (backwards != 0);
    cfg.end_correction = (endCorrection != 0);
    ltv().followPath(gLtvPath, cfg);
}

EMSCRIPTEN_KEEPALIVE
void sim_ltv_set_gains(float kV, float ksStraight, float kP) {
    gLtvGains.kV = kV;
    gLtvGains.ksStraight = ksStraight;
    gLtvGains.kP = kP;
    rebuildLtv();
}

EMSCRIPTEN_KEEPALIVE
int sim_ltv_running() { return ltv().isRunning() ? 1 : 0; }

EMSCRIPTEN_KEEPALIVE
void sim_ltv_cancel() { ltv().cancel(); }

EMSCRIPTEN_KEEPALIVE
void sim_route_push(int op, float a, float b, float c, float d) {
    gRoute.push_back({op, a, b, c, d});
}

EMSCRIPTEN_KEEPALIVE
void sim_route_run() {
    if (gRoute.empty()) return;
    if (gRouteTask != sim::kInvalidTask) sim::sched().kill(gRouteTask);
    gManualActive = false;
    gRouteMaxSpeed = 127.0f;
    gRouteTask = sim::sched().spawn([]() { execRoute(); }, "dsl-route");
}

/// Index of the command currently executing, or -1 when idle. Drives the
/// editor's active-line highlight.
EMSCRIPTEN_KEEPALIVE
int sim_route_index() { return gRouteIndex; }

}  // extern "C"

// ============================================================================
// Parameters
// ============================================================================

EMSCRIPTEN_KEEPALIVE
void sim_set_param(const char* name, float value) {
    auto& p = sim::world().params();
    const std::string k(name);

    if      (k == "responseTauMs")     p.responseTauMs = value;
    else if (k == "leftScale")         p.leftScale = value;
    else if (k == "rightScale")        p.rightScale = value;
    else if (k == "encoderSlip")       p.encoderSlip = value;
    else if (k == "imuUnderReport")    p.imuUnderReport = value;
    else if (k == "imuDriftDegPerSec") p.imuDriftDegPerSec = value;
    else if (k == "imuNoiseDeg")       p.imuNoiseDeg = value;
    else if (k == "distNoiseMm")       p.distNoiseMm = value;
    else if (k == "distMaxRangeMm")    p.distMaxRangeMm = value;
    else if (k == "distMinRangeMm")    p.distMinRangeMm = value;
    else if (k == "distPeriodMs")      p.distPeriodMs = value;
    else if (k == "distFovDeg")        p.distFovDeg = value;
    else if (k == "distConeRays")      p.distConeRays = static_cast<int>(value);
    else if (k == "distMaxIncidenceDeg") p.distMaxIncidenceDeg = value;
    else if (k == "imuPeriodMs")       p.imuPeriodMs = value;
    else if (k == "encoderPeriodMs")   p.encoderPeriodMs = value;
    else if (k == "encoderResolutionDeg") p.encoderResolutionDeg = value;
    else if (k == "lateralKp" || k == "lateralKi" || k == "lateralKd") {
        riku::PIDGains g = riku::chassisConfig.lateralGains;
        if (k == "lateralKp") g.kP = value;
        if (k == "lateralKi") g.kI = value;
        if (k == "lateralKd") g.kD = value;
        riku::chassisConfig.lateralGains = g;
        riku::chassis.setLateralGains(g);
    } else if (k == "angularKp" || k == "angularKi" || k == "angularKd") {
        riku::PIDGains g = riku::chassisConfig.angularGains;
        if (k == "angularKp") g.kP = value;
        if (k == "angularKi") g.kI = value;
        if (k == "angularKd") g.kD = value;
        riku::chassisConfig.angularGains = g;
        riku::chassis.setAngularGains(g);
    } else if (k == "ramseteB")           { gRamsete.b = value;
    } else if (k == "ramseteZeta")        { gRamsete.zeta = value;
    } else if (k == "ramseteMinSpeed")    { gRamsete.minSpeed = value;
    } else if (k == "ramseteMaxAccel")    { gRamsete.maxAccel = value;
    } else if (k == "ramseteLookahead")   { gRamsete.lookaheadDist = value;
    } else if (k == "ramseteTolerance")   { gRamsete.targetTolerance = value;
    } else if (k == "ramseteHeadingTol")  { gRamsete.headingTolerance = value;
    } else if (k == "ramseteProfile")     { gRamsete.useVelocityProfile = value != 0.0f;
    } else if (k == "mclDistSyncProp") {
        riku::chassis.mcl().setDistSyncProp(value);
    } else if (k == "seed") {
        // Pin both RNGs: the world's noise and MCL's particle jitter. Without
        // the second, a run replays the same sensor stream but a different
        // particle cloud, which is not a reproducible test.
        gSeed = static_cast<std::uint32_t>(value);
        sim::seedRng(gSeed);
        riku::chassis.mcl().setSeed(gSeed);
        riku::chassis.mcl().setPose(riku::chassis.getPose());
    }
}

// ============================================================================
// Field editing
// ============================================================================

EMSCRIPTEN_KEEPALIVE
void sim_field_clear() { sim::field().clear(); }

EMSCRIPTEN_KEEPALIVE
void sim_field_default() { sim::field().loadDefault(); }

EMSCRIPTEN_KEEPALIVE
void sim_field_add_segment(float x1, float y1, float x2, float y2) {
    sim::field().addSegment(x1, y1, x2, y2);
}

EMSCRIPTEN_KEEPALIVE
void sim_field_add_circle(float x, float y, float r) {
    sim::field().addCircle(x, y, r);
}

// ============================================================================
// State out
// ============================================================================

EMSCRIPTEN_KEEPALIVE
float* sim_particles_ptr() { return gParticleBuf.data(); }

/// Refreshes the particle buffer and returns how many were written. Call this
/// before reading sim_particles_ptr().
EMSCRIPTEN_KEEPALIVE
int sim_particle_count() {
    gParticleCount = (gLocMode & kLocMcl)
        ? riku::chassis.mcl().snapshotParticles(gParticleBuf.data(),
                                                riku::MclTracking::PARTICLE_MAX)
        : 0;
    return gParticleCount;
}

EMSCRIPTEN_KEEPALIVE
float* sim_record_ptr() { return gRecord.data(); }

EMSCRIPTEN_KEEPALIVE
int sim_record_count() { return static_cast<int>(gRecord.size() / kRecordStride); }

EMSCRIPTEN_KEEPALIVE
int sim_record_stride() { return kRecordStride; }

EMSCRIPTEN_KEEPALIVE
void sim_record_clear() {
    gRecord.clear();
    gRecordNextMs = 0.0;
}

EMSCRIPTEN_KEEPALIVE
void sim_record_enable(int on) { gRecording = (on != 0); }

EMSCRIPTEN_KEEPALIVE
float* sim_trail_true_ptr() { return gTrueTrail.xy.data(); }

EMSCRIPTEN_KEEPALIVE
int sim_trail_true_count() { return static_cast<int>(gTrueTrail.xy.size() / 2); }

EMSCRIPTEN_KEEPALIVE
float* sim_trail_odom_ptr() { return gOdomTrail.xy.data(); }

EMSCRIPTEN_KEEPALIVE
int sim_trail_odom_count() { return static_cast<int>(gOdomTrail.xy.size() / 2); }

EMSCRIPTEN_KEEPALIVE
const char* sim_state_json() {
    const auto& t = sim::world().truth();
    const riku::Pose odom = riku::chassis.getPose();
    const riku::Pose mclRaw = riku::chassis.mcl().getRawEstimate();

    const float errX = odom.x - t.x;
    const float errY = odom.y - t.y;
    const float errDist = std::sqrt(errX * errX + errY * errY);
    const float errHeadingDeg =
        riku::angleDiffRad(t.theta, odom.theta) * riku::RAD_TO_DEG;

    // N_eff is the particle filter's own health metric: it collapses toward 1
    // when a handful of particles carry all the weight, which is what happens
    // just before MCL diverges.
    const float nEff = (gLocMode & kLocMcl)
        ? riku::chassis.mcl().getEstimate().second : 0.0f;
    const riku::Pose rcl = riku::chassis.rcl().latestEstimate();

    char buf[8192];
    int n = std::snprintf(buf, sizeof(buf),
        "{\"t\":%.3f,"
        "\"true\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f},"
        "\"odom\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f},"
        "\"mcl\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f,\"nEff\":%.1f},"
        "\"rcl\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f},"
        "\"err\":{\"dist\":%.3f,\"heading\":%.2f},"
        "\"vel\":{\"lin\":%.2f,\"ang\":%.2f,\"left\":%.2f,\"right\":%.2f},"
        "\"colliding\":%s,\"routeRunning\":%s,\"settled\":%s,"
        "\"locMode\":%d,\"tasks\":%d,\"beams\":[",
        sim::preciseMillis() / 1000.0,
        t.x, t.y, riku::stdRadToVexDeg(t.theta),
        odom.x, odom.y, riku::stdRadToVexDeg(odom.theta),
        mclRaw.x, mclRaw.y, riku::stdRadToVexDeg(mclRaw.theta), nEff,
        rcl.x, rcl.y, riku::stdRadToVexDeg(rcl.theta),
        errDist, errHeadingDeg,
        t.vLinIn, t.vAngRad * riku::RAD_TO_DEG, t.leftVelIn, t.rightVelIn,
        t.colliding ? "true" : "false",
        sim_route_running() ? "true" : "false",
        riku::chassis.isSettled() ? "true" : "false",
        gLocMode, sim::sched().liveTaskCount());

    bool first = true;
    for (int i = 0; i < riku::MAX_DISTANCE_SENSORS; i++) {
        const auto& b = sim::world().beam(i);
        if (!b.active) continue;
        n += std::snprintf(buf + n, sizeof(buf) - n,
            "%s{\"i\":%d,\"ox\":%.2f,\"oy\":%.2f,\"ex\":%.2f,\"ey\":%.2f,"
            "\"r\":%.2f,\"hit\":%s,\"dir\":%.2f,\"fov\":%.1f,"
            "\"conf\":%d,\"inc\":%.1f}",
            first ? "" : ",", i, b.originX, b.originY, b.endX, b.endY,
            b.rangeIn, b.inRange ? "true" : "false",
            b.dirDeg, b.fovDeg, b.confidence, b.incidenceDeg);
        first = false;
    }

    // Per-sensor filter view: what each estimator currently believes about the
    // same four sensors the world just synthesised. Comparing "the sim produced
    // a reading" against "MCL accepted it" is how a discarded-sensor bug becomes
    // visible instead of just showing up as drift.
    n += std::snprintf(buf + n, sizeof(buf) - n, "],\"filters\":[");
    const auto& mclValid = riku::chassis.mcl().validSensors();
    const auto& mclRead  = riku::chassis.mcl().sensorReadings();
    const auto& rclSensors = riku::chassis.rcl().sensors();

    first = true;
    for (int i = 0; i < riku::MAX_DISTANCE_SENSORS; i++) {
        if (!sim::world().beam(i).active) continue;
        const auto& rs = rclSensors[i];
        n += std::snprintf(buf + n, sizeof(buf) - n,
            "%s{\"i\":%d,\"mclValid\":%s,\"mclIn\":%.2f,"
            "\"rclValid\":%s,\"rclIn\":%.2f,\"rclConf\":%d,"
            "\"rx\":%.2f,\"ry\":%.2f,\"rdx\":%.3f,\"rdy\":%.3f}",
            first ? "" : ",", i,
            mclValid[i] ? "true" : "false", mclRead[i],
            rs.valid ? "true" : "false", rs.readingInch, rs.confidence,
            rs.worldX, rs.worldY, rs.rayCos, rs.raySin);
        first = false;
    }
    std::snprintf(buf + n, sizeof(buf) - n, "]}");

    gStateJson.assign(buf);
    return gStateJson.c_str();
}

EMSCRIPTEN_KEEPALIVE
const char* sim_field_json() {
    std::string out = "{\"segments\":[";
    bool first = true;
    for (const auto& s : sim::field().segments()) {
        char b[144];
        std::snprintf(b, sizeof(b), "%s[%.2f,%.2f,%.2f,%.2f,%d]",
                      first ? "" : ",", s.x1, s.y1, s.x2, s.y2, s.kind);
        out += b;
        first = false;
    }
    out += "],\"circles\":[";
    first = true;
    for (const auto& c : sim::field().circles()) {
        char b[96];
        std::snprintf(b, sizeof(b), "%s[%.2f,%.2f,%.2f,%d]",
                      first ? "" : ",", c.x, c.y, c.r, c.kind);
        out += b;
        first = false;
    }
    out += "]}";
    gStateJson = out;
    return gStateJson.c_str();
}

}  // extern "C"
