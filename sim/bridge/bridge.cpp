// chad-sim — C API surface consumed by the browser.
//
// Everything here is a thin adapter. The algorithms under test are the real
// chad objects declared in gamespecific/config.cpp; this file only starts
// them, advances virtual time, and marshals state out to JavaScript.
#include <emscripten/emscripten.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "../platform/sim_runtime.hpp"
#include "../platform/sim_sched.hpp"
#include "../world/field.hpp"
#include "../world/world.hpp"

#include "chad/movement_control/ltv.hpp"
#include "chad/movement_control/ltvlqr.hpp"
#include "chad/movement_control/mpc_follower.hpp"

#include "gamespecific/autons.hpp"
#include "gamespecific/config.hpp"

namespace {

// --- localization mode bits -------------------------------------------------
//
// Bit 0 clear means dead reckoning only: with no tracking wheels in
// chad::odomSensors that is the IMU plus the drive motor encoders, nothing more.
constexpr int kLocMcl = 1;
constexpr int kLocRcl = 2;
constexpr int kLocEkf = 4;

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

// --- optional dead wheels ---------------------------------------------------
//
// The robot in gamespecific/config.cpp has no tracking wheels, so the library
// falls back to the drive motor encoders. These let the sim show the other
// configuration without changing the robot's own config: the pods live here, and
// swapping them in goes through the same OdomTracking::setSensors() a real robot
// would use. World publishes their travel on ports 11 and 12 either way.
pros::Rotation gVertPod(11);
pros::Rotation gHorizPod(12);
chad::TrackingWheel gVertWheel(gVertPod, 2.75f, 0.0f);
chad::TrackingWheel gHorizWheel(gHorizPod, 2.75f, 0.0f);
bool gOdomPods = false;

// Ticks spent in contact, not just a bool: a controller that grazes a post for
// one tick and one that leans on it all run read the same otherwise.
int gCollideTicks = 0;

std::string gStateJson;
std::vector<float> gParticleBuf(chad::MclTracking::PARTICLE_MAX * 4);
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

/// Mark a discontinuity in both trails. The renderer starts a new subpath at a
/// non-finite point, so a reposition leaves a gap instead of a straight line
/// drawn across the field from wherever the previous run ended.
void breakTrails() {
    const float nan = std::nanf("");
    if (!gTrueTrail.xy.empty()) pushTrail(gTrueTrail, nan, nan);
    if (!gOdomTrail.xy.empty()) pushTrail(gOdomTrail, nan, nan);
}

struct AutonEntry { const char* name; void (*fn)(); };
const AutonEntry kAutons[] = {
    {"Example",   chad::autonExample},
    {"Red Left",  chad::autonRedLeft},
    {"Blue Left", chad::autonBlueLeft},
    {"Skills",    chad::autonSkills},
};
constexpr int kAutonCount = static_cast<int>(sizeof(kAutons) / sizeof(kAutons[0]));

/// Rebuild the odometry sensor set with or without the dead wheels.
void applyOdomPods() {
    chad::OdomSensors sensors = chad::odomSensors;
    if (gOdomPods) {
        sensors.vertWheelCollection = { &gVertWheel };
        sensors.horizWheelCollection = { &gHorizWheel };
    }
    // reset() re-baselines against whichever sensors are now live -- required,
    // or the first tick measures a delta against the other sensor set -- but it
    // also zeroes the pose. Swapping pods in should not move the robot.
    const chad::Pose held = chad::chassis.odom().getPose();
    chad::chassis.odom().setSensors(sensors);
    chad::chassis.odom().reset();
    chad::chassis.odom().setPose(held);
}

void applyManualDrive() {
    if (!gManualActive) return;
    chad::drivetrain.setVoltage(gManualLeft / 12.0f * 127.0f,
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
    const chad::Pose odom = chad::chassis.getPose();
    const chad::Pose mcl = chad::chassis.mcl().getRawEstimate();

    const float ex = odom.x - t.x;
    const float ey = odom.y - t.y;

    // Columns: t, trueX, trueY, trueH, odomX, odomY, odomH, mclX, mclY, mclH,
    //          errDist, errHeading, vLin, vAng
    gRecord.push_back(static_cast<float>(now / 1000.0));
    gRecord.push_back(t.x);
    gRecord.push_back(t.y);
    gRecord.push_back(chad::stdRadToVexDeg(t.theta));
    gRecord.push_back(odom.x);
    gRecord.push_back(odom.y);
    gRecord.push_back(chad::stdRadToVexDeg(odom.theta));
    gRecord.push_back(mcl.x);
    gRecord.push_back(mcl.y);
    gRecord.push_back(chad::stdRadToVexDeg(mcl.theta));
    gRecord.push_back(std::sqrt(ex * ex + ey * ey));
    gRecord.push_back(chad::angleDiffRad(t.theta, odom.theta) * chad::RAD_TO_DEG);
    gRecord.push_back(t.vLinIn);
    gRecord.push_back(t.vAngRad * chad::RAD_TO_DEG);
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
    for (const auto& a : kAutons) chad::selector.registerAuton(a.name, a.fn);
    chad::selector.init();

    chad::registerFieldObstacles();

    // Odometry only for now; MCL/RCL are opt-in from the UI.
    chad::chassis.odom().startBackgroundOdom();
}

EMSCRIPTEN_KEEPALIVE void sim_set_localization(int mode);

EMSCRIPTEN_KEEPALIVE
void sim_reset(float x, float y, float thetaDeg) {
    const chad::Pose p{x, y, chad::vexToStdRad(thetaDeg)};
    const int wasLoc = gLocMode;
    gCollideTicks = 0;

    // Tear the run down through the library's own lifecycle calls so its
    // pros::Task pointers are released properly, rather than killing the
    // fibers out from under it.
    chad::chassis.cancelMotion();
    if (gRouteTask != sim::kInvalidTask) {
        sim::sched().kill(gRouteTask);
        gRouteTask = sim::kInvalidTask;
    }
    chad::chassis.ekf().stopTracking();
    chad::chassis.mcl().stopTracking();
    chad::chassis.rcl().stopTracking();
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
    chad::chassis.mcl().setSeed(gSeed);

    sim::world().resetTo(p);

    // Re-baseline odometry against the freshly reset hardware BEFORE setting
    // the pose. setPose() deliberately leaves mPrevHeadingRad alone (so an MCL
    // sync cannot fake a rotation), which means the first tick after a reset
    // would otherwise measure dTheta against the previous run's final heading.
    // reset() is the call that re-reads the IMU and drops the stale delta.
    applyOdomPods();
    chad::chassis.odom().reset();
    chad::chassis.setPose(p);
    chad::chassis.mcl().setPose(p);
    chad::chassis.ekf().setPose(p);

    gTrueTrail.xy.clear();
    gOdomTrail.xy.clear();
    gRecord.clear();
    gRecordNextMs = 0.0;
    gManualLeft = gManualRight = 0.0f;

    // Odometry's task was never stopped, so this only re-arms the enabled flag.
    chad::chassis.odom().startBackgroundOdom();
    if (wasLoc != 0) sim_set_localization(wasLoc);
}

EMSCRIPTEN_KEEPALIVE
void sim_teleport(float x, float y, float thetaDeg) {
    // Kidnapped-robot test: ground truth moves, the estimator is not told.
    breakTrails();
    sim::world().teleport({x, y, chad::vexToStdRad(thetaDeg)});
}

// ============================================================================
// Time
// ============================================================================

EMSCRIPTEN_KEEPALIVE
void sim_step(float ms) {
    applyManualDrive();
    sim::sched().runFor(ms);

    const auto& t = sim::world().truth();
    if (t.colliding) gCollideTicks++;
    pushTrail(gTrueTrail, t.x, t.y);
    const chad::Pose op = chad::chassis.getPose();
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
        chad::chassis.mcl().setPose(chad::chassis.getPose());
        chad::chassis.mcl().startTracking();
    } else if (!(mode & kLocMcl) && (gLocMode & kLocMcl)) {
        chad::chassis.mcl().stopTracking();
    }

    if ((mode & kLocRcl) && !(gLocMode & kLocRcl)) {
        chad::chassis.rcl().startTracking();
    } else if (!(mode & kLocRcl) && (gLocMode & kLocRcl)) {
        chad::chassis.rcl().stopTracking();
    }

    // Exactly one filter may write the pose back into odometry. With the EKF up
    // it is the EKF, and the others become measurements or diagnostics. Two
    // filters lerping the same pose toward different estimates undo each other:
    // MCL+RCL together lap at an 11.96 in peak where MCL alone manages 1.08.
    chad::chassis.mcl().setAutoSync(!(mode & kLocEkf));
    chad::chassis.rcl().setAutoSync(!(mode & kLocEkf));
    if ((mode & kLocEkf) && !(gLocMode & kLocEkf)) {
        chad::chassis.ekf().setPose(chad::chassis.getPose());
        chad::chassis.ekf().startTracking();
    } else if (!(mode & kLocEkf) && (gLocMode & kLocEkf)) {
        chad::chassis.ekf().stopTracking();
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
    chad::chassis.cancelMotion();
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
chad::RAMSETEParams gRamsete;
chad::MoveToPointParams gMoveToPoint;

// Sampled trajectory points, pushed by the browser before a run. An
// OP_FOLLOW_PATH command names a [start, count) slice of this buffer, so one
// route can follow several paths.
std::vector<chad::Pose> gPath;

// --- LTV / LQR follower -----------------------------------------------------
//
// Nothing in the project constructs an LTVPathFollower, so it ships untested.
// The sim builds one on demand so the controller can actually be exercised.
// Its trajectory arrives as the same "P = {...} / V = {...}" text blob
// prepare_trajectory() parses on a robot -- positions in METRES, velocities in
// m/s and rad/s, one sample per 10 ms tick.

chad::LtvLqrFollower* gLqr = nullptr;
chad::LtvLqrParams gLqrParams;

chad::LtvLqrFollower& lqr() {
    if (gLqr == nullptr) gLqr = new chad::LtvLqrFollower(chad::chassis, chad::drivetrain);
    return *gLqr;
}

chad::LTVPathFollower* gLtv = nullptr;
std::string gLtvPath;

// Feedforward gains are settable so they can be tuned in the sim rather than
// guessed. kV is volts per m/s: at 12 V this drivetrain runs about 1.95 m/s,
// so the physically correct value is near 6.2, not the token default.
// kV defaults to the value implied by the drivetrain itself: 12 V produces
// 450 rpm on a 3.25 in wheel, i.e. ~1.95 m/s, so kV ~= 12 / 1.95 = 6.2 V per
// m/s. Guessing low here shows up as a systematic along-track deficit, not as
// a controller fault -- 0.9 leaves the path 9 in short, 6.2 lands within 0.5.
chad::FeedforwardGains gLtvGains{
    .kV = 6.2f,
    .kaStraight = 0.0f,
    .kaTurn = 0.0f,
    .ksStraight = 0.3f,
    .ksTurn = 0.0f,
    .kP = 2.0f,
};

chad::LTVPathFollower& ltv() {
    if (gLtv == nullptr) {
        gLtv = new chad::LTVPathFollower(chad::chassis, chad::drivetrain, gLtvGains);
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
                const chad::Pose p{c.a, c.b, chad::vexToStdRad(c.c)};
                breakTrails();
                sim::world().resetTo(p);
                chad::chassis.setPose(p);
                chad::chassis.mcl().setPose(p);
                break;
            }
            case OP_MOVE_TO: {
                chad::MoveToPointParams params = gMoveToPoint;
                params.forwards = (c.c >= 0.0f);
                params.maxSpeed = gRouteMaxSpeed;
                chad::chassis.moveToPoint(c.a, c.b, timeout, params);
                break;
            }
            case OP_TURN_TO_HEADING: {
                chad::TurntoHeadingParams params;
                params.maxSpeed = gRouteMaxSpeed;
                chad::chassis.turnToHeading(c.a, timeout, params);
                break;
            }
            case OP_TURN_TO_POINT: {
                chad::TurnToPointParams params;
                params.maxSpeed = gRouteMaxSpeed;
                chad::chassis.turnToPoint(c.a, c.b, timeout, params);
                break;
            }
            case OP_DRIVE: {
                chad::MoveDistanceParams params;
                params.forwards = (c.a >= 0.0f);
                params.maxSpeed = gRouteMaxSpeed;
                chad::chassis.moveDistance(std::fabs(c.a), timeout, params);
                break;
            }
            case OP_BOOMERANG: {
                chad::BoomerangParams params;
                params.maxSpeed = gRouteMaxSpeed;
                chad::chassis.moveBoomerang(c.a, c.b, c.c, timeout, params);
                break;
            }
            case OP_ARC: {
                chad::ArcParams params;
                params.maxSpeed = gRouteMaxSpeed;
                chad::chassis.moveArc(c.a, c.b, c.c, timeout, params);
                break;
            }
            case OP_SWING: {
                chad::SwingParams params;
                params.maxSpeed = gRouteMaxSpeed;
                const auto side = (c.b >= 0.5f) ? chad::SwingSide::Right
                                                : chad::SwingSide::Left;
                chad::chassis.swingToHeading(c.a, side, timeout, params);
                break;
            }
            case OP_DELAY:
                pros::delay(static_cast<std::uint32_t>(c.a));
                break;
            case OP_WAIT_SETTLED:
                chad::chassis.waitUntilSettled();
                break;
            case OP_LATERAL_GAINS:
                chad::chassis.setLateralGains({c.a, c.b, c.c});
                break;
            case OP_ANGULAR_GAINS:
                chad::chassis.setAngularGains({c.a, c.b, c.c});
                break;
            case OP_MAX_SPEED:
                gRouteMaxSpeed = c.a;
                break;
            case OP_FOLLOW_PATH: {
                const int start = static_cast<int>(c.a);
                const int count = static_cast<int>(c.b);
                if (start < 0 || count <= 0 ||
                    start + count > static_cast<int>(gPath.size())) break;
                std::vector<chad::Pose> path(gPath.begin() + start,
                                             gPath.begin() + start + count);
                chad::RAMSETEParams params = gRamsete;
                params.maxSpeed = std::min(gRouteMaxSpeed, gRamsete.maxSpeed);
                chad::chassis.moveRAMSETE(path, timeout, params);
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
    gPath.push_back({x, y, chad::vexToStdRad(headingDeg)});
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
    chad::LTVPathFollower::ltvConfig cfg;
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

/// Follow gPath[start, start+count) with the speed-scheduled LQR.
EMSCRIPTEN_KEEPALIVE
void sim_lqr_follow(int start, int count, float timeoutMs) {
    if (start < 0 || count <= 0 || start + count > static_cast<int>(gPath.size())) return;
    if (gRouteTask != sim::kInvalidTask) sim::sched().kill(gRouteTask);
    gManualActive = false;
    // Own fiber, so the browser keeps driving the clock and can sample the run
    // while it happens -- the same reason a route runs on one.
    auto path = std::make_shared<std::vector<chad::Pose>>(
        gPath.begin() + start, gPath.begin() + start + count);
    gRouteTask = sim::sched().spawn([path, timeoutMs]() {
        lqr().followPath(*path, gLqrParams, timeoutMs);
    }, "lqr");
}

EMSCRIPTEN_KEEPALIVE
int sim_lqr_running() { return lqr().isRunning() ? 1 : 0; }

EMSCRIPTEN_KEEPALIVE
float sim_lqr_peak() { return lqr().lastPeakError(); }

EMSCRIPTEN_KEEPALIVE
float sim_lqr_final() { return lqr().lastFinalError(); }

EMSCRIPTEN_KEEPALIVE
void sim_lqr_set(const char* name, float value) {
    const std::string k(name);
    if      (k == "qx")     gLqrParams.weights.qx = value;
    else if (k == "qy")     gLqrParams.weights.qy = value;
    else if (k == "qtheta") gLqrParams.weights.qtheta = value;
    else if (k == "rv")     gLqrParams.weights.rv = value;
    else if (k == "romega") gLqrParams.weights.romega = value;
    else if (k == "maxSpeed")   gLqrParams.maxSpeed = value;
    else if (k == "maxAccel")   gLqrParams.maxAccel = value;
    else if (k == "maxLatAccel") gLqrParams.maxLateralAccel = value;
    else if (k == "tolerance")  gLqrParams.targetTolerance = value;
    else if (k == "headingTol") gLqrParams.headingTolerance = value;
    else if (k == "velCtrl")    gLqrParams.useVelocityController = (value != 0.0f);
    else if (k == "ffKv")       gLqrParams.ffGains.kV = value;
    else if (k == "ffKs")       gLqrParams.ffGains.ksStraight = value;
    else if (k == "ffKp")       gLqrParams.ffGains.kP = value;
    else if (k == "ffKi")       gLqrParams.ffGains.kI = value;
    else if (k == "ffKa")       gLqrParams.ffGains.kaStraight = value;
    else if (k == "slewRise")   gLqrParams.slewRisePerSec = value;
    else if (k == "slewFall")   gLqrParams.slewFallPerSec = value;
}

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
    else if (k == "distConeSigmaFrac") p.distConeSigmaFrac = value;
    else if (k == "collideObstacles") p.collideObstacles = (value != 0.0f);
    else if (k == "imuPeriodMs")       p.imuPeriodMs = value;
    else if (k == "encoderPeriodMs")   p.encoderPeriodMs = value;
    else if (k == "encoderResolutionDeg") p.encoderResolutionDeg = value;
    else if (k == "lateralKp" || k == "lateralKi" || k == "lateralKd") {
        chad::PIDGains g = chad::chassisConfig.lateralGains;
        if (k == "lateralKp") g.kP = value;
        if (k == "lateralKi") g.kI = value;
        if (k == "lateralKd") g.kD = value;
        chad::chassisConfig.lateralGains = g;
        chad::chassis.setLateralGains(g);
    } else if (k == "angularKp" || k == "angularKi" || k == "angularKd") {
        chad::PIDGains g = chad::chassisConfig.angularGains;
        if (k == "angularKp") g.kP = value;
        if (k == "angularKi") g.kI = value;
        if (k == "angularKd") g.kD = value;
        chad::chassisConfig.angularGains = g;
        chad::chassis.setAngularGains(g);
    } else if (k == "mclObsCone" || k == "mclObsTol") {
        auto c = chad::chassis.mcl().config();
        if (k == "mclObsCone") c.obstacleConeFrac = value;
        else                   c.obstacleRangeTol = value;
        chad::chassis.mcl().setConfig(c);
    } else if (k.rfind("ekf", 0) == 0) {
        auto c = chad::chassis.ekf().config();
        if      (k == "ekfMclSigmaScale") c.mclSigmaScale = value;
        else if (k == "ekfMclSigmaFloor") c.mclSigmaFloor = value;
        else if (k == "ekfRangeGate")     c.rangeGateChi2 = value;
        else if (k == "ekfObsTol")        c.obstacleRangeTol = value;
        else if (k == "ekfObsSigmaPad")   c.obstacleSigmaPad = value;
        else if (k == "ekfSlipProp")      c.slipNoiseProp = value;
        else if (k == "ekfVertProp")      c.vertNoiseProp = value;
        else if (k == "ekfVertBias")      c.vertBiasProp = value;
        else if (k == "ekfSlipBias")      c.slipBiasProp = value;
        else if (k == "ekfBiasDecorr")    c.biasDecorrIn = value;
        else if (k == "ekfRangeSigBase")  c.rangeSigmaBase = value;
        else if (k == "ekfRangeSigProp")  c.rangeSigmaProp = value;
        else if (k == "ekfMaxIncidence")  c.maxIncidenceRad = value;
        else if (k == "ekfImuSigma")      c.imuSigmaRad = value;
        else if (k == "ekfPosFloor")      c.posNoiseFloor = value;
        else if (k == "ekfThetaPerInch")  c.thetaNoisePerInch = value;
        else if (k == "ekfMclGate")       c.mclGateChi2 = value;
        else if (k == "ekfUseRange")      c.useRangeUpdates = (value != 0.0f);
        else if (k == "ekfResetCycles")   c.mclResetCycles = static_cast<int>(value);
        else if (k == "ekfSyncProp")      c.syncProp = value;
        else if (k == "ekfThetaSyncProp") c.thetaSyncProp = value;
        chad::chassis.ekf().setConfig(c);
    } else if (k == "lateralDFilt" || k == "angularDFilt" ||
               k == "lateralWindup" || k == "angularWindup" || k == "settleTimeMs") {
        if      (k == "lateralDFilt")  chad::chassisConfig.lateralDerivativeFilter = value;
        else if (k == "angularDFilt")  chad::chassisConfig.angularDerivativeFilter = value;
        else if (k == "lateralWindup") chad::chassisConfig.lateralWindupRange = value;
        else if (k == "angularWindup") chad::chassisConfig.angularWindupRange = value;
        else chad::chassisConfig.settleTimeMs = static_cast<std::uint32_t>(value);
        chad::chassis.setPidTuning(chad::chassisConfig);
    } else if (k.rfind("rcl", 0) == 0) {
        auto c = chad::chassis.rcl().config();
        if      (k == "rclHardSync")  c.hardSync = (value != 0.0f);
        else if (k == "rclSyncProp")  c.syncProp = value;
        else if (k == "rclSyncRate")  c.maxSyncRate = value;
        else if (k == "rclObsTol")    c.obstacleRangeTol = value;
        chad::chassis.rcl().setConfig(c);
    } else if (k.rfind("mcl", 0) == 0) {
        auto c = chad::chassis.mcl().config();
        if      (k == "mclRangeSigClose") c.rangeSigmaCloseIn = value;
        else if (k == "mclRangeSigProp")  c.rangeSigmaProp = value;
        else if (k == "mclShortWeight")   c.shortWeight = value;
        else if (k == "mclShortDecay")    c.shortDecaySigmas = value;
        else if (k == "mclRandWeight")    c.randWeight = value;
        else if (k == "mclMaxRandomFrac") c.maxRandomFrac = value;
        else if (k == "mclDeadzoneLikelihood") c.deadzoneLikelihood = value;
        else if (k == "mclResampleVar")   c.distResampleVariance = value;
        else if (k == "mclThetaResampleVar") c.thetaResampleVariance = value;
        else if (k == "mclResampleThresh") c.resampleThreshold = static_cast<int>(value);
        else if (k == "mclImeVariance")   c.imeVariance = value;
        else if (k == "mclHorizProp")     c.horizDependentVarianceProp = value;
        else if (k == "mclHorizConst")    c.horizConstantNoise = value;
        else if (k == "mclImuVariance")   c.imuVariance = value;
        else if (k == "mclMaxThetaDev")   c.maxThetaDeviation = value;
        else if (k == "mclObsConeFrac")   c.obstacleConeFrac = value;
        else if (k == "mclObsRangeTol")   c.obstacleRangeTol = value;
        else if (k == "mclSyncProp")      c.distSyncProp = value;
        chad::chassis.mcl().setConfig(c);
    } else if (k == "ramseteB")           { gRamsete.b = value;
    } else if (k == "ramseteZeta")        { gRamsete.zeta = value;
    } else if (k == "ramseteMinSpeed")    { gRamsete.minSpeed = value;
    } else if (k == "ramseteMaxAccel")    { gRamsete.maxAccel = value;
    } else if (k == "ramseteLookahead")   { gRamsete.lookaheadDist = value;
    } else if (k == "ramseteTolerance")   { gRamsete.targetTolerance = value;
    } else if (k == "ramseteHeadingTol")  { gRamsete.headingTolerance = value;
    } else if (k == "ramseteStall")       { gRamsete.stallSpeed = value;
    } else if (k == "ramseteProfile")     { gRamsete.useVelocityProfile = value != 0.0f;
    } else if (k == "ramseteLatAccel")    { gRamsete.maxLateralAccel = value;
    } else if (k == "ramseteMaxSpeed")    { gRamsete.maxSpeed = value;
    } else if (k == "ramseteVelCtrl")     { gRamsete.useVelocityController = value != 0.0f;
    } else if (k == "ramseteFfKv")        { gRamsete.ffGains.kV = value;
    } else if (k == "ramseteFfKs")        { gRamsete.ffGains.ksStraight = value;
    } else if (k == "ramseteFfKp")        { gRamsete.ffGains.kP = value;
    } else if (k == "ramseteFfKi")        { gRamsete.ffGains.kI = value;
    } else if (k == "mtpBearingHold")  { gMoveToPoint.bearingHoldDist = value;
    } else if (k == "mtpTolerance")    { gMoveToPoint.targetTolerance = value;
    } else if (k == "mclDistSyncProp") {
        chad::chassis.mcl().setDistSyncProp(value);
    } else if (k == "seed") {
        // Pin both RNGs: the world's noise and MCL's particle jitter. Without
        // the second, a run replays the same sensor stream but a different
        // particle cloud, which is not a reproducible test.
        gSeed = static_cast<std::uint32_t>(value);
        sim::seedRng(gSeed);
        chad::chassis.mcl().setSeed(gSeed);
        chad::chassis.mcl().setPose(chad::chassis.getPose());
    }
}

/// Read back a tunable. The browser's sliders used to publish their own markup
/// defaults on load, which silently overwrote whatever gamespecific/config.cpp
/// had been tuned to -- the panel has to ask what the value IS, not assert one.
/// Returns NaN for an unknown key so the caller can fall back.
EMSCRIPTEN_KEEPALIVE
float sim_get_param(const char* name) {
    const auto& p = sim::world().params();
    const std::string k(name);

    if      (k == "responseTauMs")     return p.responseTauMs;
    else if (k == "leftScale")         return p.leftScale;
    else if (k == "rightScale")        return p.rightScale;
    else if (k == "ekfSyncProp")       return chad::chassis.ekf().config().syncProp;
    else if (k == "ekfThetaSyncProp")  return chad::chassis.ekf().config().thetaSyncProp;
    else if (k == "encoderSlip")       return p.encoderSlip;
    else if (k == "imuUnderReport")    return p.imuUnderReport;
    else if (k == "imuDriftDegPerSec") return p.imuDriftDegPerSec;
    else if (k == "imuNoiseDeg")       return p.imuNoiseDeg;
    else if (k == "distNoiseMm")       return p.distNoiseMm;
    else if (k == "distMaxRangeMm")    return p.distMaxRangeMm;
    else if (k == "distMinRangeMm")    return p.distMinRangeMm;
    else if (k == "distPeriodMs")      return p.distPeriodMs;
    else if (k == "distFovDeg")        return p.distFovDeg;
    else if (k == "distConeRays")      return static_cast<float>(p.distConeRays);
    else if (k == "distMaxIncidenceDeg") return p.distMaxIncidenceDeg;
    else if (k == "distConeSigmaFrac") return p.distConeSigmaFrac;
    else if (k == "collideObstacles")  return p.collideObstacles ? 1.0f : 0.0f;
    else if (k == "imuPeriodMs")       return p.imuPeriodMs;
    else if (k == "encoderPeriodMs")   return p.encoderPeriodMs;
    else if (k == "encoderResolutionDeg") return p.encoderResolutionDeg;

    const auto& cc = chad::chassisConfig;
    if      (k == "lateralKp")     return cc.lateralGains.kP;
    else if (k == "lateralKi")     return cc.lateralGains.kI;
    else if (k == "lateralKd")     return cc.lateralGains.kD;
    else if (k == "angularKp")     return cc.angularGains.kP;
    else if (k == "angularKi")     return cc.angularGains.kI;
    else if (k == "angularKd")     return cc.angularGains.kD;
    else if (k == "lateralDFilt")  return cc.lateralDerivativeFilter;
    else if (k == "angularDFilt")  return cc.angularDerivativeFilter;
    else if (k == "lateralWindup") return cc.lateralWindupRange;
    else if (k == "angularWindup") return cc.angularWindupRange;
    else if (k == "settleTimeMs")  return static_cast<float>(cc.settleTimeMs);

    else if (k == "ramseteB")          return gRamsete.b;
    else if (k == "ramseteZeta")       return gRamsete.zeta;
    else if (k == "ramseteMinSpeed")   return gRamsete.minSpeed;
    else if (k == "ramseteMaxAccel")   return gRamsete.maxAccel;
    else if (k == "ramseteTolerance")  return gRamsete.targetTolerance;
    else if (k == "ramseteHeadingTol") return gRamsete.headingTolerance;

    const auto mc = chad::chassis.mcl().config();
    if      (k == "mclObsCone")  return mc.obstacleConeFrac;
    else if (k == "mclObsTol")   return mc.obstacleRangeTol;
    else if (k == "mclSyncProp") return mc.distSyncProp;

    return std::nanf("");
}

// ============================================================================
// Field editing
// ============================================================================

/// Swap the dead wheels in or out. 0 = motor encoders and IMU only.
EMSCRIPTEN_KEEPALIVE
void sim_set_odom_pods(int on) {
    const bool want = (on != 0);
    if (want == gOdomPods) return;
    gOdomPods = want;
    applyOdomPods();
}

EMSCRIPTEN_KEEPALIVE
int sim_odom_pods() { return gOdomPods ? 1 : 0; }

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
        ? chad::chassis.mcl().snapshotParticles(gParticleBuf.data(),
                                                chad::MclTracking::PARTICLE_MAX)
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
    const chad::Pose odom = chad::chassis.getPose();
    const chad::Pose mclRaw = chad::chassis.mcl().getRawEstimate();

    const float errX = odom.x - t.x;
    const float errY = odom.y - t.y;
    const float errDist = std::sqrt(errX * errX + errY * errY);
    const float errHeadingDeg =
        chad::angleDiffRad(t.theta, odom.theta) * chad::RAD_TO_DEG;

    // N_eff is the particle filter's own health metric: it collapses toward 1
    // when a handful of particles carry all the weight, which is what happens
    // just before MCL diverges.
    const float nEff = (gLocMode & kLocMcl)
        ? chad::chassis.mcl().getEstimate().second : 0.0f;
    const chad::Pose rcl = chad::chassis.rcl().latestEstimate();
    const chad::Pose ekf = chad::chassis.ekf().getEstimate();
    const auto ekfSigma = chad::chassis.ekf().uncertainty();
    const float mclInject = chad::chassis.mcl().randomInjectFraction();

    char buf[8192];
    int n = std::snprintf(buf, sizeof(buf),
        "{\"t\":%.3f,"
        "\"true\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f},"
        "\"odom\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f},"
        "\"mcl\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f,\"nEff\":%.1f},"
        "\"rcl\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f},"
        "\"ekf\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f,\"sx\":%.3f,\"sy\":%.3f,"
            "\"st\":%.3f,\"beams\":%d,\"reloc\":%d},"
        "\"mclInject\":%.3f,"
        "\"err\":{\"dist\":%.3f,\"heading\":%.2f},"
        "\"vel\":{\"lin\":%.2f,\"ang\":%.2f,\"left\":%.2f,\"right\":%.2f},"
        "\"colliding\":%s,\"routeRunning\":%s,\"settled\":%s,"
        "\"locMode\":%d,\"tasks\":%d,\"beams\":[",
        sim::preciseMillis() / 1000.0,
        t.x, t.y, chad::stdRadToVexDeg(t.theta),
        odom.x, odom.y, chad::stdRadToVexDeg(odom.theta),
        mclRaw.x, mclRaw.y, chad::stdRadToVexDeg(mclRaw.theta), nEff,
        rcl.x, rcl.y, chad::stdRadToVexDeg(rcl.theta),
        ekf.x, ekf.y, chad::stdRadToVexDeg(ekf.theta),
        ekfSigma.sigmaX, ekfSigma.sigmaY,
        ekfSigma.sigmaTheta * chad::RAD_TO_DEG,
        chad::chassis.ekf().beamsUsed(), chad::chassis.ekf().relocateCount(),
        mclInject,
        errDist, errHeadingDeg,
        t.vLinIn, t.vAngRad * chad::RAD_TO_DEG, t.leftVelIn, t.rightVelIn,
        t.colliding ? "true" : "false",
        sim_route_running() ? "true" : "false",
        chad::chassis.isSettled() ? "true" : "false",
        gLocMode, sim::sched().liveTaskCount());

    bool first = true;
    for (int i = 0; i < chad::MAX_DISTANCE_SENSORS; i++) {
        const auto& b = sim::world().beam(i);
        if (!b.active) continue;
        n += std::snprintf(buf + n, sizeof(buf) - n,
            "%s{\"i\":%d,\"ox\":%.2f,\"oy\":%.2f,\"ex\":%.2f,\"ey\":%.2f,"
            "\"r\":%.2f,\"hit\":%s,\"dir\":%.2f,\"fov\":%.1f,"
            "\"conf\":%d,\"inc\":%.1f,"
            "\"hx\":%.2f,\"hy\":%.2f,\"obs\":%s,\"ar\":%.2f}",
            first ? "" : ",", i, b.originX, b.originY, b.endX, b.endY,
            b.rangeIn, b.inRange ? "true" : "false",
            b.dirDeg, b.fovDeg, b.confidence, b.incidenceDeg,
            b.hitX, b.hitY, b.hitIsObstacle ? "true" : "false", b.axisRangeIn);
        first = false;
    }

    // Per-sensor filter view: what each estimator currently believes about the
    // same four sensors the world just synthesised. Comparing "the sim produced
    // a reading" against "MCL accepted it" is how a discarded-sensor bug becomes
    // visible instead of just showing up as drift.
    n += std::snprintf(buf + n, sizeof(buf) - n, "],\"filters\":[");
    const auto& mclValid = chad::chassis.mcl().validSensors();
    const auto& mclRead  = chad::chassis.mcl().sensorReadings();
    const auto& mclDead  = chad::chassis.mcl().deadzoneFraction();
    const auto& rclSensors = chad::chassis.rcl().sensors();

    first = true;
    for (int i = 0; i < chad::MAX_DISTANCE_SENSORS; i++) {
        if (!sim::world().beam(i).active) continue;
        const auto& rs = rclSensors[i];
        n += std::snprintf(buf + n, sizeof(buf) - n,
            "%s{\"i\":%d,\"mclValid\":%s,\"mclIn\":%.2f,"
            "\"rclValid\":%s,\"rclIn\":%.2f,\"rclConf\":%d,\"mclDead\":%.2f,"
            "\"rx\":%.2f,\"ry\":%.2f,\"rdx\":%.3f,\"rdy\":%.3f}",
            first ? "" : ",", i,
            mclValid[i] ? "true" : "false", mclRead[i],
            rs.valid ? "true" : "false", rs.readingInch, rs.confidence,
            mclDead[i],
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
// ============================================================================
// MPC / RTI study
// ============================================================================
//
// Two separate surfaces:
//
//   sim_mpc_* run MpcPathFollower against the world, so tracking is measured
//   through the same kinematics, device rates and quantisation as every other
//   controller in the sim.
//
//   sim_mpc_bench_json times the QP solver on its own, closed-loop on the
//   linear error plant with no world in the way. That is the number that decides
//   feasibility, and it needs thousands of solves rather than the few hundred a
//   path run provides.
//
// Wall time comes from emscripten_get_now(), NOT the virtual clock: the point is
// how long the arithmetic takes on real silicon.

namespace {

double microsNow() { return emscripten_get_now() * 1000.0; }

chad::MpcPathFollower* gMpc = nullptr;
std::vector<chad::TrajectorySample> gMpcTraj;
std::string gMpcJson;

chad::MpcPathFollower::Config defaultMpcConfig() {
    chad::MpcPathFollower::Config c;
    c.mode = chad::MpcPathFollower::Mode::MpcRti;
    c.horizon = 10;
    c.maxIters = 400;
    c.rtiIters = 1;
    c.tol = 1e-6f;
    c.weights = {1.0f, 40.0f, 4.0f, 1.0f, 0.25f};
    c.dt = 0.01f;
    c.constrainInputs = true;
    c.wheelSpeedLimit = 1.887f;      // (12 V - ksStraight) / kV
    c.slackToLinear = 0.5f;
    c.maxVoltage = 12.0f;
    c.speedBuckets = 16;
    c.gains = gLtvGains;             // identical output stage to the LTV baseline
    c.microsClock = &microsNow;
    return c;
}

chad::MpcPathFollower::Config gMpcCfg = defaultMpcConfig();

chad::MpcPathFollower& mpcFollower() {
    if (gMpc == nullptr) {
        gMpc = new chad::MpcPathFollower(chad::chassis, chad::drivetrain);
    }
    return *gMpc;
}

}  // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE
int sim_collide_ticks() { return gCollideTicks; }

EMSCRIPTEN_KEEPALIVE
void sim_mpc_traj_clear() { gMpcTraj.clear(); }

/// Append one trajectory sample. Position in INCHES, heading in standard math
/// radians, v in inches/sec, w in rad/sec. Converted to metres here so the
/// harness can work in field units.
EMSCRIPTEN_KEEPALIVE
void sim_mpc_traj_push(float x, float y, float theta, float v, float w) {
    gMpcTraj.push_back({x * chad::INCH_TO_METER, y * chad::INCH_TO_METER, theta,
                        v * chad::INCH_TO_METER, w});
}

EMSCRIPTEN_KEEPALIVE
int sim_mpc_traj_count() { return static_cast<int>(gMpcTraj.size()); }

EMSCRIPTEN_KEEPALIVE
void sim_mpc_defaults() { gMpcCfg = defaultMpcConfig(); }

EMSCRIPTEN_KEEPALIVE
void sim_mpc_set(const char* name, float value) {
    const std::string k(name);
    using Mode = chad::MpcPathFollower::Mode;

    if      (k == "mode")       gMpcCfg.mode = static_cast<Mode>(static_cast<int>(value));
    else if (k == "horizon")    gMpcCfg.horizon = static_cast<int>(value);
    else if (k == "maxIters")   gMpcCfg.maxIters = static_cast<int>(value);
    else if (k == "rtiIters")   gMpcCfg.rtiIters = static_cast<int>(value);
    else if (k == "tol")        gMpcCfg.tol = value;
    else if (k == "solver")     gMpcCfg.solver =
                                    static_cast<chad::MpcSolver>(static_cast<int>(value));
    else if (k == "qx")         gMpcCfg.weights.qx = value;
    else if (k == "qy")         gMpcCfg.weights.qy = value;
    else if (k == "qth")        gMpcCfg.weights.qTheta = value;
    else if (k == "rv")         gMpcCfg.weights.rv = value;
    else if (k == "rw")         gMpcCfg.weights.rw = value;
    else if (k == "dt")         gMpcCfg.dt = value;
    else if (k == "constrain")  gMpcCfg.constrainInputs = (value != 0.0f);
    else if (k == "wheelLimit") gMpcCfg.wheelSpeedLimit = value;
    else if (k == "slackSplit") gMpcCfg.slackToLinear = value;
    else if (k == "wheelBox")   gMpcCfg.wheelSpaceInputs = (value != 0.0f);
    else if (k == "indepClip")  gMpcCfg.independentClip = (value != 0.0f);
    else if (k == "maxVoltage") gMpcCfg.maxVoltage = value;
    else if (k == "buckets")    gMpcCfg.speedBuckets = static_cast<int>(value);
    else if (k == "kV")         gMpcCfg.gains.kV = value;
    else if (k == "kS")         gMpcCfg.gains.ksStraight = value;
    else if (k == "kP")         gMpcCfg.gains.kP = value;
    else if (k == "timing")     gMpcCfg.microsClock = (value != 0.0f) ? &microsNow : nullptr;
}

EMSCRIPTEN_KEEPALIVE
void sim_mpc_run() {
    if (gMpcTraj.empty()) return;
    gManualActive = false;
    mpcFollower().follow(gMpcTraj, gMpcCfg);
}

EMSCRIPTEN_KEEPALIVE
int sim_mpc_running() { return (gMpc != nullptr && gMpc->isRunning()) ? 1 : 0; }

EMSCRIPTEN_KEEPALIVE
void sim_mpc_cancel() { if (gMpc != nullptr) gMpc->cancel(); }

EMSCRIPTEN_KEEPALIVE
const char* sim_mpc_stats_json() {
    static const chad::MpcPathFollower::Stats empty{};
    const auto& s = (gMpc != nullptr) ? gMpc->stats() : empty;
    const int ticks = (s.ticks > 0) ? s.ticks : 1;

    char buf[1400];
    std::snprintf(buf, sizeof(buf),
        "{\"ticks\":%d,\"iters\":%d,\"maxIters\":%d,\"meanIters\":%.3f,"
        "\"rebuilds\":%d,\"convergedTicks\":%d,"
        "\"solveUsTotal\":%.1f,\"meanSolveUs\":%.4f,\"worstSolveUs\":%.4f,"
        "\"worstTickUs\":%.4f,\"setupUs\":%.2f,"
        "\"meanCrossM\":%.6f,\"peakCrossM\":%.6f,\"meanHeadingRad\":%.6f,"
        "\"endErrorM\":%.6f,\"endHeadingRad\":%.6f,"
        "\"satTicks\":%d,\"activeBoundTicks\":%d,"
        "\"peakWheelDemand\":%.4f,\"peakClipRatio\":%.4f,"
        "\"collideTicks\":%d,\"running\":%s}",
        s.ticks, s.solverIterations, s.maxIterations,
        static_cast<double>(s.solverIterations) / ticks,
        s.rebuilds, s.convergedTicks,
        s.solveMicros, s.solveMicros / ticks, s.worstSolveMicros,
        s.worstTickMicros, s.setupMicros,
        s.sumAbsCross / ticks, s.peakAbsCross, s.sumAbsHeading / ticks,
        s.endError, s.endHeading,
        s.satTicks, s.activeBoundTicks, s.peakWheelDemand, s.peakClipRatio,
        gCollideTicks, sim_mpc_running() ? "true" : "false");
    gMpcJson.assign(buf);
    return gMpcJson.c_str();
}

// ----------------------------------------------------------------------------
// Solver benchmark
// ----------------------------------------------------------------------------
//
// Closed loop on the linear error plant, which is what makes the RTI warm start
// honest: consecutive QPs differ by one tick of real plant motion, exactly as
// they do on a robot. A benchmark that re-solves the same QP from the same warm
// start measures nothing.
//
// A checksum is accumulated and reported so the optimiser cannot delete the
// solve.

namespace {

/// Advance the plant one tick: x+ = Ad x + Bd u.
void plantStep(const chad::DrivetrainPlant& p, float x[3], const float u[2]) {
    float nx[3];
    for (int i = 0; i < 3; i++) {
        nx[i] = p.Ad[i][0] * x[0] + p.Ad[i][1] * x[1] + p.Ad[i][2] * x[2] +
                p.Bd[i][0] * u[0] + p.Bd[i][1] * u[1];
    }
    x[0] = nx[0]; x[1] = nx[1]; x[2] = nx[2];
}

}  // namespace

/// @param horizon      MPC horizon N
/// @param solver       0 = fast gradient, 1 = projected Gauss-Seidel
/// @param itersPerTick 0 = solve to convergence, >0 = fixed RTI budget
/// @param constrained  1 = box active, 0 = unconstrained
/// @param ticks        closed-loop ticks to run
EMSCRIPTEN_KEEPALIVE
const char* sim_mpc_bench_json(int horizon, int solver, int itersPerTick,
                               int constrained, int ticks) {
    if (horizon < 1) horizon = 1;
    if (horizon > chad::MPC_MAX_HORIZON) horizon = chad::MPC_MAX_HORIZON;
    if (ticks < 1) ticks = 1;

    const chad::MpcWeights w = gMpcCfg.weights;
    const float dt = gMpcCfg.dt;
    const float vRef = 0.75f * gMpcCfg.wheelSpeedLimit;
    const float wRef = 0.0f;

    const chad::DrivetrainPlant plant = chad::DrivetrainPlant::discretize(vRef, dt);
    const chad::LqrSolution lqr = chad::solveDiscreteLqr(plant, w);

    static chad::CondensedMpc mpc;
    mpc.solver = static_cast<chad::MpcSolver>(solver);

    // Setup cost, averaged. Rebuilding the condensed matrices is the expensive
    // half and happens only when the reference speed crosses a bucket edge, so
    // it is priced separately from the per-tick solve.
    const int setupReps = 32;
    const double s0 = microsNow();
    for (int i = 0; i < setupReps; i++) {
        mpc.setup(plant, w, horizon, lqr.P);
    }
    const double setupUs = (microsNow() - s0) / setupReps;

    float uMin[2], uMax[2];
    if (constrained != 0) {
        // Same inscribed-box construction as MpcPathFollower::inputBox().
        const float halfTrack = 0.5f * chad::drivetrain.trackWidth * chad::INCH_TO_METER;
        const float vNeed = std::fabs(vRef), wNeed = std::fabs(wRef) * halfTrack;
        const float slack = std::fmax(0.0f, gMpcCfg.wheelSpeedLimit - vNeed - wNeed);
        const float vBudget = vNeed + 0.5f * slack;
        const float wBudget = (wNeed + 0.5f * slack) / halfTrack;
        uMin[0] = -vBudget - vRef; uMax[0] = vBudget - vRef;
        uMin[1] = -wBudget - wRef; uMax[1] = wBudget - wRef;
    } else {
        uMin[0] = uMin[1] = -1e6f;
        uMax[0] = uMax[1] = 1e6f;
    }

    // Timing methodology: emscripten_get_now() crosses into JS, and at ~0.3 us a
    // call it swamps a 1 us solve if called per solve. So the clock is read ONCE
    // around the whole loop, and an identical loop with the solve removed is
    // timed and subtracted. Its cost is measured, not assumed.
    float x[3] = {0.02f, 0.06f, 0.12f};
    mpc.zero();

    double sumCross = 0.0, peakCross = 0.0, checksum = 0.0;
    long iterSum = 0;
    int convergedCount = 0, maxIters = 0;
    // Whether the box actually BINDS. Without this the headline "MPC ties the
    // clipped LQR" is vacuous -- an inactive constraint is trivially free.
    int boundTicks = 0, mpcClipTicks = 0, lqrClipTicks = 0;
    const double t0 = microsNow();

    for (int k = 0; k < ticks; k++) {
        // Keep the error alive: without a disturbance the loop settles to zero
        // and every later solve starts from a perfect warm start, which flatters
        // RTI.
        if (k % 97 == 0) {
            x[0] += 0.02f;
            x[1] += 0.05f;
            x[2] += 0.10f;
        }

        float u[2];
        chad::MpcResult r;
        if (itersPerTick > 0) {
            if (k > 0) mpc.shift();
            r = mpc.solve(x, uMin, uMax, itersPerTick, 0.0f);
        } else {
            mpc.zero();
            r = mpc.solve(x, uMin, uMax, gMpcCfg.maxIters, gMpcCfg.tol);
        }
        mpc.firstMove(u);

        iterSum += r.iterations;
        if (r.iterations > maxIters) maxIters = r.iterations;
        if (r.converged) convergedCount++;

        // The box is a hard limit on what the drivetrain can do, so an
        // unconstrained command would be clipped downstream anyway.
        const float uApplied[2] = {chad::clamp(u[0], uMin[0], uMax[0]),
                                   chad::clamp(u[1], uMin[1], uMax[1])};
        if (r.activeBounds > 0) boundTicks++;
        if (uApplied[0] != u[0] || uApplied[1] != u[1]) mpcClipTicks++;
        plantStep(plant, x, uApplied);

        const double c = std::fabs(x[1]);
        sumCross += c;
        if (c > peakCross) peakCross = c;
        checksum += u[0] + u[1];
    }
    const double totalUs = microsNow() - t0;

    // The same loop with the solve removed: disturbance, clamp, plant step,
    // bookkeeping. Subtracted from totalUs so the reported cost is the solver.
    float xh[3] = {0.02f, 0.06f, 0.12f};
    double harnessChk = 0.0;
    const double h0 = microsNow();
    for (int k = 0; k < ticks; k++) {
        if (k % 97 == 0) { xh[0] += 0.02f; xh[1] += 0.05f; xh[2] += 0.10f; }
        const float u[2] = {mpc.solution()[0], mpc.solution()[1]};
        const float ua[2] = {chad::clamp(u[0], uMin[0], uMax[0]),
                             chad::clamp(u[1], uMin[1], uMax[1])};
        plantStep(plant, xh, ua);
        harnessChk += std::fabs(xh[1]) + u[0] + u[1];
    }
    const double harnessUs = microsNow() - h0;

    // What one clock read costs, so the floor of this measurement is on record.
    const double c0 = microsNow();
    double clockChk = 0.0;
    for (int k = 0; k < 20000; k++) clockChk += microsNow();
    const double clockUs = (microsNow() - c0) / 20000.0;

    // Same closed loop under the scheduled LQR, for a cost and tracking
    // reference point. The LQR cannot see the box, so its command is clipped.
    float xl[3] = {0.02f, 0.06f, 0.12f};
    double lqrSum = 0.0, lqrPeak = 0.0, lqrChk = 0.0;
    const double l0 = microsNow();
    for (int k = 0; k < ticks; k++) {
        if (k % 97 == 0) { xl[0] += 0.02f; xl[1] += 0.05f; xl[2] += 0.10f; }
        float u[2];
        for (int i = 0; i < 2; i++) {
            u[i] = -(lqr.K[i][0] * xl[0] + lqr.K[i][1] * xl[1] + lqr.K[i][2] * xl[2]);
        }
        lqrChk += u[0] + u[1];
        const float ua[2] = {chad::clamp(u[0], uMin[0], uMax[0]),
                             chad::clamp(u[1], uMin[1], uMax[1])};
        if (ua[0] != u[0] || ua[1] != u[1]) lqrClipTicks++;
        plantStep(plant, xl, ua);
        const double c = std::fabs(xl[1]);
        lqrSum += c;
        if (c > lqrPeak) lqrPeak = c;
    }
    const double lqrUs = ((microsNow() - l0) - harnessUs) / ticks;

    // And the price of solving Riccati every tick instead of scheduling it.
    const double r0 = microsNow();
    double riccatiChk = 0.0;
    long riccatiIters = 0;
    for (int k = 0; k < ticks; k++) {
        const chad::LqrSolution s =
            chad::solveDiscreteLqr(chad::DrivetrainPlant::discretize(vRef, dt), w);
        riccatiChk += s.K[0][0] + s.K[1][2];
        riccatiIters += s.iterations;
    }
    const double riccatiUs = (microsNow() - r0) / ticks;

    char buf[1800];
    std::snprintf(buf, sizeof(buf),
        "{\"horizon\":%d,\"vars\":%d,\"solver\":%d,\"itersPerTick\":%d,"
        "\"constrained\":%d,\"ticks\":%d,"
        "\"usPerSolve\":%.4f,\"usGross\":%.4f,\"harnessUs\":%.4f,"
        "\"clockUs\":%.4f,\"maxIters\":%d,\"meanIters\":%.3f,"
        "\"convergedFrac\":%.4f,\"setupUs\":%.3f,"
        "\"flopsPerIter\":%ld,\"flopsSetup\":%ld,"
        "\"meanCrossM\":%.6f,\"peakCrossM\":%.6f,"
        "\"lqrUsPerTick\":%.4f,\"lqrMeanCrossM\":%.6f,\"lqrPeakCrossM\":%.6f,"
        "\"riccatiUsPerTick\":%.4f,\"riccatiIters\":%.1f,"
        "\"lambdaMax\":%.4f,\"powerIters\":%d,"
        "\"checksum\":%.6f,\"lqrChecksum\":%.6f,"
        "\"boundTicks\":%d,\"mpcClipTicks\":%d,\"lqrClipTicks\":%d,"
        "\"uMin\":[%.4f,%.4f],\"uMax\":[%.4f,%.4f],\"vRef\":%.4f,"
        "\"riccatiChecksum\":%.6f,\"harnessChecksum\":%.6f,\"clockChecksum\":%.6e}",
        horizon, 2 * horizon, solver, itersPerTick, constrained, ticks,
        (totalUs - harnessUs) / ticks, totalUs / ticks, harnessUs / ticks,
        clockUs, maxIters, static_cast<double>(iterSum) / ticks,
        static_cast<double>(convergedCount) / ticks, setupUs,
        mpc.flopsPerIteration(), mpc.flopsSetup(),
        sumCross / ticks, peakCross,
        lqrUs, lqrSum / ticks, lqrPeak,
        riccatiUs, static_cast<double>(riccatiIters) / ticks,
        mpc.lipschitz(), mpc.powerIterations(),
        checksum, lqrChk,
        boundTicks, mpcClipTicks, lqrClipTicks,
        uMin[0], uMin[1], uMax[0], uMax[1], vRef,
        riccatiChk, harnessChk, clockChk);
    gMpcJson.assign(buf);
    return gMpcJson.c_str();
}

/// Correctness gate: with the DARE cost-to-go as terminal cost and no active
/// constraints, the first move of the condensed QP must equal -K x exactly, for
/// any horizon. If this drifts, the condensing or the terminal cost is wrong and
/// no tracking number below it means anything.
EMSCRIPTEN_KEEPALIVE
const char* sim_mpc_check_json(int horizon) {
    const chad::MpcWeights w = gMpcCfg.weights;
    const float dt = gMpcCfg.dt;
    const float vRef = 0.75f * gMpcCfg.wheelSpeedLimit;

    const chad::DrivetrainPlant plant = chad::DrivetrainPlant::discretize(vRef, dt);
    const chad::LqrSolution lqr = chad::solveDiscreteLqr(plant, w);

    static chad::CondensedMpc mpc;
    mpc.solver = chad::MpcSolver::GaussSeidel;
    mpc.setup(plant, w, horizon, lqr.P);

    const float lo[2] = {-1e6f, -1e6f};
    const float hi[2] = {1e6f, 1e6f};

    float worst = 0.0f;
    float lqrU[2] = {0.0f, 0.0f}, mpcU[2] = {0.0f, 0.0f};
    const float probes[4][3] = {
        {0.05f, 0.10f, 0.20f},
        {-0.03f, 0.02f, -0.15f},
        {0.00f, -0.08f, 0.05f},
        {0.12f, 0.00f, 0.00f},
    };
    for (const auto& x : probes) {
        mpc.zero();
        mpc.solve(x, lo, hi, 4000, 1e-12f);
        mpc.firstMove(mpcU);
        for (int i = 0; i < 2; i++) {
            lqrU[i] = -(lqr.K[i][0] * x[0] + lqr.K[i][1] * x[1] + lqr.K[i][2] * x[2]);
            const float d = std::fabs(lqrU[i] - mpcU[i]) / (std::fabs(lqrU[i]) + 1e-6f);
            if (d > worst) worst = d;
        }
    }

    char buf[480];
    std::snprintf(buf, sizeof(buf),
        "{\"horizon\":%d,\"worstRelErr\":%.3e,"
        "\"lqrK\":[[%.5f,%.5f,%.5f],[%.5f,%.5f,%.5f]],"
        "\"lastLqrU\":[%.6f,%.6f],\"lastMpcU\":[%.6f,%.6f],\"lqrIters\":%d}",
        horizon, worst,
        lqr.K[0][0], lqr.K[0][1], lqr.K[0][2],
        lqr.K[1][0], lqr.K[1][1], lqr.K[1][2],
        lqrU[0], lqrU[1], mpcU[0], mpcU[1], lqr.iterations);
    gMpcJson.assign(buf);
    return gMpcJson.c_str();
}

}  // extern "C"
