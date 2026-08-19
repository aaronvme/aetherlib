// riku-sim — C API surface consumed by the browser.
//
// Everything here is a thin adapter. The algorithms under test are the real
// riku objects declared in gamespecific/config.cpp; this file only starts
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

#include "riku/movement_control/ltv.hpp"
#include "riku/movement_control/ltvlqr.hpp"

#include "gamespecific/autons.hpp"
#include "gamespecific/config.hpp"

namespace {

// --- localization mode bits -------------------------------------------------
//
// Bit 0 clear means dead reckoning only: with no tracking wheels in
// riku::odomSensors that is the IMU plus the drive motor encoders, nothing more.
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
riku::TrackingWheel gVertWheel(gVertPod, 2.75f, 0.0f);
riku::TrackingWheel gHorizWheel(gHorizPod, 2.75f, 0.0f);
bool gOdomPods = false;

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
    {"Example",   riku::autonExample},
    {"Red Left",  riku::autonRedLeft},
    {"Blue Left", riku::autonBlueLeft},
    {"Skills",    riku::autonSkills},
};
constexpr int kAutonCount = static_cast<int>(sizeof(kAutons) / sizeof(kAutons[0]));

/// Rebuild the odometry sensor set with or without the dead wheels.
void applyOdomPods() {
    riku::OdomSensors sensors = riku::odomSensors;
    if (gOdomPods) {
        sensors.vertWheelCollection = { &gVertWheel };
        sensors.horizWheelCollection = { &gHorizWheel };
    }
    // reset() re-baselines against whichever sensors are now live -- required,
    // or the first tick measures a delta against the other sensor set -- but it
    // also zeroes the pose. Swapping pods in should not move the robot.
    const riku::Pose held = riku::chassis.odom().getPose();
    riku::chassis.odom().setSensors(sensors);
    riku::chassis.odom().reset();
    riku::chassis.odom().setPose(held);
}

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
    riku::chassis.ekf().stopTracking();
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
    applyOdomPods();
    riku::chassis.odom().reset();
    riku::chassis.setPose(p);
    riku::chassis.mcl().setPose(p);
    riku::chassis.ekf().setPose(p);

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
    breakTrails();
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

    // Exactly one filter may write the pose back into odometry. With the EKF up
    // it is the EKF, and the others become measurements or diagnostics. Two
    // filters lerping the same pose toward different estimates undo each other:
    // MCL+RCL together lap at an 11.96 in peak where MCL alone manages 1.08.
    riku::chassis.mcl().setAutoSync(!(mode & kLocEkf));
    riku::chassis.rcl().setAutoSync(!(mode & kLocEkf));
    if ((mode & kLocEkf) && !(gLocMode & kLocEkf)) {
        riku::chassis.ekf().setPose(riku::chassis.getPose());
        riku::chassis.ekf().startTracking();
    } else if (!(mode & kLocEkf) && (gLocMode & kLocEkf)) {
        riku::chassis.ekf().stopTracking();
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
riku::MoveToPointParams gMoveToPoint;

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

riku::LtvLqrFollower* gLqr = nullptr;
riku::LtvLqrParams gLqrParams;

riku::LtvLqrFollower& lqr() {
    if (gLqr == nullptr) gLqr = new riku::LtvLqrFollower(riku::chassis, riku::drivetrain);
    return *gLqr;
}

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
                breakTrails();
                sim::world().resetTo(p);
                riku::chassis.setPose(p);
                riku::chassis.mcl().setPose(p);
                break;
            }
            case OP_MOVE_TO: {
                riku::MoveToPointParams params = gMoveToPoint;
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
                params.maxSpeed = std::min(gRouteMaxSpeed, gRamsete.maxSpeed);
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

/// Follow gPath[start, start+count) with the speed-scheduled LQR.
EMSCRIPTEN_KEEPALIVE
void sim_lqr_follow(int start, int count, float timeoutMs) {
    if (start < 0 || count <= 0 || start + count > static_cast<int>(gPath.size())) return;
    if (gRouteTask != sim::kInvalidTask) sim::sched().kill(gRouteTask);
    gManualActive = false;
    // Own fiber, so the browser keeps driving the clock and can sample the run
    // while it happens -- the same reason a route runs on one.
    auto path = std::make_shared<std::vector<riku::Pose>>(
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
    } else if (k == "mclObsCone" || k == "mclObsTol") {
        auto c = riku::chassis.mcl().config();
        if (k == "mclObsCone") c.obstacleConeFrac = value;
        else                   c.obstacleRangeTol = value;
        riku::chassis.mcl().setConfig(c);
    } else if (k.rfind("ekf", 0) == 0) {
        auto c = riku::chassis.ekf().config();
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
        riku::chassis.ekf().setConfig(c);
    } else if (k == "lateralDFilt" || k == "angularDFilt" ||
               k == "lateralWindup" || k == "angularWindup" || k == "settleTimeMs") {
        if      (k == "lateralDFilt")  riku::chassisConfig.lateralDerivativeFilter = value;
        else if (k == "angularDFilt")  riku::chassisConfig.angularDerivativeFilter = value;
        else if (k == "lateralWindup") riku::chassisConfig.lateralWindupRange = value;
        else if (k == "angularWindup") riku::chassisConfig.angularWindupRange = value;
        else riku::chassisConfig.settleTimeMs = static_cast<std::uint32_t>(value);
        riku::chassis.setPidTuning(riku::chassisConfig);
    } else if (k.rfind("rcl", 0) == 0) {
        auto c = riku::chassis.rcl().config();
        if      (k == "rclHardSync")  c.hardSync = (value != 0.0f);
        else if (k == "rclSyncProp")  c.syncProp = value;
        else if (k == "rclSyncRate")  c.maxSyncRate = value;
        else if (k == "rclObsTol")    c.obstacleRangeTol = value;
        riku::chassis.rcl().setConfig(c);
    } else if (k.rfind("mcl", 0) == 0) {
        auto c = riku::chassis.mcl().config();
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
        riku::chassis.mcl().setConfig(c);
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
    else if (k == "ekfSyncProp")       return riku::chassis.ekf().config().syncProp;
    else if (k == "ekfThetaSyncProp")  return riku::chassis.ekf().config().thetaSyncProp;
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

    const auto& cc = riku::chassisConfig;
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

    const auto mc = riku::chassis.mcl().config();
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
    const riku::Pose ekf = riku::chassis.ekf().getEstimate();
    const auto ekfSigma = riku::chassis.ekf().uncertainty();
    const float mclInject = riku::chassis.mcl().randomInjectFraction();

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
        t.x, t.y, riku::stdRadToVexDeg(t.theta),
        odom.x, odom.y, riku::stdRadToVexDeg(odom.theta),
        mclRaw.x, mclRaw.y, riku::stdRadToVexDeg(mclRaw.theta), nEff,
        rcl.x, rcl.y, riku::stdRadToVexDeg(rcl.theta),
        ekf.x, ekf.y, riku::stdRadToVexDeg(ekf.theta),
        ekfSigma.sigmaX, ekfSigma.sigmaY,
        ekfSigma.sigmaTheta * riku::RAD_TO_DEG,
        riku::chassis.ekf().beamsUsed(), riku::chassis.ekf().relocateCount(),
        mclInject,
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
    const auto& mclValid = riku::chassis.mcl().validSensors();
    const auto& mclRead  = riku::chassis.mcl().sensorReadings();
    const auto& mclDead  = riku::chassis.mcl().deadzoneFraction();
    const auto& rclSensors = riku::chassis.rcl().sensors();

    first = true;
    for (int i = 0; i < riku::MAX_DISTANCE_SENSORS; i++) {
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
