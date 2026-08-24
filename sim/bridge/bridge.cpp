// chad-sim — C API surface consumed by the browser.
//
// Everything here is a thin adapter. The algorithms under test are the real
// chad objects declared in gamespecific/config.cpp; this file only starts
// them, advances virtual time, and marshals state out to JavaScript.
#include <emscripten/emscripten.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <type_traits>
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

#include "bridge_internal.hpp"

// The shared state is named unqualified throughout this file; it lives in a
// namespace only so the other bridge translation units can reach it.
using namespace simbridge;

namespace simbridge {
int gCollideTicks = 0;
bool gManualActive = false;

// kV is volts per m/s, and it is the gain the followers are most sensitive to:
// guessing low shows up as a systematic along-track deficit rather than as a
// controller fault. 6.2 is 12 V / 1.95 m/s, the speed of a 3.25 in wheel at
// 450 rpm -- but config.cpp now measures this drivetrain at 2.6738 in, i.e.
// 1.60 m/s, which implies 7.5. Left at 6.2 because every follower figure in
// sim/README.md was measured with it; re-measure before changing it.
chad::FeedforwardGains gLtvGains{
    .kV = 6.2f,
    .kaStraight = 0.0f,
    .kaTurn = 0.0f,
    .ksStraight = 0.3f,
    .ksTurn = 0.0f,
    .kP = 2.0f,
};
}  // namespace simbridge

namespace {

// --- localization mode bits -------------------------------------------------
//
// Bit 0 clear means dead reckoning only: with no tracking wheels in
// chad::odomSensors that is the IMU plus the drive motor encoders, nothing more.
constexpr int kLocMcl = 1;

/// Virtual time sim_reset spends letting the cancelled tasks park before the
/// clock is rewound. One 10 ms control period, so every loop gets a full tick.
constexpr double kResetDrainMs = 10.0;
constexpr int kLocRcl = 2;
constexpr int kLocEkf = 4;

int  gLocMode = 0;
bool gStarted = false;

// Pinned RNG seed. Held here so sim_reset() can restore the exact same
// starting conditions rather than continuing from a drifted generator.
std::uint32_t gSeed = 0xC0FFEEu;

// Manual drive command in volts, applied when no routine is running.
// gManualActive is in simbridge -- every controller entry point clears it.
float gManualLeft = 0.0f;
float gManualRight = 0.0f;

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

std::string gStateJson;
std::vector<float> gParticleBuf(chad::MclTracking::PARTICLE_MAX * 4);
int gParticleCount = 0;

/// Append-only formatting into a fixed buffer.
///
/// The idiom this replaces -- `n += snprintf(buf + n, sizeof(buf) - n, ...)` --
/// overflows the buffer rather than truncating: snprintf returns what it WOULD
/// have written, so once the text exceeds the buffer `n` runs past its end and
/// `sizeof(buf) - n` becomes an enormous size_t. Eight sensors of beam and
/// filter detail were already within a factor of two of the old 8 KB bound.
struct JsonBuf {
    char data[16384];
    int n{0};

    template <typename... Args>
    void addf(const char* fmt, Args... args) {
        const int cap = static_cast<int>(sizeof(data)) - 1;
        if (n < 0 || n >= cap) return;
        const int written = std::snprintf(data + n, static_cast<std::size_t>(cap - n + 1),
                                          fmt, args...);
        if (written < 0) return;
        n += (written < cap - n) ? written : (cap - n);
    }

    /// Literal text, no formatting. Separate so a constant string is never
    /// handed to snprintf as its format.
    void add(const char* text) { addf("%s", text); }
};

/// Function-local, so a 16 KB buffer is not on the caller's stack -- the browser
/// polls this from the main thread but a fiber could too, and Asyncify stacks
/// are 32 KB.
JsonBuf& jsonBuf() {
    static JsonBuf buf;
    return buf;
}

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

// --- drive command readback -------------------------------------------------
//
// What the library is actually asking the motors for, in volts, with the port
// sign undone the same way the world undoes it. The one observable that can tell
// "the motion handed its speed to the next one" from "the motion stopped", and
// the only way to assert the safety property that a drained motion queue leaves
// nothing commanded.

float sideCommandVolts(const pros::MotorGroup* g) {
    if (g == nullptr || g->ports().empty()) return 0.0f;
    double mv = 0.0;
    for (std::uint8_t port : g->ports()) {
        const auto& m = sim::bus().motors[port];
        mv += m.reversed ? -m.commandedMv : m.commandedMv;
    }
    return static_cast<float>(mv / static_cast<double>(g->ports().size()) / 1000.0);
}

// --- total drive-encoder outage --------------------------------------------
//
// A real V5 reports PROS_ERR from a port whose device has gone, and the
// drivetrain's fault test is exactly that: a reading at or past INT32_MAX. The
// world republishes the real drive ports every encoder tick, so poking those
// cannot hold a fault; instead the sense groups are pointed at ports the world
// never writes, carrying a value no encoder could report. Same code path as
// unplugging every drive motor: EncoderAverage::live drops to 0.
//
// 3-6 are free in gamespecific/config.cpp and must stay off the drivetrain's
// command groups, which are the only ports the world writes. Blue, so that
// resolving externalGearRatio() while these are installed gives the 450/600 the
// real sense groups give.
pros::MotorGroup gDeadLeftSense({3, 4}, pros::MotorGears::blue);
pros::MotorGroup gDeadRightSense({5, 6}, pros::MotorGears::blue);
constexpr double kFaultReading = 4.0e9;   // > INT32_MAX, finite, like PROS_ERR

/// 0 healthy, 1 every port faulted, 2 every port reading a constant zero. The
/// second is a different failure and the more dangerous one: the readings are
/// perfectly valid, so nothing downstream can tell them from a robot parked at
/// its baseline.
int gEncodersDead = 0;
pros::MotorGroup* gRealLeftSense = nullptr;
pros::MotorGroup* gRealRightSense = nullptr;

void poisonDeadEncoders() {
    const double value = (gEncodersDead == 2) ? 0.0 : kFaultReading;
    for (std::uint8_t port : {3, 4, 5, 6}) {
        auto& m = sim::bus().motors[port];
        m.positionDeg = value;
        m.velocityRpm = value;
    }
}

void applyDeadEncoders() {
    if (gEncodersDead != 0) {
        poisonDeadEncoders();
        chad::drivetrain.leftSenseMotors = &gDeadLeftSense;
        chad::drivetrain.rightSenseMotors = &gDeadRightSense;
    } else {
        chad::drivetrain.leftSenseMotors = gRealLeftSense;
        chad::drivetrain.rightSenseMotors = gRealRightSense;
    }
}

// --- MCL resample counter ---------------------------------------------------
//
// MclCore reports resampledLast() but keeps no cumulative count, so this is
// --- motion trace -----------------------------------------------------------
//
// Chassis::setTelemetry() hands out one ChassisSample per motion tick. `type` is
// the running MotionType as the library itself reports it (the same int a
// telemetry CSV records) and `inTolerance` is SettleMonitor::inTolerance(), so
// this is where "when did the settle test first pass" comes from -- as opposed
// to "when did the motion return", which the runner times separately. The gap
// between the two is the settle hold. A motion can return without this ever
// having passed: a curved path arrives with a heading residual outside
// tolerance, so its toleranceMs stays -1 and that is honest, not a fault.
// The record's own `settled` field is a different thing -- Chassis::isSettled(),
// i.e. the motion queue is empty.
int    gTraceType = 0;
int    gTraceTicks = 0;
double gTraceFirstToleranceMs = -1.0;
float  gTracePeakLat = 0.0f;
float  gTracePeakAng = 0.0f;

void resetTrace() {
    gTraceType = 0;
    gTraceTicks = 0;
    gTraceFirstToleranceMs = -1.0;
    gTracePeakLat = gTracePeakAng = 0.0f;
}

void onChassisSample(const chad::ChassisSample& s) {
    gTraceType = s.type;
    gTraceTicks++;
    gTracePeakLat = std::max(gTracePeakLat, std::fabs(s.lateralOut));
    gTracePeakAng = std::max(gTracePeakAng, std::fabs(s.angularOut));
    if (s.inTolerance && gTraceFirstToleranceMs < 0.0) {
        gTraceFirstToleranceMs = sim::preciseMillis();
    }
}

// --- motion program ---------------------------------------------------------
//
// One scripted motion and one record of how it ended. Declared here because
// sim_reset clears them; the runner is further down with the C API it serves.

struct MotionStep {
    int type{0};                 // chad::MotionType, as an int
    float a{0}, b{0}, c{0};
    float timeout{0.0f};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float tolerance{-1.0f};      // negative = ChassisConfig default
    float earlyExit{0.0f};
    float delayAfterMs{0.0f};
    bool back{false};
    bool async{false};
    bool holdWindow{false};
};

struct MotionRecord {
    int index{0};
    int type{0};                 // as the library's own telemetry reports it
    int outcome{0};
    int ticks{0};
    bool arrived{false};
    bool settled{false};
    float ms{0.0f};              // motion start to the blocking call returning
    float toleranceMs{-1.0f};    // first tick the settle test passed, from start
    float vIn{0.0f}, vOut{0.0f};       // truth speed entering / leaving, in/s
    float cmdIn{0.0f}, cmdOut{0.0f};   // mean drive command, volts
    float peakLat{0.0f}, peakAng{0.0f};
    float x{0.0f}, y{0.0f}, h{0.0f};
    float trueX{0.0f}, trueY{0.0f};
};

std::vector<MotionStep> gMotionSteps;
std::vector<MotionRecord> gMotionRecords;
bool gMotionActive = false;
std::string gMotionJson;

// Defined further down, next to the state each one owns; declared here because
// sim_init() and sim_reset() are the first things in the file.
void takeParamBaseline();
void cancelStrayFollowers();

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

    // The encoder source to come back to after a simulated outage.
    gRealLeftSense = chad::drivetrain.leftSenseMotors;
    gRealRightSense = chad::drivetrain.rightSenseMotors;

    // One ChassisSample per motion tick: the running MotionType and whether the
    // settle test is currently passing, neither of which is reachable any other
    // way.
    chad::chassis.setTelemetry(onChassisSample);

    // Odometry only for now; MCL/RCL are opt-in from the UI.
    chad::chassis.odom().startBackgroundOdom();

    // Last, so it captures the configuration config.cpp actually ships rather
    // than whatever the constructors defaulted to.
    takeParamBaseline();
}

EMSCRIPTEN_KEEPALIVE void sim_set_localization(int mode);
// Owned by bridge_traj.cpp / bridge_mpc.cpp; both are no-ops when idle.
EMSCRIPTEN_KEEPALIVE void sim_traj_cancel();
EMSCRIPTEN_KEEPALIVE void sim_mpc_cancel();
EMSCRIPTEN_KEEPALIVE int sim_traj_running();
EMSCRIPTEN_KEEPALIVE int sim_mpc_running();

EMSCRIPTEN_KEEPALIVE
void sim_reset(float x, float y, float thetaDeg) {
    const chad::Pose p{x, y, chad::vexToStdRad(thetaDeg)};
    const int wasLoc = gLocMode;
    gCollideTicks = 0;
    gMotionRecords.clear();
    gMotionActive = false;
    resetTrace();
    // A reset is a clean robot: injected faults do not survive it.
    gEncodersDead = 0;
    applyDeadEncoders();

    // Tear the run down through the library's own lifecycle calls so its
    // pros::Task pointers are released properly, rather than killing the
    // fibers out from under it.
    chad::chassis.cancelMotion();
    if (gRouteTask != sim::kInvalidTask) {
        sim::sched().kill(gRouteTask);
        gRouteTask = sim::kInvalidTask;
    }
    // The trajectory, MPC and LTV followers run on fibers this function does not
    // own, and a live one keeps driving the robot straight through the reset.
    cancelStrayFollowers();
    chad::chassis.ekf().stopTracking();
    chad::chassis.mcl().stopTracking();
    chad::chassis.rcl().stopTracking();
    gLocMode = 0;

    // Let every task notice the cancel and park itself, on the old clock, before
    // it is rewound. Unconditional on purpose: stopTracking() above already
    // spends virtual time in a pros::delay loop WHEN a filter was up, and
    // nothing at all when none was, which is why the first trial after sim_init
    // was not comparable with any later one. Details in sim/README.md.
    sim::sched().runFor(kResetDrainMs);

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
    // Re-asserted every step: sim_reset zeroes every port on the bus, and a
    // fault that healed itself on the next reset would be a fault nobody could
    // test through one.
    if (gEncodersDead != 0) poisonDeadEncoders();
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

/// Stop every follower that owns a fiber or a pros::Task of its own. Guarded on
/// the pointers rather than calling ltv()/lqr(), which would construct a
/// follower on every reset just to cancel it.
void cancelStrayFollowers() {
    if (gLtv != nullptr) gLtv->cancel();
    if (gLqr != nullptr) gLqr->cancel();
    sim_traj_cancel();
    sim_mpc_cancel();
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
               k == "lateralWindup" || k == "angularWindup" || k == "settleTimeMs" ||
               k == "settleSpeedIn" || k == "settleSpeedDeg" ||
               k == "stallSpeedIn" || k == "stallSpeedDeg" ||
               k == "stallTimeMs" || k == "stallGraceMs") {
        // setPidTuning carries the whole exit test, not just the gains, so the
        // stall thresholds are reachable from here -- they decide when a motion
        // gives up and nothing else in the sim could set them.
        auto& cc = chad::chassisConfig;
        if      (k == "lateralDFilt")  cc.lateralDerivativeFilter = value;
        else if (k == "angularDFilt")  cc.angularDerivativeFilter = value;
        else if (k == "lateralWindup") cc.lateralWindupRange = value;
        else if (k == "angularWindup") cc.angularWindupRange = value;
        else if (k == "settleSpeedIn") cc.settleSpeedIn = value;
        else if (k == "settleSpeedDeg") cc.settleSpeedDeg = value;
        else if (k == "stallSpeedIn")  cc.stallSpeedIn = value;
        else if (k == "stallSpeedDeg") cc.stallSpeedDeg = value;
        else if (k == "stallTimeMs")   cc.stallTimeMs = static_cast<std::uint32_t>(value);
        else if (k == "stallGraceMs")  cc.stallGraceMs = static_cast<std::uint32_t>(value);
        else cc.settleTimeMs = static_cast<std::uint32_t>(value);
        chad::chassis.setPidTuning(cc);
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
        // The roughening the cloud ramps to once the measurements stop fitting.
        // Decoupling it from distResampleVariance is the whole kidnap-recovery
        // mechanism, and it had no knob here at all.
        else if (k == "mclSearchVar")     c.distSearchVariance = value;
        else if (k == "mclThetaSearchVar") c.thetaSearchVariance = value;
        else if (k == "mclSearchMismatchFull") c.searchMismatchFull = value;
        else if (k == "mclSearchMismatchFloor") c.searchMismatchFloor = value;
        else if (k == "mclLostFitSigmas") c.lostFitSigmas = value;
        else if (k == "mclInjectResampleFloor") c.injectResampleFloor = value;
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

// ---------------------------------------------------------------------------
// Reading parameters back, and the baseline sim_init established
// ---------------------------------------------------------------------------
//
// sim_reset deliberately does NOT restore any of this: it CONSUMES the knobs
// written before it, so restoring would remove the ability to configure the
// world a reset establishes. The measurement behind that and the full contract
// are in sim/README.md, "What sim_reset restores, and what it does not".

namespace {

/// Every knob sim_set_param can move, as one copyable value. Two of these turn
/// "is this trial running the configuration config.cpp ships?" from a
/// hand-written list into a comparison.
struct ParamSet {
    sim::RobotParams world{};
    std::decay_t<decltype(chad::chassis.mcl().config())> mcl{};
    std::decay_t<decltype(chad::chassis.ekf().config())> ekf{};
    std::decay_t<decltype(chad::chassis.rcl().config())> rcl{};
    chad::ChassisConfig chassis{};
    chad::RAMSETEParams ramsete{};
    chad::MoveToPointParams mtp{};
    std::uint32_t seed{0};
};

ParamSet liveParams() {
    ParamSet s;
    s.world = sim::world().params();
    s.mcl = chad::chassis.mcl().config();
    s.ekf = chad::chassis.ekf().config();
    s.rcl = chad::chassis.rcl().config();
    s.chassis = chad::chassisConfig;
    s.ramsete = gRamsete;
    s.mtp = gMoveToPoint;
    s.seed = gSeed;
    return s;
}

/// `seed` is the one key this does not write. It names the trial rather than the
/// model, sim_reset already re-derives both RNGs from it, and restoring it would
/// silently re-randomise a sweep that had pinned it. sim_param_restored()
/// reports that at runtime, so it is not true only in this comment.
void applyParams(const ParamSet& s) {
    sim::world().params() = s.world;
    chad::chassis.mcl().setConfig(s.mcl);
    chad::chassis.ekf().setConfig(s.ekf);
    chad::chassis.rcl().setConfig(s.rcl);
    chad::chassisConfig = s.chassis;
    chad::chassis.setPidTuning(s.chassis);
    gRamsete = s.ramsete;
    gMoveToPoint = s.mtp;
}

/// One key out of one set. sim_get_param and the baseline read both go through
/// here, so a slider and a leak check cannot be reading different things.
float readParam(const ParamSet& s, const std::string& k) {
    if      (k == "responseTauMs")     return s.world.responseTauMs;
    else if (k == "leftScale")         return s.world.leftScale;
    else if (k == "rightScale")        return s.world.rightScale;
    else if (k == "ekfSyncProp")       return s.ekf.syncProp;
    else if (k == "ekfThetaSyncProp")  return s.ekf.thetaSyncProp;
    else if (k == "encoderSlip")       return s.world.encoderSlip;
    else if (k == "imuUnderReport")    return s.world.imuUnderReport;
    else if (k == "imuDriftDegPerSec") return s.world.imuDriftDegPerSec;
    else if (k == "imuNoiseDeg")       return s.world.imuNoiseDeg;
    else if (k == "distNoiseMm")       return s.world.distNoiseMm;
    else if (k == "distMaxRangeMm")    return s.world.distMaxRangeMm;
    else if (k == "distMinRangeMm")    return s.world.distMinRangeMm;
    else if (k == "distPeriodMs")      return s.world.distPeriodMs;
    else if (k == "distFovDeg")        return s.world.distFovDeg;
    else if (k == "distConeRays")      return static_cast<float>(s.world.distConeRays);
    else if (k == "distMaxIncidenceDeg") return s.world.distMaxIncidenceDeg;
    else if (k == "distConeSigmaFrac") return s.world.distConeSigmaFrac;
    else if (k == "collideObstacles")  return s.world.collideObstacles ? 1.0f : 0.0f;
    else if (k == "imuPeriodMs")       return s.world.imuPeriodMs;
    else if (k == "encoderPeriodMs")   return s.world.encoderPeriodMs;
    else if (k == "encoderResolutionDeg") return s.world.encoderResolutionDeg;

    const auto& cc = s.chassis;
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
    else if (k == "settleSpeedIn") return cc.settleSpeedIn;
    else if (k == "settleSpeedDeg") return cc.settleSpeedDeg;
    else if (k == "stallSpeedIn")  return cc.stallSpeedIn;
    else if (k == "stallSpeedDeg") return cc.stallSpeedDeg;
    else if (k == "stallTimeMs")   return static_cast<float>(cc.stallTimeMs);
    else if (k == "stallGraceMs")  return static_cast<float>(cc.stallGraceMs);

    else if (k == "seed")          return static_cast<float>(s.seed);

    else if (k == "ramseteB")          return s.ramsete.b;
    else if (k == "ramseteZeta")       return s.ramsete.zeta;
    else if (k == "ramseteMinSpeed")   return s.ramsete.minSpeed;
    else if (k == "ramseteMaxAccel")   return s.ramsete.maxAccel;
    else if (k == "ramseteLookahead")  return s.ramsete.lookaheadDist;
    else if (k == "ramseteTolerance")  return s.ramsete.targetTolerance;
    else if (k == "ramseteHeadingTol") return s.ramsete.headingTolerance;
    else if (k == "ramseteStall")      return s.ramsete.stallSpeed;
    else if (k == "ramseteProfile")    return s.ramsete.useVelocityProfile ? 1.0f : 0.0f;
    else if (k == "ramseteLatAccel")   return s.ramsete.maxLateralAccel;
    else if (k == "ramseteMaxSpeed")   return s.ramsete.maxSpeed;
    else if (k == "ramseteVelCtrl")    return s.ramsete.useVelocityController ? 1.0f : 0.0f;
    else if (k == "ramseteFfKv")       return s.ramsete.ffGains.kV;
    else if (k == "ramseteFfKs")       return s.ramsete.ffGains.ksStraight;
    else if (k == "ramseteFfKp")       return s.ramsete.ffGains.kP;
    else if (k == "ramseteFfKi")       return s.ramsete.ffGains.kI;
    else if (k == "mtpBearingHold")    return s.mtp.bearingHoldDist;
    else if (k == "mtpTolerance")      return s.mtp.targetTolerance;

    // Every key sim_set_param accepts is readable, or a slider cannot show the
    // value it is about to overwrite and a test cannot assert that a set landed.
    if (k.rfind("ekf", 0) == 0) {
        const auto& c = s.ekf;
        if      (k == "ekfMclSigmaScale") return c.mclSigmaScale;
        else if (k == "ekfMclSigmaFloor") return c.mclSigmaFloor;
        else if (k == "ekfRangeGate")     return c.rangeGateChi2;
        else if (k == "ekfObsTol")        return c.obstacleRangeTol;
        else if (k == "ekfObsSigmaPad")   return c.obstacleSigmaPad;
        else if (k == "ekfSlipProp")      return c.slipNoiseProp;
        else if (k == "ekfVertProp")      return c.vertNoiseProp;
        else if (k == "ekfVertBias")      return c.vertBiasProp;
        else if (k == "ekfSlipBias")      return c.slipBiasProp;
        else if (k == "ekfBiasDecorr")    return c.biasDecorrIn;
        else if (k == "ekfRangeSigBase")  return c.rangeSigmaBase;
        else if (k == "ekfRangeSigProp")  return c.rangeSigmaProp;
        else if (k == "ekfMaxIncidence")  return c.maxIncidenceRad;
        else if (k == "ekfImuSigma")      return c.imuSigmaRad;
        else if (k == "ekfPosFloor")      return c.posNoiseFloor;
        else if (k == "ekfThetaPerInch")  return c.thetaNoisePerInch;
        else if (k == "ekfMclGate")       return c.mclGateChi2;
        else if (k == "ekfUseRange")      return c.useRangeUpdates ? 1.0f : 0.0f;
        else if (k == "ekfResetCycles")   return static_cast<float>(c.mclResetCycles);
        return std::nanf("");
    }

    if (k.rfind("rcl", 0) == 0) {
        const auto& c = s.rcl;
        if      (k == "rclHardSync") return c.hardSync ? 1.0f : 0.0f;
        else if (k == "rclSyncProp") return c.syncProp;
        else if (k == "rclSyncRate") return c.maxSyncRate;
        else if (k == "rclObsTol")   return c.obstacleRangeTol;
        return std::nanf("");
    }

    if (k.rfind("mcl", 0) == 0) {
        const auto& c = s.mcl;
        if      (k == "mclObsCone")       return c.obstacleConeFrac;
        else if (k == "mclObsTol")        return c.obstacleRangeTol;
        else if (k == "mclSyncProp")      return c.distSyncProp;
        else if (k == "mclDistSyncProp")  return c.distSyncProp;
        else if (k == "mclRangeSigClose") return c.rangeSigmaCloseIn;
        else if (k == "mclRangeSigProp")  return c.rangeSigmaProp;
        else if (k == "mclShortWeight")   return c.shortWeight;
        else if (k == "mclShortDecay")    return c.shortDecaySigmas;
        else if (k == "mclRandWeight")    return c.randWeight;
        else if (k == "mclMaxRandomFrac") return c.maxRandomFrac;
        else if (k == "mclDeadzoneLikelihood") return c.deadzoneLikelihood;
        else if (k == "mclResampleVar")   return c.distResampleVariance;
        else if (k == "mclThetaResampleVar") return c.thetaResampleVariance;
        else if (k == "mclSearchVar")     return c.distSearchVariance;
        else if (k == "mclThetaSearchVar") return c.thetaSearchVariance;
        else if (k == "mclSearchMismatchFull") return c.searchMismatchFull;
        else if (k == "mclSearchMismatchFloor") return c.searchMismatchFloor;
        else if (k == "mclLostFitSigmas") return c.lostFitSigmas;
        else if (k == "mclInjectResampleFloor") return c.injectResampleFloor;
        else if (k == "mclResampleThresh") return static_cast<float>(c.resampleThreshold);
        else if (k == "mclImeVariance")   return c.imeVariance;
        else if (k == "mclHorizProp")     return c.horizDependentVarianceProp;
        else if (k == "mclHorizConst")    return c.horizConstantNoise;
        else if (k == "mclImuVariance")   return c.imuVariance;
        else if (k == "mclMaxThetaDev")   return c.maxThetaDeviation;
        else if (k == "mclObsConeFrac")   return c.obstacleConeFrac;
        else if (k == "mclObsRangeTol")   return c.obstacleRangeTol;
        return std::nanf("");
    }

    return std::nanf("");
}

ParamSet gBaseline;

const char* const kParamNames[] = {
    "responseTauMs", "leftScale", "rightScale", "encoderSlip", "imuUnderReport",
    "imuDriftDegPerSec", "imuNoiseDeg", "distNoiseMm", "distMaxRangeMm",
    "distMinRangeMm", "distPeriodMs", "distFovDeg", "distConeRays",
    "distMaxIncidenceDeg", "distConeSigmaFrac", "collideObstacles",
    "imuPeriodMs", "encoderPeriodMs", "encoderResolutionDeg", "lateralKp",
    "lateralKi", "lateralKd", "angularKp", "angularKi", "angularKd",
    "mclObsCone", "mclObsTol", "ekfMclSigmaScale", "ekfMclSigmaFloor",
    "ekfRangeGate", "ekfObsTol", "ekfObsSigmaPad", "ekfSlipProp", "ekfVertProp",
    "ekfVertBias", "ekfSlipBias", "ekfBiasDecorr", "ekfRangeSigBase",
    "ekfRangeSigProp", "ekfMaxIncidence", "ekfImuSigma", "ekfPosFloor",
    "ekfThetaPerInch", "ekfMclGate", "ekfUseRange", "ekfResetCycles",
    "ekfSyncProp", "ekfThetaSyncProp", "lateralDFilt", "angularDFilt",
    "lateralWindup", "angularWindup", "settleTimeMs", "settleSpeedIn",
    "settleSpeedDeg", "stallSpeedIn", "stallSpeedDeg", "stallTimeMs",
    "stallGraceMs", "rclHardSync", "rclSyncProp", "rclSyncRate", "rclObsTol",
    "mclRangeSigClose", "mclRangeSigProp", "mclShortWeight", "mclShortDecay",
    "mclRandWeight", "mclMaxRandomFrac", "mclDeadzoneLikelihood", "mclResampleVar",
    "mclThetaResampleVar", "mclSearchVar", "mclThetaSearchVar", "mclSearchMismatchFull",
    "mclSearchMismatchFloor", "mclLostFitSigmas", "mclInjectResampleFloor",
    "mclResampleThresh", "mclImeVariance", "mclHorizProp", "mclHorizConst",
    "mclImuVariance", "mclMaxThetaDev", "mclObsConeFrac", "mclObsRangeTol",
    "mclSyncProp", "ramseteB", "ramseteZeta", "ramseteMinSpeed", "ramseteMaxAccel",
    "ramseteLookahead", "ramseteTolerance", "ramseteHeadingTol", "ramseteStall",
    "ramseteProfile", "ramseteLatAccel", "ramseteMaxSpeed", "ramseteVelCtrl",
    "ramseteFfKv", "ramseteFfKs", "ramseteFfKp", "ramseteFfKi", "mtpBearingHold",
    "mtpTolerance", "mclDistSyncProp", "seed",
};
constexpr int kParamCount =
    static_cast<int>(sizeof(kParamNames) / sizeof(kParamNames[0]));

std::string gParamJson;

void takeParamBaseline() { gBaseline = liveParams(); }

bool paramRestored(const std::string& k) { return k != "seed"; }

int restorableParamCount() {
    int n = 0;
    for (int i = 0; i < kParamCount; i++) n += paramRestored(kParamNames[i]) ? 1 : 0;
    return n;
}

/// Exact comparison on purpose: these are float32 fields written through
/// float32, so a value that round-tripped is bit-identical and anything else is
/// a real difference.
///
/// Restored keys only. A moved `seed` is what every trial does on purpose, so
/// counting it would make "dirty == 0" fail for correct callers and stop being
/// an assertion anyone could use.
int countDirtyParams(const ParamSet& live) {
    int n = 0;
    for (int i = 0; i < kParamCount; i++) {
        const std::string k(kParamNames[i]);
        if (paramRestored(k) && readParam(live, k) != readParam(gBaseline, k)) n++;
    }
    return n;
}

}  // namespace

/// Read back a tunable. The browser's sliders used to publish their own markup
/// defaults on load, which silently overwrote whatever gamespecific/config.cpp
/// had been tuned to -- the panel has to ask what the value IS, not assert one.
/// Returns NaN for an unknown key so the caller can fall back.
EMSCRIPTEN_KEEPALIVE
float sim_get_param(const char* name) {
    return readParam(liveParams(), std::string(name));
}

/// The value sim_init() found for the same key. NaN for an unknown key.
EMSCRIPTEN_KEEPALIVE
float sim_param_baseline(const char* name) {
    return readParam(gBaseline, std::string(name));
}

/// How many keys sim_set_param accepts, and the i'th name. Enumerable so a
/// caller can sweep, diff or display the whole set without keeping a second copy
/// of the list -- sim/test/ui.mjs parses both if/else chains out of this file and
/// fails if the three ever disagree.
EMSCRIPTEN_KEEPALIVE
int sim_param_count() { return kParamCount; }

EMSCRIPTEN_KEEPALIVE
const char* sim_param_name(int i) {
    return (i >= 0 && i < kParamCount) ? kParamNames[i] : "";
}

/// 1 if sim_params_restore() writes this key, 0 if it deliberately does not,
/// -1 if sim_set_param does not know it at all.
EMSCRIPTEN_KEEPALIVE
int sim_param_restored(const char* name) {
    const std::string k(name);
    bool known = false;
    for (int i = 0; i < kParamCount && !known; i++) known = (k == kParamNames[i]);
    if (!known) return -1;
    return paramRestored(k) ? 1 : 0;
}

/// Put every restored key back to what sim_init() found.
EMSCRIPTEN_KEEPALIVE
void sim_params_restore() { applyParams(gBaseline); }

/// Every restored key whose current value differs from the baseline, with both
/// values. One call at the top of a trial asserts independence over the whole
/// set. The probe that blinded the beams for every later trial DID list
/// distMaxRangeMm in its hand-written baseline -- in the block that ran before
/// the reset, which is why listing keys by hand is not the fix.
EMSCRIPTEN_KEEPALIVE
const char* sim_params_dirty_json() {
    const ParamSet live = liveParams();
    std::string items;
    int n = 0;
    for (int i = 0; i < kParamCount; i++) {
        const std::string k(kParamNames[i]);
        if (!paramRestored(k)) continue;
        const float now = readParam(live, k);
        const float was = readParam(gBaseline, k);
        if (now == was) continue;
        char b[160];
        std::snprintf(b, sizeof(b), "%s{\"k\":\"%s\",\"is\":%.6g,\"was\":%.6g}",
                      n ? "," : "", kParamNames[i], now, was);
        items += b;
        n++;
    }
    gParamJson = "{\"count\":" + std::to_string(n) +
                 ",\"of\":" + std::to_string(restorableParamCount()) +
                 ",\"params\":[" + items + "]}";
    return gParamJson.c_str();
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
void sim_field_add_segment_kind(float x1, float y1, float x2, float y2, int kind) {
    sim::field().addSegment(x1, y1, x2, y2, kind);
}

EMSCRIPTEN_KEEPALIVE
void sim_field_add_circle(float x, float y, float r) {
    sim::field().addCircle(x, y, r);
}

// ============================================================================
// Motion outcomes, settling and chaining
// ============================================================================
//
// The Route DSL runs motions but reports nothing about how they ended, and its
// commands carry no earlyExitRange or minSpeed -- so the whole outcome and
// chaining API was unreachable from here. This runs a scripted list of motions
// on its own fiber through the ordinary blocking Chassis API and keeps one
// record per motion: the outcome, whether it arrived, how long it took, when the
// settle test first passed, and the drive speed and command on both sides of the
// call. The last of those is what makes a handover distinguishable from a stop.

namespace {

float meanDriveCommand() {
    return 0.5f * (sideCommandVolts(chad::drivetrain.leftMotors) +
                   sideCommandVolts(chad::drivetrain.rightMotors));
}

void runMotionStep(const MotionStep& s) {
    const float tol = s.tolerance;
    switch (s.type) {
    case static_cast<int>(chad::MotionType::Distance): {
        chad::MoveDistanceParams p;
        p.forwards = (s.a >= 0.0f) != s.back;
        p.maxSpeed = s.maxSpeed;
        p.minSpeed = s.minSpeed;
        if (tol >= 0.0f) p.targetTolerance = tol;
        p.earlyExitRange = s.earlyExit;
        p.holdForTimeout = s.holdWindow;
        chad::chassis.moveDistance(std::fabs(s.a), s.timeout, p, s.async);
        break;
    }
    case static_cast<int>(chad::MotionType::Point): {
        chad::MoveToPointParams p;
        p.forwards = !s.back;
        p.maxSpeed = s.maxSpeed;
        p.minSpeed = s.minSpeed;
        if (tol >= 0.0f) p.targetTolerance = tol;
        p.earlyExitRange = s.earlyExit;
        chad::chassis.moveToPoint(s.a, s.b, s.timeout, p, s.async);
        break;
    }
    case static_cast<int>(chad::MotionType::Heading): {
        chad::TurntoHeadingParams p;
        p.maxSpeed = s.maxSpeed;
        p.minSpeed = s.minSpeed;
        if (tol >= 0.0f) p.targetTolerance = tol;
        p.holdForTimeout = s.holdWindow;
        chad::chassis.turnToHeading(s.a, s.timeout, p, s.async);
        break;
    }
    case static_cast<int>(chad::MotionType::TurnPoint): {
        chad::TurnToPointParams p;
        p.maxSpeed = s.maxSpeed;
        p.minSpeed = s.minSpeed;
        if (tol >= 0.0f) p.targetTolerance = tol;
        chad::chassis.turnToPoint(s.a, s.b, s.timeout, p, s.async);
        break;
    }
    case static_cast<int>(chad::MotionType::Arc): {
        chad::ArcParams p;
        p.forwards = !s.back;
        p.maxSpeed = s.maxSpeed;
        p.minSpeed = s.minSpeed;
        if (tol >= 0.0f) p.targetTolerance = tol;
        chad::chassis.moveArc(s.a, s.b, s.c, s.timeout, p, s.async);
        break;
    }
    case static_cast<int>(chad::MotionType::Boomerang): {
        chad::BoomerangParams p;
        p.forwards = !s.back;
        p.maxSpeed = s.maxSpeed;
        p.minSpeed = s.minSpeed;
        if (tol >= 0.0f) p.targetTolerance = tol;
        chad::chassis.moveBoomerang(s.a, s.b, s.c, s.timeout, p, s.async);
        break;
    }
    case static_cast<int>(chad::MotionType::Swing): {
        chad::SwingParams p;
        p.maxSpeed = s.maxSpeed;
        p.minSpeed = s.minSpeed;
        if (tol >= 0.0f) p.targetTolerance = tol;
        const auto side = (s.b >= 0.5f) ? chad::SwingSide::Right
                                        : chad::SwingSide::Left;
        chad::chassis.swingToHeading(s.a, side, s.timeout, p, s.async);
        break;
    }
    default:
        break;
    }
}

void execMotionProgram() {
    for (std::size_t i = 0; i < gMotionSteps.size(); i++) {
        const MotionStep& s = gMotionSteps[i];
        MotionRecord r;
        r.index = static_cast<int>(i);
        r.vIn = sim::world().truth().vLinIn;
        r.cmdIn = meanDriveCommand();

        resetTrace();
        const double t0 = sim::preciseMillis();
        runMotionStep(s);
        const double t1 = sim::preciseMillis();

        r.ms = static_cast<float>(t1 - t0);
        r.toleranceMs = (gTraceFirstToleranceMs >= 0.0)
            ? static_cast<float>(gTraceFirstToleranceMs - t0) : -1.0f;
        r.type = gTraceType;
        r.ticks = gTraceTicks;
        r.peakLat = gTracePeakLat;
        r.peakAng = gTracePeakAng;
        r.outcome = static_cast<int>(chad::chassis.lastOutcome());
        r.arrived = chad::chassis.lastMotionArrived();
        r.settled = chad::chassis.isSettled();
        r.vOut = sim::world().truth().vLinIn;
        r.cmdOut = meanDriveCommand();
        const chad::Pose p = chad::chassis.getPose();
        r.x = p.x;
        r.y = p.y;
        r.h = chad::stdRadToVexDeg(p.theta);
        r.trueX = sim::world().truth().x;
        r.trueY = sim::world().truth().y;
        gMotionRecords.push_back(r);

        if (s.delayAfterMs > 0.0f) {
            pros::delay(static_cast<std::uint32_t>(s.delayAfterMs));
        }
    }
    gMotionActive = false;
}

}  // namespace

EMSCRIPTEN_KEEPALIVE
void sim_motion_clear() {
    gMotionSteps.clear();
    gMotionRecords.clear();
}

/// Append a motion. `type` is chad::MotionType as an int, so a test naming 9
/// gets a swing and the record's own `type` -- which comes back off the
/// library's telemetry -- says whether the library agrees.
EMSCRIPTEN_KEEPALIVE
void sim_motion_push(int type, float a, float b, float c) {
    MotionStep s;
    s.type = type;
    s.a = a;
    s.b = b;
    s.c = c;
    gMotionSteps.push_back(s);
}

/// Set a parameter on the most recently pushed motion.
EMSCRIPTEN_KEEPALIVE
void sim_motion_set(const char* name, float value) {
    if (gMotionSteps.empty() || name == nullptr) return;
    MotionStep& s = gMotionSteps.back();
    const std::string k(name);
    if      (k == "timeout")    s.timeout = value;
    else if (k == "maxSpeed")   s.maxSpeed = value;
    else if (k == "minSpeed")   s.minSpeed = value;
    else if (k == "tolerance")  s.tolerance = value;
    else if (k == "earlyExit")  s.earlyExit = value;
    else if (k == "delayAfter") s.delayAfterMs = value;
    else if (k == "back")       s.back = (value != 0.0f);
    else if (k == "async")      s.async = (value != 0.0f);
    else if (k == "holdWindow") s.holdWindow = (value != 0.0f);
}

EMSCRIPTEN_KEEPALIVE
void sim_motion_run() {
    if (gMotionSteps.empty()) return;
    if (gRouteTask != sim::kInvalidTask) sim::sched().kill(gRouteTask);
    gMotionRecords.clear();
    gManualActive = false;
    gMotionActive = true;
    // Own fiber, so waitUntilSettled() and the blocking API behave exactly as
    // they do on the brain and the caller keeps driving the clock.
    gRouteTask = sim::sched().spawn([]() { execMotionProgram(); }, "motion");
}

EMSCRIPTEN_KEEPALIVE
int sim_motion_running() { return (gMotionActive && sim_route_running()) ? 1 : 0; }

EMSCRIPTEN_KEEPALIVE
int sim_motion_done() { return static_cast<int>(gMotionRecords.size()); }

/// Left and right drive command in volts, port signs undone. Zero here is the
/// safety property every outcome except ChainedOut has to end with.
EMSCRIPTEN_KEEPALIVE
float sim_drive_cmd_left() { return sideCommandVolts(chad::drivetrain.leftMotors); }

EMSCRIPTEN_KEEPALIVE
float sim_drive_cmd_right() { return sideCommandVolts(chad::drivetrain.rightMotors); }

EMSCRIPTEN_KEEPALIVE
const char* sim_motion_json() {
    std::string out = "[";
    bool first = true;
    for (const auto& r : gMotionRecords) {
        char b[512];
        std::snprintf(b, sizeof(b),
            "%s{\"i\":%d,\"type\":%d,\"outcome\":%d,\"name\":\"%s\","
            "\"arrived\":%s,\"settled\":%s,\"ms\":%.1f,\"toleranceMs\":%.1f,"
            "\"ticks\":%d,\"vIn\":%.2f,\"vOut\":%.2f,\"cmdIn\":%.2f,"
            "\"cmdOut\":%.2f,\"peakLat\":%.1f,\"peakAng\":%.1f,"
            "\"x\":%.3f,\"y\":%.3f,\"h\":%.2f,\"trueX\":%.3f,\"trueY\":%.3f}",
            first ? "" : ",", r.index, r.type, r.outcome,
            chad::outcomeName(static_cast<chad::MotionOutcome>(r.outcome)),
            r.arrived ? "true" : "false", r.settled ? "true" : "false",
            r.ms, r.toleranceMs, r.ticks, r.vIn, r.vOut, r.cmdIn, r.cmdOut,
            r.peakLat, r.peakAng, r.x, r.y, r.h, r.trueX, r.trueY);
        out += b;
        first = false;
    }
    out += "]";
    gMotionJson = out;
    return gMotionJson.c_str();
}

/// Cancel whatever is running through the library's own path, so the recorded
/// outcome is the one a routine would see.
EMSCRIPTEN_KEEPALIVE
void sim_motion_cancel() { chad::chassis.cancelMotion(); }

// ============================================================================
// Fault injection and MCL seeding
// ============================================================================

/// Fault every drive encoder at once. 0 restores them, 1 makes every port
/// report PROS_ERR, 2 makes every port report a valid constant zero -- the
/// failure a scalar average cannot tell from standing still. Cleared by
/// sim_reset.
EMSCRIPTEN_KEEPALIVE
void sim_kill_drive_encoders(int mode) {
    gEncodersDead = (mode < 0 || mode > 2) ? 0 : mode;
    applyDeadEncoders();
}

EMSCRIPTEN_KEEPALIVE
int sim_drive_encoders_dead() { return gEncodersDead; }

/// Reseed MCL's particle RNG without touching the world's. Same seed replays a
/// divergence; a different one is an independent run against identical sensor
/// data, which is the only way to tell a filter problem from a world problem.
/// The cloud is re-scattered, or the run would still carry the old seed's
/// initialisation.
EMSCRIPTEN_KEEPALIVE
void sim_mcl_seed(int seed) {
    chad::chassis.mcl().setSeed(static_cast<std::uint32_t>(seed));
    chad::chassis.mcl().setPose(chad::chassis.getPose());
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

    auto& ekfT = chad::chassis.ekf();
    const auto ekfCfg = ekfT.config();
    const auto ekfBeams = ekfT.beamStates();
    const auto ekfInnov = ekfT.innovations();
    auto& od = chad::chassis.odom();
    const chad::EncoderAverage encPos = chad::drivetrain.averagePositionDegLive();
    const chad::WheelReading encVel = chad::drivetrain.wheelVelocitiesLive();

    auto& buf = jsonBuf();
    buf.n = 0;
    buf.addf(
        "{\"t\":%.3f,"
        "\"true\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f},"
        "\"odom\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f},"
        "\"mcl\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f,\"nEff\":%.1f,"
            "\"gen\":%u,\"resamples\":%u,\"mismatch\":%.3f,"
            "\"fit\":%.2f,\"searchMis\":%.3f},"
        "\"rcl\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f},"
        "\"ekf\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f,\"sx\":%.3f,\"sy\":%.3f,"
            "\"st\":%.3f,\"beams\":%d,\"reloc\":%d},"
        "\"mclInject\":%.3f,"
        "\"err\":{\"dist\":%.3f,\"heading\":%.2f},"
        "\"vel\":{\"lin\":%.2f,\"ang\":%.2f,\"left\":%.2f,\"right\":%.2f},"
        "\"colliding\":%s,\"routeRunning\":%s,\"settled\":%s,"
        "\"locMode\":%d,\"tasks\":%d,",
        sim::preciseMillis() / 1000.0,
        t.x, t.y, chad::stdRadToVexDeg(t.theta),
        odom.x, odom.y, chad::stdRadToVexDeg(odom.theta),
        mclRaw.x, mclRaw.y, chad::stdRadToVexDeg(mclRaw.theta), nEff,
        chad::chassis.mcl().estimateGeneration(),
        chad::chassis.mcl().resampleCount(),
        chad::chassis.mcl().likelihoodMismatch(),
        chad::chassis.mcl().fitSigmas(),
        chad::chassis.mcl().searchMismatch(),
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

    // Diagnostics. Every filter here can fail by silently discarding what it is
    // told, and a filter that rejects everything looks exactly like a healthy
    // one from the pose alone. `gate` is the Mahalanobis threshold each family is
    // being held to; the per-beam innovation and state are in "filters" below.
    buf.addf(
        "\"diag\":{"
        "\"ekf\":{\"gateR\":%.2f,\"gateM\":%.2f,\"gateH\":%.2f,"
            "\"rejR\":%u,\"rejH\":%u,\"rejM\":%u,"
            "\"streakR\":%d,\"streakBlind\":%d,\"streakM\":%d,"
            "\"noReturn\":%u,\"deadzoned\":%u,\"grazed\":%u,"
            "\"imuSkips\":%u,\"repairs\":%u},"
        "\"odom\":{\"faults\":%u,\"outages\":%u,\"jumps\":%u,\"hjumps\":%u,"
            "\"imuFaults\":%u},"
        "\"enc\":{\"live\":%d,\"faulted\":%d,\"valid\":%s,"
            "\"velLive\":%d,\"velFaulted\":%d,\"velValid\":%s},"
        "\"params\":{\"dirty\":%d,\"of\":%d}},",
        ekfCfg.rangeGateChi2, ekfCfg.mclGateChi2, ekfCfg.imuGateChi2,
        ekfT.rangeRejects(), ekfT.headingRejects(), ekfT.mclRejects(),
        ekfT.rangeRejectStreak(), ekfT.rangeBlindStreak(), ekfT.mclRejectStreak(),
        ekfT.rangeNoReturns(), ekfT.rangeDeadzoned(), ekfT.rangeGrazed(),
        ekfT.imuSkips(), ekfT.covarianceRepairs(),
        od.distanceFaults(), od.distanceOutages(), od.jumpRejects(),
        od.headingJumpRejects(), od.imuFaults(),
        encPos.live, encPos.faulted, encPos.valid() ? "true" : "false",
        encVel.live, encVel.faulted, encVel.valid() ? "true" : "false",
        countDirtyParams(liveParams()), restorableParamCount());

    // The live motion outcome. Chassis::lastOutcome() and lastMotionArrived()
    // were only reachable through sim_motion_json, i.e. only for a scripted
    // motion program -- so a route, an auton or a trajectory follow reported
    // nothing about how it ended. `toleranceAtMs` is when the settle test first
    // passed on the CURRENT motion, in the same virtual ms as "t"; -1 means it
    // never has, which is honest for a curved path that arrives off-tangent.
    // cmdL/cmdR are the drive command with the port signs undone: nonzero after
    // a motion returns is the chaining seam, and zero is a full stop.
    buf.addf(
        "\"motion\":{\"outcome\":%d,\"name\":\"%s\",\"arrived\":%s,"
        "\"type\":%d,\"ticks\":%d,\"toleranceAtMs\":%.1f,"
        "\"peakLat\":%.1f,\"peakAng\":%.1f,\"cmdL\":%.2f,\"cmdR\":%.2f,"
        "\"records\":%d,\"trajRunning\":%s,\"mpcRunning\":%s},",
        static_cast<int>(chad::chassis.lastOutcome()),
        chad::outcomeName(chad::chassis.lastOutcome()),
        chad::chassis.lastMotionArrived() ? "true" : "false",
        gTraceType, gTraceTicks, gTraceFirstToleranceMs,
        gTracePeakLat, gTracePeakAng,
        sideCommandVolts(chad::drivetrain.leftMotors),
        sideCommandVolts(chad::drivetrain.rightMotors),
        static_cast<int>(gMotionRecords.size()),
        sim_traj_running() ? "true" : "false",
        sim_mpc_running() ? "true" : "false");

    buf.add("\"beams\":[");
    bool first = true;
    for (int i = 0; i < chad::MAX_DISTANCE_SENSORS; i++) {
        const auto& b = sim::world().beam(i);
        if (!b.active) continue;
        buf.addf(
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
    // visible instead of just showing up as drift. `ekfBeam` is EkfCore::BeamState
    // and `ekfInnov` the beam's residual in inches, so a gate that is throwing
    // away good measurements is visible per sensor rather than as drift.
    buf.add("],\"filters\":[");
    const auto& mclValid = chad::chassis.mcl().validSensors();
    const auto& mclRead  = chad::chassis.mcl().sensorReadings();
    const auto& mclDead  = chad::chassis.mcl().deadzoneFraction();
    const auto& rclSensors = chad::chassis.rcl().sensors();

    first = true;
    for (int i = 0; i < chad::MAX_DISTANCE_SENSORS; i++) {
        if (!sim::world().beam(i).active) continue;
        const auto& rs = rclSensors[i];
        buf.addf(
            "%s{\"i\":%d,\"mclValid\":%s,\"mclIn\":%.2f,"
            "\"rclValid\":%s,\"rclIn\":%.2f,\"rclConf\":%d,\"mclDead\":%.2f,"
            "\"rx\":%.2f,\"ry\":%.2f,\"rdx\":%.3f,\"rdy\":%.3f,"
            "\"ekfBeam\":%d,\"ekfInnov\":%.3f}",
            first ? "" : ",", i,
            mclValid[i] ? "true" : "false", mclRead[i],
            rs.valid ? "true" : "false", rs.readingInch, rs.confidence,
            mclDead[i],
            rs.worldX, rs.worldY, rs.rayCos, rs.raySin,
            static_cast<int>(ekfBeams[i]), ekfInnov[i]);
        first = false;
    }
    buf.add("]}");

    gStateJson.assign(buf.data, static_cast<std::size_t>(buf.n));
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

EMSCRIPTEN_KEEPALIVE
int sim_collide_ticks() { return gCollideTicks; }

}  // extern "C"
