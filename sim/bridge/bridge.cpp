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
#include "../world/actors.hpp"
#include "../world/field.hpp"
#include "../world/pieces.hpp"
#include "../world/toggles.hpp"
#include "../world/world.hpp"

#include "chad/movement_control/spline.hpp"

#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"
#include "gamespecific/claw.hpp"
#include "gamespecific/config.hpp"
#include "gamespecific/controls.hpp"
#include "gamespecific/lift.hpp"

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
float gManualLift = 0.0f;
float gManualRight = 0.0f;
bool  gManualActive = false;
/// One-shot stall for the next chassis telemetry callback, which runs on the
/// motion task: stands in for a motion task that cannot reach its cancel check.
std::uint32_t gMotionStallMs = 0;
bool gMotionStallHooked = false;
/// <SG5>: each robot starts touching one Pin. Off by default, because a
/// routine written without one grabs whatever is in front of the jaws and a
/// preload that was not asked for changes what that is.
bool  gPreload = false;
/// The Preload placed by the last reset, so the next one can take it away
/// again. Cleared whenever the piece set is rebuilt, because loadDefault()
/// restarts numbering and a stale id would then name somebody else.
int   gPreloadId = -1;

sim::TaskId gRouteTask = sim::kInvalidTask;

// --- match period ----------------------------------------------------------
//
// A routine that needs 17 s is not a 15 s routine, and the only place that ever
// shows up otherwise is at a competition. The clock starts with the run and
// cuts the drive at expiry, so a route that does not fit says so here instead.
constexpr float kAutonPeriodMs  = 15000.0f;
constexpr float kSkillsPeriodMs = 60000.0f;
int    gPeriodMode = 0;          // 0 off, 1 auton, 2 skills
double gPeriodStartMs = -1.0;
bool   gPeriodExpired = false;

// Whether the field is carrying game pieces. They start off: a pin or a cup is
// a real obstacle that no filter has in its map, so loading them by default
// would quietly change what every localization run measures -- the MCL
// disturbance battery went from one failure to three when they were on. They
// are a thing you ask for, and once asked for a reset puts the same ones back
// so two runs of a routine face the same field.
bool gPiecesLoaded = false;

/// Arm the far-side gate whenever the period starts, so the sim matches what
/// registerAutonDisableLine() does on the robot without the routine having to
/// call it.
bool gAutoFarSide = false;

float periodLengthMs() {
    if (gPeriodMode == 1) return kAutonPeriodMs;
    if (gPeriodMode == 2) return kSkillsPeriodMs;
    return 0.0f;
}

// Defined with the rest of the route machinery further down. Declared here so
// sim_init() can wrap it into the physics tick.
void sampleRouteStep();
void tickPeriod(double dtMs);

// --- optional dead wheels ---------------------------------------------------
//
// The robot's own tracking wheels come from gamespecific/config.cpp, and World
// publishes their travel on their own ports with their own offsets. These are a
// second, centred pair for comparing against without changing the robot's
// config: swapping them in goes through the same OdomTracking::setSensors() a
// real robot would use. World publishes their travel on ports 11 and 12 either
// way.
pros::Rotation gVertPod(11);
pros::Rotation gHorizPod(12);
chad::TrackingWheel gVertWheel(gVertPod, 2.75f, 0.0f);
chad::TrackingWheel gHorizWheel(gHorizPod, 2.75f, 0.0f);
// 0 = the robot's own sensors from config.cpp, 1 = the centred pair above,
// 2 = no tracking wheels: drive motor encoders and the IMU only.
int gOdomPods = 0;

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


struct AutonEntry { const char* name; const char* symbol; void (*fn)(); };

/// The symbol is stringised so sim/build.sh can pair each entry with the .cpp
/// that defines it, and the browser can draw that source as a planned path.
#define CHAD_AUTON(label, f) { label, #f, chad::f }

void autonNone() {}

// The first two mirror src/main.cpp exactly, so an index here is the same
// routine the brain's selector has at that index. Anything after them is sim
// only, appended rather than interleaved so that stays true.
//
// The list named autonBlue3PinPid, which has never existed in this tree -- the
// sim has not compiled since, and nothing said so because the one gate that
// builds it was not being run.
const AutonEntry kAutons[] = {
    CHAD_AUTON("One Field Two Wall", oneFieldTwoWall),
    { "None", "autonNone", autonNone },

    CHAD_AUTON("Red 3 Pin",     autonRed3Pin),
    CHAD_AUTON("Skills",        autonSkills),
    CHAD_AUTON("UNc",           autonTemp),
    CHAD_AUTON("Red 3 Pin Far", autonRed3PinFar),
    CHAD_AUTON("Example",       autonExample),
};
constexpr int kAutonCount = static_cast<int>(sizeof(kAutons) / sizeof(kAutons[0]));

/// Rebuild the odometry sensor set with or without the dead wheels.
void applyOdomPods() {
    chad::OdomSensors sensors = chad::odomSensors;
    if (gOdomPods == 1) {
        sensors.vertWheelCollection = { &gVertWheel };
        sensors.horizWheelCollection = { &gHorizWheel };
    } else if (gOdomPods == 2) {
        sensors.vertWheelCollection.clear();
        sensors.horizWheelCollection.clear();
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
    // Hold the lift where it is when nothing is asking for it, so it does not
    // sag while you drive. A routine owns the lift instead, which is why this
    // only runs while the manual drive does.
    chad::liftMotor.move(gManualLift != 0.0f
                             ? static_cast<int>(gManualLift)
                             : static_cast<int>(std::lround(liftHoldPower())));
}

/// Put the Preload in the jaws. <SG5> says one Pin, touching the robot and
/// nothing else, and red loads red/yellow while blue loads blue/yellow -- so
/// which colour it is comes from which side of the Autonomous Line the robot
/// was placed on, the same way registerAutonDisableLine() picks its half.
void placePreload() {
    if (!gPreload) {
        if (gPreloadId >= 0) sim::pieces().remove(gPreloadId);
        gPreloadId = -1;
        sim::pieces().publish();
        return;
    }

    const auto& tr = sim::world().truth();
    const chad::Pose pose{tr.x, tr.y, tr.theta};
    float cx = 0.0f, cy = 0.0f, cdir = 0.0f;
    sim::pieces().clawPoint(pose, cx, cy, cdir);

    // Where a carried piece rides, so the claw is already on it when it shuts.
    const float reach = 2.0f + sim::kPinRadiusIn;
    const float px = cx + std::cos(cdir) * reach;
    const float py = cy + std::sin(cdir) * reach;

    // Moved, not replaced. Removing and re-adding walked the id up by one on
    // every reset, and the HUD names the piece it is holding.
    sim::Piece* p = (gPreloadId >= 0) ? sim::pieces().get(gPreloadId) : nullptr;
    if (p == nullptr) {
        gPreloadId = sim::pieces().add(sim::PieceKind::Pin, px, py, 0.0f);
        p = sim::pieces().get(gPreloadId);
        if (p == nullptr) return;
    } else {
        p->x = px;
        p->y = py;
        p->theta = 0.0f;
        p->goal = -1;
        p->inCup = -1;
        p->level = 0;
    }
    p->upright = true;
    p->color = (tr.x + tr.y < 0.0f) ? sim::PinColor::RedYellow
                                    : sim::PinColor::BlueYellow;

    // A closed claw is already gripping it -- that is how the robot is put on
    // the field, and it has to arrive that way rather than being picked up a
    // tick later once the sensor notices something in front of the jaws.
    if (::clawClosed()) {
        sim::pieces().hold(gPreloadId);
    } else if (sim::pieces().heldId() >= 0) {
        sim::pieces().release();
    }
    sim::pieces().markDirty();
    sim::pieces().publish();
}

/// Arm the auton-period read gate on the diagonal, from wherever the robot is
/// now. Mirrors chad::registerAutonDisableLine(), which the routines call for
/// themselves -- this is for driving the sim by hand.
void armFarSideGate(float lifetimeMs) {
    chad::chassis.rcl().setFarSideGateFromLine(
        chad::Field::FIELD_HALF, -chad::Field::FIELD_HALF,
        -chad::Field::FIELD_HALF, chad::Field::FIELD_HALF, lifetimeMs);
}

void tickPeriod(double dtMs) {
    (void)dtMs;
    if (gPeriodMode == 0 || gPeriodStartMs < 0.0 || gPeriodExpired) return;
    if (sim::preciseMillis() - gPeriodStartMs < periodLengthMs()) return;

    gPeriodExpired = true;
    // Cut the drive here and leave the route task for sim_step to unwind: this
    // runs inside the scheduler, and killing the fiber that is currently on the
    // stack takes the whole run down with it.
    chad::chassis.cancelMotion();
    chad::drivetrain.setVoltage(0.0f, 0.0f);
    gManualLeft = gManualRight = gManualLift = 0.0f;
}

// --- run recorder -----------------------------------------------------------
//
// A flat float ring the browser reads directly out of the heap and plots. One
// row per sample; keep kRecordStride in step with the column list below.

constexpr int kRecordStride = 18;
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
    //          errDist, errHeading, vLin, vAng,
    //          rclResidX, rclResidY, rclSyncStep, rclVotes
    // Keep kRecordStride in step with this list.
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

    // What RCL is asking for against what it is allowed to apply. A residual
    // that stays large while the step stays at the rate limit is the filter
    // being held back; one that oscillates is it fighting itself.
    const auto& r = chad::chassis.rcl();
    gRecord.push_back(r.residualX());
    gRecord.push_back(r.residualY());
    gRecord.push_back(r.lastSyncStepIn());
    gRecord.push_back(static_cast<float>(r.activeSensorCount()));
}

}  // namespace

extern "C" {

// ============================================================================
// Lifecycle
// ============================================================================

// Defined next to sim_set_param and sim_get_param, which it goes through.
EMSCRIPTEN_KEEPALIVE void sim_tuning_apply();

EMSCRIPTEN_KEEPALIVE
void sim_init() {
    if (gStarted) return;
    gStarted = true;

    sim::seedRng(gSeed);
    sim::world().init();
    sim::toggles().loadDefault();

    // World::init() installs itself as the physics function; wrap it so route
    // clearance is sampled on the same 10 ms tick the physics runs on rather
    // than once per browser frame.
    sim::setPhysicsFn([](double dtMs) {
        // Actors first: their footprints have to be on the field BEFORE the
        // distance sensors are synthesised, or every beam is one tick behind
        // the robot it is supposed to be ranging.
        sim::actors().step(dtMs);
        sim::actors().publish();
        // Pieces follow the claw, so they move with the robot's TRUE pose and
        // have to be placed before the beams are synthesised -- a pin in the
        // jaws is something the claw sensor can see.
        const auto& tr = sim::world().truth();
        const auto& rp = sim::world().params();
        sim::pieces().step({tr.x, tr.y, tr.theta}, ::clawClosed(),
                           liftNearestPin(liftPosition()),
                           0.5f * rp.robotLengthIn, 0.5f * rp.robotWidthIn,
                           dtMs);
        sim::world().step(dtMs);
        // After the world, so it sees the pose the robot actually reached: a
        // toggle is flipped by being pressed against the wall, and the press is
        // exactly the case where the commanded move was refused.
        sim::toggles().step({tr.x, tr.y, tr.theta},
                            0.5f * rp.robotLengthIn, 0.5f * rp.robotWidthIn,
                            sim::world().truth().vLinIn);
        tickPeriod(dtMs);
        sampleRouteStep();
    });

    // Same registration order as src/main.cpp so indices line up with the
    // brain's auton selector.
    for (const auto& a : kAutons) chad::selector.registerAuton(a.name, a.fn);
    chad::selector.init();

    chad::registerFieldObstacles();

    // Odometry only for now; MCL/RCL are opt-in from the UI.
    chad::chassis.odom().startBackgroundOdom();

    sim_tuning_apply();
}

EMSCRIPTEN_KEEPALIVE void sim_set_localization(int mode);

EMSCRIPTEN_KEEPALIVE
void sim_reset(float x, float y, float thetaDeg) {
    const chad::Pose p{x, y, chad::vexToStdRad(thetaDeg)};
    const int wasLoc = gLocMode;

    // Tear the run down through the library's own lifecycle calls so its
    // pros::Task pointers are released properly, rather than killing the
    // fibers out from under it.
    chad::chassis.cancelMotion();
    if (gRouteTask != sim::kInvalidTask) {
        sim::sched().kill(gRouteTask);
        gRouteTask = sim::kInvalidTask;
    }
    // A killed fiber never runs runMirrored's restore.
    chad::chassis.setMirror(chad::FieldMirror::None);
    pros::sim_input::competitionStatus() = 0;
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
    // the pose: reset() re-reads the encoders and IMU and drops whatever delta
    // the previous run left, so the first tick measures from here.
    applyOdomPods();
    chad::chassis.odom().reset();
    chad::chassis.setPose(p);
    // Seeded immediately before its own resample, so the starting cloud does
    // not depend on whether anything else drew from the generator first.
    chad::chassis.mcl().setSeed(gSeed);
    chad::chassis.mcl().setPose(p);
    chad::chassis.ekf().setPose(p);

    gTrueTrail.xy.clear();
    gOdomTrail.xy.clear();
    gRecord.clear();
    gRecordNextMs = 0.0;
    gManualLeft = gManualRight = gManualLift = 0.0f;

    sim::actors().rewind();
    // A re-run has to face the same field, or the second attempt is measuring
    // where the last one happened to leave the pins. An empty field stays empty.
    // loadDefault() renumbers from 1, so anything holding an id from before it
    // is holding somebody else's now.
    if (gPiecesLoaded) { sim::pieces().loadDefault(); gPreloadId = -1; }
    placePreload();
    sim::toggles().resetStates();
    gPeriodStartMs = (gPeriodMode != 0) ? 0.0 : -1.0;
    gPeriodExpired = false;
    chad::chassis.rcl().clearFarSideGate();
    if (gAutoFarSide) armFarSideGate(periodLengthMs());

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

/// autons::resetFromBeamsNear() as a routine calls it. 1 = pose replaced.
EMSCRIPTEN_KEEPALIVE
int sim_beam_reset_near(float maxJumpIn) {
    return chad::autons::resetFromBeamsNear(maxJumpIn) ? 1 : 0;
}

// ============================================================================
// Time
// ============================================================================

EMSCRIPTEN_KEEPALIVE
void sim_step(float ms) {
    applyManualDrive();
    sim::sched().runFor(ms);

    // tickPeriod cut the drive from inside the scheduler; the fiber can only be
    // destroyed out here, where it is not the one running.
    if (gPeriodExpired && gRouteTask != sim::kInvalidTask) {
        sim::sched().kill(gRouteTask);
        gRouteTask = sim::kInvalidTask;
        chad::drivetrain.setVoltage(0.0f, 0.0f);
    }

    const auto& t = sim::world().truth();
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

/// Drive the lift by hand, -127..127. Anything nonzero cancels a lift motion a
/// routine left running, because two things commanding it at once is how you
/// get a lift that will not move and no reason why.
EMSCRIPTEN_KEEPALIVE
void sim_set_lift(float power) {
    if (power != 0.0f && liftBusy()) liftCancel();
    gManualLift = power;
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

/// The C++ function name, which keys the source the browser draws a plan from.
EMSCRIPTEN_KEEPALIVE
const char* sim_auton_symbol(int i) {
    if (i < 0 || i >= kAutonCount) return "";
    return kAutons[i].symbol;
}

/// Rewind the clock for a fresh run, so the HUD's t and every duration in the
/// report are measured from the start of the routine rather than from boot.
/// Paired with resyncSleepers(), or the background loops stay due at times
/// that no longer exist and stall for one period.
void beginRun() {
    if (gRouteTask != sim::kInvalidTask) sim::sched().kill(gRouteTask);
    gRouteTask = sim::kInvalidTask;
    chad::chassis.setMirror(chad::FieldMirror::None);
    gManualActive = false;
    sim::resetClock();
    sim::sched().resyncSleepers();
    // The sampler keys off the clock it was just handed a new zero for.
    gRecordNextMs = 0.0;
    breakTrails();

    // The traffic has to start where it started last time or a re-run measures
    // the actors' phase rather than the change under test.
    sim::actors().rewind();

    gPeriodStartMs = (gPeriodMode != 0) ? 0.0 : -1.0;
    gPeriodExpired = false;
    if (gAutoFarSide) armFarSideGate(periodLengthMs());
}

EMSCRIPTEN_KEEPALIVE
void sim_run_auton(int index) {
    if (index < 0 || index >= kAutonCount) return;
    beginRun();
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
    chad::chassis.setMirror(chad::FieldMirror::None);
    gManualActive = true;
    gManualLeft = gManualRight = gManualLift = 0.0f;
}

/// Competition status bits as PROS reports them: disabled 1, autonomous 2,
/// connected 4.
EMSCRIPTEN_KEEPALIVE
void sim_set_competition(int status) {
    pros::sim_input::competitionStatus() = static_cast<std::uint8_t>(status);
}

/// One master-controller stick, -127..127, in PROS channel order.
EMSCRIPTEN_KEEPALIVE
void sim_set_analog(int channel, int value) {
    if (channel < 0 || channel > 3) return;
    pros::sim_input::state(0).analog[channel] = value;
}

/// Field control switching to driver: what opcontrol() in src/main.cpp does
/// on entry (main.cpp is not compiled here), then driverControl() on its own
/// fiber, as PROS runs it.
EMSCRIPTEN_KEEPALIVE
void sim_enter_driver() {
    if (gRouteTask != sim::kInvalidTask) sim::sched().kill(gRouteTask);
    gRouteTask = sim::kInvalidTask;
    gManualActive = false;

    chad::chassis.cancelMotion();
    liftCancel();
    chad::chassis.setMirror(chad::FieldMirror::None);
    chad::clearAutonDisableLine();
    gRouteTask = sim::sched().spawn([]() { chad::driverControl(); }, "driver");
}

EMSCRIPTEN_KEEPALIVE
void sim_abort_motion() { chad::chassis.abortMotion(); }

/// Hold the motion task inside its next telemetry callback for `ms`.
EMSCRIPTEN_KEEPALIVE
void sim_stall_motion_task(int ms) {
    gMotionStallMs = static_cast<std::uint32_t>(ms > 0 ? ms : 0);
    if (gMotionStallHooked) return;
    gMotionStallHooked = true;
    chad::chassis.setTelemetry([](const chad::ChassisSample&) {
        if (gMotionStallMs == 0) return;
        const std::uint32_t ms = gMotionStallMs;
        gMotionStallMs = 0;
        pros::delay(ms);
    });
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
    OP_CLAW,             // a: 0 retract, 1 extend, 2 toggle
    OP_TANK,             // a: left, b: right, open loop, returns at once
};

// One `.field = value` from a designated initializer in the route source.
// Carried by name rather than by index so the browser does not need the
// layout of eight different params structs -- only the field names, which it
// validates against the headers before pushing.
struct RouteOpt {
    std::string name;
    float value;
};

struct RouteCmd {
    int op;
    float a, b, c, d;
    bool async{false};
    std::vector<RouteOpt> opts;
};

// What one command actually did. This is the point of running a route in the
// sim rather than eyeballing the drawn path: a motion that quietly sat out its
// timeout looks identical on the field to one that settled.
struct RouteStep {
    int op{0};
    float startMs{0.0f};
    float durationMs{0.0f};
    float timeoutMs{0.0f};
    bool  ranOutTimeout{false};
    float targetErrIn{-1.0f};   // truth vs the commanded point, -1 = no point
    float headingErrDeg{-1.0f}; // truth vs the commanded heading, -1 = no turn
    float estErrIn{0.0f};       // truth vs what the robot believes
    float minClearIn{0.0f};     // closest the body came to a wall or post
    bool  collided{false};
    int   optsIgnored{0};
};

// One motion-loop tick plus a margin. A motion that returns this close to its
// timeout cannot have exited on tolerance.
constexpr float kTimeoutSlackMs = 25.0f;

std::vector<RouteCmd> gRoute;
std::vector<RouteStep> gReport;
int gRouteIndex = -1;
float gRouteMaxSpeed = 127.0f;
// Set while a command is executing, so the physics hook knows which report
// entry to fold clearance into.
bool gRouteSampling = false;

// Per-motion defaults, tunable from the browser via sim_set_param. Every op
// starts from the one for its own type rather than from a fresh struct, so a
// value tuned in the panel reaches the motion the route actually runs. A
// route's own options still override these per command.
chad::RAMSETEParams gRamsete;
chad::MoveToPointParams gMoveToPoint;
chad::MoveDistanceParams gDrive;
chad::TurntoHeadingParams gTurn;
chad::TurnToPointParams gTurnPoint;
chad::BoomerangParams gBoomerang;
chad::ArcParams gArc;
chad::SwingParams gSwing;

// Sampled trajectory points, pushed by the browser before a run. An
// OP_FOLLOW_PATH command names a [start, count) slice of this buffer, so one
// route can follow several paths.
std::vector<chad::Pose> gPath;

/// Apply one option to whichever params struct is in hand.
///
/// `requires` rather than eight hand-written appliers: the structs overlap
/// heavily -- every one carries maxSpeed, minSpeed and targetTolerance -- and
/// the duplicated version drifts the moment a field is added to one of them.
/// A field this does not know is counted rather than dropped: silently
/// ignoring a .lead the route asked for would make the sim disagree with the
/// code it was handed, which is the one thing it must not do.
///
/// extern "C++" because this section of the file sits inside the bridge's
/// extern "C" block, and a template cannot be declared with C linkage.
extern "C++" {

template <typename P>
bool applyRouteOpt(P& p, const RouteOpt& o) {
    const float v = o.value;
    const bool flag = v != 0.0f;
    if constexpr (requires { p.forwards; }) {
        if (o.name == "forwards") { p.forwards = flag; return true; }
    }
    if constexpr (requires { p.direction; }) {
        if (o.name == "direction") {
            p.direction = static_cast<chad::TurnDirection>(static_cast<int>(v));
            return true;
        }
    }
    if constexpr (requires { p.maxSpeed; }) {
        if (o.name == "maxSpeed") { p.maxSpeed = v; return true; }
    }
    if constexpr (requires { p.minSpeed; }) {
        if (o.name == "minSpeed") { p.minSpeed = v; return true; }
    }
    if constexpr (requires { p.targetTolerance; }) {
        if (o.name == "targetTolerance") { p.targetTolerance = v; return true; }
    }
    if constexpr (requires { p.earlyExitRange; }) {
        if (o.name == "earlyExitRange") { p.earlyExitRange = v; return true; }
    }
    if constexpr (requires { p.lead; }) {
        if (o.name == "lead") { p.lead = v; return true; }
    }
    if constexpr (requires { p.leadDecay; }) {
        if (o.name == "leadDecay") { p.leadDecay = v; return true; }
    }
    if constexpr (requires { p.headingTolerance; }) {
        if (o.name == "headingTolerance") { p.headingTolerance = v; return true; }
    }
    if constexpr (requires { p.bearingHoldDist; }) {
        if (o.name == "bearingHoldDist") { p.bearingHoldDist = v; return true; }
    }
    if constexpr (requires { p.offset; }) {
        if (o.name == "offset") { p.offset = v; return true; }
    }
    if constexpr (requires { p.b; }) {
        if (o.name == "b") { p.b = v; return true; }
    }
    if constexpr (requires { p.zeta; }) {
        if (o.name == "zeta") { p.zeta = v; return true; }
    }
    if constexpr (requires { p.stallSpeed; }) {
        if (o.name == "stallSpeed") { p.stallSpeed = v; return true; }
    }
    if constexpr (requires { p.lookaheadDist; }) {
        if (o.name == "lookaheadDist") { p.lookaheadDist = v; return true; }
    }
    // The browser splines the preview at this spacing, so failing to pass it on
    // made the drawn curve and the driven curve different curves -- the one
    // thing the sim is supposed to be authoritative about. It showed up in the
    // report's ignored column rather than silently, but only if you read it.
    if constexpr (requires { p.smoothSpacingIn; }) {
        if (o.name == "smoothSpacingIn") { p.smoothSpacingIn = v; return true; }
    }
    if constexpr (requires { p.useVelocityProfile; }) {
        if (o.name == "useVelocityProfile") {
            p.useVelocityProfile = flag;
            return true;
        }
    }
    if constexpr (requires { p.maxAccel; }) {
        if (o.name == "maxAccel") { p.maxAccel = v; return true; }
    }
    if constexpr (requires { p.maxLateralAccel; }) {
        if (o.name == "maxLateralAccel") { p.maxLateralAccel = v; return true; }
    }
    if constexpr (requires { p.useVelocityController; }) {
        if (o.name == "useVelocityController") {
            p.useVelocityController = flag;
            return true;
        }
    }
    if constexpr (requires { p.settleTimeMs; }) {
        if (o.name == "settleTimeMs") {
            p.settleTimeMs = static_cast<std::uint32_t>(v);
            return true;
        }
    }
    return false;
}

template <typename P>
int applyRouteOpts(P& p, const std::vector<RouteOpt>& opts) {
    int ignored = 0;
    for (const auto& o : opts) {
        if (!applyRouteOpt(p, o)) ignored++;
    }
    return ignored;
}

}  // extern "C++"

/// Fold the current instant into the running command's report entry.
///
/// Called from the physics hook, so clearance is sampled every tick rather
/// than once per browser frame: at full speed the robot covers 2 in a tick,
/// and a per-frame sample would step straight over a post.
void sampleRouteStep() {
    if (!gRouteSampling || gReport.empty()) return;
    RouteStep& st = gReport.back();
    const auto& t = sim::world().truth();
    const auto& rp = sim::world().params();

    // Allow for the footprint in the direction the obstacle actually lies,
    // via the rectangle's support function. The circumscribed radius would be
    // simpler and is what this did first, but half a body diagonal is 10.6 in
    // on a 15 in square and the usual start pose is 10.2 in off the wall, so
    // every clearance came out negative and the column said nothing.
    const auto near = sim::field().nearest(t.x, t.y);
    const float dx = near.x - t.x, dy = near.y - t.y;
    const float len = std::hypot(dx, dy);
    float halfExtent = 0.5f * std::max(rp.robotLengthIn, rp.robotWidthIn);
    if (len > 1e-6f) {
        const float c = std::cos(t.theta), s = std::sin(t.theta);
        const float fx = (dx * c + dy * s) / len;      // obstacle in robot frame
        const float fy = (-dx * s + dy * c) / len;
        halfExtent = 0.5f * (rp.robotLengthIn * std::fabs(fx) +
                             rp.robotWidthIn * std::fabs(fy));
    }
    const float clear = near.dist - halfExtent;
    if (clear < st.minClearIn) st.minClearIn = clear;
    if (t.colliding) st.collided = true;
}

void execRoute() {
    gReport.clear();
    gReport.reserve(gRoute.size());

    // True until the first motion runs, which is what makes a setPose an
    // opening pose rather than a re-localization.
    bool startPhase = true;

    for (std::size_t i = 0; i < gRoute.size(); i++) {
        gRouteIndex = static_cast<int>(i);
        const RouteCmd& c = gRoute[i];
        const float timeout = (c.d > 0.0f) ? c.d : 0.0f;

        RouteStep step;
        step.op = c.op;
        step.startMs = static_cast<float>(sim::millis());
        step.timeoutMs = timeout;
        step.minClearIn = 1e9f;
        gReport.push_back(step);
        gRouteSampling = true;
        sampleRouteStep();      // one sample, so an instant command still reports

        const auto& t0 = sim::world().truth();
        const float startX = t0.x, startY = t0.y;

        // Where the command meant to end up, for the error column. Turns and
        // gain changes leave these unset and report -1.
        bool hasPoint = false;
        float targetX = 0.0f, targetY = 0.0f;
        bool hasHeading = false;
        float targetHeadingDeg = 0.0f;

        switch (c.op) {
            case OP_SET_POSE: {
                const chad::Pose p{c.a, c.b, chad::vexToStdRad(c.c)};
                // A later setPose corrects the estimate only: moving ground
                // truth would erase the error the sim exists to show.
                if (startPhase) {
                    breakTrails();
                    sim::world().resetTo(p);
                }
                chad::chassis.setPose(p);
                chad::chassis.mcl().setPose(p);
                break;
            }
            case OP_MOVE_TO: {
                chad::MoveToPointParams params = gMoveToPoint;
                params.forwards = (c.c >= 0.0f);
                params.maxSpeed = gRouteMaxSpeed;
                gReport.back().optsIgnored = applyRouteOpts(params, c.opts);
                hasPoint = true;
                targetX = c.a;
                targetY = c.b;
                chad::chassis.moveToPoint(c.a, c.b, timeout, params, c.async);
                break;
            }
            case OP_TURN_TO_HEADING: {
                chad::TurntoHeadingParams params = gTurn;
                params.maxSpeed = gRouteMaxSpeed;
                gReport.back().optsIgnored = applyRouteOpts(params, c.opts);
                hasHeading = true;
                targetHeadingDeg = c.a;
                chad::chassis.turnToHeading(c.a, timeout, params, c.async);
                break;
            }
            case OP_TURN_TO_POINT: {
                chad::TurnToPointParams params = gTurnPoint;
                params.maxSpeed = gRouteMaxSpeed;
                gReport.back().optsIgnored = applyRouteOpts(params, c.opts);
                chad::chassis.turnToPoint(c.a, c.b, timeout, params, c.async);
                // The heading it should have reached is only known once the
                // motion is over: it is measured from wherever the robot
                // actually ended up, not from where it started.
                hasHeading = true;
                targetHeadingDeg = chad::stdRadToVexDeg(
                    std::atan2(c.b - sim::world().truth().y,
                               c.a - sim::world().truth().x));
                break;
            }
            case OP_DRIVE: {
                chad::MoveDistanceParams params = gDrive;
                params.maxSpeed = gRouteMaxSpeed;
                gReport.back().optsIgnored = applyRouteOpts(params, c.opts);
                // Pass the signed distance straight through. Splitting it into
                // fabs() and a forwards flag here overrode any .forwards the
                // route asked for, and Chassis::moveDistance already flips the
                // flag itself for a negative target -- so the sim was applying
                // a rule the robot does not have.
                chad::chassis.moveDistance(c.a, timeout, params, c.async);
                break;
            }
            case OP_BOOMERANG: {
                chad::BoomerangParams params = gBoomerang;
                params.maxSpeed = gRouteMaxSpeed;
                gReport.back().optsIgnored = applyRouteOpts(params, c.opts);
                hasPoint = true;
                targetX = c.a;
                targetY = c.b;
                hasHeading = true;
                targetHeadingDeg = c.c;
                chad::chassis.moveBoomerang(c.a, c.b, c.c, timeout, params, c.async);
                break;
            }
            case OP_ARC: {
                chad::ArcParams params = gArc;
                params.maxSpeed = gRouteMaxSpeed;
                gReport.back().optsIgnored = applyRouteOpts(params, c.opts);
                hasPoint = true;
                targetX = c.a;
                targetY = c.b;
                chad::chassis.moveArc(c.a, c.b, c.c, timeout, params, c.async);
                break;
            }
            case OP_SWING: {
                chad::SwingParams params = gSwing;
                params.maxSpeed = gRouteMaxSpeed;
                gReport.back().optsIgnored = applyRouteOpts(params, c.opts);
                const auto side = (c.b >= 0.5f) ? chad::SwingSide::Right
                                                : chad::SwingSide::Left;
                hasHeading = true;
                targetHeadingDeg = c.a;
                chad::chassis.swingToHeading(c.a, side, timeout, params, c.async);
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
            // Open loop, and deliberately non-blocking: the routines that use
            // it set a voltage, delay, then set another. Chassis::tank() has
            // its own isSettled() guard, so a tank issued under a running
            // async motion is dropped here exactly as it is on the robot.
            case OP_TANK:
                chad::chassis.tank(c.a, c.b);
                break;
            case OP_CLAW: {
                const int how = static_cast<int>(c.a);
                if (how == 0) ::clawOpen();
                else if (how == 1) ::clawClose();
                else ::clawToggle();
                break;
            }
            case OP_FOLLOW_PATH: {
                const int start = static_cast<int>(c.a);
                const int count = static_cast<int>(c.b);
                if (start < 0 || count <= 0 ||
                    start + count > static_cast<int>(gPath.size())) break;
                std::vector<chad::Pose> path(gPath.begin() + start,
                                             gPath.begin() + start + count);
                chad::RAMSETEParams params = gRamsete;
                params.maxSpeed = std::min(gRouteMaxSpeed, gRamsete.maxSpeed);
                gReport.back().optsIgnored = applyRouteOpts(params, c.opts);
                hasPoint = true;
                targetX = path.back().x;
                targetY = path.back().y;
                chad::chassis.moveRAMSETE(path, timeout, params, c.async);
                break;
            }
            default:
                break;
        }

        if ((c.op >= OP_MOVE_TO && c.op <= OP_SWING) || c.op == OP_FOLLOW_PATH) {
            startPhase = false;
        }

        gRouteSampling = false;
        sampleRouteStep();

        RouteStep& st = gReport.back();
        st.durationMs = static_cast<float>(sim::millis()) - st.startMs;
        // Heuristic, and the only one available: nothing in the Chassis API says
        // why a motion returned. A blocking motion that used its whole timeout
        // was cut off rather than settled, within one motion-loop tick.
        st.ranOutTimeout = timeout > 0.0f && !c.async &&
                           st.durationMs >= timeout - kTimeoutSlackMs;

        const auto& t = sim::world().truth();
        const chad::Pose est = chad::chassis.getPose();
        st.estErrIn = std::hypot(t.x - est.x, t.y - est.y);
        if (hasPoint) {
            st.targetErrIn = std::hypot(t.x - targetX, t.y - targetY);
        } else if (c.op == OP_DRIVE) {
            // No commanded point to miss, so report the shortfall: straight-line
            // travel against the distance asked for.
            const float moved = std::hypot(t.x - startX, t.y - startY);
            st.targetErrIn = std::fabs(moved - std::fabs(c.a));
        }
        if (hasHeading) {
            st.headingErrDeg = std::fabs(chad::radToDeg(chad::wrapRad(
                chad::vexToStdRad(targetHeadingDeg) - t.theta)));
        }
        if (st.minClearIn > 1e8f) st.minClearIn = 0.0f;
    }
    gRouteIndex = -1;
    gRouteSampling = false;
}

}  // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE
void sim_route_clear() {
    gRoute.clear();
    gReport.clear();
    gRouteIndex = -1;
    gRouteMaxSpeed = 127.0f;
    gRouteSampling = false;
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

/// The curve moveRAMSETE will drive for a slice of uploaded control points,
/// as JSON, for the browser to draw. Headings come back as VEX bearings.
///
/// Runs the real chad::splinePath -- the same call moveRAMSETE makes on the
/// path it is handed -- so the drawn curve is not a browser-side lookalike
/// that could drift from what the robot does. gPath is left alone; this only
/// reads it.
EMSCRIPTEN_KEEPALIVE
const char* sim_path_preview_json(int start, int count, float spacingIn) {
    static std::string out;
    out = "[";
    if (start < 0 || count <= 0 ||
        start + count > static_cast<int>(gPath.size())) {
        out += "]";
        return out.c_str();
    }

    const std::vector<chad::Pose> controls(gPath.begin() + start,
                                          gPath.begin() + start + count);
    const auto curve = (spacingIn > 0.0f) ? chad::splinePath(controls, spacingIn)
                                          : controls;
    char buf[96];
    for (std::size_t i = 0; i < curve.size(); i++) {
        std::snprintf(buf, sizeof(buf), "%s{\"x\":%.2f,\"y\":%.2f,\"h\":%.1f}",
                      i ? "," : "", curve[i].x, curve[i].y,
                      chad::stdRadToVexDeg(curve[i].theta));
        out += buf;
    }
    out += "]";
    return out.c_str();
}

EMSCRIPTEN_KEEPALIVE
void sim_route_push(int op, float a, float b, float c, float d) {
    gRoute.push_back({op, a, b, c, d});
}

/// Set one `.field = value` on the command pushed most recently.
///
/// Booleans and enums come across as floats: 0/1 for a bool, the enumerator's
/// integer value for TurnDirection. The browser validates the name against the
/// params headers before calling, so an unknown name here means the sim cannot
/// apply that field, not that the route misspelled it -- either way it is
/// counted and shown in the report rather than dropped.
EMSCRIPTEN_KEEPALIVE
void sim_route_opt(const char* name, float value) {
    if (gRoute.empty() || name == nullptr) return;
    gRoute.back().opts.push_back({name, value});
}

/// Mark the most recent command async, matching a trailing `true` on the real
/// call. The route then runs on to the next statement while the motion is
/// still going, exactly as the compiled auton would.
EMSCRIPTEN_KEEPALIVE
void sim_route_async() {
    if (!gRoute.empty()) gRoute.back().async = true;
}

EMSCRIPTEN_KEEPALIVE
void sim_route_run() {
    if (gRoute.empty()) return;
    beginRun();
    gRouteMaxSpeed = 127.0f;
    gRouteTask = sim::sched().spawn([]() { execRoute(); }, "dsl-route");
}

/// Index of the command currently executing, or -1 when idle. Drives the
/// editor's active-line highlight.
EMSCRIPTEN_KEEPALIVE
int sim_route_index() { return gRouteIndex; }

/// Per-command outcome for the last run: how long each took, whether it was
/// cut off by its timeout, how far it ended from what it was aimed at, and the
/// closest the body came to a wall or post while it ran.
EMSCRIPTEN_KEEPALIVE
const char* sim_route_report_json() {
    static std::string out;
    out = "[";
    char buf[320];
    for (std::size_t i = 0; i < gReport.size(); i++) {
        const RouteStep& r = gReport[i];
        std::snprintf(buf, sizeof(buf),
                      "%s{\"i\":%d,\"op\":%d,\"t0\":%.0f,\"ms\":%.0f,"
                      "\"timeout\":%.0f,\"cutOff\":%s,\"targetErr\":%.2f,"
                      "\"headingErr\":%.2f,\"estErr\":%.2f,\"clear\":%.2f,"
                      "\"hit\":%s,\"ignored\":%d}",
                      i ? "," : "", static_cast<int>(i), r.op, r.startMs,
                      r.durationMs, r.timeoutMs, r.ranOutTimeout ? "true" : "false",
                      r.targetErrIn, r.headingErrDeg, r.estErrIn, r.minClearIn,
                      r.collided ? "true" : "false", r.optsIgnored);
        out += buf;
    }
    out += "]";
    return out.c_str();
}

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
    else if (k == "distSeePieces")     p.distSeePieces = (value != 0.0f);
    else if (k == "clawMountX")        p.clawMountX = value;
    else if (k == "clawMountY")        p.clawMountY = value;
    else if (k == "clawMountDeg")      p.clawMountDeg = value;
    else if (k == "liftDegPerSec")     p.liftDegPerSec = value;
    else if (k == "alignerWidthIn")    p.alignerWidthIn = value;
    else if (k == "alignerMouthIn")    p.alignerMouthIn = value;
    else if (k == "alignerDepthIn")    p.alignerDepthIn = value;
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
    } else if (k.rfind("rcl", 0) == 0) {
        auto c = chad::chassis.rcl().config();
        if      (k == "rclSyncProp")     c.syncProp = value;
        else if (k == "rclMaxSyncRate")  c.maxSyncRate = value;
        else if (k == "rclMaxSyncDist")  c.maxSyncDist = value;
        else if (k == "rclMaxInnov")     c.maxWallInnovation = value;
        else if (k == "rclAngleTol")     c.angleTolerance = value;
        else if (k == "rclConfMin")      c.confidenceThreshold = static_cast<int>(value);
        else if (k == "rclStaleHoldMs")  c.staleHoldMs = value;
        else if (k == "rclMaxTurnRate")  c.maxTurnRateRadS = value;
        else if (k == "rclLatencyMs")    c.sensorLatencyMs = value;
        else if (k == "rclAxisAgree")    c.axisAgreeIn = value;
        else if (k == "rclPairTol")      c.pairSpanTolIn = value;
        else if (k == "rclRecoverProp")  c.recoverySyncProp = value;
        else if (k == "rclDisplaceCyc")  c.displaceCycles = static_cast<int>(value);
        else if (k == "rclRecoverCyc")   c.recoverCycles = static_cast<int>(value);
        else if (k == "rclObsCone")      c.obstacleConeFrac = value;
        else if (k == "rclObsTol")       c.obstacleRangeTol = value;
        else if (k == "rclNoReturnLim")  c.noReturnWallLimit = value;
        chad::chassis.rcl().setConfig(c);
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
        else if (k == "mclInjectFloor")   c.injectResampleFloor = value;
        else if (k == "mclInjectRadius")  c.injectRadiusIn = value;
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
    } else if (k == "ramseteSmooth")      { gRamsete.smoothSpacingIn = value;
    } else if (k == "ramseteSettleMs")    {
        gRamsete.settleTimeMs = static_cast<std::uint32_t>(value);
    } else if (k == "mtpBearingHold")  { gMoveToPoint.bearingHoldDist = value;
    } else if (k == "mtpTolerance")    { gMoveToPoint.targetTolerance = value;
    } else if (k == "mtpLead")         { gMoveToPoint.lead = value;
    } else if (k == "boomLead")        { gBoomerang.lead = value;
    } else if (k == "boomLeadDecay")   { gBoomerang.leadDecay = value;
    } else if (k == "boomMinSpeed")    { gBoomerang.minSpeed = value;
    } else if (k == "boomTolerance")   { gBoomerang.targetTolerance = value;
    } else if (k == "boomHeadingTol")  { gBoomerang.headingTolerance = value;
    } else if (k == "arcTolerance")    { gArc.targetTolerance = value;
    } else if (k == "arcMinSpeed")     { gArc.minSpeed = value;
    } else if (k == "driveTolerance")  { gDrive.targetTolerance = value;
    } else if (k == "turnTolerance")   { gTurn.targetTolerance = value;
                                         gTurnPoint.targetTolerance = value;
                                         gSwing.targetTolerance = value;
    } else if (k == "targetTolIn" || k == "targetTolDeg" ||
               k == "bearingHold") {
        // ChassisConfig's fallbacks: every motion whose own tolerance is
        // negative reads these, which is most of what a route emits.
        if      (k == "targetTolIn")  chad::chassisConfig.targetToleranceIn = value;
        else if (k == "targetTolDeg") chad::chassisConfig.targetToleranceDeg = value;
        else                          chad::chassisConfig.bearingHoldDist = value;
        chad::chassis.setSettleTolerances(chad::chassisConfig.targetToleranceIn,
                                         chad::chassisConfig.targetToleranceDeg,
                                         chad::chassisConfig.bearingHoldDist);
    } else if (k == "lateralSlew") {
        chad::chassisConfig.lateralSlewPerSec = value;
        chad::chassis.setLateralSlew(value);
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

    // Geometry, readable so the browser can state what it is simulating instead
    // of carrying its own copy. World::init() takes these from chad::drivetrain,
    // so they are the robot's numbers, not RobotParams' pre-init placeholders --
    // which is exactly what the drivetrain panel used to print, and why it
    // claimed a 3.25 in wheel on a 2.6738 in drivetrain.
    else if (k == "trackWidthIn")      return p.trackWidthIn;
    else if (k == "wheelDiameterIn")   return p.wheelDiameterIn;
    else if (k == "maxWheelRpm")       return p.maxWheelRpm;
    else if (k == "robotLengthIn")     return p.robotLengthIn;
    else if (k == "robotWidthIn")      return p.robotWidthIn;
    else if (k == "mclParticleCount")
        return static_cast<float>(chad::chassis.mcl().config().particleCount);
    else if (k == "distSensorCount") {
        // Counted from the config rather than from the live beams, which are
        // not populated until the first reset or physics tick.
        int n = 0;
        for (int i = 0; i < chad::MAX_DISTANCE_SENSORS; i++) {
            if (chad::distSensors[i] != nullptr) n++;
        }
        return static_cast<float>(n);
    }

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

    else if (k == "lateralSlew")   return cc.lateralSlewPerSec;
    else if (k == "targetTolIn")   return cc.targetToleranceIn;
    else if (k == "targetTolDeg")  return cc.targetToleranceDeg;
    else if (k == "bearingHold")   return cc.bearingHoldDist;

    else if (k == "ramseteB")          return gRamsete.b;
    else if (k == "ramseteZeta")       return gRamsete.zeta;
    else if (k == "ramseteMinSpeed")   return gRamsete.minSpeed;
    else if (k == "ramseteMaxAccel")   return gRamsete.maxAccel;
    else if (k == "ramseteLatAccel")   return gRamsete.maxLateralAccel;
    else if (k == "ramseteMaxSpeed")   return gRamsete.maxSpeed;
    else if (k == "ramseteStall")      return gRamsete.stallSpeed;
    else if (k == "ramseteSmooth")     return gRamsete.smoothSpacingIn;
    else if (k == "ramseteSettleMs")
        return static_cast<float>(gRamsete.settleTimeMs);
    else if (k == "ramseteTolerance")  return gRamsete.targetTolerance;
    else if (k == "ramseteHeadingTol") return gRamsete.headingTolerance;
    else if (k == "ramseteVelCtrl")
        return gRamsete.useVelocityController ? 1.0f : 0.0f;
    else if (k == "ramseteFfKv")       return gRamsete.ffGains.kV;
    else if (k == "ramseteFfKs")       return gRamsete.ffGains.ksStraight;
    else if (k == "ramseteFfKp")       return gRamsete.ffGains.kP;

    else if (k == "mtpTolerance")   return gMoveToPoint.targetTolerance;
    else if (k == "mtpBearingHold") return gMoveToPoint.bearingHoldDist;
    else if (k == "mtpLead")        return gMoveToPoint.lead;
    else if (k == "boomLead")       return gBoomerang.lead;
    else if (k == "boomLeadDecay")  return gBoomerang.leadDecay;
    else if (k == "boomMinSpeed")   return gBoomerang.minSpeed;
    else if (k == "boomTolerance")  return gBoomerang.targetTolerance;
    else if (k == "boomHeadingTol") return gBoomerang.headingTolerance;
    else if (k == "arcTolerance")   return gArc.targetTolerance;
    else if (k == "arcMinSpeed")    return gArc.minSpeed;
    else if (k == "driveTolerance") return gDrive.targetTolerance;
    else if (k == "turnTolerance")  return gTurn.targetTolerance;

    const auto mc = chad::chassis.mcl().config();
    if      (k == "mclObsCone")  return mc.obstacleConeFrac;
    else if (k == "mclObsTol")   return mc.obstacleRangeTol;
    else if (k == "mclSyncProp") return mc.distSyncProp;

    if      (k == "distSeePieces") return sim::world().params().distSeePieces ? 1.0f : 0.0f;
    else if (k == "clawMountX")    return sim::world().params().clawMountX;
    else if (k == "clawMountY")    return sim::world().params().clawMountY;
    else if (k == "clawMountDeg")  return sim::world().params().clawMountDeg;
    else if (k == "liftDegPerSec") return sim::world().params().liftDegPerSec;
    else if (k == "alignerWidthIn") return sim::world().params().alignerWidthIn;
    else if (k == "alignerMouthIn") return sim::world().params().alignerMouthIn;
    else if (k == "alignerDepthIn") return sim::world().params().alignerDepthIn;

    const auto rc = chad::chassis.rcl().config();
    if      (k == "rclSyncProp")    return rc.syncProp;
    else if (k == "rclMaxSyncRate") return rc.maxSyncRate;
    else if (k == "rclMaxSyncDist") return rc.maxSyncDist;
    else if (k == "rclMaxInnov")    return rc.maxWallInnovation;
    else if (k == "rclAngleTol")    return rc.angleTolerance;
    else if (k == "rclConfMin")     return static_cast<float>(rc.confidenceThreshold);
    else if (k == "rclStaleHoldMs") return rc.staleHoldMs;
    else if (k == "rclMaxTurnRate") return rc.maxTurnRateRadS;
    else if (k == "rclLatencyMs")   return rc.sensorLatencyMs;
    else if (k == "rclAxisAgree")   return rc.axisAgreeIn;
    else if (k == "rclPairTol")     return rc.pairSpanTolIn;
    else if (k == "rclRecoverProp") return rc.recoverySyncProp;
    else if (k == "rclDisplaceCyc") return static_cast<float>(rc.displaceCycles);
    else if (k == "rclRecoverCyc")  return static_cast<float>(rc.recoverCycles);
    else if (k == "rclObsCone")     return rc.obstacleConeFrac;
    else if (k == "rclObsTol")      return rc.obstacleRangeTol;
    else if (k == "rclNoReturnLim") return rc.noReturnWallLimit;

    return std::nanf("");
}

// ============================================================================
// Simulator tuning
// ============================================================================
//
// What the sim runs with, applied over gamespecific/config.cpp at sim_init.
//
// These are NOT the robot's constants and are not meant to become them. The
// sim's drivetrain is a first-order lag with no mass, torque or traction, so a
// gain that is right here is optimistic there. What they are for is the one
// thing the sim is actually authoritative about: a route drawn in the browser
// and the route the sim drives being the same route. Every waypoint the editor
// can place is measured against ground truth by sim/test/accuracy.mjs, and
// sim/test/tune_routing.mjs sweeps outward from whatever is written here.
//
// Only entries that differ from the library default are listed, so this doubles
// as the record of what the sim disagrees with the robot about. Each key is one
// sim_set_param understands, which is also what makes the browser's panel show
// these values rather than the header's.
struct SimTuning { const char* key; float value; };

constexpr SimTuning kSimTuning[] = {
    // The along-track loop. Around the robot's gain the last inch takes 1.5 s
    // to close, so the settle debounce expires with the robot still half an
    // inch out; the nine point-to-point motions in the battery took 16.5 s at
    // the 9.012 it shipped when this was measured, against 12.5 s here. It
    // ships 9.5 now, which is the same story a little faster.
    //
    // 12 and not more, because moveDistance and moveToPoint have no velocity
    // profile -- they are P+D against a first-order lag, so the only thing
    // stopping them is the gain coming out of saturation in time. The plant
    // needs v*tau to stop, 4.4 in from full speed, and 127/kP is how far out
    // the output leaves saturation: 10.6 in at kP 12, 7.9 in at kP 16. Past
    // that the arithmetic stops working and the robot sails through the
    // waypoint and reverses.
    {"lateralKp", 12.0f},

    // kD has to be overridden WITH kP, not inherited. This entry pinned kP to
    // 12 and left kD alone, which was fine until the robot's measured gains
    // went from {9.012, 0, 1.15} to {9.5, 0, 0.4}: kP 12 against kD 0.4 is
    // under-damped in this plant, and the whole gate went red -- 0.89 in of
    // overshoot and one reversal on every long straight. Nothing caught it
    // because the sim did not compile for the several commits either side of
    // that change.
    //
    // Swept on the full battery at the default 70 ms lag: kD 0.4 fails 8 cases,
    // 0.6 passes but at worst=0.58, and 0.7 through 1.4 all pass with the cost
    // flat from 2.22 to 2.61. 0.8 is in the flat part with room above the 0.6
    // cliff. It holds from 50 ms to 85 ms of lag; at 100 ms the battery does
    // fail, but on turn overshoot and RAMSETE endpoints rather than anything
    // along-track, and identically at kD 1.15 -- so that ceiling belongs to the
    // angular gains and the follower, not here.
    {"lateralKd", 0.8f},

    // There is deliberately NO targetTolIn entry. There was one, at 0.30, and
    // its reason was that the robot's 1.0 in had to exceed a localization error
    // the sim's exact dead reckoning does not have. The robot ships 0.2 now, so
    // the override had inverted into loosening the tolerance rather than
    // tightening it. Swept on the battery: inheriting 0.2 scores cost 1.94
    // against 0.30's 2.25 and lands 0.194 in out against 0.209, for 1.0 s more
    // over the whole battery. 0.15 is better again (1.82) and 0.12 breaks --
    // a motion sits out its timeout -- so the floor is real, just below 0.2.
    // Inherited rather than re-pinned at 0.15, because a divergence has to earn
    // its keep and 0.005 in does not.

    // Boomerang's speed scale is a hard distToTarget/6, so inside 6 in the
    // along-track term is throttled quadratically and the approach crawls.
    // minSpeed is the only floor available on it, and it floors the SCALE
    // rather than the output -- so setting it to maxSpeed switches the throttle
    // off entirely and leaves the plain P+D loop, which at kP 12 stops without
    // overshooting on its own. The throttle was protecting against an
    // overshoot the gain no longer has: 0.22 in in 4.5 s with the floor at 60,
    // 0.18 in in 3.0 s without it, and zero overshoot either way.
    {"boomLead", 0.7f},
    {"boomLeadDecay", 0.15f},
    {"boomMinSpeed", 127.0f},

    // A gentler profile than the robot's 50. RAMSETE's endpoint error IS its
    // overshoot -- the profile decelerates to zero at the last point, but the
    // follower exits inside tolerance and coasts the rest -- so arriving
    // slower is the whole of the fix: 0.48 in past the last point at 50,
    // 0.18 in at 30, for 1.4 s over five paths.
    {"ramseteMaxAccel", 30.0f},
    // b, and this one is the aggression to distrust: cross-track falls
    // monotonically to b=144 with nothing anywhere in the sim getting worse,
    // which is what a plant with no mass looks like when you ask it about
    // gain. It is here anyway because the robot's 48 does not merely track
    // worse, it times a path out: 0.89 in of cross-track and 19.2 s over the
    // five, against 13.3 s. 72 is the least that finishes; 96 has margin and
    // is where the returns flatten.
    {"ramseteB", 96.0f},
};

// Swept and deliberately left alone, so the next person does not re-sweep
// them:
//
//   lateralSlew        0 is 0.4 s per motion faster than the robot's 450 and
//                      no more accurate. The slew is there so a tall robot
//                      does not tip on the stop, which is a cost this plant
//                      cannot show.
//   angularKp/Kd/DFilt 176.607 already turns to 0.14 deg. 210 reads 0.14 and
//                      250 reads 0.59, so there is a peak near 210 and no
//                      reason to chase it.
//   targetTolDeg       1.5 already settles at 0.14 deg.
//   bearingHold        flat from 0.5 to 2.0 here. Small values chase pose
//                      noise, which this default plant does not have.
//   ramseteZeta        1.0 tracks 0.04 in tighter and is outside the
//                      documented 0 < zeta < 1.
//   ramseteLatAccel    the robot's 60 is 0.7 s faster over five paths than 40
//                      and identical at the endpoint, for 0.08 in more
//                      cross-track. Not worth an override.
//   ramseteSettleMs    must stay 0. 60 ms times out 2 of 5 paths: position
//                      and heading have to hold together, and past the last
//                      point the profile has no authority left to fix
//                      heading with.
//   ramseteStall       must stay 1.5. At 0 the same 2 paths time out.
//   ramseteVelCtrl     off. Closing the loop scores better at kP 2 (0.31
//                      against 0.82) and at kP 6 and 12 the robot stops dead
//                      0.26 and 0.36 in short of the last point and sits
//                      there until the timeout -- the loop cannot deliver the
//                      creep that closes the last fraction of an inch. A
//                      default that good only 3x below where it fails hard is
//                      not worth the 0.5 of cost.

// What each key held before the table was applied, captured once so the effect
// of the tuning is measurable rather than asserted. Without it,
// `accuracy.mjs --stock` would need its own copy of the library defaults, and a
// second copy of a default is how the documentation ends up lying.
float gStock[std::size(kSimTuning)];
bool gStockCaptured = false;

EMSCRIPTEN_KEEPALIVE
void sim_tuning_apply() {
    if (!gStockCaptured) {
        for (std::size_t i = 0; i < std::size(kSimTuning); i++) {
            gStock[i] = sim_get_param(kSimTuning[i].key);
        }
        gStockCaptured = true;
    }
    for (const auto& t : kSimTuning) sim_set_param(t.key, t.value);
}

/// Put back what gamespecific/config.cpp and the headers actually say. This is
/// the robot's tuning, running against the sim's plant -- useful for seeing
/// what the overrides buy, not for deciding anything about the robot.
EMSCRIPTEN_KEEPALIVE
void sim_tuning_stock() {
    if (!gStockCaptured) return;
    for (std::size_t i = 0; i < std::size(kSimTuning); i++) {
        sim_set_param(kSimTuning[i].key, gStock[i]);
    }
}

/// The table as JSON, tuned value against the value it replaced, so the sweeps
/// and the browser can list what the sim overrode without a second copy of it
/// in JavaScript.
EMSCRIPTEN_KEEPALIVE
const char* sim_tuning_json() {
    static std::string out;
    out = "[";
    char buf[128];
    for (std::size_t i = 0; i < std::size(kSimTuning); i++) {
        std::snprintf(buf, sizeof(buf),
                      "%s{\"key\":\"%s\",\"sim\":%.4f,\"robot\":%.4f}",
                      i ? "," : "", kSimTuning[i].key, kSimTuning[i].value,
                      gStockCaptured ? gStock[i] : 0.0f);
        out += buf;
    }
    out += "]";
    return out.c_str();
}

// ============================================================================
// Other robots on the field
// ============================================================================

EMSCRIPTEN_KEEPALIVE
int sim_actor_add(float x, float y, float headingDeg, int team) {
    return sim::actors().add(x, y, chad::vexToStdRad(headingDeg), team);
}

EMSCRIPTEN_KEEPALIVE
void sim_actor_remove(int id) {
    sim::actors().remove(id);
    sim::actors().publish();
}

EMSCRIPTEN_KEEPALIVE
void sim_actor_clear() {
    sim::actors().clear();
    sim::field().clearDynamic();
}

EMSCRIPTEN_KEEPALIVE
void sim_actor_set_pose(int id, float x, float y, float headingDeg) {
    if (auto* a = sim::actors().get(id)) {
        a->x = x;
        a->y = y;
        a->theta = chad::vexToStdRad(headingDeg);
        sim::actors().publish();
    }
}

EMSCRIPTEN_KEEPALIVE
void sim_actor_set_size(int id, float lengthIn, float widthIn) {
    if (auto* a = sim::actors().get(id)) {
        a->lengthIn = lengthIn;
        a->widthIn = widthIn;
        sim::actors().publish();
    }
}

EMSCRIPTEN_KEEPALIVE
void sim_actor_set_flags(int id, int enabled, int patrol, int loop, int faceTravel) {
    if (auto* a = sim::actors().get(id)) {
        a->enabled = (enabled != 0);
        a->patrol = (patrol != 0);
        a->loop = (loop != 0);
        a->faceTravel = (faceTravel != 0);
        sim::actors().publish();
    }
}

EMSCRIPTEN_KEEPALIVE
void sim_actor_set_speed(int id, float speedIn) {
    if (auto* a = sim::actors().get(id)) a->speedIn = speedIn;
}

/// Replace the whole patrol path. Points are pushed one at a time because
/// marshalling a float array across the boundary for four points is not worth
/// the heap juggling on either side.
EMSCRIPTEN_KEEPALIVE
void sim_actor_path_clear(int id) {
    if (auto* a = sim::actors().get(id)) {
        a->path.clear();
        a->leg = 0;
        a->legT = 0.0f;
    }
}

EMSCRIPTEN_KEEPALIVE
void sim_actor_path_push(int id, float x, float y) {
    if (auto* a = sim::actors().get(id)) {
        a->path.push_back(x);
        a->path.push_back(y);
    }
}

EMSCRIPTEN_KEEPALIVE
const char* sim_actors_json() {
    std::string out = "[";
    bool first = true;
    for (const auto& a : sim::actors().all()) {
        char b[256];
        std::snprintf(b, sizeof(b),
            "%s{\"id\":%d,\"x\":%.2f,\"y\":%.2f,\"h\":%.2f,\"team\":%d,"
            "\"len\":%.2f,\"wid\":%.2f,\"on\":%s,\"patrol\":%s,\"loop\":%s,"
            "\"face\":%s,\"speed\":%.1f,\"path\":[",
            first ? "" : ",", a.id, a.x, a.y, chad::stdRadToVexDeg(a.theta),
            a.team, a.lengthIn, a.widthIn,
            a.enabled ? "true" : "false", a.patrol ? "true" : "false",
            a.loop ? "true" : "false", a.faceTravel ? "true" : "false",
            a.speedIn);
        out += b;
        for (std::size_t i = 0; i < a.path.size(); i++) {
            char c[32];
            std::snprintf(c, sizeof(c), "%s%.2f", i ? "," : "", a.path[i]);
            out += c;
        }
        out += "]}";
        first = false;
    }
    out += "]";
    gStateJson = out;
    return gStateJson.c_str();
}

// ============================================================================
// Game pieces
// ============================================================================

EMSCRIPTEN_KEEPALIVE
int sim_piece_add(int kind, float x, float y, float headingDeg) {
    const auto k = (kind == 1) ? sim::PieceKind::Cup : sim::PieceKind::Pin;
    const int id = sim::pieces().add(k, x, y, chad::vexToStdRad(headingDeg));
    sim::pieces().publish();
    return id;
}

EMSCRIPTEN_KEEPALIVE
void sim_piece_remove(int id) {
    sim::pieces().remove(id);
    sim::pieces().publish();
}

/// 1 when Rapier is driving the pieces, 0 when they are on the built-in
/// kinematic model. A silent fallback would look like a solver that simply
/// does nothing, which is the hardest kind of broken to notice.
EMSCRIPTEN_KEEPALIVE
int sim_pieces_external() {
    return sim::pieces().externalPhysics() ? 1 : 0;
}

EMSCRIPTEN_KEEPALIVE
void sim_piece_clear() {
    sim::pieces().clear();
    sim::pieces().publish();
    gPiecesLoaded = false;
    gPreloadId = -1;
}

EMSCRIPTEN_KEEPALIVE
void sim_piece_default() {
    sim::pieces().loadDefault();
    gPiecesLoaded = true;
    gPreloadId = -1;
}

/// Drag one around. A piece in the jaws is released first -- moving it by hand
/// while the claw still claims it would have the next tick snap it back.
EMSCRIPTEN_KEEPALIVE
void sim_piece_set_pose(int id, float x, float y, float headingDeg) {
    if (sim::pieces().heldId() == id) sim::pieces().release();
    if (auto* p = sim::pieces().get(id)) {
        p->x = x;
        p->y = y;
        p->theta = chad::vexToStdRad(headingDeg);
        p->goal = -1;
        p->inCup = -1;
        p->held = false;
        // Rapier owns where a piece is; a pose written behind its back is one
        // it undoes on the next step.
        sim::pieces().markDirty();
        sim::pieces().publish();
    }
}

/// Work the claw by hand: 0 opens, 1 closes, anything else toggles. Polarity
/// is gamespecific/claw.cpp's (it grips retracted). Without this the only
/// way to pick anything up was to run a whole routine, which is no way to
/// check whether a grab works.
EMSCRIPTEN_KEEPALIVE
void sim_set_claw(int how) {
    if (how == 0) ::clawOpen();
    else if (how == 1) ::clawClose();
    else ::clawToggle();
}

/// Colour, and which way up. Separate from the pose so a scenario can lay a
/// pin on its side without also having to say where it is -- a lying pin is
/// 6.5 in of obstacle and an upright one 1.6, which is the whole reason to be
/// able to set it.
EMSCRIPTEN_KEEPALIVE
void sim_piece_set_flags(int id, int color, int upright, int clearUp) {
    if (auto* p = sim::pieces().get(id)) {
        if (color >= 0 && color <= 3) p->color = static_cast<sim::PinColor>(color);
        p->upright = upright != 0;
        p->clearUp = clearUp != 0;
        sim::pieces().markDirty();
        sim::pieces().publish();
    }
}

EMSCRIPTEN_KEEPALIVE
const char* sim_pieces_json() {
    std::string out = "{\"clawX\":";
    char head[192];
    float cx = 0.0f, cy = 0.0f, cdir = 0.0f;
    const auto& tr = sim::world().truth();
    sim::pieces().clawPoint({tr.x, tr.y, tr.theta}, cx, cy, cdir);
    const auto& cb = sim::world().clawBeam();
    std::snprintf(head, sizeof(head),
                  "%.2f,\"clawY\":%.2f,\"clawDir\":%.2f,\"clawRange\":%.2f,"
                  "\"clawHit\":%s,\"closed\":%s,\"held\":%d,"
                  "\"liftDeg\":%.1f,\"liftPin\":%d,\"pieces\":[",
                  cx, cy, cdir * chad::RAD_TO_DEG,
                  cb.inRange ? cb.rangeIn : -1.0f,
                  cb.inRange ? "true" : "false",
                  ::clawClosed() ? "true" : "false",
                  sim::pieces().heldId(),
                  liftPosition(), liftNearestPin(liftPosition()));
    out += head;

    bool first = true;
    for (const auto& p : sim::pieces().all()) {
        char b[256];
        std::snprintf(b, sizeof(b),
            "%s{\"id\":%d,\"kind\":%d,\"x\":%.2f,\"y\":%.2f,\"h\":%.1f,"
            "\"r\":%.2f,\"held\":%s,\"goal\":%d,\"level\":%d,"
            "\"color\":%d,\"up\":%s,\"clear\":%s,\"cup\":%d}",
            first ? "" : ",", p.id, static_cast<int>(p.kind), p.x, p.y,
            chad::stdRadToVexDeg(p.theta), p.radiusIn,
            p.held ? "true" : "false", p.goal, p.level,
            static_cast<int>(p.color), p.upright ? "true" : "false",
            p.clearUp ? "true" : "false", p.inCup);
        out += b;
        first = false;
    }
    out += "]}";
    gStateJson = out;
    return gStateJson.c_str();
}

// ============================================================================
// Toggles
// ============================================================================

EMSCRIPTEN_KEEPALIVE
const char* sim_toggles_json() {
    std::string out = "[";
    bool first = true;
    for (const auto& t : sim::toggles().all()) {
        char b[160];
        std::snprintf(b, sizeof(b),
            "%s{\"id\":%d,\"wall\":%d,\"x\":%.2f,\"y\":%.2f,\"len\":%.2f,"
            "\"state\":%d,\"on\":%s}",
            first ? "" : ",", t.id, t.wall, t.x, t.y, t.lengthIn,
            static_cast<int>(t.state), t.contact ? "true" : "false");
        out += b;
        first = false;
    }
    out += "]";
    gStateJson = out;
    return gStateJson.c_str();
}

/// Set one by hand: 0 neutral, 1 red, 2 blue. Ramming is the real way in, but
/// a scenario that starts with a quadrant already owned should not have to be
/// driven into existence.
EMSCRIPTEN_KEEPALIVE
void sim_toggle_set(int id, int state) {
    const int s = ((state % 3) + 3) % 3;
    sim::toggles().setState(id, static_cast<sim::ToggleState>(s));
}

/// Turn one on by a face, the way driving into it would. Which colour comes
/// next depends on the wall, so the browser must not do the arithmetic itself.
/// Start every reset with a Preload in the jaws, or stop doing so.
EMSCRIPTEN_KEEPALIVE
void sim_set_preload(int on) {
    gPreload = (on != 0);
}

EMSCRIPTEN_KEEPALIVE
int sim_preload(void) {
    return gPreload ? 1 : 0;
}

EMSCRIPTEN_KEEPALIVE
void sim_toggle_cycle(int id) {
    sim::toggles().cycle(id);
}

EMSCRIPTEN_KEEPALIVE
void sim_toggle_reset() {
    sim::toggles().resetStates();
}

// ============================================================================
// Match period
// ============================================================================

/// 0 = no clock, 1 = 15 s autonomous, 2 = 60 s skills.
EMSCRIPTEN_KEEPALIVE
void sim_set_period_mode(int mode) {
    gPeriodMode = (mode < 0 || mode > 2) ? 0 : mode;
    gPeriodExpired = false;
    gPeriodStartMs = (gPeriodMode != 0) ? sim::preciseMillis() : -1.0;
}

EMSCRIPTEN_KEEPALIVE
int sim_period_mode() { return gPeriodMode; }

/// Milliseconds left, or -1 when no clock is running.
EMSCRIPTEN_KEEPALIVE
float sim_period_remaining_ms() {
    if (gPeriodMode == 0 || gPeriodStartMs < 0.0) return -1.0f;
    const float left = periodLengthMs() -
                       static_cast<float>(sim::preciseMillis() - gPeriodStartMs);
    return left > 0.0f ? left : 0.0f;
}

// ============================================================================
// Auton read gate ("autonomous disable line")
// ============================================================================

/// Arm or disarm the far-side gate by hand. Lifetime 0 means until cleared.
EMSCRIPTEN_KEEPALIVE
void sim_set_far_side_gate(int on, float lifetimeMs) {
    if (on) armFarSideGate(lifetimeMs);
    else    chad::chassis.rcl().clearFarSideGate();
}

/// Re-arm it automatically whenever a run starts, which is what the routines
/// do for themselves via registerAutonDisableLine().
EMSCRIPTEN_KEEPALIVE
void sim_set_auto_far_side(int on) { gAutoFarSide = (on != 0); }

EMSCRIPTEN_KEEPALIVE
int sim_auto_far_side() { return gAutoFarSide ? 1 : 0; }

// ============================================================================
// Field editing
// ============================================================================

/// Choose what odometry reads. 0 = the robot's own config, 1 = the sim's
/// centred dead wheels, 2 = motor encoders and IMU only.
EMSCRIPTEN_KEEPALIVE
void sim_set_odom_pods(int mode) {
    const int want = (mode < 0 || mode > 2) ? 0 : mode;
    if (want == gOdomPods) return;
    gOdomPods = want;
    applyOdomPods();
}

EMSCRIPTEN_KEEPALIVE
int sim_odom_pods() { return gOdomPods; }

EMSCRIPTEN_KEEPALIVE
void sim_field_clear() { sim::field().clear(); }

EMSCRIPTEN_KEEPALIVE
void sim_field_default() { sim::field().loadDefault(); }

/// `kind` is carried rather than defaulted so a round trip through the editor
/// preserves it: 0 marks the perimeter, and the physics needs to know a wall is
/// a wall. Re-adding the perimeter as kind 2 used to leave a field the
/// raycaster and the collision test disagreed about.
EMSCRIPTEN_KEEPALIVE
void sim_field_add_segment(float x1, float y1, float x2, float y2, int kind) {
    sim::field().addSegment(x1, y1, x2, y2, kind);
}

EMSCRIPTEN_KEEPALIVE
void sim_field_add_circle(float x, float y, float r, int kind) {
    sim::field().addCircle(x, y, r, kind);
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
    const auto& rclT = chad::chassis.rcl();
    const chad::Pose rcl = rclT.latestEstimate();
    const chad::Pose ekf = chad::chassis.ekf().getEstimate();
    const auto ekfSigma = chad::chassis.ekf().uncertainty();
    const float mclInject = chad::chassis.mcl().randomInjectFraction();

    char buf[16384];
    int n = std::snprintf(buf, sizeof(buf),
        "{\"t\":%.3f,"
        "\"true\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f},"
        "\"odom\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f},"
        "\"mcl\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f,\"nEff\":%.1f},"
        "\"rcl\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f},"
        "\"ekf\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.2f,\"sx\":%.3f,\"sy\":%.3f,"
            "\"st\":%.3f,\"beams\":%d,\"reloc\":%d},"
        "\"mclInject\":%.3f,"
        "\"rcl2\":{\"displaced\":%s,\"spread\":%.2f,\"sync\":%.3f,"
            "\"rx\":%.3f,\"ry\":%.3f,\"turn\":%.2f,\"held\":%s,\"votes\":%d,"
            "\"gate\":%s,\"gnx\":%.3f,\"gny\":%.3f,\"gd\":%.2f,\"gms\":%.0f},"
        "\"period\":{\"mode\":%d,\"left\":%.0f},"
        "\"err\":{\"dist\":%.3f,\"heading\":%.2f},"
        "\"vel\":{\"lin\":%.2f,\"ang\":%.2f,\"left\":%.2f,\"right\":%.2f},"
        "\"colliding\":%s,\"routeRunning\":%s,\"settled\":%s,"
        "\"claw\":%d,"
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
        rclT.isDisplaced() ? "true" : "false", rclT.spreadIn(),
        rclT.lastSyncStepIn(), rclT.residualX(), rclT.residualY(),
        rclT.turnRateRadS(), rclT.turnGateHeld() ? "true" : "false",
        rclT.activeSensorCount(),
        rclT.farSideGateActive() ? "true" : "false",
        rclT.config().farSideNx, rclT.config().farSideNy,
        rclT.config().farSideD, rclT.farSideGateRemainingMs(),
        gPeriodMode, sim_period_remaining_ms(),
        errDist, errHeadingDeg,
        t.vLinIn, t.vAngRad * chad::RAD_TO_DEG, t.leftVelIn, t.rightVelIn,
        t.colliding ? "true" : "false",
        sim_route_running() ? "true" : "false",
        chad::chassis.isSettled() ? "true" : "false",
        ::clawClosed() ? 1 : 0,
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
            "\"rx\":%.2f,\"ry\":%.2f,\"rdx\":%.3f,\"rdy\":%.3f,"
            "\"cls\":%d,\"fresh\":%s,\"pair\":%s,\"out\":%s,\"innov\":%.2f,"
            "\"trust\":%.3f}",
            first ? "" : ",", i,
            mclValid[i] ? "true" : "false", mclRead[i],
            rs.valid ? "true" : "false", rs.readingInch, rs.confidence,
            mclDead[i],
            rs.worldX, rs.worldY, rs.rayCos, rs.raySin,
            static_cast<int>(rs.cls),
            rs.fresh ? "true" : "false",
            rs.pairVerified ? "true" : "false",
            rs.outlier ? "true" : "false",
            rs.innovationIn, rs.trust);
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
