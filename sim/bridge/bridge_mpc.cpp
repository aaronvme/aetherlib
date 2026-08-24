// chad-sim — the MPC / RTI study surface.
//
// Split out of bridge.cpp because it is a self-contained experiment rather than
// part of the simulator proper, and because it is the one area under active
// change. Shared simulator state comes in through bridge_internal.hpp; this
// file defines nothing the rest of the bridge needs.
#include <emscripten/emscripten.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "../platform/sim_runtime.hpp"
#include "../platform/sim_sched.hpp"
#include "../world/world.hpp"

#include "chad/movement_control/mpc_follower.hpp"

#include "gamespecific/config.hpp"

#include "bridge_internal.hpp"

using namespace simbridge;

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
//   path run provides. It is also where a constraint can be judged exactly:
//   with no model mismatch, this tick's predicted stage 1 IS next tick's state.
//
// Wall time comes from emscripten_get_now(), NOT the virtual clock: the point is
// how long the arithmetic takes on real silicon.

namespace {

double microsNow() { return emscripten_get_now() * 1000.0; }

chad::MpcPathFollower* gMpc = nullptr;
std::vector<chad::TrajectorySample> gMpcTraj;
std::string gMpcJson;

/// One solver instance behind every study entry point. Each one re-runs setup()
/// and clearConstraints(), so a corridor left over from a previous call cannot
/// silently change the problem the next one solves.
chad::CondensedMpc gStudy;

// Per-tick solver diagnostics, as a flat float32 ring the harness reads straight
// out of HEAPF32. Aggregates cannot separate a run that left a corridor once
// from one that leaned on it the whole way, and a JSON array of a few thousand
// ticks is not worth parsing.
constexpr int kTraceStride = 15;
constexpr int kTraceMax = 4096;
float gTrace[kTraceMax * kTraceStride];
int gTraceCount = 0;

/// Study knobs that are not part of the follower's own config: initial error,
/// the reference slew that makes a command-rate limit differ from a
/// correction-rate limit, and whether the periodic kick is applied.
struct BenchOpts {
    float ex0{0.02f};
    float ey0{0.06f};
    float theta0{0.12f};
    /// |d omega_ref| per tick, rad/s. Zero leaves the reference constant, in
    /// which case a rate limit on the command and one on the correction are the
    /// same constraint and nothing distinguishes them.
    float wSlew{0.0f};
    /// Ticks per leg of the reference triangle wave; the amplitude is
    /// wSlew * wLeg, so every consecutive pair differs by exactly wSlew.
    int wLeg{40};
    bool disturb{true};
};
BenchOpts gBench;

chad::MpcPathFollower::Config defaultMpcConfig() {
    chad::MpcPathFollower::Config c;
    c.mode = chad::MpcPathFollower::Mode::MpcRti;
    c.horizon = 10;
    // maxIters and rtiIters are deliberately NOT set: the sim exercises the
    // budget the library ships, and a second copy here is what drifts.
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

float halfTrackM() { return 0.5f * chad::drivetrain.trackWidth * chad::INCH_TO_METER; }

/// Advance the plant one tick: x+ = Ad x + Bd u.
void plantStep(const chad::DrivetrainPlant& p, float x[3], const float u[2]) {
    float nx[3];
    for (int i = 0; i < 3; i++) {
        nx[i] = p.Ad[i][0] * x[0] + p.Ad[i][1] * x[1] + p.Ad[i][2] * x[2] +
                p.Bd[i][0] * u[0] + p.Bd[i][1] * u[1];
    }
    x[0] = nx[0]; x[1] = nx[1]; x[2] = nx[2];
}

/// Roll the solved input sequence through the plant and return the worst
/// |row . x_k| over stages 1..N for the two corridor rows. Deliberately NOT the
/// solver's own condensing: re-deriving a'Phi_k x0 + a'G_k U would only prove
/// the formula was typed twice.
void planExtremes(const chad::DrivetrainPlant& p, const float x0[3], const float* U,
                  int horizon, float& worstCross, float& worstHeading) {
    float x[3] = {x0[0], x0[1], x0[2]};
    worstCross = 0.0f;
    worstHeading = 0.0f;
    for (int k = 0; k < horizon; k++) {
        const float u[2] = {U[2 * k + 0], U[2 * k + 1]};
        plantStep(p, x, u);
        const float cy = std::fabs(x[1]), ct = std::fabs(x[2]);
        if (cy > worstCross) worstCross = cy;
        if (ct > worstHeading) worstHeading = ct;
    }
}

/// The follower's own inscribed box, at a reference twist. Duplicated rather
/// than shared because MpcPathFollower::inputBox is private; it is four lines and
/// sim/test/mpc.mjs pins the two against each other through a follower run.
void studyBox(float vRef, float wRef, bool constrained, float uMin[2], float uMax[2]) {
    if (!constrained) {
        uMin[0] = uMin[1] = -1e6f;
        uMax[0] = uMax[1] = 1e6f;
        return;
    }
    const float limit = gMpcCfg.wheelSpeedLimit;
    const float ht = halfTrackM();
    const float vNeed = std::fabs(vRef), wNeed = std::fabs(wRef) * ht;
    const float need = vNeed + wNeed;
    float vBudget, wSpeed;
    if (need >= limit) {
        const float kk = (need > 1e-6f) ? limit / need : 0.0f;
        vBudget = vNeed * kk;
        wSpeed = wNeed * kk;
    } else {
        const float slack = limit - need;
        const float a = chad::clamp(gMpcCfg.slackToLinear, 0.0f, 1.0f);
        vBudget = vNeed + a * slack;
        wSpeed = wNeed + (1.0f - a) * slack;
    }
    const float wBudget = (ht > 1e-6f) ? wSpeed / ht : 0.0f;
    uMin[0] = -vBudget - vRef; uMax[0] = vBudget - vRef;
    uMin[1] = -wBudget - wRef; uMax[1] = wBudget - wRef;
}

/// Install the configured general constraints on gStudy. Must run after setup(),
/// because addStateConstraint budgets its rows against the current horizon, and
/// the rate limit has to be in place first so that budget counts it.
void studyConstraints(bool& rejected) {
    rejected = false;
    gStudy.clearConstraints();
    if (gMpcCfg.inputRateLimit[0] > 0.0f || gMpcCfg.inputRateLimit[1] > 0.0f) {
        gStudy.setInputRateLimit(gMpcCfg.inputRateLimit);
    }
    if (gMpcCfg.crossTrackCorridor > 0.0f) {
        const float a[3] = {0.0f, 1.0f, 0.0f};
        if (!gStudy.addStateConstraint(a, -gMpcCfg.crossTrackCorridor,
                                      gMpcCfg.crossTrackCorridor)) {
            rejected = true;
        }
    }
    if (gMpcCfg.headingCorridor > 0.0f) {
        const float a[3] = {0.0f, 0.0f, 1.0f};
        if (!gStudy.addStateConstraint(a, -gMpcCfg.headingCorridor,
                                      gMpcCfg.headingCorridor)) {
            rejected = true;
        }
    }
    gStudy.admm.rho = gMpcCfg.admmRho;
    gStudy.admm.rhoScale = gMpcCfg.admmRhoScale;
    gStudy.admm.checkEvery = gMpcCfg.admmCheckEvery;
}

}  // namespace

extern "C" {

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
void sim_mpc_defaults() {
    gMpcCfg = defaultMpcConfig();
    gBench = BenchOpts{};
}

/// Returns 0 for an unrecognised key. A silently ignored knob is a test that
/// measures the default configuration while claiming to measure a constrained
/// one, so the harness asserts on this.
EMSCRIPTEN_KEEPALIVE
int sim_mpc_set(const char* name, float value) {
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
    else if (k == "varying")    gMpcCfg.timeVaryingHorizon = (value != 0.0f);
    else if (k == "kV")         gMpcCfg.gains.kV = value;
    else if (k == "kS")         gMpcCfg.gains.ksStraight = value;
    else if (k == "kP")         gMpcCfg.gains.kP = value;
    else if (k == "timing")     gMpcCfg.microsClock = (value != 0.0f) ? &microsNow : nullptr;
    // General constraints.
    else if (k == "rateV")      gMpcCfg.inputRateLimit[0] = value;
    else if (k == "rateW")      gMpcCfg.inputRateLimit[1] = value;
    else if (k == "rateTotal")  gMpcCfg.rateLimitTotalCommand = (value != 0.0f);
    else if (k == "corridor")   gMpcCfg.crossTrackCorridor = value;
    else if (k == "headCorr")   gMpcCfg.headingCorridor = value;
    else if (k == "admmRho")    gMpcCfg.admmRho = value;
    else if (k == "admmScale")  gMpcCfg.admmRhoScale = value;
    else if (k == "admmCheck")  gMpcCfg.admmCheckEvery = static_cast<int>(value);
    else if (k == "shift")      gMpcCfg.shiftWarmStart = (value != 0.0f);
    else if (k == "certify")    gMpcCfg.certifyOptimality = (value != 0.0f);
    // Bench-only knobs.
    else if (k == "bEx0")       gBench.ex0 = value;
    else if (k == "bEy0")       gBench.ey0 = value;
    else if (k == "bTheta0")    gBench.theta0 = value;
    else if (k == "bSlew")      gBench.wSlew = value;
    else if (k == "bLeg")       gBench.wLeg = static_cast<int>(value);
    else if (k == "bDisturb")   gBench.disturb = (value != 0.0f);
    else return 0;
    return 1;
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

    char buf[2000];
    std::snprintf(buf, sizeof(buf),
        "{\"ticks\":%d,\"iters\":%d,\"maxIters\":%d,\"meanIters\":%.3f,"
        "\"rebuilds\":%d,\"convergedTicks\":%d,"
        "\"solveUsTotal\":%.1f,\"meanSolveUs\":%.4f,\"worstSolveUs\":%.4f,"
        "\"worstTickUs\":%.4f,\"setupUs\":%.2f,"
        "\"meanCrossM\":%.6f,\"peakCrossM\":%.6f,\"meanHeadingRad\":%.6f,"
        "\"endErrorM\":%.6f,\"endHeadingRad\":%.6f,"
        "\"satTicks\":%d,\"activeBoundTicks\":%d,"
        "\"peakWheelDemand\":%.4f,\"peakClipRatio\":%.4f,"
        "\"admmTicks\":%d,\"constraintRows\":%d,\"factorizationFailures\":%d,"
        "\"relaxedRowTicks\":%d,\"infeasibleTicks\":%d,\"droppedRowTicks\":%d,"
        "\"constraintsRejected\":%s,"
        "\"worstPrimalResidual\":%.6e,\"worstDualResidual\":%.6e,"
        "\"collideTicks\":%d,\"running\":%s}",
        s.ticks, s.solverIterations, s.maxIterations,
        static_cast<double>(s.solverIterations) / ticks,
        s.rebuilds, s.convergedTicks,
        s.solveMicros, s.solveMicros / ticks, s.worstSolveMicros,
        s.worstTickMicros, s.setupMicros,
        s.sumAbsCross / ticks, s.peakAbsCross, s.sumAbsHeading / ticks,
        s.endError, s.endHeading,
        s.satTicks, s.activeBoundTicks, s.peakWheelDemand, s.peakClipRatio,
        s.admmTicks, s.constraintRows, s.factorizationFailures,
        s.relaxedRowTicks, s.infeasibleTicks, s.droppedRowTicks,
        s.constraintsRejected ? "true" : "false",
        s.worstPrimalResidual, s.worstDualResidual,
        gCollideTicks, sim_mpc_running() ? "true" : "false");
    gMpcJson.assign(buf);
    return gMpcJson.c_str();
}

// ----------------------------------------------------------------------------
// Per-tick trace
// ----------------------------------------------------------------------------

EMSCRIPTEN_KEEPALIVE int sim_mpc_trace_ptr() {
    return static_cast<int>(reinterpret_cast<std::uintptr_t>(gTrace));
}
EMSCRIPTEN_KEEPALIVE int sim_mpc_trace_count() { return gTraceCount; }
EMSCRIPTEN_KEEPALIVE int sim_mpc_trace_stride() { return kTraceStride; }
EMSCRIPTEN_KEEPALIVE int sim_mpc_trace_capacity() { return kTraceMax; }

// ----------------------------------------------------------------------------
// Row budget
// ----------------------------------------------------------------------------
//
// addStateConstraint budgets ROWS, not spec count, and its contract is that
// acceptance means installation. This exercises the edge of that budget: what
// was accepted, what the accepted set plans, and what actually reached C after a
// solve. A spec accepted whose rows then vanish shows up as constraintRows below
// plannedRows with droppedRows > 0.

/// @param horizon   MPC horizon
/// @param rateAxes  0, 1 or 2 axes carrying an input-rate limit
/// @param specs     state constraints to attempt
/// @param span      stages each spec covers, 0 = the whole horizon
EMSCRIPTEN_KEEPALIVE
const char* sim_mpc_rows_json(int horizon, int rateAxes, int specs, int span) {
    const chad::MpcWeights w = gMpcCfg.weights;
    const float dt = gMpcCfg.dt;
    const float vRef = 0.75f * gMpcCfg.wheelSpeedLimit;
    const chad::DrivetrainPlant plant = chad::DrivetrainPlant::discretize(vRef, dt);
    const chad::LqrSolution lqr = chad::solveDiscreteLqr(plant, w);

    gStudy.solver = chad::MpcSolver::FastGradient;
    gStudy.clearConstraints();
    gStudy.setup(plant, w, horizon, lqr.P);

    const float rate[2] = {(rateAxes >= 1) ? 0.05f : 0.0f, (rateAxes >= 2) ? 0.05f : 0.0f};
    if (rateAxes > 0) gStudy.setInputRateLimit(rate);
    const int rowsAfterRate = gStudy.plannedRows();

    int accepted = 0;
    for (int i = 0; i < specs; i++) {
        // A distinct row direction per spec, so none of them condense to an
        // all-zero row that rebuildQp is entitled to discard.
        const float a[3] = {(i == 2) ? 1.0f : 0.0f, (i % 2 == 0) ? 1.0f : 0.0f,
                            (i % 2 == 0) ? 0.0f : 1.0f};
        const int last = (span > 0) ? span : 0;
        if (gStudy.addStateConstraint(a, -0.05f, 0.05f, 1, last)) accepted++;
    }
    const int planned = gStudy.plannedRows();

    const float x0[3] = {0.02f, 0.03f, 0.05f};
    float uMin[2], uMax[2];
    studyBox(vRef, 0.0f, true, uMin, uMax);
    gStudy.zero();
    gStudy.certifyOptimality = true;
    const chad::MpcResult r = gStudy.solve(x0, uMin, uMax, 200, 1e-5f);

    char buf[640];
    std::snprintf(buf, sizeof(buf),
        "{\"horizon\":%d,\"maxQpRows\":%d,\"maxStateRows\":%d,\"maxSpecs\":%d,"
        "\"rateAxes\":%d,\"rowsAfterRate\":%d,\"specsAttempted\":%d,"
        "\"specsAccepted\":%d,\"stateConstraints\":%d,\"plannedRows\":%d,"
        "\"constraintRows\":%d,\"droppedRows\":%d,\"usedAdmm\":%s,"
        "\"factorized\":%s,\"rho\":%.6e,\"primalResidual\":%.6e,"
        "\"sizeofMpc\":%d,\"sizeofFollower\":%d,\"sizeofQpRow\":%d}",
        gStudy.horizon(), chad::MPC_MAX_QP_ROWS, chad::MPC_MAX_STATE_ROWS,
        chad::MPC_MAX_STATE_CONSTRAINTS, rateAxes, rowsAfterRate, specs,
        accepted, gStudy.stateConstraints(), planned,
        r.constraintRows, r.droppedRows, r.usedAdmm ? "true" : "false",
        gStudy.qpFactorized() ? "true" : "false", gStudy.qpRho(),
        r.primalResidual,
        static_cast<int>(sizeof(chad::CondensedMpc)),
        static_cast<int>(sizeof(chad::MpcPathFollower)),
        static_cast<int>(sizeof(float) * chad::MPC_MAX_VARS));
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
// The constraint configuration comes from the same gMpcCfg the follower uses --
// inputRateLimit, crossTrackCorridor, headingCorridor, rateLimitTotalCommand,
// shiftWarmStart, certifyOptimality -- so the timing here and the tracking in a
// world run describe the same solver settings. Reference deltas and the rate
// anchor are fed the way MpcPathFollower::feedRateLimit does, because that is
// what makes a rate limit bound the COMMAND rather than the correction.
//
// A checksum is accumulated and reported so the optimiser cannot delete the
// solve.

/// @param horizon      MPC horizon N
/// @param solver       0 = fast gradient, 1 = projected Gauss-Seidel, 2 = ADMM
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
    const float ht = halfTrackM();

    const chad::DrivetrainPlant plant = chad::DrivetrainPlant::discretize(vRef, dt);
    const chad::LqrSolution lqr = chad::solveDiscreteLqr(plant, w);

    gStudy.solver = static_cast<chad::MpcSolver>(solver);
    gStudy.certifyOptimality = gMpcCfg.certifyOptimality;

    // Setup cost, averaged. Rebuilding the condensed matrices is the expensive
    // half and happens only when the reference speed crosses a bucket edge, so
    // it is priced separately from the per-tick solve. Constraints go on after
    // each setup: the state rows condense against G.
    //
    // The one-iteration solve is not padding. rebuildQp() and the Cholesky of
    // H + sigma I + rho C'C are LAZY -- they run on the first solve after a
    // rebuild -- so leaving them out would price the rebuild at the cost of
    // condensing alone and hide an O(n^3) factorization that a real tick pays.
    bool rejected = false;
    const float warmX[3] = {0.01f, 0.01f, 0.01f};
    float warmLo[2], warmHi[2];
    studyBox(vRef, 0.0f, constrained != 0, warmLo, warmHi);
    const int setupReps = 32;
    const double s0 = microsNow();
    for (int i = 0; i < setupReps; i++) {
        gStudy.setup(plant, w, horizon, lqr.P);
        studyConstraints(rejected);
        gStudy.solve(warmX, warmLo, warmHi, 1, 0.0f);
    }
    const double setupUs = (microsNow() - s0) / setupReps;

    // A reference angular rate that actually moves, so a limit on the command
    // rate and a limit on the correction rate are different constraints. Exact
    // triangle wave: consecutive samples differ by wSlew, always.
    const int refLen = ticks + chad::MPC_MAX_HORIZON + 2;
    std::vector<float> refW(refLen, 0.0f);
    {
        const float slew = (std::isfinite(gBench.wSlew) && gBench.wSlew > 0.0f)
                               ? gBench.wSlew : 0.0f;
        const int leg = (gBench.wLeg > 0) ? gBench.wLeg : 1;
        const float amp = slew * static_cast<float>(leg);
        float v = 0.0f;
        int dir = 1;
        for (int k = 0; k < refLen; k++) {
            refW[k] = v;
            v += static_cast<float>(dir) * slew;
            if (std::fabs(v) >= amp && amp > 0.0f) dir = -dir;
        }
    }
    const float wPeak = std::fabs(gBench.wSlew) * static_cast<float>(
                            (gBench.wLeg > 0) ? gBench.wLeg : 1);

    // Budgets are taken at the worst-case reference twist so they hold for the
    // whole run; only the offset by the current reference moves per tick.
    float boxAtPeak[2][2];
    studyBox(vRef, wPeak, constrained != 0, boxAtPeak[0], boxAtPeak[1]);
    const float vBudget = boxAtPeak[1][0] + vRef;
    const float wBudget = boxAtPeak[1][1] + wPeak;

    const bool rateActive = (gMpcCfg.inputRateLimit[0] > 0.0f ||
                             gMpcCfg.inputRateLimit[1] > 0.0f);
    const int hz = chad::clamp(horizon, 1, chad::MPC_MAX_HORIZON);

    float x[3] = {gBench.ex0, gBench.ey0, gBench.theta0};
    gStudy.zero();
    gTraceCount = 0;

    double sumCross = 0.0, peakCross = 0.0, checksum = 0.0;
    long iterSum = 0;
    int convergedCount = 0, maxIters = 0;
    // Whether the box actually BINDS. Without this the headline "MPC ties the
    // clipped LQR" is vacuous -- an inactive constraint is trivially free.
    int boundTicks = 0, mpcClipTicks = 0, lqrClipTicks = 0;
    // General-path diagnostics.
    int admmTicks = 0, droppedTicks = 0, relaxedTicks = 0, infeasTicks = 0;
    int factFailTicks = 0, capTicks = 0;
    float maxPrimal = -1.0f, maxDual = -1.0f;
    double sumPrimal = 0.0;
    int primalSamples = 0;
    // Realized and PLANNED corridor excursions, and the rate actually delivered.
    float maxPlanCross = 0.0f, maxPlanHeading = 0.0f;
    float maxPlanViolation = 0.0f, maxRealViolation = 0.0f;
    float maxCmdRate = 0.0f, maxCorrRate = 0.0f;
    // The rate profile of the PLAN, not of the committed move. setReferenceDeltas
    // moves only this: the committed command's rate is fixed by the anchor alone,
    // so dropping the deltas is invisible in maxCmdRate and shows up only here.
    float maxPlanCmdRate = 0.0f, maxPlanCorrRate = 0.0f;
    float lastCmd[2] = {0.0f, 0.0f}, lastU[2] = {0.0f, 0.0f};
    bool haveLast = false;
    int rows = 0;
    float delta[chad::MPC_MAX_VARS];
    const double t0 = microsNow();

    for (int k = 0; k < ticks; k++) {
        // Keep the error alive: without a disturbance the loop settles to zero
        // and every later solve starts from a perfect warm start, which flatters
        // RTI.
        if (gBench.disturb && k % 97 == 0) {
            x[0] += 0.02f;
            x[1] += 0.05f;
            x[2] += 0.10f;
        }

        const float wRef = refW[k];
        float uMin[2] = {-vBudget - vRef, -wBudget - wRef};
        float uMax[2] = {vBudget - vRef, wBudget - wRef};
        if (constrained == 0) {
            uMin[0] = uMin[1] = -1e6f;
            uMax[0] = uMax[1] = 1e6f;
        }

        if (rateActive) {
            float anchor[2] = {0.0f, 0.0f};
            if (haveLast) {
                if (gMpcCfg.rateLimitTotalCommand) {
                    anchor[0] = lastCmd[0] - vRef;
                    anchor[1] = lastCmd[1] - wRef;
                } else {
                    anchor[0] = lastU[0];
                    anchor[1] = lastU[1];
                }
            }
            gStudy.setPreviousInput(anchor);
            if (gMpcCfg.rateLimitTotalCommand) {
                for (int j = 1; j < hz; j++) {
                    delta[2 * j + 0] = 0.0f;            // constant linear reference
                    delta[2 * j + 1] = refW[k + j] - refW[k + j - 1];
                }
                delta[0] = delta[1] = 0.0f;
                gStudy.setReferenceDeltas(delta, 2 * hz);
            } else {
                gStudy.setReferenceDeltas(nullptr, 0);
            }
        }

        float u[2];
        chad::MpcResult r;
        if (itersPerTick > 0) {
            if (k > 0 && gMpcCfg.shiftWarmStart) gStudy.shift();
            r = gStudy.solve(x, uMin, uMax, itersPerTick, 0.0f);
        } else {
            gStudy.zero();
            r = gStudy.solve(x, uMin, uMax, gMpcCfg.maxIters, gMpcCfg.tol);
        }
        gStudy.firstMove(u);

        iterSum += r.iterations;
        if (r.iterations > maxIters) maxIters = r.iterations;
        if (r.converged) convergedCount++;
        if (r.usedAdmm) admmTicks++;
        if (r.droppedRows > 0) droppedTicks++;
        if (r.relaxedRows > 0) relaxedTicks++;
        if (r.primalInfeasible) infeasTicks++;
        if (r.factorizationFailed) factFailTicks++;
        if (r.hitIterationCap) capTicks++;
        if (r.constraintRows > rows) rows = r.constraintRows;
        if (r.primalResidual >= 0.0f) {
            if (r.primalResidual > maxPrimal) maxPrimal = r.primalResidual;
            sumPrimal += r.primalResidual;
            primalSamples++;
        }
        if (r.dualResidual >= 0.0f && r.dualResidual > maxDual) maxDual = r.dualResidual;

        {
            const float* U = gStudy.solution();
            for (int j = 1; j < hz; j++) {
                const float dCorr = U[2 * j + 1] - U[2 * (j - 1) + 1];
                const float dCmd = dCorr + (refW[k + j] - refW[k + j - 1]);
                if (std::fabs(dCmd) > maxPlanCmdRate) maxPlanCmdRate = std::fabs(dCmd);
                if (std::fabs(dCorr) > maxPlanCorrRate) maxPlanCorrRate = std::fabs(dCorr);
            }
        }

        // What the plan actually predicts, rolled through the plant rather than
        // read back out of the solver's own condensing.
        float planCross = 0.0f, planHeading = 0.0f;
        planExtremes(plant, x, gStudy.solution(), hz, planCross, planHeading);
        if (planCross > maxPlanCross) maxPlanCross = planCross;
        if (planHeading > maxPlanHeading) maxPlanHeading = planHeading;
        if (gMpcCfg.crossTrackCorridor > 0.0f) {
            const float e = planCross - gMpcCfg.crossTrackCorridor;
            if (e > maxPlanViolation) maxPlanViolation = e;
        }
        if (gMpcCfg.headingCorridor > 0.0f) {
            const float e = planHeading - gMpcCfg.headingCorridor;
            if (e > maxPlanViolation) maxPlanViolation = e;
        }

        // The box is a hard limit on what the drivetrain can do, so an
        // unconstrained command would be clipped downstream anyway.
        const float uApplied[2] = {chad::clamp(u[0], uMin[0], uMax[0]),
                                   chad::clamp(u[1], uMin[1], uMax[1])};
        if (r.activeBounds > 0) boundTicks++;
        if (uApplied[0] != u[0] || uApplied[1] != u[1]) mpcClipTicks++;

        const float cmd[2] = {vRef + uApplied[0], wRef + uApplied[1]};
        if (haveLast) {
            const float dc = std::fabs(cmd[1] - lastCmd[1]);
            const float du = std::fabs(uApplied[1] - lastU[1]);
            if (dc > maxCmdRate) maxCmdRate = dc;
            if (du > maxCorrRate) maxCorrRate = du;
        }

        float* row = (gTraceCount < kTraceMax) ? &gTrace[kTraceStride * gTraceCount++]
                                               : nullptr;
        if (row != nullptr) {
            row[0] = static_cast<float>(r.iterations);
            row[1] = r.primalResidual;
            row[2] = r.dualResidual;
            row[3] = r.converged ? 1.0f : 0.0f;
            row[4] = static_cast<float>(r.activeRows);
            row[5] = static_cast<float>(r.droppedRows);
            row[6] = static_cast<float>(r.relaxedRows);
            row[7] = r.primalInfeasible ? 1.0f : 0.0f;
            row[8] = x[1];
            row[9] = x[2];
            row[10] = uApplied[1];
            row[11] = cmd[1];
            row[12] = wRef;
            row[13] = planCross;
            row[14] = static_cast<float>(r.activeBounds);
        }

        lastCmd[0] = cmd[0]; lastCmd[1] = cmd[1];
        lastU[0] = uApplied[0]; lastU[1] = uApplied[1];
        haveLast = true;

        plantStep(plant, x, uApplied);

        const double c = std::fabs(x[1]);
        sumCross += c;
        if (c > peakCross) peakCross = c;
        if (gMpcCfg.crossTrackCorridor > 0.0f) {
            const float e = static_cast<float>(c) - gMpcCfg.crossTrackCorridor;
            if (e > maxRealViolation) maxRealViolation = e;
        }
        checksum += u[0] + u[1];
    }
    const double totalUs = microsNow() - t0;

    // The same loop with the solve removed: disturbance, box offset, reference
    // deltas, plan rollout, clamp, plant step, bookkeeping. Subtracted from
    // totalUs so the reported cost is the solver.
    float xh[3] = {gBench.ex0, gBench.ey0, gBench.theta0};
    double harnessChk = 0.0;
    const double h0 = microsNow();
    for (int k = 0; k < ticks; k++) {
        if (gBench.disturb && k % 97 == 0) { xh[0] += 0.02f; xh[1] += 0.05f; xh[2] += 0.10f; }
        const float wRef = refW[k];
        const float uMin[2] = {-vBudget - vRef, -wBudget - wRef};
        const float uMax[2] = {vBudget - vRef, wBudget - wRef};
        if (rateActive && gMpcCfg.rateLimitTotalCommand) {
            for (int j = 1; j < hz; j++) {
                delta[2 * j + 0] = 0.0f;
                delta[2 * j + 1] = refW[k + j] - refW[k + j - 1];
            }
            harnessChk += delta[2 * (hz - 1) + 1];
        }
        const float u[2] = {gStudy.solution()[0], gStudy.solution()[1]};
        const float ua[2] = {chad::clamp(u[0], uMin[0], uMax[0]),
                             chad::clamp(u[1], uMin[1], uMax[1])};
        float pc = 0.0f, ph = 0.0f;
        planExtremes(plant, xh, gStudy.solution(), hz, pc, ph);
        const float* U = gStudy.solution();
        for (int j = 1; j < hz; j++) {
            const float dCorr = U[2 * j + 1] - U[2 * (j - 1) + 1];
            harnessChk += std::fabs(dCorr + (refW[k + j] - refW[k + j - 1])) +
                          std::fabs(dCorr);
        }
        plantStep(plant, xh, ua);
        harnessChk += std::fabs(xh[1]) + u[0] + u[1] + pc + ph + wRef;
    }
    const double harnessUs = microsNow() - h0;

    // What one clock read costs, so the floor of this measurement is on record.
    const double c0 = microsNow();
    double clockChk = 0.0;
    for (int k = 0; k < 20000; k++) clockChk += microsNow();
    const double clockUs = (microsNow() - c0) / 20000.0;

    // Same closed loop under the scheduled LQR, for a cost and tracking
    // reference point. The LQR cannot see the box, so its command is clipped.
    float xl[3] = {gBench.ex0, gBench.ey0, gBench.theta0};
    double lqrSum = 0.0, lqrPeak = 0.0, lqrChk = 0.0;
    float lqrMaxCmdRate = 0.0f, lqrLastCmd = 0.0f;
    bool lqrHave = false;
    const double l0 = microsNow();
    for (int k = 0; k < ticks; k++) {
        if (gBench.disturb && k % 97 == 0) { xl[0] += 0.02f; xl[1] += 0.05f; xl[2] += 0.10f; }
        const float wRef = refW[k];
        const float uMin[2] = {-vBudget - vRef, -wBudget - wRef};
        const float uMax[2] = {vBudget - vRef, wBudget - wRef};
        float u[2];
        for (int i = 0; i < 2; i++) {
            u[i] = -(lqr.K[i][0] * xl[0] + lqr.K[i][1] * xl[1] + lqr.K[i][2] * xl[2]);
        }
        lqrChk += u[0] + u[1];
        const float ua[2] = {chad::clamp(u[0], uMin[0], uMax[0]),
                             chad::clamp(u[1], uMin[1], uMax[1])};
        if (ua[0] != u[0] || ua[1] != u[1]) lqrClipTicks++;
        const float cmd = wRef + ua[1];
        if (lqrHave) {
            const float d = std::fabs(cmd - lqrLastCmd);
            if (d > lqrMaxCmdRate) lqrMaxCmdRate = d;
        }
        lqrLastCmd = cmd;
        lqrHave = true;
        plantStep(plant, xl, ua);
        const double c = std::fabs(xl[1]);
        lqrSum += c;
        if (c > lqrPeak) lqrPeak = c;
    }
    // Raw, NOT harness-subtracted. The harness loop rolls the whole plan
    // forward and recomputes the reference deltas; the LQR loop does neither, so
    // subtracting it would over-subtract -- and did, reporting a negative cost.
    // Two 1x3 dot products are below this harness's resolution anyway; compare
    // against harnessUs to see the floor.
    const double lqrUs = (microsNow() - l0) / ticks;

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

    const double usPerSolve = (totalUs - harnessUs) / ticks;
    const long flopsIter = gStudy.flopsPerIteration();
    const double meanIters = static_cast<double>(iterSum) / ticks;

    char buf[2600];
    std::snprintf(buf, sizeof(buf),
        "{\"horizon\":%d,\"vars\":%d,\"solver\":%d,\"itersPerTick\":%d,"
        "\"constrained\":%d,\"ticks\":%d,"
        "\"usPerSolve\":%.4f,\"usGross\":%.4f,\"harnessUs\":%.4f,"
        "\"clockUs\":%.4f,\"maxIters\":%d,\"meanIters\":%.3f,"
        "\"convergedFrac\":%.4f,\"setupUs\":%.3f,"
        "\"flopsPerIter\":%ld,\"flopsSetup\":%ld,\"flopsPerTick\":%.0f,"
        "\"flopsWorstTick\":%.0f,"
        "\"meanCrossM\":%.6f,\"peakCrossM\":%.6f,"
        "\"lqrGrossUsPerTick\":%.4f,\"lqrMeanCrossM\":%.6f,\"lqrPeakCrossM\":%.6f,"
        "\"lqrMaxCmdRate\":%.6f,"
        "\"riccatiUsPerTick\":%.4f,\"riccatiIters\":%.1f,"
        "\"lambdaMax\":%.4f,\"powerIters\":%d,"
        "\"checksum\":%.6f,\"lqrChecksum\":%.6f,"
        "\"boundTicks\":%d,\"mpcClipTicks\":%d,\"lqrClipTicks\":%d,"
        "\"vBudget\":%.4f,\"wBudget\":%.4f,\"vRef\":%.4f,\"wPeak\":%.4f,"
        "\"rows\":%d,\"plannedRows\":%d,\"admmTicks\":%d,\"droppedTicks\":%d,"
        "\"relaxedTicks\":%d,\"infeasTicks\":%d,\"factFailTicks\":%d,"
        "\"capTicks\":%d,\"constraintsRejected\":%s,\"rho\":%.6e,"
        "\"maxPrimal\":%.6e,\"meanPrimal\":%.6e,\"maxDual\":%.6e,"
        "\"maxPlanCrossM\":%.6f,\"maxPlanHeadingRad\":%.6f,"
        "\"maxPlanViolationM\":%.6e,\"maxRealViolationM\":%.6e,"
        "\"maxCmdRate\":%.6f,\"maxCorrRate\":%.6f,"
        "\"maxPlanCmdRate\":%.6f,\"maxPlanCorrRate\":%.6f,"
        "\"corridor\":%.6f,\"headCorr\":%.6f,\"rateW\":%.6f,\"rateTotal\":%s,"
        "\"shift\":%s,\"certify\":%s,\"slew\":%.6f,"
        "\"riccatiChecksum\":%.6f,\"harnessChecksum\":%.6f,\"clockChecksum\":%.6e}",
        horizon, 2 * horizon, solver, itersPerTick, constrained, ticks,
        usPerSolve, totalUs / ticks, harnessUs / ticks,
        clockUs, maxIters, meanIters,
        static_cast<double>(convergedCount) / ticks, setupUs,
        flopsIter, gStudy.flopsSetup(), meanIters * static_cast<double>(flopsIter),
        // A control loop misses a deadline on its WORST tick, not its mean
        // one, and iteration counts here run to the cap. Setup lands on the
        // tick that re-condenses, so it belongs in the same total.
        static_cast<double>(maxIters) * static_cast<double>(flopsIter) +
            static_cast<double>(gStudy.flopsSetup()),
        sumCross / ticks, peakCross,
        lqrUs, lqrSum / ticks, lqrPeak, lqrMaxCmdRate,
        riccatiUs, static_cast<double>(riccatiIters) / ticks,
        gStudy.lipschitz(), gStudy.powerIterations(),
        checksum, lqrChk,
        boundTicks, mpcClipTicks, lqrClipTicks,
        vBudget, wBudget, vRef, wPeak,
        rows, gStudy.plannedRows(), admmTicks, droppedTicks,
        relaxedTicks, infeasTicks, factFailTicks,
        capTicks, rejected ? "true" : "false", gStudy.qpRho(),
        maxPrimal, (primalSamples > 0) ? sumPrimal / primalSamples : -1.0, maxDual,
        maxPlanCross, maxPlanHeading, maxPlanViolation, maxRealViolation,
        maxCmdRate, maxCorrRate, maxPlanCmdRate, maxPlanCorrRate,
        gMpcCfg.crossTrackCorridor, gMpcCfg.headingCorridor,
        gMpcCfg.inputRateLimit[1], gMpcCfg.rateLimitTotalCommand ? "true" : "false",
        gMpcCfg.shiftWarmStart ? "true" : "false",
        gMpcCfg.certifyOptimality ? "true" : "false", gBench.wSlew,
        riccatiChk, harnessChk, clockChk);
    gMpcJson.assign(buf);
    return gMpcJson.c_str();
}

/// Correctness gate: with the DARE cost-to-go as terminal cost and no active
/// constraints, the first move of the condensed QP must equal -K x exactly, for
/// any horizon and on EITHER solver path. If this drifts, the condensing or the
/// terminal cost is wrong and no tracking number below it means anything.
///
/// Forcing MpcSolver::Admm on a box-only problem is what makes the general path
/// comparable to the fast path: the same QP, the same answer, a different
/// algorithm.
EMSCRIPTEN_KEEPALIVE
const char* sim_mpc_check_json(int horizon, int solver) {
    const chad::MpcWeights w = gMpcCfg.weights;
    const float dt = gMpcCfg.dt;
    const float vRef = 0.75f * gMpcCfg.wheelSpeedLimit;

    const chad::DrivetrainPlant plant = chad::DrivetrainPlant::discretize(vRef, dt);
    const chad::LqrSolution lqr = chad::solveDiscreteLqr(plant, w);

    gStudy.solver = static_cast<chad::MpcSolver>(solver);
    gStudy.clearConstraints();
    gStudy.setup(plant, w, horizon, lqr.P);
    gStudy.certifyOptimality = true;
    // The dual residual floors near 1e-6 from float32 cancellation in a dense H
    // matvec, so an absolute tolerance under that would never terminate. The
    // primal here is a box the iterate never leaves.
    gStudy.admm.epsDualAbs = 1e-5f;
    gStudy.admm.epsDualRel = 0.0f;
    gStudy.admm.checkEvery = 8;
    // Honoured so a harness can drive rho off the cliff on purpose: a check
    // that cannot be made to fail is not measuring the ADMM path.
    gStudy.admm.rho = gMpcCfg.admmRho;
    gStudy.admm.rhoScale = gMpcCfg.admmRhoScale;

    const float lo[2] = {-1e6f, -1e6f};
    const float hi[2] = {1e6f, 1e6f};

    float worst = 0.0f;
    int worstIters = 0;
    float lqrU[2] = {0.0f, 0.0f}, mpcU[2] = {0.0f, 0.0f};
    const float probes[4][3] = {
        {0.05f, 0.10f, 0.20f},
        {-0.03f, 0.02f, -0.15f},
        {0.00f, -0.08f, 0.05f},
        {0.12f, 0.00f, 0.00f},
    };
    for (const auto& x : probes) {
        gStudy.zero();
        const chad::MpcResult r = gStudy.solve(x, lo, hi, 4000, 1e-12f);
        if (r.iterations > worstIters) worstIters = r.iterations;
        gStudy.firstMove(mpcU);
        for (int i = 0; i < 2; i++) {
            lqrU[i] = -(lqr.K[i][0] * x[0] + lqr.K[i][1] * x[1] + lqr.K[i][2] * x[2]);
            const float d = std::fabs(lqrU[i] - mpcU[i]) / (std::fabs(lqrU[i]) + 1e-6f);
            if (d > worst) worst = d;
        }
    }
    gStudy.admm = chad::CondensedMpc::AdmmConfig{};

    char buf[600];
    std::snprintf(buf, sizeof(buf),
        "{\"horizon\":%d,\"solver\":%d,\"worstRelErr\":%.3e,\"maxIters\":%d,"
        "\"lqrK\":[[%.5f,%.5f,%.5f],[%.5f,%.5f,%.5f]],"
        "\"lastLqrU\":[%.6f,%.6f],\"lastMpcU\":[%.6f,%.6f],\"lqrIters\":%d}",
        horizon, solver, worst, worstIters,
        lqr.K[0][0], lqr.K[0][1], lqr.K[0][2],
        lqr.K[1][0], lqr.K[1][1], lqr.K[1][2],
        lqrU[0], lqrU[1], mpcU[0], mpcU[1], lqr.iterations);
    gMpcJson.assign(buf);
    return gMpcJson.c_str();
}

}  // extern "C"
