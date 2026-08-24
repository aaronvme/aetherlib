// Critic evidence file for the constrained MPC (qp.hpp / mpc.cpp / rti.hpp).
//
// Tests named "defect*" encode the contract the headers advertise. They were
// written to FAIL and are now regressions: each one guards the defect it is
// named after, so a revert to the old behaviour reopens a named failure rather
// than a mystery. Tests named "vacuous*" prove that another test in the suite
// cannot fail. Tests named "confirms*" record what was verified rather than
// assumed, and "diag*" print a measurement without asserting one.
//
// Where a defect's premise became obsolete -- the row budget was raised, the
// addRow guard was fixed -- the case was rewritten to assert the NEW contract
// rather than deleted, because the old cliff is exactly what a future change to
// MPC_MAX_QP_ROWS or MPC_MAX_HORIZON would recreate.
#include "chad/movement_control/mpc.hpp"
#include "chad/movement_control/mpc_lqr.hpp"
#include "chad/movement_control/qp.hpp"
#include "chad/movement_control/rti.hpp"

#include <cmath>
#include <cstdio>
#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {

MpcWeights loose() { return {1.0f, 0.01f, 0.01f, 1.0f, 1.0f}; }
MpcWeights normal() { return {1.0f, 1.0f, 2.0f, 0.6f, 0.4f}; }

/// Forward-simulate the plant with a solved input sequence. out[k] is x_{k+1},
/// so out[k-1] is stage k in the library's 1-based stage numbering. Shares no
/// code with the condensing.
void rollout(const DrivetrainPlant& p, const float x0[3], const float* u, int n,
             float out[MPC_MAX_HORIZON][3]) {
    float x[3] = {x0[0], x0[1], x0[2]};
    for (int k = 0; k < n; k++) {
        float nx[3];
        for (int r = 0; r < 3; r++) {
            nx[r] = p.Ad[r][0] * x[0] + p.Ad[r][1] * x[1] + p.Ad[r][2] * x[2] +
                    p.Bd[r][0] * u[2 * k + 0] + p.Bd[r][1] * u[2 * k + 1];
        }
        for (int r = 0; r < 3; r++) x[r] = nx[r];
        for (int r = 0; r < 3; r++) out[k][r] = x[r];
    }
}

float peakAxis(const DrivetrainPlant& p, const float x0[3], const float* u, int n, int axis) {
    float xs[MPC_MAX_HORIZON][3];
    rollout(p, x0, u, n, xs);
    float worst = 0.0f;
    for (int k = 0; k < n; k++) worst = std::fmax(worst, std::fabs(xs[k][axis]));
    return worst;
}

}  // namespace

// ===========================================================================
// DEFECT 1 -- addStateConstraint() accepted up to MPC_MAX_STATE_CONSTRAINTS = 4
// specs while MPC_MAX_QP_ROWS funded only 2 full-horizon ones once a two-axis
// rate limit was present. The extra rows were dropped by `if (r < 0) continue;`
// in rebuildQp() and the solve still reported converged, on a QP that was not
// the one the caller asked for.
//
// The contract now: the budget funds every spec the spec count allows, the add
// counts the rows it will need and refuses rather than over-promising, and a row
// lost anyway sets MpcResult::droppedRows and forbids converged. So acceptance
// means installation, and this asserts it end to end -- the corridor the third
// spec asked for has to hold under forward simulation, not just appear in a row
// count.
// ===========================================================================
CHAD_TEST(zzcritic, defectStateRowBudgetSilentlyDropsAConstraint) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    const int horizon = MPC_MAX_HORIZON;
    CondensedMpc mpc;
    mpc.setup(plant, loose(), horizon);
    // Loose enough that the corridor is reachable. The original version of this
    // test used du = {0.05, 0.10}, under which |y| <= 0.04 is INFEASIBLE: at
    // 0.10 rad/s per tick the 0.35 rad heading cannot be killed before the
    // cross-track passes 0.041 m at stage 4, so no row budget could rescue it
    // and asserting the corridor there tested nothing about row accounting.
    const float du[2] = {1.0f, 2.0f};
    mpc.setInputRateLimit(du);
    const float ax[3] = {1.0f, 0.0f, 0.0f};
    const float at[3] = {0.0f, 0.0f, 1.0f};
    const float ay[3] = {0.0f, 1.0f, 0.0f};
    const float corridor = 0.04f;
    // Two harmless specs first, then the one that matters. All three must get
    // their rows, which is what the old budget could not fund.
    CHECK(mpc.addStateConstraint(ax, -10.0f, 10.0f));
    CHECK(mpc.addStateConstraint(at, -10.0f, 10.0f));
    CHECK(mpc.addStateConstraint(ay, -corridor, corridor));
    CHECK_EQ(mpc.stateConstraints(), 3);

    // 2N box + 2N rate + 3N state.
    const int wanted = 2 * 2 * horizon + 3 * horizon;
    CHECK_EQ(mpc.plannedRows(), wanted);

    const float uMin[2] = {-2.0f, -5.0f};
    const float uMax[2] = {2.0f, 5.0f};
    const float x0[3] = {0.0f, 0.0f, 0.35f};
    const float anchor[2] = {0.0f, 0.0f};
    mpc.zero();
    mpc.setPreviousInput(anchor);
    const MpcResult res = mpc.solve(x0, uMin, uMax, 40000, 1e-4f);
    const float peak = peakAxis(plant, x0, mpc.solution(), horizon, 1);

    std::printf("[defect1] cap=%d rows=%d wanted=%d dropped=%d conv=%d peakCross=%.4f\n",
                MPC_MAX_QP_ROWS, res.constraintRows, wanted, res.droppedRows,
                res.converged ? 1 : 0, static_cast<double>(peak));

    CHECK_EQ(res.constraintRows, wanted);
    CHECK_EQ(res.droppedRows, 0);
    CHECK(res.converged);
    CHECK(peak <= corridor + 3e-3f);
    // The corridor is what holds the cross-track down, not the weights: loose()
    // barely penalises y, so an unconstrained solve leaves it far outside.
    CondensedMpc open;
    open.setup(plant, loose(), horizon);
    open.setInputRateLimit(du);
    open.zero();
    open.setPreviousInput(anchor);
    open.solve(x0, uMin, uMax, 40000, 1e-4f);
    CHECK(peakAxis(plant, x0, open.solution(), horizon, 1) > 2.0f * corridor);
}

/// The only coverage of the drop path itself. Since the row budget now funds
/// every spec the spec count allows, a drop cannot come from the budget any
/// more -- what remains is a condensed row that is not representable. G's
/// cross-track entries scale with j * vRef * dt, so an absurd reference speed
/// plus a large-but-finite normal vector puts a'G_k past FLT_MAX while H itself
/// stays finite, and AdmmQp::addRow refuses it.
///
/// What this pins is the ACCOUNTING: the refusal is counted, the row count says
/// one fewer than planned, and the committed input is still box-feasible. It
/// cannot isolate the `droppedRows > 0 => !converged` forcing, because a plant
/// this badly conditioned does not converge at any budget with or without the
/// dropped row -- see the note in the report.
CHAD_TEST(zzcritic, aCondensedRowThatCannotBeRepresentedIsCountedNotSwallowed) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0e6f, 0.02f);
    const int horizon = 10;
    const float uMin[2] = {-0.3f, -0.6f};
    const float uMax[2] = {0.3f, 0.6f};
    const float x0[3] = {0.1f, 0.05f, 0.02f};

    CondensedMpc mpc;
    mpc.setup(plant, normal(), horizon);
    const float big[3] = {0.0f, 1.0e35f, 0.0f};
    CHECK(mpc.addStateConstraint(big, -1.0f, 1.0f));
    CHECK_EQ(mpc.plannedRows(), 3 * horizon);
    mpc.zero();
    const MpcResult res = mpc.solve(x0, uMin, uMax, 500, 1e-4f);
    std::printf("[dropped] planned=%d rows=%d dropped=%d conv=%d u0=%.6f\n",
                mpc.plannedRows(), res.constraintRows, res.droppedRows,
                res.converged ? 1 : 0, static_cast<double>(mpc.solution()[0]));
    CHECK_EQ(res.droppedRows, 1);
    CHECK_EQ(res.constraintRows, mpc.plannedRows() - res.droppedRows);
    CHECK(!res.converged);
    // The tick still has to produce something the hardware can take.
    for (int i = 0; i < mpc.vars(); i++) {
        CHECK_FINITE(mpc.solution()[i]);
        CHECK(mpc.solution()[i] >= uMin[i % 2] - 1e-6f);
        CHECK(mpc.solution()[i] <= uMax[i % 2] + 1e-6f);
    }

    // A representable normal over the same stages: nothing dropped.
    CondensedMpc fine;
    fine.setup(plant, normal(), horizon);
    const float small[3] = {0.0f, 1.0f, 0.0f};
    CHECK(fine.addStateConstraint(small, -1.0f, 1.0f));
    fine.zero();
    const MpcResult ok = fine.solve(x0, uMin, uMax, 500, 1e-4f);
    CHECK_EQ(ok.droppedRows, 0);
    CHECK_EQ(ok.constraintRows, fine.plannedRows());
}

/// The tightest configuration a caller can ask for, and the one MpcPathFollower
/// itself uses. The follower's maximum -- a rate limit on both axes plus both
/// corridors -- used to sit exactly on MPC_MAX_QP_ROWS with nothing to spare;
/// the point of this case is now the margin, and that the CEILING of the whole
/// advertised configuration still fits, since that is what makes "accepted" and
/// "installed" the same thing. Fails if the row budget shrinks, if the horizon
/// or the spec count grows without it, or if the follower's own row count grows
/// into the margin.
CHAD_TEST(zzcritic, rowBudgetFundsTheAdvertisedMaximumWithFollowerMargin) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    const int horizon = MPC_MAX_HORIZON;
    const float du[2] = {0.05f, 0.10f};
    const float ay[3] = {0.0f, 1.0f, 0.0f};
    const float at[3] = {0.0f, 0.0f, 1.0f};
    const float ax[3] = {1.0f, 0.0f, 0.0f};
    const float axy[3] = {1.0f, 1.0f, 0.0f};
    const float uMin[2] = {-2.0f, -5.0f};
    const float uMax[2] = {2.0f, 5.0f};
    const float x0[3] = {0.0f, 0.0f, 0.35f};

    CondensedMpc follower;
    follower.setup(plant, loose(), horizon);
    follower.setInputRateLimit(du);
    CHECK(follower.addStateConstraint(ay, -0.04f, 0.04f));
    CHECK(follower.addStateConstraint(at, -0.2f, 0.2f));
    const int used = follower.plannedRows();
    const int margin = MPC_MAX_QP_ROWS - used;
    follower.zero();
    const MpcResult fr = follower.solve(x0, uMin, uMax, 200, 1e-3f);
    std::printf("[margin] follower rows=%d of %d, margin=%d\n", fr.constraintRows,
                MPC_MAX_QP_ROWS, margin);
    CHECK_EQ(used, 6 * horizon);          // 2N box + 2N rate + 2N corridor
    CHECK_EQ(fr.constraintRows, used);
    CHECK_EQ(fr.droppedRows, 0);
    CHECK(margin >= 2 * horizon);         // room for two more full-horizon specs

    // The ceiling: MPC_MAX_STATE_CONSTRAINTS full-horizon specs on top of the
    // same two-axis rate limit. This is the configuration the old budget could
    // not fund, and it must fund it exactly rather than by luck.
    CondensedMpc ceiling;
    ceiling.setup(plant, loose(), horizon);
    ceiling.setInputRateLimit(du);
    CHECK(ceiling.addStateConstraint(ay, -1.0f, 1.0f));
    CHECK(ceiling.addStateConstraint(at, -1.0f, 1.0f));
    CHECK(ceiling.addStateConstraint(ax, -1.0f, 1.0f));
    CHECK(ceiling.addStateConstraint(axy, -1.0f, 1.0f));
    CHECK_EQ(ceiling.stateConstraints(), MPC_MAX_STATE_CONSTRAINTS);
    // Refused by the SPEC budget, not silently absorbed.
    CHECK(!ceiling.addStateConstraint(ay, -1.0f, 1.0f));
    CHECK_EQ(ceiling.stateConstraints(), MPC_MAX_STATE_CONSTRAINTS);

    ceiling.zero();
    const MpcResult cr = ceiling.solve(x0, uMin, uMax, 200, 1e-3f);
    std::printf("[margin] ceiling rows=%d of %d planned=%d dropped=%d\n", cr.constraintRows,
                MPC_MAX_QP_ROWS, ceiling.plannedRows(), cr.droppedRows);
    CHECK_EQ(ceiling.plannedRows(), 8 * horizon);
    CHECK(ceiling.plannedRows() <= MPC_MAX_QP_ROWS);
    CHECK_EQ(cr.constraintRows, ceiling.plannedRows());
    CHECK_EQ(cr.droppedRows, 0);
}

// ===========================================================================
// DEFECT 2 -- AdmmQp::addRow documents "returns -1 if ... the row is empty /
// non-finite". A NaN alongside a finite nonzero is accepted, because
// `m > peak` is false for NaN so the NaN never reaches the isfinite() test.
// ===========================================================================
CHAD_TEST(zzcritic, defectAddRowAcceptsANonFiniteCoefficient) {
    AdmmQp<4, 4> qp;
    qp.begin(3);
    float row[4] = {1.0f, std::nanf(""), 0.0f, 0.0f};
    const int r = qp.addRow(row);
    std::printf("[defect2] addRow({1, NaN, 0}) -> %d rows=%d stored=(%.3f, %.3f)\n", r,
                qp.rows(), r >= 0 ? static_cast<double>(qp.rowOf(r)[0]) : 0.0,
                r >= 0 ? static_cast<double>(qp.rowOf(r)[1]) : 0.0);
    CHECK_EQ(r, -1);
    CHECK_EQ(qp.rows(), 0);
}

/// Replaces a case that DOCUMENTED the old broken behaviour -- that a row mixing
/// a finite nonzero with a NaN was accepted. The guard is fixed, so what needs
/// pinning is that the two rejection branches are reached INDEPENDENTLY. The
/// original bug survived precisely because the shipped test's {NaN, 0, 0} row
/// tripped the empty-row branch first and never reached the finiteness test:
/// NaN > peak is false, so a NaN never becomes the peak and a peak test never
/// sees it.
CHAD_TEST(zzcritic, addRowRejectsEmptyAndNonFiniteRowsByDifferentBranches) {
    constexpr float kInf = std::numeric_limits<float>::infinity();

    // Branch 1, empty: all-zero, every entry finite.
    AdmmQp<4, 4> empty;
    empty.begin(3);
    float zeros[4]{};
    CHECK_EQ(empty.addRow(zeros), -1);
    CHECK_EQ(empty.rows(), 0);

    // Branch 2, non-finite, with a finite nonzero present so the empty-row
    // branch is NOT what rejects it. One case per non-finite kind, and one with
    // the bad entry AFTER the peak -- which is the ordering the old code let
    // through.
    const float bad[3] = {std::nanf(""), kInf, -kInf};
    for (int k = 0; k < 3; k++) {
        for (int after = 0; after < 2; after++) {
            AdmmQp<4, 4> qp;
            qp.begin(3);
            float row[4]{};
            row[after ? 0 : 2] = 5.0f;      // the finite peak
            row[after ? 2 : 0] = bad[k];
            CHECK_EQ(qp.addRow(row), -1);
            CHECK_EQ(qp.rows(), 0);
        }
    }

    // And a row that is neither empty nor non-finite is still accepted, or the
    // two above would pass for a guard that rejected everything.
    AdmmQp<4, 4> good;
    good.begin(3);
    float ok[4] = {1.0f, -2.0f, 0.0f, 0.0f};
    CHECK_EQ(good.addRow(ok), 0);
    CHECK_EQ(good.rows(), 1);
    CHECK_EQ(good.addUnitRow(1, std::nanf("")), -1);
    CHECK_EQ(good.addUnitRow(1, kInf), -1);
    CHECK_EQ(good.rows(), 1);
}

// ===========================================================================
// DEFECT 3 -- MpcPathFollower::Config::tol defaults to 1e-6, and that is below
// the float32 noise floor of the dual residual, so Mode::MpcConverged with any
// general constraint never converges and always burns its whole 400-iteration
// budget. Same tol is used for BOTH epsPrimal and epsDual even though the two
// are in different units.
// ===========================================================================
CHAD_TEST(zzcritic, defectFollowerDefaultToleranceIsUnreachableOnTheAdmmPath) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, loose(), 10);                 // Config::horizon default
    const float a[3] = {0.0f, 1.0f, 0.0f};
    CHECK(mpc.addStateConstraint(a, -0.04f, 0.04f));
    mpc.admm.checkEvery = 1;
    const float uMin[2] = {-2.0f, -5.0f};
    const float uMax[2] = {2.0f, 5.0f};
    const float x0[3] = {0.0f, 0.0f, 0.35f};

    MpcResult atDefault{};
    for (float tol : {1e-3f, 1e-4f, 1e-5f, 1e-6f}) {
        mpc.zero();
        const MpcResult res = mpc.solve(x0, uMin, uMax, 400, tol);   // maxIters default
        std::printf("[defect3] tol=%.0e iters=%d conv=%d cap=%d prim=%.3e dual=%.3e\n",
                    static_cast<double>(tol), res.iterations, res.converged ? 1 : 0,
                    res.hitIterationCap ? 1 : 0, static_cast<double>(res.primalResidual),
                    static_cast<double>(res.dualResidual));
        if (tol == 1e-6f) atDefault = res;
    }
    // The primal residual gets to 1e-8; only the dual floors out. A single
    // tolerance for two different unit systems is what makes this unreachable.
    CHECK(atDefault.primalResidual < 1e-6f);
    CHECK(atDefault.converged);
    CHECK(!atDefault.hitIterationCap);
    CHECK(atDefault.iterations < 400);
}

// ===========================================================================
// DEFECT 4 -- mpc_follower.hpp used to say of crossTrackCorridor: "The solver
// plans inside it rather than merely being penalised for leaving it, which is
// the whole difference between a weight and a constraint." That is false at the
// default rtiIters = 1, and it is not fixable by tuning: one ADMM iteration
// cannot satisfy a general row, and a corridor is the one constraint solve()
// cannot clamp on the way out, because it bounds a state the robot reaches later
// rather than the number being committed now.
//
// So the claim was corrected rather than made true, and the violation is now
// REPORTABLE: MpcResult::primalResidual is measured whether or not
// certifyOptimality is set, since it is one pass over the row support rather
// than the dual's dense H matvec. What this case pins is that the violation
// exists, that the reported number bounds it from above, and that both shrink
// with the budget -- so a future "optimisation" that stops measuring the primal,
// or a regression that stops honouring the corridor at all, fails here.
// ===========================================================================
CHAD_TEST(zzcritic, defectRtiBudgetDoesNotHonourTheStateCorridor) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    const int horizon = 12;
    const float corridor = 0.04f;
    const float a[3] = {0.0f, 1.0f, 0.0f};
    const float uMin[2] = {-2.0f, -5.0f};
    const float uMax[2] = {2.0f, 5.0f};
    const float x0[3] = {0.0f, 0.0f, 0.35f};

    float over[3]{}, reported[3]{}, stateViol[3]{};
    int idx = 0;
    for (int iters : {1, 4, 20}) {
        CondensedMpc mpc;
        mpc.setup(plant, loose(), horizon);
        CHECK(mpc.addStateConstraint(a, -corridor, corridor));
        RealTimeIteration rti(&mpc);
        rti.config.iterationsPerTick = iters;
        rti.config.tol = 0.0f;
        rti.config.shiftWarmStart = false;
        rti.config.trackPreviousInput = false;
        CHECK(!rti.config.certifyOptimality);   // the default: no dual, no verdict
        float u[2];
        float worst = 0.0f, worstPrim = 0.0f, worstState = 0.0f;
        for (int t = 0; t < 5; t++) {
            const MpcResult res = rti.step(x0, uMin, uMax, u);
            worst = std::fmax(worst, peakAxis(plant, x0, mpc.solution(), horizon, 1));
            worstState = std::fmax(worstState, res.stateViolation[0]);
            // Nothing certified the solve, so it must not claim it converged and
            // must not pretend to a dual residual -- but the primal violation
            // has to be a number the caller can act on.
            CHECK(!res.converged);
            CHECK(res.dualResidual < 0.0f);
            CHECK(res.kktResidual < 0.0f);
            CHECK(res.primalResidual >= 0.0f);
            CHECK_EQ(res.relaxedRows, 0);       // no single row's bounds crossed
            worstPrim = std::fmax(worstPrim, res.primalResidual);
        }
        over[idx] = worst - corridor;
        reported[idx] = worstPrim;
        stateViol[idx] = worstState;
        std::printf("[defect4] iters=%2d peakCross=%.4f over=%.4f (%.1f%%) "
                    "reportedPrimal=%.4f stateViol=%.4f\n",
                    iters, static_cast<double>(worst), static_cast<double>(over[idx]),
                    100.0 * (static_cast<double>(worst) / corridor - 1.0),
                    static_cast<double>(worstPrim), static_cast<double>(worstState));
        // stateViolation is the corridor distance in METRES, so it has to equal
        // the excursion a forward simulation finds -- not merely bound it. The
        // two are computed from the same U by different routes: this one through
        // the condensed row a'G_k, the rollout through the plant recursion.
        CHECK_NEAR(worstState, over[idx], 1e-5);
        idx++;
    }
    // The corridor IS violated at a one-iteration budget. Asserted, so that a
    // change which quietly made it hold would force the header claim to be
    // revisited rather than left understating the solver.
    CHECK(over[0] > 3e-3f);
    // And the reported residual bounds the violation, so a caller does not need
    // a forward simulation to find out.
    CHECK(reported[0] >= over[0]);
    // Spending the budget shrinks both, by more than an order of magnitude.
    CHECK(over[2] < 0.1f * over[0]);
    CHECK(reported[2] < 0.1f * reported[0]);
    CHECK(stateViol[2] < 0.1f * stateViol[0]);
}

// ===========================================================================
// DEFECT 4b -- the residual above bounds the corridor violation here only
// because this configuration happens to attain its row maximum on a corridor
// row. mpc_follower.hpp described it as "an upper bound on how far outside, in
// metres"; it is a maximum over rows whose units DIFFER, and on a corridor the
// solve cannot satisfy it is attained on a box row instead -- in m/s, off the
// pre-clamp iterate, three orders of magnitude away from any distance.
//
// The distance this checks against is built by rolling the plan through the
// plant. No solver-reported quantity is fed back into it, so the comparison
// cannot pass by construction.
// ===========================================================================
CHAD_TEST(zzcritic, defectPrimalResidualIsAMixedUnitRowMaximumNotADistance) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.415f, 0.01f);
    const int horizon = 10;
    // Tighter than the ~2 cm of cross-track a 10-tick horizon can command, so
    // the corridor is not merely active but unreachable, which is what makes
    // the ADMM iterate run away and the two numbers separate.
    const float corridor = 0.002f;
    const float a[3] = {0.0f, 1.0f, 0.0f};
    // MpcPathFollower::inputBox at vRef = 0.75 * 1.887 m/s, its own asymmetric
    // inscribed box, so the box rows carry a realistic scale.
    const float uMin[2] = {-3.066f, -1.558f};
    const float uMax[2] = {0.236f, 1.558f};
    const float x0[3] = {0.02f, 0.05f, 0.12f};

    CondensedMpc mpc;
    mpc.setup(plant, MpcWeights{1.0f, 40.0f, 4.0f, 1.0f, 0.25f}, horizon);
    CHECK(mpc.addStateConstraint(a, -corridor, corridor));
    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 1;
    rti.config.tol = 0.0f;
    rti.config.trackPreviousInput = false;

    float u[2];
    float worstPrimal = 0.0f, worstState = 0.0f, worstRollout = 0.0f;
    int stateRowAttainedViolation = 0;
    for (int t = 0; t < 40; t++) {
        const MpcResult res = rti.step(x0, uMin, uMax, u);
        CHECK_EQ(res.relaxedRows, 0);
        worstPrimal = std::fmax(worstPrimal, res.primalResidual);
        worstState = std::fmax(worstState, res.stateViolation[0]);
        const float peak = peakAxis(plant, x0, mpc.solution(), horizon, 1);
        worstRollout = std::fmax(worstRollout, peak - corridor);
        if (res.worstViolationSpec >= 0) stateRowAttainedViolation++;
    }
    std::printf("[defect4b] primalResidual=%.4e  stateViolation=%.4e  "
                "rollout=%.4e  ratio=%.0f  stateRowAttained=%d/40\n",
                static_cast<double>(worstPrimal), static_cast<double>(worstState),
                static_cast<double>(worstRollout),
                static_cast<double>(worstPrimal / worstRollout),
                stateRowAttainedViolation);

    // The corridor really is missed, by a distance a forward simulation agrees
    // with to float precision. This is the number that is in metres.
    CHECK(worstRollout > 1e-2f);
    CHECK_NEAR(worstState, worstRollout, 1e-5);

    // And primalResidual is nowhere near it. Two orders of magnitude is far
    // more than any unit conversion could excuse, which is the point: it is not
    // a distance at all. Measured ~6000x.
    CHECK(worstPrimal > 100.0f * worstRollout);

    // The reason: the row attaining the worst violation is never a state row.
    // Zero is the assertion, not a threshold -- a box row in m/s wins every
    // tick, and solve() clamps that row on the way out while the corridor
    // cannot be clamped at all.
    CHECK_EQ(stateRowAttainedViolation, 0);
}

// ===========================================================================
// DEFECT 5 -- two rows whose OWN bounds are each consistent but whose
// intersection is empty. `relaxedRows` only catches a single row crossing, so
// there is no primal-infeasibility signal at all, and with the RTI default of
// certifyOptimality = false the residuals are not even computed. The box here
// is what MpcPathFollower::inputBox produces when the reference speed exceeds
// the drivetrain's own wheel-speed limit, and the anchor is tick 1's zero.
// ===========================================================================
CHAD_TEST(zzcritic, defectNoPrimalInfeasibilityDetection) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, loose(), 10);
    const float du[2] = {0.01f, 0.02f};
    mpc.setInputRateLimit(du);
    const float uMin[2] = {-3.6f, -5.0f};        // vRef 2.0 against a 1.6 limit
    const float uMax[2] = {-0.4f, 5.0f};
    const float x0[3] = {0.2f, 0.1f, 0.05f};
    const float anchor[2] = {0.0f, 0.0f};

    // Row "u_0 in [-3.6, -0.4]" and row "u_0 in [-0.01, 0.01]" cannot both hold.
    mpc.certifyOptimality = false;               // the RealTimeIteration default
    mpc.zero();
    mpc.setPreviousInput(anchor);
    const MpcResult blind = mpc.solve(x0, uMin, uMax, 50, 1e-4f);
    std::printf("[defect5] blind: relaxed=%d prim=%.3e dual=%.3e conv=%d u0=%.4f\n",
                blind.relaxedRows, static_cast<double>(blind.primalResidual),
                static_cast<double>(blind.dualResidual), blind.converged ? 1 : 0,
                static_cast<double>(mpc.solution()[0]));
    // Something has to say the constraint set was empty.
    CHECK(blind.relaxedRows > 0 || blind.primalResidual > 0.0f);

    mpc.certifyOptimality = true;
    mpc.zero();
    mpc.setPreviousInput(anchor);
    float lastPrim = 0.0f;
    for (int t = 0; t < 2000; t++) {
        const MpcResult res = mpc.solve(x0, uMin, uMax, 1, 1e-4f);
        lastPrim = res.primalResidual;
        CHECK_FINITE(mpc.solution()[0]);
    }
    std::printf("[defect5] certified after 2000 ticks: prim=%.4f u0=%.4f (box edge %.2f)\n",
                static_cast<double>(lastPrim), static_cast<double>(mpc.solution()[0]),
                static_cast<double>(uMax[0]));
    // Recorded, not asserted: the iterate does NOT diverge, it parks at a fixed
    // point with a large standing primal residual. The committed move is the box
    // edge, which is safe; the diagnostics are what is missing.
    CHECK(lastPrim > 0.1f);
}

// ===========================================================================
// VACUOUS 1 -- rti/resetClearsTheWarmStart used to assert `|u[0]| <= |warm| +
// 1e-6`, which holds identically when reset() is skipped, so a reset() that did
// nothing would pass; and CHECK_FINITE(u[0]) cannot fail either, because solve()
// clamps the output into the box unconditionally.
//
// Kept after the rewrite because it also records the trap the strengthened
// version fell into: on this state, axis 0 is within 4e-5 of the optimum after
// two iterations from zero, so even an EQUALITY against the warm value cannot
// separate reset from no-reset there. The rewritten case asserts on axis 1, and
// a second case covers the ADMM path, which is the only one that touches the
// slack and multipliers at all.
// ===========================================================================
CHAD_TEST(zzcritic, vacuousResetClearsTheWarmStart) {
    const float boxMin[2] = {-0.30f, -0.60f};
    const float boxMax[2] = {0.30f, 0.60f};
    const float x0[3] = {0.5f, 0.5f, 0.25f};

    CondensedMpc mpc;
    mpc.setup(DrivetrainPlant::discretize(1.2f, 0.02f), normal(), 10);
    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 2;
    float u[2];
    for (int i = 0; i < 30; i++) rti.step(x0, boxMin, boxMax, u);
    const float warm[2] = {u[0], u[1]};

    RealTimeIteration cold(&mpc);      // a fresh binding is the same as a reset
    cold.config.iterationsPerTick = 2;
    float uc[2];
    cold.step(x0, boxMin, boxMax, uc);

    rti.step(x0, boxMin, boxMax, u);             // the old test called reset() here
    std::printf("[vacuous1] warm=(%.6f,%.6f) noReset=(%.6f,%.6f) reset=(%.6f,%.6f)\n",
                static_cast<double>(warm[0]), static_cast<double>(warm[1]),
                static_cast<double>(u[0]), static_cast<double>(u[1]),
                static_cast<double>(uc[0]), static_cast<double>(uc[1]));
    CHECK(std::fabs(u[0]) <= std::fabs(warm[0]) + 1e-6f);   // the old assertion
    CHECK_FINITE(u[0]);
    // The trap: axis 0 cannot tell a reset solve from a warm one at all...
    CHECK(std::fabs(uc[0] - warm[0]) < 1e-3f);
    // ...while axis 1 can, by twenty times the margin.
    CHECK(std::fabs(uc[1] - warm[1]) > 1e-2f);
}

// ===========================================================================
// VACUOUS 2 -- mpc/stepNormAloneDoesNotClaimConvergence. Both branches of its
// if/else restate `converged == (kktResidual <= tol)`, which is the line of
// code under test (mpc.cpp:513). No mutation of the solver can fail it.
// ===========================================================================
CHAD_TEST(zzcritic, vacuousStepNormAloneDoesNotClaimConvergence) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.4f, 0.02f);
    const float uMin[2] = {-1.0e6f, -1.0e6f};
    const float uMax[2] = {1.0e6f, 1.0e6f};
    const float x0[3] = {1.0f, 1.0f, 0.5f};
    for (int budget : {1, 3, 9, 40}) {
        for (float tol : {1e-4f, 1e-2f, 1.0f}) {
            CondensedMpc mpc;
            mpc.setup(plant, normal(), 16);
            mpc.zero();
            const MpcResult r = mpc.solve(x0, uMin, uMax, budget, tol);
            // The identity that makes the real test's if/else a tautology.
            CHECK_EQ(r.converged, (r.kktResidual >= 0.0f && r.kktResidual <= tol));
            CHECK(r.kktResidual >= 0.0f);        // the "else" branch, always true
        }
    }
}

// ===========================================================================
// VACUOUS 3 -- mpc/stateCorridorOverAStageRangeOnlyConstrainsThatRange asserts
// only a row COUNT, which is identical for [5,9] and for the off-by-one [4,8]
// and [6,10]. The test cannot catch the off-by-one it is named for.
// ===========================================================================
CHAD_TEST(zzcritic, vacuousStageRangeTestOnlyCountsRows) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    const int horizon = 12;
    const float uMin[2] = {-2.0f, -5.0f};
    const float uMax[2] = {2.0f, 5.0f};
    const float x0[3] = {0.0f, 0.0f, 0.35f};
    const float a[3] = {0.0f, 1.0f, 0.0f};
    int rows[3]{};
    int idx = 0;
    for (int first : {4, 5, 6}) {
        CondensedMpc mpc;
        mpc.setup(plant, loose(), horizon);
        CHECK(mpc.addStateConstraint(a, -0.04f, 0.04f, first, first + 4));
        mpc.zero();
        rows[idx++] = mpc.solve(x0, uMin, uMax, 20000, 1e-4f).constraintRows;
    }
    std::printf("[vacuous3] rows for [4,8]=%d [5,9]=%d [6,10]=%d\n", rows[0], rows[1],
                rows[2]);
    CHECK_EQ(rows[0], 2 * horizon + 5);
    CHECK_EQ(rows[1], 2 * horizon + 5);
    CHECK_EQ(rows[2], 2 * horizon + 5);
}

// ===========================================================================
// CONFIRMS -- the stage range really is 1-based and inclusive, established by
// forward simulation rather than by a row count. A single-stage corridor at
// stage k puts x_k exactly on the bound and leaves later stages outside it.
// ===========================================================================
CHAD_TEST(zzcritic, confirmsStateStageIndexingIsOneBasedInclusive) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    const int horizon = 12;
    const float uMin[2] = {-2.0f, -5.0f};
    const float uMax[2] = {2.0f, 5.0f};
    const float x0[3] = {0.0f, 0.0f, 0.35f};
    const float corridor = 0.04f;
    const float a[3] = {0.0f, 1.0f, 0.0f};

    CondensedMpc open;
    open.setup(plant, loose(), horizon);
    open.zero();
    open.solve(x0, uMin, uMax, 20000, 1e-8f);
    float xo[MPC_MAX_HORIZON][3];
    rollout(plant, x0, open.solution(), horizon, xo);
    // Cross-track grows monotonically here, so "x_k is at the bound" pins k.
    for (int k = 1; k < horizon; k++) CHECK(xo[k][1] > xo[k - 1][1]);

    for (int stage : {6, 9, 12}) {
        CondensedMpc mpc;
        mpc.setup(plant, loose(), horizon);
        CHECK(mpc.addStateConstraint(a, -corridor, corridor, stage, stage));
        mpc.zero();
        CHECK(mpc.solve(x0, uMin, uMax, 40000, 1e-4f).converged);
        float xs[MPC_MAX_HORIZON][3];
        rollout(plant, x0, mpc.solution(), horizon, xs);
        std::printf("[confirms] single-stage %2d -> x_%d=%.4f x_%d=%.4f x_%d=%.4f\n", stage,
                    stage - 1, static_cast<double>(xs[stage - 2][1]), stage,
                    static_cast<double>(xs[stage - 1][1]), stage + 1,
                    stage < horizon ? static_cast<double>(xs[stage][1]) : 0.0);
        // Stage `stage` sits on the bound...
        CHECK_NEAR(std::fabs(xs[stage - 1][1]), corridor, 1e-3);
        // ...and the neighbours do not, which is what rules out an off-by-one.
        // Only checkable mid-horizon: with the corridor on the LAST stage the
        // plan is free to overshoot on the way there and come back to it, so
        // stage N-1 above the bound is admissible rather than evidence.
        if (stage < horizon) {
            CHECK(std::fabs(xs[stage - 2][1]) < corridor - 1e-3f);
            CHECK(std::fabs(xs[stage][1]) > corridor + 1e-4f);
        }
    }
}

/// The iteration cap holds even when checkEvery never lands on a multiple, and
/// the reported residual describes the returned iterate because the last
/// iteration is always checked.
CHAD_TEST(zzcritic, confirmsIterationCapSurvivesSparseResidualChecks) {
    AdmmQp<4, 4> qp;
    qp.begin(4);
    float H[4][4]{};
    for (int i = 0; i < 4; i++) H[i][i] = 1.0f;
    for (int i = 0; i < 4; i++) CHECK_EQ(qp.addUnitRow(i, 1.0f), i);
    for (int r = 0; r < 4; r++) { qp.lower()[r] = -1.0f; qp.upper()[r] = 1.0f; }
    CHECK(qp.factorize(H, 1.0f, 1e-6f));
    const float q[4] = {50.0f, -60.0f, 30.0f, -20.0f};
    for (int check : {0, 7, 1000}) {
        qp.resetState();
        const QpResult res = qp.solve(q, 3, 1e-9f, 1e-9f, check);
        CHECK_EQ(res.iterations, 3);
        CHECK(res.hitIterationCap);
        CHECK(!res.converged);
        // The primal is measured either way -- it is a pass over the row
        // support, not an H matvec. Only the dual is gated on certification,
        // which is why `converged` is.
        CHECK(res.primalResidual > 0.0f);
        if (check == 0) CHECK(res.dualResidual < 0.0f);
        else CHECK(res.dualResidual > 0.0f);
    }
}

/// setReferenceDeltas(nullptr) must clear stale deltas, or the rate bound keeps
/// applying yesterday's reference steps. Not covered by the existing suite.
CHAD_TEST(zzcritic, confirmsNullReferenceDeltasClearThePreviousOnes) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.3f, 0.02f);
    const int horizon = 10;
    const float uMin[2] = {-0.6f, -1.2f};
    const float uMax[2] = {0.6f, 1.2f};
    const float x0[3] = {0.35f, 0.45f, 0.25f};
    const float du[2] = {0.03f, 0.08f};
    const float anchor[2] = {0.0f, 0.0f};

    float dRef[MPC_MAX_VARS]{};
    for (int k = 1; k < horizon; k++) {
        dRef[2 * k + 0] = 0.025f;
        dRef[2 * k + 1] = -0.060f;
    }

    CondensedMpc cleared;
    cleared.setup(plant, normal(), horizon);
    cleared.setInputRateLimit(du);
    cleared.zero();
    cleared.setPreviousInput(anchor);
    cleared.setReferenceDeltas(dRef, 2 * horizon);
    CHECK(cleared.solve(x0, uMin, uMax, 8000, 1e-5f).converged);
    cleared.setReferenceDeltas(nullptr, 0);
    cleared.zero();
    CHECK(cleared.solve(x0, uMin, uMax, 8000, 1e-5f).converged);

    CondensedMpc never;
    never.setup(plant, normal(), horizon);
    never.setInputRateLimit(du);
    never.zero();
    never.setPreviousInput(anchor);
    CHECK(never.solve(x0, uMin, uMax, 8000, 1e-5f).converged);

    for (int i = 0; i < never.vars(); i++) {
        CHECK_NEAR(cleared.solution()[i], never.solution()[i], 3e-3);
    }
}

/// RESOLUTION of the shiftWarmStart finding. The report measured shifting as
/// costing 1.2-4.4x the iterations of leaving the iterate alone in every regime,
/// with the row-shift map deriving correctly by hand -- so the cost was recorded
/// as unattributed. Two causes, both now identified, and neither is the map:
///
///   1. The 4.4x outlier on a never-binding rate limit was the ADMM dual warm
///      start being wiped at every re-condensing (DEFECT 6). With that fixed the
///      same measurement reads 1.7x, and 0.76x once the terminal cost is right.
///   2. Everything left is time inconsistency. Shifting assumes u_1 is the next
///      tick's u_0, which holds only when the terminal cost is the
///      infinite-horizon cost-to-go. Every measurement in the report ran with
///      terminalP = nullptr, i.e. the terminal stage merely weighted by Q --
///      exactly the regime rti.hpp documents as "turn it off".
///
/// This asserts the mechanism directly: the ordering of "shifted sequence" vs
/// "unshifted sequence" as a predictor of the NEXT tick's optimum flips with the
/// terminal cost. MpcPathFollower::condense() supplies the DARE P, so the
/// default of true is right there; it is wrong for a bare CondensedMpc built
/// without one.
CHAD_TEST(zzcritic, shiftIsTheRightWarmStartExactlyWhenTheTerminalCostIsTheCostToGo) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.2f, 0.02f);
    const float uMin[2] = {-0.30f, -0.60f};
    const float uMax[2] = {0.30f, 0.60f};
    // Interior of the box: with every stage saturated both sequences are the
    // constant vector, the shift is the identity and the comparison is vacuous.
    const float x0[3] = {0.05f, 0.06f, 0.03f};
    const int N = 10;
    const LqrSolution lqr = solveDiscreteLqr(plant, normal());
    CHECK(lqr.converged);

    float shiftErr[2]{}, holdErr[2]{};
    for (int useP = 0; useP < 2; useP++) {
        const float (*P)[3] = (useP == 1) ? lqr.P : nullptr;
        CondensedMpc a;
        a.setup(plant, normal(), N, P);
        a.zero();
        a.solve(x0, uMin, uMax, 200000, 1e-7f);
        float uOpt[MPC_MAX_VARS];
        int saturated = 0;
        for (int i = 0; i < 2 * N; i++) {
            uOpt[i] = a.solution()[i];
            const float lo = (i % 2 == 0) ? uMin[0] : uMin[1];
            const float hi = (i % 2 == 0) ? uMax[0] : uMax[1];
            if (uOpt[i] <= lo + 1e-6f || uOpt[i] >= hi - 1e-6f) saturated++;
        }
        CHECK_EQ(saturated, 0);

        float nx[3];
        for (int r = 0; r < 3; r++) {
            nx[r] = plant.Ad[r][0] * x0[0] + plant.Ad[r][1] * x0[1] + plant.Ad[r][2] * x0[2] +
                    plant.Bd[r][0] * uOpt[0] + plant.Bd[r][1] * uOpt[1];
        }
        CondensedMpc b;
        b.setup(plant, normal(), N, P);
        b.zero();
        b.solve(nx, uMin, uMax, 200000, 1e-7f);

        for (int i = 0; i < 2 * N; i++) {
            // shift() advances by one block and repeats the last one.
            const float shifted = (i + 2 < 2 * N) ? uOpt[i + 2] : uOpt[2 * N - 2 + (i % 2)];
            shiftErr[useP] = std::fmax(shiftErr[useP], std::fabs(shifted - b.solution()[i]));
            holdErr[useP] = std::fmax(holdErr[useP], std::fabs(uOpt[i] - b.solution()[i]));
        }
        std::printf("[shift-cause] terminalP=%d  ||shift(U*)-U*next||=%.6f "
                    "||U*-U*next||=%.6f  %s\n",
                    useP, static_cast<double>(shiftErr[useP]),
                    static_cast<double>(holdErr[useP]),
                    shiftErr[useP] < holdErr[useP] ? "shift wins" : "hold wins");
    }
    CHECK(shiftErr[1] < holdErr[1]);   // with the cost-to-go, shifting is right
    CHECK(shiftErr[0] > holdErr[0]);   // without it, it is not
}

/// The premise the existing shift tests actually run under: a scalar decay of
/// the state. Recorded to show shifting LOSES there, so those tests' passing
/// tells you nothing about mRowShiftFrom being aligned.
CHAD_TEST(zzcritic, vacuousShiftTestsRunOnAPremiseThatDefeatsShifting) {
    const float uMin[2] = {-0.30f, -0.60f};
    const float uMax[2] = {0.30f, 0.60f};
    const float du[2] = {0.02f, 0.05f};

    int total[2] = {0, 0};
    for (int mode = 0; mode < 2; mode++) {
        CondensedMpc mpc;
        mpc.setup(DrivetrainPlant::discretize(1.2f, 0.02f), normal(), 10);
        mpc.setInputRateLimit(du);
        mpc.admm.checkEvery = 1;
        RealTimeIteration rti(&mpc);
        rti.config.iterationsPerTick = 4000;
        rti.config.tol = 1e-4f;
        rti.config.certifyOptimality = true;
        rti.config.shiftWarmStart = (mode == 1);
        rti.config.trackPreviousInput = true;
        float x0[3] = {0.30f, 0.40f, 0.15f};
        float u[2];
        for (int t = 0; t < 40; t++) {
            total[mode] += rti.step(x0, uMin, uMax, u).iterations;
            x0[0] *= 0.97f;
            x0[1] *= 0.97f;
            x0[2] *= 0.97f;
        }
    }
    std::printf("[vacuous4] scalar-decay iterations over 40 ticks: noShift=%d shift=%d\n",
                total[0], total[1]);
    CHECK(total[1] > total[0]);   // shifting costs MORE under this premise
}

// ===========================================================================
// ISOLATION for the shift finding: box path vs general path, model-consistent
// propagation, with and without shift(), and -- the dimension the report was
// missing -- with and without the DARE terminal cost. The ratios reverse across
// that column on the general path, which is the whole story.
// ===========================================================================
CHAD_TEST(zzcritic, diagShiftIsolation) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.2f, 0.02f);
    const float uMin[2] = {-0.30f, -0.60f};
    const float uMax[2] = {0.30f, 0.60f};
    const LqrSolution lqr = solveDiscreteLqr(plant, normal());

    struct Case { const char* label; bool general; float du; bool track; };
    const Case cases[] = {
        {"box only            ", false, 0.0f,   false},
        {"admm, box rows only ", false, 0.0f,   false},
        {"rate du=10 (loose)  ", true,  10.0f,  false},
        {"rate du=0.02 (binds)", true,  0.02f,  true},
    };

    for (int useP = 0; useP < 2; useP++) {
        for (int c = 0; c < 4; c++) {
            int total[2] = {0, 0};
            for (int mode = 0; mode < 2; mode++) {
                CondensedMpc mpc;
                mpc.setup(plant, normal(), 10, (useP == 1) ? lqr.P : nullptr);
                if (c == 1) mpc.solver = MpcSolver::Admm;
                if (cases[c].general) {
                    const float du[2] = {cases[c].du, cases[c].du * 2.5f};
                    mpc.setInputRateLimit(du);
                }
                mpc.admm.checkEvery = 1;
                RealTimeIteration rti(&mpc);
                rti.config.iterationsPerTick = 20000;
                rti.config.tol = 1e-4f;
                rti.config.certifyOptimality = true;
                rti.config.shiftWarmStart = (mode == 1);
                rti.config.trackPreviousInput = cases[c].track;
                float x0[3] = {0.30f, 0.40f, 0.15f};
                float u[2];
                for (int t = 0; t < 40; t++) {
                    total[mode] += rti.step(x0, uMin, uMax, u).iterations;
                    float nx[3];
                    for (int r = 0; r < 3; r++) {
                        nx[r] = plant.Ad[r][0] * x0[0] + plant.Ad[r][1] * x0[1] +
                                plant.Ad[r][2] * x0[2] + plant.Bd[r][0] * u[0] +
                                plant.Bd[r][1] * u[1];
                    }
                    for (int r = 0; r < 3; r++) x0[r] = nx[r];
                }
            }
            std::printf("[shift-iso] P=%d %s noShift=%5d shift=%5d  ratio=%.2f\n", useP,
                        cases[c].label, total[0], total[1],
                        static_cast<double>(total[1]) / static_cast<double>(total[0]));
        }
    }
}

// ===========================================================================
// DEFECT 6 -- every re-condensing throws the ADMM dual/slack warm start away.
// rebuildQp() calls AdmmQp::begin(), and each row is then re-added through
// commit(), which sets mW[r] = mY[r] = 0 (qp.hpp:336-337). So the multipliers
// the previous tick paid for are gone. mpc_test's
// warmStartSurvivesARecondensingAtTheSameHorizon only checks the PRIMAL
// sequence, so this is invisible to the suite. MpcPathFollower re-condenses
// whenever either end of the horizon crosses a speed bucket, with
// timeVaryingHorizon on by default.
// ===========================================================================
CHAD_TEST(zzcritic, defectRecondensingWipesTheAdmmDualWarmStart) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    const float uMin[2] = {-0.3f, -0.6f};
    const float uMax[2] = {0.3f, 0.6f};
    const float x0[3] = {0.25f, 0.35f, 0.12f};
    const float du[2] = {0.02f, 0.05f};
    const float anchor[2] = {0.0f, 0.0f};

    CondensedMpc mpc;
    mpc.setup(plant, normal(), 10);
    mpc.setInputRateLimit(du);
    mpc.admm.checkEvery = 1;
    mpc.zero();
    mpc.setPreviousInput(anchor);
    const MpcResult cold = mpc.solve(x0, uMin, uMax, 8000, 1e-5f);
    CHECK(cold.converged);
    const MpcResult warm = mpc.solve(x0, uMin, uMax, 8000, 1e-5f);
    CHECK(warm.converged);

    // Re-condense at the SAME horizon and the SAME plant: nothing about the QP
    // has changed, so a warm start that survived would cost what `warm` cost.
    mpc.setup(plant, normal(), 10);
    mpc.setInputRateLimit(du);
    const MpcResult after = mpc.solve(x0, uMin, uMax, 8000, 1e-5f);
    CHECK(after.converged);
    std::printf("[defect6] cold=%d warm=%d after-identical-recondense=%d\n", cold.iterations,
                warm.iterations, after.iterations);
    CHECK(after.iterations <= warm.iterations + 1);
}

/// The identical rebuild above is the easy case: every row key matches and C is
/// unchanged, so a carry-over that happened to be keyed on the row INDEX would
/// pass it. What a real run does is cross a speed bucket, which changes G and
/// therefore every state and box row's coefficients while leaving what each row
/// MEANS alone. That is the case the keying exists for.
CHAD_TEST(zzcritic, warmStartSurvivesARecondensingAtADifferentSpeed) {
    const float uMin[2] = {-0.3f, -0.6f};
    const float uMax[2] = {0.3f, 0.6f};
    const float x0[3] = {0.25f, 0.35f, 0.12f};
    const float du[2] = {0.02f, 0.05f};
    const float anchor[2] = {0.0f, 0.0f};

    for (float v2 : {1.05f, 1.2f, 1.5f, 2.0f}) {
        CondensedMpc mpc;
        mpc.setup(DrivetrainPlant::discretize(1.0f, 0.02f), normal(), 10);
        mpc.setInputRateLimit(du);
        mpc.admm.checkEvery = 1;
        mpc.zero();
        mpc.setPreviousInput(anchor);
        const MpcResult cold = mpc.solve(x0, uMin, uMax, 8000, 1e-5f);
        CHECK(cold.converged);

        mpc.setup(DrivetrainPlant::discretize(v2, 0.02f), normal(), 10);
        mpc.setInputRateLimit(du);
        const MpcResult after = mpc.solve(x0, uMin, uMax, 8000, 1e-5f);
        CHECK(after.converged);

        // The same new plant from scratch, for the cold price at that speed.
        CondensedMpc fresh;
        fresh.setup(DrivetrainPlant::discretize(v2, 0.02f), normal(), 10);
        fresh.setInputRateLimit(du);
        fresh.admm.checkEvery = 1;
        fresh.zero();
        fresh.setPreviousInput(anchor);
        const MpcResult cold2 = fresh.solve(x0, uMin, uMax, 8000, 1e-5f);
        CHECK(cold2.converged);

        std::printf("[bucket] v 1.0 -> %.2f  cold=%d afterRecondense=%d coldAtNewV=%d\n",
                    static_cast<double>(v2), cold.iterations, after.iterations,
                    cold2.iterations);
        // Measured 10, 11, 15, 16 against 19, 19, 20, 20 -- ratios 1.90, 1.73,
        // 1.33, 1.25. It was 16-26 against 74 before rhoScale went from 1 to 3,
        // so the carry-over is worth less now only because the cold solve got
        // four times cheaper.
        //
        // The strong margin is asked of the near buckets only, and that is not
        // a weakening: the follower runs 16 buckets over 1.887 m/s, so a real
        // edge is a 0.118 m/s step and 1.0 -> 2.0 is eight of them at once,
        // which no tick does. Every case still has to beat the cold price.
        CHECK(after.iterations < cold2.iterations);
        if (v2 <= 1.2f) CHECK(after.iterations * 3 < cold2.iterations * 2);
        // And the answer is the same one the cold solve at that speed finds, so
        // the saving is not bought by stopping somewhere else.
        for (int i = 0; i < mpc.vars(); i++) {
            CHECK_NEAR(mpc.solution()[i], fresh.solution()[i], 3e-3);
        }
    }
}

/// Analytic cost of the follower's default constrained configuration, for the
/// 10 ms budget claim in rti.hpp.
CHAD_TEST(zzcritic, diagFlopBudgetOfTheDefaultConstrainedConfig) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    const float a[3] = {0.0f, 1.0f, 0.0f};
    const float at[3] = {0.0f, 0.0f, 1.0f};
    const float du[2] = {0.05f, 0.10f};
    for (int horizon : {10, MPC_MAX_HORIZON}) {
        CondensedMpc mpc;
        mpc.setup(plant, loose(), horizon);
        mpc.setInputRateLimit(du);
        CHECK(mpc.addStateConstraint(a, -0.04f, 0.04f));
        CHECK(mpc.addStateConstraint(at, -0.2f, 0.2f));
        const float uMin[2] = {-2.0f, -5.0f};
        const float uMax[2] = {2.0f, 5.0f};
        const float x0[3] = {0.0f, 0.0f, 0.35f};
        mpc.zero();
        const MpcResult res = mpc.solve(x0, uMin, uMax, 400, 1e-6f);
        std::printf("[flops] N=%2d rows=%d flops/iter=%ld setup=%ld  iters@tol1e-6=%d "
                    "=> %ld flops/tick\n",
                    horizon, res.constraintRows, mpc.flopsPerIteration(), mpc.flopsSetup(),
                    res.iterations, mpc.flopsPerIteration() * res.iterations);
    }
}

// ===========================================================================
// The largest gap the critic report named: no test checked a CONSTRAINED
// condensed-MPC optimum against an independent solve. Everything else either
// counted rows, or asserted box feasibility -- which is the unconditional output
// clamp in solve(), not the solver -- or re-derived the condensing's own
// formula, which proves only that a formula was typed twice.
//
// The reference below shares nothing with the condensing. It solves the SPARSE
// optimal control problem -- states kept as decision variables, the dynamics
// kept as equality constraints -- in double precision by exhaustive active-set
// enumeration, the same method mpcqp/matchesActiveSetEnumeration uses on the
// bare QP. A wrong Phi, G, H or F, a state row condensed against the wrong
// stage, or a rate row on the wrong pair of columns all show up as a
// disagreement in u.
// ===========================================================================
namespace {

constexpr int kOcpMaxN = 3;
constexpr int kOcpVars = 5 * kOcpMaxN;      // x_1..x_N, then u_0..u_{N-1}
constexpr int kOcpIneq = 10;
constexpr int kOcpDim = kOcpVars + 3 * kOcpMaxN + kOcpIneq;

struct RefOcp {
    int n{0};              // horizon
    int nv{0};             // decision variables
    int neq{0};
    int m{0};              // inequality rows
    double H[kOcpVars][kOcpVars]{};
    double Aeq[3 * kOcpMaxN][kOcpVars]{};
    double beq[3 * kOcpMaxN]{};
    double C[kOcpIneq][kOcpVars]{};
    double lo[kOcpIneq]{};
    double hi[kOcpIneq]{};
};

int ocpX(int k, int r) { return 3 * (k - 1) + r; }                         // k = 1..N
int ocpU(const RefOcp& p, int k, int a) { return 3 * p.n + 2 * k + a; }    // k = 0..N-1

bool ocpLinear(int n, double A[kOcpDim][kOcpDim], double* b, double* x) {
    for (int c = 0; c < n; c++) {
        int piv = c;
        for (int r = c + 1; r < n; r++) {
            if (std::fabs(A[r][c]) > std::fabs(A[piv][c])) piv = r;
        }
        if (std::fabs(A[piv][c]) < 1e-11) return false;
        if (piv != c) {
            for (int j = 0; j < n; j++) {
                const double t = A[piv][j];
                A[piv][j] = A[c][j];
                A[c][j] = t;
            }
            const double t = b[piv];
            b[piv] = b[c];
            b[c] = t;
        }
        for (int r = c + 1; r < n; r++) {
            const double f = A[r][c] / A[c][c];
            if (f == 0.0) continue;
            for (int j = c; j < n; j++) A[r][j] -= f * A[c][j];
            b[r] -= f * b[c];
        }
    }
    for (int r = n - 1; r >= 0; r--) {
        double s = b[r];
        for (int j = r + 1; j < n; j++) s -= A[r][j] * x[j];
        x[r] = s / A[r][r];
    }
    return true;
}

/// Cost 0.5 z'Hz with H = 2 blkdiag(Q,..,Qterm, R,..,R), dynamics as
/// x_{k+1} - Ad x_k - Bd u_k = 0. No condensing anywhere.
void ocpBuild(RefOcp& p, int horizon, const DrivetrainPlant& plant, const MpcWeights& w,
              const float P[3][3], const float x0[3]) {
    p.n = horizon;
    p.nv = 5 * horizon;
    p.neq = 3 * horizon;
    p.m = 0;

    const double q[3] = {w.qx, w.qy, w.qTheta};
    for (int k = 1; k <= horizon; k++) {
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                const double v = (k == horizon && P != nullptr)
                                     ? static_cast<double>(P[r][c])
                                     : ((r == c) ? q[r] : 0.0);
                p.H[ocpX(k, r)][ocpX(k, c)] = 2.0 * v;
            }
        }
    }
    for (int k = 0; k < horizon; k++) {
        p.H[ocpU(p, k, 0)][ocpU(p, k, 0)] = 2.0 * static_cast<double>(w.rv);
        p.H[ocpU(p, k, 1)][ocpU(p, k, 1)] = 2.0 * static_cast<double>(w.rw);
    }

    for (int k = 0; k < horizon; k++) {
        for (int r = 0; r < 3; r++) {
            const int row = 3 * k + r;
            p.Aeq[row][ocpX(k + 1, r)] = 1.0;
            if (k == 0) {
                double s = 0.0;
                for (int c = 0; c < 3; c++) s += static_cast<double>(plant.Ad[r][c]) * x0[c];
                p.beq[row] = s;
            } else {
                for (int c = 0; c < 3; c++) {
                    p.Aeq[row][ocpX(k, c)] = -static_cast<double>(plant.Ad[r][c]);
                }
                p.beq[row] = 0.0;
            }
            for (int a = 0; a < 2; a++) {
                p.Aeq[row][ocpU(p, k, a)] = -static_cast<double>(plant.Bd[r][a]);
            }
        }
    }
}

void ocpAddRow(RefOcp& p, int col, double lo, double hi) {
    p.C[p.m][col] = 1.0;
    p.lo[p.m] = lo;
    p.hi[p.m] = hi;
    p.m++;
}

/// Enumerate every inequality as inactive, at its lower bound, or at its upper
/// bound. H is positive definite and Aeq has full row rank, so the first
/// candidate that is primal feasible with the right inequality-multiplier signs
/// is the unique global optimum.
bool ocpSolve(const RefOcp& p, double z[kOcpVars], int* activeOut) {
    int total = 1;
    for (int r = 0; r < p.m; r++) total *= 3;
    const int maxActive = p.nv - p.neq;

    for (int code = 0; code < total; code++) {
        int act[kOcpIneq]{}, side[kOcpIneq]{};
        int na = 0;
        int c = code;
        for (int r = 0; r < p.m; r++) {
            const int st = c % 3;
            c /= 3;
            if (st != 0) { act[na] = r; side[na] = st; na++; }
        }
        if (na > maxActive) continue;

        const int dim = p.nv + p.neq + na;
        double A[kOcpDim][kOcpDim]{};
        double b[kOcpDim]{}, x[kOcpDim]{};
        for (int i = 0; i < p.nv; i++) {
            for (int j = 0; j < p.nv; j++) A[i][j] = p.H[i][j];
            b[i] = 0.0;                        // the linear term is zero here
        }
        for (int e = 0; e < p.neq; e++) {
            for (int j = 0; j < p.nv; j++) {
                A[j][p.nv + e] = p.Aeq[e][j];
                A[p.nv + e][j] = p.Aeq[e][j];
            }
            b[p.nv + e] = p.beq[e];
        }
        for (int k = 0; k < na; k++) {
            const int col = p.nv + p.neq + k;
            for (int j = 0; j < p.nv; j++) {
                A[j][col] = p.C[act[k]][j];
                A[col][j] = p.C[act[k]][j];
            }
            b[col] = (side[k] == 1) ? p.lo[act[k]] : p.hi[act[k]];
        }
        if (!ocpLinear(dim, A, b, x)) continue;

        bool ok = true;
        for (int r = 0; r < p.m && ok; r++) {
            double cz = 0.0;
            for (int j = 0; j < p.nv; j++) cz += p.C[r][j] * x[j];
            if (cz < p.lo[r] - 1e-9 || cz > p.hi[r] + 1e-9) ok = false;
        }
        // H z + Aeq' nu + C_act' lam = 0, so a row held at its UPPER bound needs
        // lam >= 0 and one at its lower needs lam <= 0. Equality multipliers are
        // unsigned.
        for (int k = 0; k < na && ok; k++) {
            const double lam = x[p.nv + p.neq + k];
            if (side[k] == 2 && lam < -1e-9) ok = false;
            if (side[k] == 1 && lam > 1e-9) ok = false;
        }
        if (!ok) continue;

        for (int j = 0; j < p.nv; j++) z[j] = x[j];
        if (activeOut != nullptr) *activeOut = na;
        return true;
    }
    return false;
}

}  // namespace

CHAD_TEST(zzcritic, condensedOptimumMatchesASparseOcpUnderAStateCorridor) {
    // dt = 0.05 rather than the usual 0.02: at 20 ms a three-stage horizon moves
    // the cross-track by under a millimetre, so no corridor value is both
    // feasible and binding, and a comparison against a non-binding constraint
    // measures nothing. The code path is identical either way.
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.05f);
    const int horizon = kOcpMaxN;
    const MpcWeights w = normal();
    const float uMin[2] = {-2.0f, -5.0f};
    const float uMax[2] = {2.0f, 5.0f};
    const float x0[3] = {0.0f, 0.30f, 0.0f};
    const float ay[3] = {0.0f, 1.0f, 0.0f};
    // Unconstrained this plan leaves y_2 at 0.2968; the tightest y_2 the box
    // allows is 0.2625. 0.28 is therefore both reachable and binding.
    const float corridor = 0.28f;
    // Terminal cost is the DARE cost-to-go, so it is assemble()'s terminalP
    // branch under test rather than the Q fallback.
    const LqrSolution lqr = solveDiscreteLqr(plant, w);
    CHECK(lqr.converged);

    RefOcp p;
    ocpBuild(p, horizon, plant, w, lqr.P, x0);
    for (int k = 0; k < horizon; k++) {
        for (int a = 0; a < 2; a++) ocpAddRow(p, ocpU(p, k, a), uMin[a], uMax[a]);
    }
    // Stages 2..N only, so an off-by-one in the stage range moves the answer.
    for (int k = 2; k <= horizon; k++) ocpAddRow(p, ocpX(k, 1), -corridor, corridor);

    double ref[kOcpVars]{};
    int active = 0;
    CHECK(ocpSolve(p, ref, &active));

    CondensedMpc mpc;
    mpc.setup(plant, w, horizon, lqr.P);
    CHECK(mpc.addStateConstraint(ay, -corridor, corridor, 2, horizon));
    mpc.admm.checkEvery = 1;
    mpc.zero();
    const MpcResult res = mpc.solve(x0, uMin, uMax, 200000, 1e-7f);
    CHECK(res.converged);
    CHECK(res.usedAdmm);
    CHECK_EQ(res.constraintRows, 2 * horizon + (horizon - 1));

    std::printf("[ocp-corridor] active=%d rows=%d iters=%d u_ref=(%.6f,%.6f) "
                "u_mpc=(%.6f,%.6f)\n",
                active, res.constraintRows, res.iterations, ref[ocpU(p, 0, 0)],
                ref[ocpU(p, 0, 1)], static_cast<double>(mpc.solution()[0]),
                static_cast<double>(mpc.solution()[1]));

    // The constraint has to bind, or this compares two unconstrained solves.
    CHECK(active > 0);
    CondensedMpc open;
    open.setup(plant, w, horizon, lqr.P);
    open.zero();
    open.solve(x0, uMin, uMax, 200000, 1e-7f);
    CHECK(std::fabs(open.solution()[1] - mpc.solution()[1]) > 1e-2f);

    for (int k = 0; k < horizon; k++) {
        for (int a = 0; a < 2; a++) {
            CHECK_NEAR(mpc.solution()[2 * k + a], ref[ocpU(p, k, a)], 3e-4);
        }
    }
    // And the state trajectory the reference predicts is the one the plant
    // actually produces from that input, which is the condensing itself.
    float xs[MPC_MAX_HORIZON][3];
    rollout(plant, x0, mpc.solution(), horizon, xs);
    for (int k = 1; k <= horizon; k++) {
        for (int r = 0; r < 3; r++) CHECK_NEAR(xs[k - 1][r], ref[ocpX(k, r)], 3e-4);
    }
}

CHAD_TEST(zzcritic, condensedOptimumMatchesASparseOcpUnderARateLimit) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    const int horizon = kOcpMaxN;
    const MpcWeights w = normal();
    const float uMin[2] = {-0.30f, -0.60f};
    const float uMax[2] = {0.30f, 0.60f};
    const float x0[3] = {0.20f, 0.15f, 0.25f};
    // Angular axis only, so the rate rows are a strict subset and the row stride
    // in rebuildQp() is exercised rather than assumed. Unconstrained the plan
    // wants u1 = (-0.0754, -0.0500, -0.0248), so an anchor of -0.02 with du =
    // 0.04 binds the head row and the reference step at stage 1 binds the first
    // difference row.
    const float du[2] = {0.0f, 0.04f};
    const float anchor[2] = {0.10f, -0.02f};
    // ref_k - ref_{k-1}, indexed [2k + axis] for k = 1..N-1.
    float dRef[MPC_MAX_VARS]{};
    dRef[2 * 1 + 1] = 0.03f;
    dRef[2 * 2 + 1] = -0.02f;

    RefOcp p;
    ocpBuild(p, horizon, plant, w, nullptr, x0);
    for (int k = 0; k < horizon; k++) {
        for (int a = 0; a < 2; a++) ocpAddRow(p, ocpU(p, k, a), uMin[a], uMax[a]);
    }
    // Head: u_0 against the committed command. Then u_k - u_{k-1}, with the
    // reference step folded into the bound.
    ocpAddRow(p, ocpU(p, 0, 1), anchor[1] - du[1], anchor[1] + du[1]);
    for (int k = 1; k < horizon; k++) {
        const double d = dRef[2 * k + 1];
        p.C[p.m][ocpU(p, k, 1)] = 1.0;
        p.C[p.m][ocpU(p, k - 1, 1)] = -1.0;
        p.lo[p.m] = -static_cast<double>(du[1]) - d;
        p.hi[p.m] = static_cast<double>(du[1]) - d;
        p.m++;
    }

    double ref[kOcpVars]{};
    int active = 0;
    CHECK(ocpSolve(p, ref, &active));

    CondensedMpc mpc;
    mpc.setup(plant, w, horizon);
    mpc.setInputRateLimit(du);
    mpc.setPreviousInput(anchor);
    mpc.setReferenceDeltas(dRef, 2 * horizon);
    mpc.admm.checkEvery = 1;
    mpc.zero();
    const MpcResult res = mpc.solve(x0, uMin, uMax, 200000, 1e-7f);
    CHECK(res.converged);
    CHECK_EQ(res.constraintRows, 2 * horizon + horizon);   // box + one-axis rate

    std::printf("[ocp-rate] active=%d rows=%d iters=%d u_ref=(%.6f,%.6f) "
                "u_mpc=(%.6f,%.6f)\n",
                active, res.constraintRows, res.iterations, ref[ocpU(p, 0, 0)],
                ref[ocpU(p, 0, 1)], static_cast<double>(mpc.solution()[0]),
                static_cast<double>(mpc.solution()[1]));

    CHECK(active > 0);
    CondensedMpc open;
    open.setup(plant, w, horizon);
    open.zero();
    open.solve(x0, uMin, uMax, 200000, 1e-7f);
    CHECK(std::fabs(open.solution()[1] - mpc.solution()[1]) > 1e-3f);

    for (int k = 0; k < horizon; k++) {
        for (int a = 0; a < 2; a++) {
            CHECK_NEAR(mpc.solution()[2 * k + a], ref[ocpU(p, k, a)], 3e-4);
        }
    }
}
