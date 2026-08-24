/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include "chad/movement_control/mpc_plant.hpp"
#include "chad/movement_control/mpc_lqr.hpp"
#include "chad/movement_control/qp.hpp"

namespace chad {

// Linear MPC over DrivetrainPlant. The states are condensed out, so the
// decision variable is the input sequence alone:
//
//     min 1/2 U' H U + (F x0)' U   s.t.  l <= C U <= u
//
// H and F depend only on reference speed, which is why setup() is the expensive
// half and solve() touches nothing but preallocated members.
//
// Two solvers sit behind one entry point.
//
//   With nothing but a box on the inputs, FISTA or projected Gauss-Seidel
//   handle it directly: the projection is a clamp and one iteration is one H
//   matvec. That is the common case and it stays the fast path.
//
//   With any GENERAL row -- an input-rate limit, a corridor on a predicted
//   state, a terminal set -- there is no cheap projection, and the problem goes
//   to the ADMM solver in qp.hpp. That costs a triangular solve per iteration
//   instead of a matvec, which is why the dispatch exists rather than running
//   everything through the general path.
//
// The point of constraining at all is that an LQR cannot respect a limit -- it
// emits the unconstrained optimum and something downstream clips it, which is a
// different and generally worse trajectory than planning inside the limit. A
// rate limit is the sharper version of the same argument: a voltage step that
// is admissible on paper still slips the wheels or browns out the brain.
//
// Give it the DARE solution P from lqr.hpp as the terminal cost and, with no
// constraint active, the first move equals the LQR move at any horizon. That
// equivalence is the correctness gate in sim/test/mpc.mjs.

constexpr int MPC_MAX_HORIZON = 20;
constexpr int MPC_MAX_VARS = 2 * MPC_MAX_HORIZON;
constexpr int MPC_MAX_STATE_CONSTRAINTS = 4;
/// Every accepted spec may span the whole horizon, so the row budget has to
/// fund the spec budget: anything smaller silently reduces the advertised four
/// constraints to however many fit. Costs mC, which is MPC_MAX_QP_ROWS x
/// MPC_MAX_VARS floats.
constexpr int MPC_MAX_STATE_ROWS = MPC_MAX_STATE_CONSTRAINTS * MPC_MAX_HORIZON;
/// Input box, input rate, and condensed state rows.
constexpr int MPC_MAX_QP_ROWS = 2 * MPC_MAX_VARS + MPC_MAX_STATE_ROWS;

enum class MpcSolver {
    /// Projected gradient with Nesterov momentum (FISTA). Fixed step 1/L,
    /// L = lambda_max(H). One iteration is one H matvec.
    FastGradient,
    /// One projected Gauss-Seidel sweep, i.e. coordinate-wise exact
    /// minimisation with clamping. Same flop count per sweep as the gradient
    /// step, strictly better contraction on a diagonally dominant H.
    GaussSeidel,
    /// OSQP-style ADMM on l <= C U <= u. Selected automatically whenever a
    /// general constraint is present; naming it here forces it even on a
    /// box-only problem, which is what makes the two paths comparable.
    Admm,
};

struct MpcResult {
    int iterations{0};
    /// ||U_k - U_{k-1}||_inf on the last iteration. Negative on the ADMM path,
    /// which does not produce it.
    float stepInf{0.0f};
    /// Box path: ||U - proj(U - alpha grad f(U))||_inf, in input units -- how
    /// far one more projected step would move, zero exactly at the constrained
    /// optimum, so unlike stepInf it cannot be small merely because progress
    /// stalled. ADMM path: max of the two residuals below. Negative when
    /// certification was skipped.
    float kktResidual{-1.0f};
    /// Set only when the exit criterion was actually evaluated and met. A solve
    /// with certification switched off reports false, because nothing was
    /// checked -- not true, because the step norm is not a certificate.
    bool converged{false};
    int activeBounds{0};      // decision variables sitting on a bound
    int restarts{0};          // momentum restarts (FastGradient only)

    /// The iteration budget ran out without a certified exit. Separates "spent
    /// the budget" from "stopped early without checking".
    bool hitIterationCap{false};

    bool usedAdmm{false};
    /// max_r |C_r U - w_r|: ADMM's splitting inconsistency, a maximum over rows
    /// in DIFFERENT units. The box rows are in input units, the rate rows in
    /// input units per tick, a cross-track corridor in metres; the reported
    /// number is whichever is largest and carries that row's unit, not metres.
    /// On a diverging iterate it is essentially always a box or rate row, both
    /// of which solve() clamps on the way out anyway.
    ///
    /// So this is a solver health indicator, not a corridor violation. Use
    /// stateViolation[] for the latter. Always measured on the general path,
    /// including when certifyOptimality is off. Negative only on the box path.
    float primalResidual{-1.0f};
    /// dist(a'x_k, [lo, hi]) for state constraint `s`, worst over the stages
    /// that spec spans, in the SPEC's own units -- metres for a cross-track
    /// corridor, radians for a heading one. Indexed in the order
    /// addStateConstraint() accepted them. This is the number that answers "how
    /// far outside the corridor did the plan go", which primalResidual does not.
    ///
    /// Measured on the solution as RETURNED, after the rate and box clamps, so
    /// it describes the plan the caller executes. Zero means satisfied, which is
    /// safe: it is a distance, and zero is the good case rather than an
    /// unmeasured one. Read only the first stateConstraints() entries.
    float stateViolation[MPC_MAX_STATE_CONSTRAINTS]{};
    /// max over EVERY row of its distance to its own bounds -- same mixed-units
    /// caveat as primalResidual, but a violation rather than an inconsistency.
    float worstViolation{0.0f};
    /// Spec index when a state row attained worstViolation, -1 when a box or
    /// rate row did. Says what unit worstViolation is in.
    int worstViolationSpec{-1};
    /// ||H U + q + C' y||_inf, in input units. Needs an H matvec, so it is
    /// measured only when certifyOptimality is set. Negative when it was not,
    /// which is also why `converged` is then false.
    float dualResidual{-1.0f};
    int constraintRows{0};
    int activeRows{0};
    /// Rows whose bounds crossed -- an unreachable rate limit, say -- and were
    /// collapsed to their midpoint so the QP stayed feasible.
    int relaxedRows{0};
    /// A constraint the caller asked for could not be installed, so the QP that
    /// was solved is not the QP that was requested. Forces converged = false: a
    /// certified optimum of the wrong problem is worse than no answer.
    int droppedRows{0};
    /// The constraint set was detected empty. See QpResult::primalInfeasible --
    /// sufficient, not necessary, and no certificate.
    bool primalInfeasible{false};
    float rho{0.0f};
    /// The KKT matrix lost positive definiteness. The solve fell back to the
    /// box, so the input is still deliverable, but the general rows were
    /// ignored.
    bool factorizationFailed{false};
    /// x0 contained a non-finite entry and was treated as zero error.
    bool stateNotFinite{false};
    /// The solve produced a non-finite input; the solution was zeroed.
    bool solutionNotFinite{false};
};

class CondensedMpc {
public:
    /// Build the condensed QP. `terminalP` may be null, in which case the
    /// terminal state is weighted by Q like every other stage.
    /// @param terminalP  DARE cost-to-go, or null to weight the last stage by Q
    /// @param inputR     full 2x2 input weight, or null for diag(w.rv, w.rw).
    ///                   Needed when the input has been transformed (see
    ///                   DrivetrainPlant::withWheelInputs), because the
    ///                   transformed weight is no longer diagonal.
    void setup(const DrivetrainPlant& plant, const MpcWeights& w, int horizon,
               const float terminalP[3][3] = nullptr,
               const float inputR[2][2] = nullptr);

    /// Condense a horizon whose plant changes stage to stage -- one entry per
    /// step, in prediction order. The horizon length is `count`, clamped to
    /// MPC_MAX_HORIZON. Use this when the reference speed varies over the
    /// horizon; `setup` assumes it does not.
    void setupVarying(const DrivetrainPlant* stages, int count, const MpcWeights& w,
                      const float terminalP[3][3] = nullptr,
                      const float inputR[2][2] = nullptr);

    /// Solve to `maxIters` or until the exit criterion holds. Warm starts from
    /// whatever is currently in the solution buffer, and from the stored slack
    /// and multipliers on the general path.
    ///
    /// `tol` is the PRIMAL tolerance: ||U - proj(U - alpha grad f)||_inf in
    /// input units on the box path, ||C U - w||_inf in each row's own units on
    /// the general path. The general path's dual criterion is separate --
    /// admm.epsDualAbs / admm.epsDualRel -- because it is measured in gradient
    /// units and floors out around 1e-6 in float32 whatever the budget.
    MpcResult solve(const float x0[3], const float uMin[2], const float uMax[2],
                    int maxIters, float tol);

    /// Shift the solution one step forward, repeating the last move, and shift
    /// the constraint slack and multipliers with it. The standard
    /// real-time-iteration warm start: at the next tick the horizon has
    /// advanced by one, so u_1 is the new u_0.
    void shift();

    /// Clear the primal iterate and, on the general path, the ADMM slack w and
    /// multipliers y as well -- everything a warm start could carry. Leaves the
    /// constraint SPEC alone, including the rate anchor, which is caller data
    /// rather than solver state, so a reset does NOT free the head of a rate
    /// limit; clearConstraints() drops that.
    void zero();

    // ---- general constraints -------------------------------------------------
    //
    // The spec set here survives a re-condensing: the ROWS depend on G and are
    // rebuilt with it, but what the caller asked for does not change. Anything
    // structural marks the factorization stale, and it is redone on the next
    // solve -- i.e. once per horizon rebuild, never per iteration.

    /// Drop every general constraint. The next solve takes the box fast path.
    void clearConstraints();

    /// |(u_k + ref_k) - (u_{k-1} + ref_{k-1})| <= duMax, per axis, over the
    /// whole horizon, with the head of the horizon anchored by
    /// setPreviousInput(). A non-positive entry leaves that axis alone; both
    /// non-positive is the same as not calling this at all.
    void setInputRateLimit(const float duMax[2]);

    /// The decision-variable value the previous tick committed, i.e. the input
    /// it actually commanded minus THIS tick's reference. Per-tick data: it
    /// changes only the bounds, so it never triggers a refactorization.
    void setPreviousInput(const float uPrev[2]);

    /// ref_k - ref_{k-1} in decision-variable units, indexed [2*k + axis] for
    /// k = 1..N-1; the first block is unused. Null clears them, which reads the
    /// rate limit as a bound on the CORRECTION rather than on the command.
    ///
    /// These shape stages 1..N-1 of the plan only. The committed input is held
    /// to the rate limit by a hard clamp against the anchor in solve(), so
    /// clearing the deltas leaves max|delta command| exactly at the limit --
    /// they buy a plan that is consistent with that clamp, not the clamp itself.
    void setReferenceDeltas(const float* d, int count);

    /// lo <= a' x_k <= hi over stages [firstStage, lastStage], 1-based and
    /// inclusive, with lastStage <= 0 meaning the end of the horizon. Condenses
    /// into rows on the inputs: a'x_k = a'Phi_k x0 + (a'G_k) U, so the row is
    /// a'G_k and the free response moves the bounds.
    ///
    /// Returns false when the spec budget or the ROW budget is spent, counted
    /// against the horizon and rate limit in force right now. Acceptance means
    /// installation: a spec that returned true and then lost its rows would be
    /// a constraint the caller believes it has.
    bool addStateConstraint(const float a[3], float lo, float hi,
                            int firstStage = 1, int lastStage = 0);

    /// Rows the current spec set will generate at the current horizon: box,
    /// rate, and state. Compare against MPC_MAX_QP_ROWS.
    int plannedRows() const;

    bool hasGeneralConstraints() const {
        return mRateActive || mStateCount > 0;
    }
    int constraintRows() const { return mQp.rows(); }
    bool qpFactorized() const { return mQp.factorized(); }
    float qpRho() const { return mQp.rho(); }
    int stateConstraints() const { return mStateCount; }

    struct AdmmConfig {
        /// Explicit penalty. Zero derives it from the diagonals of H and C'C,
        /// then multiplies by rhoScale. rho is never touched inside the
        /// iteration -- doing so would force a refactorization -- so it is
        /// fixed for the life of a factorization.
        float rho{0.0f};
        /// 3, from a sweep over eight constraint shapes, scored on worst-case
        /// iterations to bring the committed move within 1e-4 of a converged
        /// double-precision reference. This knob owns the RATE-limited shapes: a
        /// rate limit alone goes 226 -> 64 and a tight one 479 -> 155. It does
        /// not help the corridor shapes, which is what rhoStateWeight is for.
        ///
        /// Together the two take the worst case over all eight shapes from 479
        /// to 155. rhoScale alone reaches 334, rhoStateWeight alone stays at 479
        /// -- the shape that sets the worst case at rhoScale 1 has no state rows
        /// at all, so neither knob is sufficient by itself.
        ///
        /// The accuracy axis moves much less than the iteration axis: worst
        /// error over the eight shapes at a four-iteration budget goes 1.64 ->
        /// 1.46, and at one iteration 2.14 -> 1.60. Chosen on iterations.
        float rhoScale{3.0f};
        /// Weight on rho for the STATE rows only. A corridor contributes rows
        /// that are nearly collinear from one stage to the next, so their C'C
        /// conditions the KKT matrix badly at the rho the box and rate rows
        /// want. 0.3 is where three measurements agree:
        ///
        ///  - worst-case iterations over the eight shapes are 155 at both 0.3
        ///    and 0.1 against 334 at 1, and the corridor shapes specifically go
        ///    334 -> 116 slack, 78 -> 34 binding, 138 -> 49 on a heading row;
        ///  - the committed move at a ONE-iteration budget on a corridor that
        ///    genuinely binds is 0.41 off the optimum at 0.3 and 0.82 at 1;
        ///  - but the PLAN's corridor excursion at that budget is 9.1e-4 m at
        ///    0.3 against 2.1e-3 at 0.1, so pushing further down trades a
        ///    prediction the caller reads for nothing more in iterations.
        ///
        /// Not below 0.1: at 0.01 the corridor stops being enforced, and on
        /// problems where it binds the solve then fails to converge at all
        /// within 2000 iterations.
        float rhoStateWeight{0.3f};
        /// Proximal regularisation. Keeps the KKT matrix definite even when H
        /// is only semidefinite.
        float sigma{1e-6f};
        /// Over-relaxation, OSQP's default. Must lie in (0, 2).
        float alpha{1.6f};
        /// Residuals cost an H matvec, so they are not evaluated every
        /// iteration. The cap is always checked.
        int checkEvery{4};
        /// Dual exit criterion, eps = epsDualAbs + epsDualRel * dualScale. The
        /// dual residual lives in gradient units, not the primal's row units,
        /// and a dense float32 H matvec cancels at around 1e-6 however long the
        /// iteration runs -- an absolute dual tolerance under that never
        /// terminates. Non-positive epsDualAbs uses solve()'s own tol.
        float epsDualAbs{0.0f};
        float epsDualRel{1e-3f};
    };
    AdmmConfig admm;

    const float* solution() const { return mU; }
    void firstMove(float u[2]) const { u[0] = mU[0]; u[1] = mU[1]; }

    int horizon() const { return mN; }
    int vars() const { return 2 * mN; }
    float lipschitz() const { return mL; }
    int powerIterations() const { return mPowerIters; }
    MpcSolver solver{MpcSolver::FastGradient};

    /// Certify convergence against a residual instead of trusting the step
    /// norm. Costs one extra H matvec per solve on the box path and one per
    /// check on the ADMM path, so a fixed-budget real-time iteration turns it
    /// off -- and then reports converged = false, because it does not know.
    bool certifyOptimality{true};

    /// 1/2 U' H U + (F x0)' U for the current solution. Requires solve() to
    /// have run at this x0, since F x0 is cached there.
    float objective() const;

    /// Analytic flop counts, for cost projection onto hardware that cannot be
    /// profiled directly. Multiply-and-add counted as two flops.
    long flopsPerIteration() const;
    long flopsSetup() const;

private:
    enum class RowKind : unsigned char { Box, RateHead, Rate, State };

    struct StateSpec {
        float a[3]{};
        float lo{0.0f};
        float hi{0.0f};
        int first{1};
        int last{0};
    };

    /// ||U - proj(U - alpha grad f(U))||_inf at the current solution.
    float stationarityInf(const float uMin[2], const float uMax[2]) const;

    /// Build H, F and the step size from whatever is in mPhi and mG.
    void assemble(const MpcWeights& w, const float terminalP[3][3],
                  const float inputR[2][2]);

    MpcResult solveBox(const float uMin[2], const float uMax[2], int maxIters, float tol);
    MpcResult solveAdmm(const float x0[3], const float uMin[2], const float uMax[2],
                        int maxIters, float tol);

    /// Rebuild C from the current condensing and refactor. Called lazily, so
    /// the cost lands on the first solve after a rebuild and nowhere else.
    void rebuildQp();
    void fillQpBounds(const float x0[3], const float uMin[2], const float uMax[2]);

    /// Rows a spec occupies at the current horizon.
    int rowsForSpec(const StateSpec& s) const;

    /// How far each state spec's prediction falls outside its own [lo, hi], on
    /// the solution as RETURNED -- after solve()'s output clamps -- so it
    /// describes the plan the caller gets and not the solver's raw iterate.
    /// Costs the state rows' nonzeros, nothing when there are no state specs.
    void measureStateViolation(const float x0[3], MpcResult& res) const;

    /// Order-preserving identity of a row, so the multipliers can be carried
    /// across a re-condensing by what a row MEANS rather than by its index.
    /// Monotone in generation order, which makes the carry-over a merge scan.
    /// `axis` is the spec index for a State row -- two specs can name the same
    /// stage, and the kind and stage alone would collide.
    static int rowKey(unsigned char kind, unsigned char axis, short stage);

    int mN{0};
    int mAssembledVars{0};
    float mH[MPC_MAX_VARS][MPC_MAX_VARS]{};
    float mF[MPC_MAX_VARS][3]{};
    float mDiagInv[MPC_MAX_VARS]{};
    float mL{1.0f};
    float mStep{1.0f};
    int mPowerIters{0};

    // Condensing scratch. Members rather than locals so setup() does not put
    // ~10 KB on a task stack.
    float mPhi[3 * MPC_MAX_HORIZON][3]{};
    float mG[3 * MPC_MAX_HORIZON][MPC_MAX_VARS]{};

    // Solver state.
    float mU[MPC_MAX_VARS]{};
    float mY[MPC_MAX_VARS]{};
    float mUprev[MPC_MAX_VARS]{};
    float mGrad[MPC_MAX_VARS]{};
    float mLin[MPC_MAX_VARS]{};    // F * x0

    // General-constraint state.
    AdmmQp<MPC_MAX_VARS, MPC_MAX_QP_ROWS> mQp;
    StateSpec mStateSpec[MPC_MAX_STATE_CONSTRAINTS]{};
    int mStateCount{0};
    bool mRateActive{false};
    bool mQpDirty{true};
    float mRateMax[2]{0.0f, 0.0f};
    float mRateAnchor[2]{0.0f, 0.0f};
    float mRefDelta[MPC_MAX_VARS]{};

    unsigned char mRowKind[MPC_MAX_QP_ROWS]{};
    unsigned char mRowAxis[MPC_MAX_QP_ROWS]{};
    short mRowStage[MPC_MAX_QP_ROWS]{};
    short mRowShiftFrom[MPC_MAX_QP_ROWS]{};
    float mRowLo[MPC_MAX_QP_ROWS]{};
    float mRowHi[MPC_MAX_QP_ROWS]{};
    float mRowPhi[MPC_MAX_QP_ROWS][3]{};
    float mRowScratch[MPC_MAX_VARS]{};
    float mRowRho[MPC_MAX_QP_ROWS]{};

    // Multipliers and slack carried across a re-condensing, keyed on row
    // identity. Members rather than locals: rebuildQp() runs on a control task.
    int mSavedKey[MPC_MAX_QP_ROWS]{};
    float mSavedW[MPC_MAX_QP_ROWS]{};
    float mSavedY[MPC_MAX_QP_ROWS]{};
    int mDroppedRows{0};
};

}  // namespace chad
