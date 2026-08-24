#include "chad/movement_control/mpc.hpp"

#include "chad/movement_control/mpc_lqr.hpp"
#include "chad/movement_control/riccati.hpp"
#include "chad_test.hpp"

using namespace chad;

namespace {

constexpr float kFree = 1.0e6f;

MpcWeights weights() { return {1.0f, 1.0f, 2.0f, 0.6f, 0.4f}; }

void lqrMove(const LqrSolution& lqr, const float x[3], float u[2]) {
    for (int i = 0; i < 2; i++) {
        u[i] = -(lqr.K[i][0] * x[0] + lqr.K[i][1] * x[1] + lqr.K[i][2] * x[2]);
    }
}

/// Deliberately slow reference: many projected Gauss-Seidel sweeps on the same
/// QP, independent of whichever solver is under test.
void referenceSolve(CondensedMpc& mpc, const float x0[3], const float uMin[2],
                    const float uMax[2], float out[2]) {
    const MpcSolver saved = mpc.solver;
    mpc.solver = MpcSolver::GaussSeidel;
    mpc.zero();
    mpc.solve(x0, uMin, uMax, 20000, 0.0f);
    mpc.firstMove(out);
    mpc.solver = saved;
}

}  // namespace

CHAD_TEST(mpc, unconstrainedFirstMoveEqualsLqr) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.2f, 0.02f);
    const MpcWeights w = weights();
    const LqrSolution lqr = solveDiscreteLqr(plant, w, 4000, 1e-9f);
    CHECK(lqr.converged);

    CondensedMpc mpc;
    mpc.setup(plant, w, 12, lqr.P);

    const float uMin[2] = {-kFree, -kFree};
    const float uMax[2] = {kFree, kFree};
    const float x0[3] = {0.10f, 0.20f, 0.05f};

    mpc.zero();
    const MpcResult res = mpc.solve(x0, uMin, uMax, 5000, 1e-7f);
    CHECK(res.converged);
    CHECK(res.kktResidual >= 0.0f);
    CHECK(res.kktResidual < 1e-7f);

    float mpcU[2], lqrU[2];
    mpc.firstMove(mpcU);
    lqrMove(lqr, x0, lqrU);
    CHECK_NEAR(mpcU[0], lqrU[0], 2e-3);
    CHECK_NEAR(mpcU[1], lqrU[1], 2e-3);
}

CHAD_TEST(mpc, lqrEquivalenceHoldsAtEveryHorizon) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(0.9f, 0.02f);
    const MpcWeights w = weights();
    const LqrSolution lqr = solveDiscreteLqr(plant, w, 4000, 1e-9f);
    const float uMin[2] = {-kFree, -kFree};
    const float uMax[2] = {kFree, kFree};
    const float x0[3] = {-0.05f, 0.12f, -0.03f};

    float lqrU[2];
    lqrMove(lqr, x0, lqrU);

    for (int horizon : {1, 2, 5, 15}) {
        CondensedMpc mpc;
        mpc.setup(plant, w, horizon, lqr.P);
        mpc.zero();
        mpc.solve(x0, uMin, uMax, 5000, 1e-9f);
        float u[2];
        mpc.firstMove(u);
        CHECK_NEAR(u[0], lqrU[0], 3e-3);
        CHECK_NEAR(u[1], lqrU[1], 3e-3);
    }
}

CHAD_TEST(mpc, zeroStateGivesZeroInput) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 8);
    const float uMin[2] = {-1.0f, -1.0f};
    const float uMax[2] = {1.0f, 1.0f};
    const float x0[3] = {0.0f, 0.0f, 0.0f};
    mpc.zero();
    mpc.solve(x0, uMin, uMax, 500, 1e-9f);
    float u[2];
    mpc.firstMove(u);
    CHECK_NEAR(u[0], 0.0f, 1e-6);
    CHECK_NEAR(u[1], 0.0f, 1e-6);
}

CHAD_TEST(mpc, boundsAreRespectedAndCounted) {
    // Box feasibility on its own tests the unconditional clamp on the way out
    // of solve(), not the solver -- a solver returning uniform garbage would
    // pass it. So this also fixes WHICH bound: the state is positive on every
    // axis, so every correction has to saturate at its LOWER bound, and the
    // whole sequence has to match an independent solve.
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 10);

    const float uMin[2] = {-0.05f, -0.10f};
    const float uMax[2] = {0.05f, 0.10f};
    const float x0[3] = {2.0f, 2.0f, 1.0f};

    mpc.zero();
    const MpcResult res = mpc.solve(x0, uMin, uMax, 2000, 1e-9f);

    const float* u = mpc.solution();
    for (int i = 0; i < mpc.vars(); i++) {
        const float lo = (i % 2 == 0) ? uMin[0] : uMin[1];
        const float hi = (i % 2 == 0) ? uMax[0] : uMax[1];
        CHECK(u[i] >= lo - 1e-6f);
        CHECK(u[i] <= hi + 1e-6f);
        // WHICH bound: the error is positive on every axis, so every correction
        // is negative. A solver returning uniform garbage fails here.
        CHECK(u[i] < 0.0f);
    }
    CHECK(res.activeBounds >= mpc.vars() - 2);

    CondensedMpc gs;
    gs.solver = MpcSolver::GaussSeidel;
    gs.setup(plant, weights(), 10);
    gs.zero();
    gs.solve(x0, uMin, uMax, 20000, 0.0f);
    for (int i = 0; i < mpc.vars(); i++) CHECK_NEAR(u[i], gs.solution()[i], 1e-6);
}

CHAD_TEST(mpc, solversAgreeOnTheConstrainedOptimum) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.1f, 0.02f);
    const MpcWeights w = weights();
    const float uMin[2] = {-0.30f, -0.60f};
    const float uMax[2] = {0.30f, 0.60f};
    const float x0[3] = {0.4f, -0.5f, 0.25f};

    CondensedMpc gs;
    gs.solver = MpcSolver::GaussSeidel;
    gs.setup(plant, w, 10);
    gs.zero();
    gs.solve(x0, uMin, uMax, 20000, 0.0f);
    float uGs[2];
    gs.firstMove(uGs);

    CondensedMpc fg;
    fg.solver = MpcSolver::FastGradient;
    fg.setup(plant, w, 10);
    fg.zero();
    const MpcResult res = fg.solve(x0, uMin, uMax, 20000, 1e-9f);
    float uFg[2];
    fg.firstMove(uFg);

    CHECK_NEAR(uFg[0], uGs[0], 1e-3);
    CHECK_NEAR(uFg[1], uGs[1], 1e-3);
    CHECK(res.kktResidual < 1e-5f);
}

CHAD_TEST(mpc, matchesAnIndependentReferenceSolve) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(0.7f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 8);

    const float uMin[2] = {-0.2f, -0.4f};
    const float uMax[2] = {0.2f, 0.4f};
    const float x0[3] = {0.3f, 0.6f, -0.2f};

    float reference[2];
    referenceSolve(mpc, x0, uMin, uMax, reference);

    mpc.zero();
    mpc.solve(x0, uMin, uMax, 5000, 1e-9f);
    float u[2];
    mpc.firstMove(u);
    CHECK_NEAR(u[0], reference[0], 1e-3);
    CHECK_NEAR(u[1], reference[1], 1e-3);
}

CHAD_TEST(mpc, objectiveDecreasesWithBudget) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.3f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 12);

    const float uMin[2] = {-0.25f, -0.5f};
    const float uMax[2] = {0.25f, 0.5f};
    const float x0[3] = {0.5f, 0.4f, 0.2f};

    float previous = 0.0f;
    for (int budget : {1, 2, 5, 20, 100, 500}) {
        mpc.zero();
        mpc.solve(x0, uMin, uMax, budget, 0.0f);
        const float f = mpc.objective();
        if (budget > 1) CHECK(f <= previous + 1e-5f);
        previous = f;
    }
    CHECK(previous < 0.0f);
}

CHAD_TEST(mpc, warmStartReachesTheSameOptimum) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 10);

    const float uMin[2] = {-0.2f, -0.4f};
    const float uMax[2] = {0.2f, 0.4f};
    const float x0[3] = {0.25f, -0.3f, 0.1f};

    mpc.zero();
    mpc.solve(x0, uMin, uMax, 5000, 1e-9f);
    float cold[2];
    mpc.firstMove(cold);

    mpc.shift();
    mpc.solve(x0, uMin, uMax, 5000, 1e-9f);
    float warm[2];
    mpc.firstMove(warm);

    CHECK_NEAR(warm[0], cold[0], 1e-4);
    CHECK_NEAR(warm[1], cold[1], 1e-4);
}

CHAD_TEST(mpc, warmStartCostsFewerIterations) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 10);

    const float uMin[2] = {-0.2f, -0.4f};
    const float uMax[2] = {0.2f, 0.4f};
    const float x0[3] = {0.25f, -0.3f, 0.1f};

    mpc.zero();
    const MpcResult cold = mpc.solve(x0, uMin, uMax, 5000, 1e-7f);
    const MpcResult warm = mpc.solve(x0, uMin, uMax, 5000, 1e-7f);
    // A margin, not `<=`: re-solving from the answer must be nearly free, and
    // `warm <= cold` is satisfied by a warm start that does nothing at all.
    CHECK(cold.iterations > 8);
    CHECK(warm.iterations * 4 <= cold.iterations);
    CHECK(warm.converged);
}

CHAD_TEST(mpc, horizonIsClampedNotOverrun) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 9999);
    CHECK_EQ(mpc.horizon(), MPC_MAX_HORIZON);
    CHECK_EQ(mpc.vars(), 2 * MPC_MAX_HORIZON);

    mpc.setup(plant, weights(), 0);
    CHECK_EQ(mpc.horizon(), 1);
}

CHAD_TEST(mpc, certificationCanBeDisabled) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 6);
    mpc.certifyOptimality = false;

    const float uMin[2] = {-0.2f, -0.4f};
    const float uMax[2] = {0.2f, 0.4f};
    const float x0[3] = {0.2f, 0.1f, 0.05f};
    mpc.zero();
    const MpcResult res = mpc.solve(x0, uMin, uMax, 1, 0.0f);
    CHECK(res.kktResidual < 0.0f);
    CHECK_EQ(res.iterations, 1);
}

CHAD_TEST(mpc, stepNormAloneDoesNotClaimConvergence) {
    // The step norm can go small while the answer is still wrong: FISTA's
    // momentum stalls. Measured against an INDEPENDENT optimum, not against the
    // solver's own exit condition, so a `converged = (stepInf <= tol)` would
    // fail here rather than restating itself.
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    // Almost no input weight, so H is dominated by G'QG and badly conditioned:
    // coordinate descent then takes tiny steps a long way from the optimum,
    // which is the regime where the step norm is a liar.
    const MpcWeights w{1.0f, 1.0f, 2.0f, 1e-4f, 1e-4f};
    const float uMin[2] = {-0.20f, -0.40f};
    const float uMax[2] = {0.20f, 0.40f};
    const float x0[3] = {0.10f, 0.10f, 0.05f};
    const float tol = 1e-3f;

    CondensedMpc reference;
    reference.solver = MpcSolver::GaussSeidel;
    reference.setup(plant, w, 20);
    reference.zero();
    reference.solve(x0, uMin, uMax, 20000, 0.0f);   // spend everything
    float truth[MPC_MAX_VARS];
    for (int i = 0; i < reference.vars(); i++) truth[i] = reference.solution()[i];

    CondensedMpc mpc;
    mpc.setup(plant, w, 20);
    mpc.solver = MpcSolver::GaussSeidel;

    // The step norm reaches the tolerance and stops the solve early. That must
    // not become a verdict: nothing was checked, so nothing is known.
    mpc.certifyOptimality = false;
    mpc.zero();
    const MpcResult blind = mpc.solve(x0, uMin, uMax, 400, tol);
    CHECK(blind.iterations < 400);        // the step norm really did stop it
    CHECK(blind.stepInf <= tol);
    CHECK(!blind.converged);
    CHECK(blind.kktResidual < 0.0f);

    // A budget too small to be optimal must not certify, and the answer at that
    // budget has to be measurably wrong -- otherwise the assertion above is
    // free.
    mpc.certifyOptimality = true;
    mpc.zero();
    const MpcResult stub = mpc.solve(x0, uMin, uMax, 1, 0.0f);
    CHECK(!stub.converged);
    CHECK(stub.hitIterationCap);
    float worst = 0.0f;
    for (int i = 0; i < mpc.vars(); i++) {
        worst = std::fmax(worst, std::fabs(mpc.solution()[i] - truth[i]));
    }
    CHECK(worst > 1e-3f);

    // And when it does certify, the answer agrees with the independent solve.
    mpc.zero();
    const MpcResult done = mpc.solve(x0, uMin, uMax, 20000, 1e-8f);
    CHECK(done.converged);
    CHECK(done.kktResidual >= 0.0f);
    CHECK(done.kktResidual <= 1e-8f);
    for (int i = 0; i < mpc.vars(); i++) CHECK_NEAR(mpc.solution()[i], truth[i], 1e-4);
}

CHAD_TEST(mpc, wheelSpaceInputsGiveTheSameTwist) {
    const float trackWidth = 0.30f;
    const DrivetrainPlant twist = DrivetrainPlant::discretize(1.0f, 0.02f);
    const DrivetrainPlant wheel = twist.withWheelInputs(trackWidth);

    const MpcWeights w = weights();
    const float x0[3] = {0.15f, 0.25f, 0.08f};
    const float uMin[2] = {-kFree, -kFree};
    const float uMax[2] = {kFree, kFree};

    CondensedMpc twistMpc;
    twistMpc.setup(twist, w, 10);
    twistMpc.zero();
    twistMpc.solve(x0, uMin, uMax, 5000, 1e-9f);
    float uTwist[2];
    twistMpc.firstMove(uTwist);

    // Transformed input weight: T' diag(rv, rw) T for the same physical cost.
    const float invB = 1.0f / trackWidth;
    const float t00 = 0.5f, t01 = 0.5f;
    const float t10 = -invB, t11 = invB;
    const float R[2][2] = {
        {w.rv * t00 * t00 + w.rw * t10 * t10, w.rv * t00 * t01 + w.rw * t10 * t11},
        {w.rv * t01 * t00 + w.rw * t11 * t10, w.rv * t01 * t01 + w.rw * t11 * t11}};

    CondensedMpc wheelMpc;
    wheelMpc.setup(wheel, w, 10, nullptr, R);
    wheelMpc.zero();
    wheelMpc.solve(x0, uMin, uMax, 5000, 1e-9f);
    float uWheel[2];
    wheelMpc.firstMove(uWheel);

    const float dv = 0.5f * (uWheel[0] + uWheel[1]);
    const float dw = invB * (uWheel[1] - uWheel[0]);
    CHECK_NEAR(dv, uTwist[0], 5e-3);
    CHECK_NEAR(dw, uTwist[1], 5e-2);
}

CHAD_TEST(mpc, dareDoublingAgreesWithProductionValueIteration) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.25f, 0.02f);
    const MpcWeights w = weights();
    const LqrSolution reference = solveDiscreteLqr(plant, w, 8000, 1e-10f);
    CHECK(reference.converged);

    const float Q[3][3] = {{w.qx, 0.0f, 0.0f}, {0.0f, w.qy, 0.0f}, {0.0f, 0.0f, w.qTheta}};
    const float R[2][2] = {{w.rv, 0.0f}, {0.0f, w.rw}};
    const DareSolution<3, 2> doubled = solveDare<3, 2>(plant.Ad, plant.Bd, Q, R);
    CHECK(doubled.converged);
    CHECK(doubled.iterations < reference.iterations);

    const float scale = linalg::maxAbs<3, 3>(reference.P);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            CHECK_NEAR(doubled.P[i][j], reference.P[i][j], 5e-3 * scale + 1e-4);
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 3; j++)
            CHECK_NEAR(doubled.K[i][j], reference.K[i][j], 5e-3 * (std::fabs(reference.K[i][j]) + 1.0f));
}

namespace {

/// Backward Riccati sweep over a stage-varying horizon. Returns the gain that
/// applies at time zero, which is what the MPC's first move must equal when no
/// constraint is active.
void tvlqrFirstGain(const DrivetrainPlant* stages, int n, const float Q[3][3],
                    const float R[2][2], const float terminalP[3][3], float K0[2][3]) {
    float S[3][3];
    linalg::copy<3, 3>(terminalP, S);

    for (int k = n - 1; k >= 0; k--) {
        const DrivetrainPlant& stage = stages[k];

        float sa[3][3];
        linalg::multiply<3, 3, 3>(S, stage.Ad, sa);
        float atsa[3][3];
        linalg::multiplyTransposedA<3, 3, 3>(stage.Ad, sa, atsa);

        float bts[2][3];
        linalg::multiplyTransposedA<2, 3, 3>(stage.Bd, S, bts);
        float m[2][2];
        linalg::multiply<2, 2, 3>(bts, stage.Bd, m);
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++) m[i][j] += R[i][j];
        linalg::symmetrise<2>(m);

        float minv[2][2];
        linalg::inverse<2>(m, minv);

        float btsa[2][3];
        linalg::multiply<2, 3, 3>(bts, stage.Ad, btsa);
        float k_stage[2][3];
        linalg::multiply<2, 3, 2>(minv, btsa, k_stage);

        float correction[3][3];
        linalg::multiplyTransposedA<3, 3, 2>(btsa, k_stage, correction);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                S[i][j] = Q[i][j] + atsa[i][j] - correction[i][j];
        linalg::symmetrise<3>(S);

        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 3; j++) K0[i][j] = k_stage[i][j];
    }
}

}  // namespace

CHAD_TEST(mpc, varyingSetupMatchesConstantSetupWhenStagesAgree) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.1f, 0.02f);
    const MpcWeights w = weights();
    const int horizon = 8;

    DrivetrainPlant stages[8];
    for (int i = 0; i < horizon; i++) stages[i] = plant;

    CondensedMpc constant;
    constant.setup(plant, w, horizon);

    CondensedMpc varying;
    varying.setupVarying(stages, horizon, w);

    const float uMin[2] = {-0.3f, -0.6f};
    const float uMax[2] = {0.3f, 0.6f};
    const float x0[3] = {0.3f, 0.4f, 0.15f};

    constant.zero();
    constant.solve(x0, uMin, uMax, 4000, 1e-7f);
    varying.zero();
    varying.solve(x0, uMin, uMax, 4000, 1e-7f);

    CHECK_EQ(varying.horizon(), constant.horizon());
    for (int i = 0; i < constant.vars(); i++) {
        CHECK_NEAR(varying.solution()[i], constant.solution()[i], 1e-4);
    }
}

CHAD_TEST(mpc, varyingHorizonMatchesTheTimeVaryingLqr) {
    const int horizon = 10;
    DrivetrainPlant stages[10];
    // A reference braking into the end of a path: the case a single-speed
    // condensing cannot represent.
    for (int i = 0; i < horizon; i++) {
        const float v = 1.6f - 0.12f * static_cast<float>(i);
        stages[i] = DrivetrainPlant::discretize(v, 0.02f);
    }

    const MpcWeights w = weights();
    const float Q[3][3] = {{w.qx, 0.0f, 0.0f}, {0.0f, w.qy, 0.0f}, {0.0f, 0.0f, w.qTheta}};
    const float R[2][2] = {{w.rv, 0.0f}, {0.0f, w.rw}};
    const DareSolution<3, 2> terminal =
        solveDare<3, 2>(stages[horizon - 1].Ad, stages[horizon - 1].Bd, Q, R);
    CHECK(terminal.converged);

    float K0[2][3]{};
    tvlqrFirstGain(stages, horizon, Q, R, terminal.P, K0);

    CondensedMpc mpc;
    mpc.setupVarying(stages, horizon, w, terminal.P);

    const float uMin[2] = {-kFree, -kFree};
    const float uMax[2] = {kFree, kFree};
    const float x0[3] = {0.08f, 0.22f, -0.06f};
    mpc.zero();
    const MpcResult res = mpc.solve(x0, uMin, uMax, 8000, 1e-7f);
    CHECK(res.converged);

    float u[2];
    mpc.firstMove(u);
    for (int i = 0; i < 2; i++) {
        const float expected = -(K0[i][0] * x0[0] + K0[i][1] * x0[1] + K0[i][2] * x0[2]);
        CHECK_NEAR(u[i], expected, 3e-3);
    }
}

CHAD_TEST(mpc, varyingHorizonDiffersFromItsAverageSpeed) {
    const int horizon = 10;
    DrivetrainPlant stages[10];
    float sum = 0.0f;
    for (int i = 0; i < horizon; i++) {
        const float v = 2.0f - 0.18f * static_cast<float>(i);
        stages[i] = DrivetrainPlant::discretize(v, 0.02f);
        sum += v;
    }

    const MpcWeights w = weights();
    const float uMin[2] = {-kFree, -kFree};
    const float uMax[2] = {kFree, kFree};
    const float x0[3] = {0.0f, 0.30f, 0.0f};

    CondensedMpc varying;
    varying.setupVarying(stages, horizon, w);
    varying.zero();
    varying.solve(x0, uMin, uMax, 8000, 1e-7f);
    float uVarying[2];
    varying.firstMove(uVarying);

    CondensedMpc averaged;
    averaged.setup(DrivetrainPlant::discretize(sum / horizon, 0.02f), w, horizon);
    averaged.zero();
    averaged.solve(x0, uMin, uMax, 8000, 1e-7f);
    float uAveraged[2];
    averaged.firstMove(uAveraged);

    CHECK(std::fabs(uVarying[1] - uAveraged[1]) > 1e-4f);
}

CHAD_TEST(mpc, varyingSetupRejectsDegenerateInput) {
    CondensedMpc mpc;
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    mpc.setup(plant, weights(), 5);
    const int before = mpc.horizon();

    mpc.setupVarying(nullptr, 4, weights());
    CHECK_EQ(mpc.horizon(), before);
    mpc.setupVarying(&plant, 0, weights());
    CHECK_EQ(mpc.horizon(), before);

    DrivetrainPlant many[MPC_MAX_HORIZON];
    for (int i = 0; i < MPC_MAX_HORIZON; i++) many[i] = plant;
    mpc.setupVarying(many, MPC_MAX_HORIZON + 50, weights());
    CHECK_EQ(mpc.horizon(), MPC_MAX_HORIZON);
}

// ===========================================================================
// General constraints
// ===========================================================================

namespace {

/// Roll the plant forward with a solved input sequence and return the worst
/// |x_k[axis]| over stages 1..n. Independent of the condensing: it uses only
/// Ad and Bd, so it catches a G that was built wrong.
float peakStateOverHorizon(const DrivetrainPlant& p, const float x0[3], const float* u,
                           int n, int axis) {
    float x[3] = {x0[0], x0[1], x0[2]};
    float worst = 0.0f;
    for (int k = 0; k < n; k++) {
        float nx[3];
        for (int r = 0; r < 3; r++) {
            nx[r] = p.Ad[r][0] * x[0] + p.Ad[r][1] * x[1] + p.Ad[r][2] * x[2] +
                    p.Bd[r][0] * u[2 * k + 0] + p.Bd[r][1] * u[2 * k + 1];
        }
        for (int r = 0; r < 3; r++) x[r] = nx[r];
        const float m = std::fabs(x[axis]);
        if (m > worst) worst = m;
    }
    return worst;
}

/// |x_k[axis]| stage by stage, 1-based: stage(k) for k = 1..n.
float stateAtStage(const DrivetrainPlant& p, const float x0[3], const float* u,
                   int stage, int axis) {
    float x[3] = {x0[0], x0[1], x0[2]};
    for (int k = 0; k < stage; k++) {
        float nx[3];
        for (int r = 0; r < 3; r++) {
            nx[r] = p.Ad[r][0] * x[0] + p.Ad[r][1] * x[1] + p.Ad[r][2] * x[2] +
                    p.Bd[r][0] * u[2 * k + 0] + p.Bd[r][1] * u[2 * k + 1];
        }
        for (int r = 0; r < 3; r++) x[r] = nx[r];
    }
    return std::fabs(x[axis]);
}

// ---------------------------------------------------------------------------
// Independent reference for the CONDENSED QP: the condensing rebuilt in double
// from the ZOH discretisation by matrix recursion (not the closed forms
// DrivetrainPlant uses), and the constrained optimum found by exhaustive
// active-set enumeration. Every row is tried inactive, at its lower bound and
// at its upper bound; each combination is an equality-constrained KKT system,
// and the first candidate that is primal feasible with the right multiplier
// signs is the unique global optimum because H is positive definite.
//
// Shares no code with CondensedMpc or AdmmQp. Small on purpose -- N = 3 keeps
// 3^12 active sets tractable.
// ---------------------------------------------------------------------------

constexpr int EN = 6;    // decision variables, 2 * N
constexpr int EM = 12;   // rows

struct EnumQp {
    int n{EN};
    int m{0};
    double H[EN][EN]{};
    double q[EN]{};
    double C[EM][EN]{};
    double l[EM]{};
    double u[EM]{};
};

bool enumSolveLinear(int n, double A[EN + EM][EN + EM], double* b, double* x) {
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

bool enumerateOptimum(const EnumQp& p, double z[EN]) {
    int total = 1;
    for (int r = 0; r < p.m; r++) total *= 3;

    for (int code = 0; code < total; code++) {
        int act[EM]{}, side[EM]{};
        int na = 0;
        int c = code;
        bool overflow = false;
        for (int r = 0; r < p.m; r++) {
            const int st = c % 3;
            c /= 3;
            if (st == 0) continue;
            if (na >= p.n) { overflow = true; break; }
            act[na] = r;
            side[na] = st;
            na++;
        }
        if (overflow) continue;

        const int dim = p.n + na;
        double A[EN + EM][EN + EM]{};
        double b[EN + EM]{}, x[EN + EM]{};
        for (int i = 0; i < p.n; i++) {
            for (int j = 0; j < p.n; j++) A[i][j] = p.H[i][j];
            b[i] = -p.q[i];
        }
        for (int k = 0; k < na; k++) {
            for (int j = 0; j < p.n; j++) {
                A[j][p.n + k] = p.C[act[k]][j];
                A[p.n + k][j] = p.C[act[k]][j];
            }
            b[p.n + k] = (side[k] == 1) ? p.l[act[k]] : p.u[act[k]];
        }
        if (!enumSolveLinear(dim, A, b, x)) continue;

        bool ok = true;
        for (int r = 0; r < p.m && ok; r++) {
            double cz = 0.0;
            for (int j = 0; j < p.n; j++) cz += p.C[r][j] * x[j];
            if (cz < p.l[r] - 1e-9 || cz > p.u[r] + 1e-9) ok = false;
        }
        for (int k = 0; k < na && ok; k++) {
            const double lam = x[p.n + k];
            if (side[k] == 2 && lam < -1e-9) ok = false;
            if (side[k] == 1 && lam > 1e-9) ok = false;
        }
        if (!ok) continue;

        for (int j = 0; j < p.n; j++) z[j] = x[j];
        return true;
    }
    return false;
}

/// Condense in double by recursion. Phi[k] = Ad^(k+1) and Gb[k][j] is the 3x2
/// block multiplying u_j in x_{k+1}.
struct Condensed3 {
    double Phi[3][3][3]{};
    double Gb[3][3][3][2]{};
};

Condensed3 condenseByRecursion(double v, double dt) {
    const double Ad[3][3] = {{1, 0, 0}, {0, 1, v * dt}, {0, 0, 1}};
    const double Bd[3][2] = {{dt, 0}, {0, 0.5 * v * dt * dt}, {0, dt}};
    Condensed3 c;
    for (int k = 0; k < 3; k++) {
        if (k == 0) {
            for (int r = 0; r < 3; r++)
                for (int j = 0; j < 3; j++) c.Phi[0][r][j] = Ad[r][j];
        } else {
            for (int r = 0; r < 3; r++)
                for (int j = 0; j < 3; j++) {
                    double s = 0.0;
                    for (int m = 0; m < 3; m++) s += Ad[r][m] * c.Phi[k - 1][m][j];
                    c.Phi[k][r][j] = s;
                }
            for (int j = 0; j < k; j++)
                for (int r = 0; r < 3; r++)
                    for (int a = 0; a < 2; a++) {
                        double s = 0.0;
                        for (int m = 0; m < 3; m++) s += Ad[r][m] * c.Gb[k - 1][j][m][a];
                        c.Gb[k][j][r][a] = s;
                    }
        }
        for (int r = 0; r < 3; r++)
            for (int a = 0; a < 2; a++) c.Gb[k][k][r][a] = Bd[r][a];
    }
    return c;
}

/// Worst |u_k - u_{k-1} + dRef_k| on one axis, with stage 0 measured against
/// the anchor. `dRef` may be null.
float worstRate(const float* u, int n, int axis, float anchor, const float* dRef) {
    float worst = std::fabs(u[axis] - anchor);
    for (int k = 1; k < n; k++) {
        const float d = (dRef != nullptr) ? dRef[2 * k + axis] : 0.0f;
        const float m = std::fabs(u[2 * k + axis] - u[2 * (k - 1) + axis] + d);
        if (m > worst) worst = m;
    }
    return worst;
}

}  // namespace

CHAD_TEST(mpc, constrainedCondensedOptimumMatchesActiveSetEnumeration) {
    // The gap every other constrained test leaves: box + input-rate + state
    // corridor, all three at once, checked against a solve that shares no code
    // with the condensing or the solver. Agreement on all six decision
    // variables pins H, F, the state row a'G_k, the free response a'Phi_k x0,
    // the rate anchor and the whole ADMM iteration simultaneously.
    const double v = 1.5, dt = 0.10;
    const int N = 3;
    const MpcWeights w{1.0f, 0.05f, 2.0f, 0.6f, 0.4f};
    const float uMin[2] = {-0.30f, -3.00f};
    const float uMax[2] = {0.30f, 3.00f};
    const float x0[3] = {0.10f, 0.0f, 0.20f};
    const float du[2] = {0.0f, 1.3f};      // angular axis only: 3 rate rows
    const float anchor[2] = {0.0f, -1.0f};
    const float corridor = 0.0155f;
    const float a[3] = {0.0f, 1.0f, 0.0f};

    const Condensed3 c = condenseByRecursion(v, dt);
    const double Q[3] = {w.qx, w.qy, w.qTheta};
    const double R[2] = {w.rv, w.rw};

    EnumQp p;
    p.n = 2 * N;
    // H = 2 (G' Qbar G + Rbar), Qbar = diag(Q) on every stage.
    for (int i = 0; i < p.n; i++) {
        for (int j = 0; j < p.n; j++) {
            double s = 0.0;
            for (int k = 0; k < N; k++) {
                for (int r = 0; r < 3; r++) {
                    s += Q[r] * c.Gb[k][i / 2][r][i % 2] * c.Gb[k][j / 2][r][j % 2];
                }
            }
            if (i == j) s += R[i % 2];
            p.H[i][j] = 2.0 * s;
        }
        double s = 0.0;
        for (int k = 0; k < N; k++) {
            for (int r = 0; r < 3; r++) {
                double free = 0.0;
                for (int cc = 0; cc < 3; cc++) free += c.Phi[k][r][cc] * x0[cc];
                s += Q[r] * c.Gb[k][i / 2][r][i % 2] * free;
            }
        }
        p.q[i] = 2.0 * s;
    }

    int m = 0;
    for (int i = 0; i < p.n; i++) {          // box
        p.C[m][i] = 1.0;
        p.l[m] = uMin[i % 2];
        p.u[m] = uMax[i % 2];
        m++;
    }
    for (int k = 0; k < N; k++) {            // rate, angular axis
        if (k == 0) {
            p.C[m][1] = 1.0;
            p.l[m] = anchor[1] - du[1];
            p.u[m] = anchor[1] + du[1];
        } else {
            p.C[m][2 * k + 1] = 1.0;
            p.C[m][2 * (k - 1) + 1] = -1.0;
            p.l[m] = -du[1];
            p.u[m] = du[1];
        }
        m++;
    }
    for (int k = 0; k < N; k++) {            // corridor on cross-track
        for (int j = 0; j <= k; j++) {
            for (int ax = 0; ax < 2; ax++) p.C[m][2 * j + ax] = c.Gb[k][j][1][ax];
        }
        double free = 0.0;
        for (int cc = 0; cc < 3; cc++) free += c.Phi[k][1][cc] * x0[cc];
        p.l[m] = -corridor - free;
        p.u[m] = corridor - free;
        m++;
    }
    p.m = m;
    CHECK_EQ(p.m, 4 * N);   // 2N box + N rate + N state

    double ref[EN]{};
    CHECK(enumerateOptimum(p, ref));

    const DrivetrainPlant plant =
        DrivetrainPlant::discretize(static_cast<float>(v), static_cast<float>(dt));
    CondensedMpc mpc;
    mpc.setup(plant, w, N);
    mpc.setInputRateLimit(du);
    CHECK(mpc.addStateConstraint(a, -corridor, corridor));
    mpc.admm.checkEvery = 1;
    mpc.zero();
    mpc.setPreviousInput(anchor);
    const MpcResult res = mpc.solve(x0, uMin, uMax, 40000, 1e-7f);
    CHECK(res.converged);
    CHECK_EQ(res.constraintRows, p.m);

    for (int i = 0; i < p.n; i++) CHECK_NEAR(mpc.solution()[i], ref[i], 3e-4);

    // The optimum has to be genuinely constrained, or the agreement above is
    // only about the unconstrained solve. Count the active rows by kind, in the
    // reference: rows 0..5 box, 6..8 rate, 9..11 state.
    int activeRate = 0, activeState = 0;
    for (int r = 2 * N; r < p.m; r++) {
        double cz = 0.0;
        for (int j = 0; j < p.n; j++) cz += p.C[r][j] * ref[j];
        if (cz > p.l[r] + 1e-7 && cz < p.u[r] - 1e-7) continue;
        if (r < 3 * N) activeRate++;
        else activeState++;
    }
    CHECK(activeRate >= 1);
    CHECK(activeState >= 1);

    // And the corridor is not something the box gave away for free.
    CondensedMpc open;
    open.setup(plant, w, N);
    open.zero();
    open.solve(x0, uMin, uMax, 20000, 1e-8f);
    CHECK(peakStateOverHorizon(plant, x0, open.solution(), N, 1) > 3.0f * corridor);
    CHECK(peakStateOverHorizon(plant, x0, mpc.solution(), N, 1) <= corridor + 1e-5f);
}

CHAD_TEST(mpc, admmReproducesTheBoxSolverOnABoxOnlyProblem) {
    // Two unrelated algorithms -- projected momentum gradient and an operator
    // split with a Cholesky -- on the same QP. Agreement is evidence about the
    // QP, not about either implementation.
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.1f, 0.02f);
    const MpcWeights w = weights();
    const float uMin[2] = {-0.30f, -0.60f};
    const float uMax[2] = {0.30f, 0.60f};
    const float x0[3] = {0.4f, -0.5f, 0.25f};

    CondensedMpc box;
    box.setup(plant, w, 10);
    box.zero();
    box.solve(x0, uMin, uMax, 20000, 1e-9f);

    CondensedMpc admm;
    admm.solver = MpcSolver::Admm;
    admm.setup(plant, w, 10);
    admm.zero();
    const MpcResult res = admm.solve(x0, uMin, uMax, 6000, 1e-5f);
    CHECK(res.usedAdmm);
    CHECK(res.converged);
    CHECK(!res.hitIterationCap);
    CHECK_EQ(res.constraintRows, 2 * admm.horizon());
    CHECK(res.stepInf < 0.0f);   // not produced by this path, and says so

    for (int i = 0; i < box.vars(); i++) {
        CHECK_NEAR(admm.solution()[i], box.solution()[i], 2e-3);
    }
}

CHAD_TEST(mpc, inputRateLimitBoundsEveryStepAndBinds) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.2f, 0.02f);
    const MpcWeights w = weights();
    const int horizon = 10;
    const float uMin[2] = {-0.50f, -1.00f};
    const float uMax[2] = {0.50f, 1.00f};
    const float x0[3] = {0.4f, 0.5f, 0.30f};
    const float du[2] = {0.02f, 0.05f};
    const float anchor[2] = {0.0f, 0.0f};

    CondensedMpc open;
    open.setup(plant, w, horizon);
    open.zero();
    open.solve(x0, uMin, uMax, 8000, 1e-8f);
    // Without the rows the answer violates the limit, so satisfying it below is
    // not something the problem gave away for free.
    CHECK(worstRate(open.solution(), horizon, 0, anchor[0], nullptr) > du[0]);
    CHECK(worstRate(open.solution(), horizon, 1, anchor[1], nullptr) > du[1]);

    CondensedMpc rate;
    rate.setup(plant, w, horizon);
    rate.setInputRateLimit(du);
    rate.zero();
    rate.setPreviousInput(anchor);
    const MpcResult res = rate.solve(x0, uMin, uMax, 8000, 1e-5f);
    CHECK(res.usedAdmm);
    CHECK(res.converged);
    CHECK_EQ(rate.constraintRows(), 4 * horizon);
    CHECK(res.activeRows > 0);

    for (int a = 0; a < 2; a++) {
        CHECK(worstRate(rate.solution(), horizon, a, anchor[a], nullptr) <= du[a] + 2e-4f);
    }
    for (int i = 0; i < rate.vars(); i++) {
        const float lo = (i % 2 == 0) ? uMin[0] : uMin[1];
        const float hi = (i % 2 == 0) ? uMax[0] : uMax[1];
        CHECK(rate.solution()[i] >= lo - 1e-6f);
        CHECK(rate.solution()[i] <= hi + 1e-6f);
    }
}

CHAD_TEST(mpc, rateLimitAnchorMovesTheFirstMove) {
    // The head of the horizon is bounded against what was actually commanded
    // last tick, so a different anchor has to give a different first move.
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    const MpcWeights w = weights();
    const float uMin[2] = {-0.5f, -1.0f};
    const float uMax[2] = {0.5f, 1.0f};
    const float x0[3] = {0.3f, 0.4f, 0.2f};
    const float du[2] = {0.03f, 0.06f};

    float moves[2][2];
    const float anchors[2] = {0.0f, 0.20f};
    for (int t = 0; t < 2; t++) {
        CondensedMpc mpc;
        mpc.setup(plant, w, 10);
        mpc.setInputRateLimit(du);
        mpc.zero();
        const float anchor[2] = {anchors[t], 0.0f};
        mpc.setPreviousInput(anchor);
        CHECK(mpc.solve(x0, uMin, uMax, 8000, 1e-5f).converged);
        mpc.firstMove(moves[t]);
        CHECK(std::fabs(moves[t][0] - anchors[t]) <= du[0] + 2e-4f);
    }
    CHECK(std::fabs(moves[1][0] - moves[0][0]) > du[0]);
}

CHAD_TEST(mpc, referenceDeltasEnterTheRateBound) {
    // With reference deltas the bounded quantity is the TOTAL command's step,
    // so a reference that is itself ramping eats the allowance and the
    // correction has to give ground.
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.3f, 0.02f);
    const MpcWeights w = weights();
    const int horizon = 10;
    const float uMin[2] = {-0.6f, -1.2f};
    const float uMax[2] = {0.6f, 1.2f};
    const float x0[3] = {0.35f, 0.45f, 0.25f};
    const float du[2] = {0.03f, 0.08f};
    const float anchor[2] = {0.0f, 0.0f};

    float dRef[MPC_MAX_VARS]{};
    for (int k = 1; k < horizon; k++) {
        dRef[2 * k + 0] = 0.025f;    // reference accelerating at every stage
        dRef[2 * k + 1] = -0.060f;
    }

    CondensedMpc mpc;
    mpc.setup(plant, w, horizon);
    mpc.setInputRateLimit(du);
    mpc.zero();
    mpc.setPreviousInput(anchor);
    mpc.setReferenceDeltas(dRef, 2 * horizon);
    const MpcResult res = mpc.solve(x0, uMin, uMax, 8000, 1e-5f);
    CHECK(res.converged);

    for (int a = 0; a < 2; a++) {
        CHECK(worstRate(mpc.solution(), horizon, a, anchor[a], dRef) <= du[a] + 3e-4f);
    }
    // Same problem with the deltas dropped: the correction then uses the whole
    // allowance itself, which is what the deltas exist to prevent.
    CondensedMpc blind;
    blind.setup(plant, w, horizon);
    blind.setInputRateLimit(du);
    blind.zero();
    blind.setPreviousInput(anchor);
    CHECK(blind.solve(x0, uMin, uMax, 8000, 1e-5f).converged);
    CHECK(worstRate(blind.solution(), horizon, 0, anchor[0], dRef) > du[0] + 3e-4f);
}

CHAD_TEST(mpc, looseRateLimitReproducesTheBoxSolution) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    const MpcWeights w = weights();
    const float uMin[2] = {-0.25f, -0.5f};
    const float uMax[2] = {0.25f, 0.5f};
    const float x0[3] = {0.2f, 0.3f, 0.15f};

    CondensedMpc box;
    box.setup(plant, w, 10);
    box.zero();
    box.solve(x0, uMin, uMax, 20000, 1e-9f);

    CondensedMpc loose;
    loose.setup(plant, w, 10);
    const float du[2] = {100.0f, 100.0f};   // far outside the box: never active
    loose.setInputRateLimit(du);
    loose.zero();
    const MpcResult res = loose.solve(x0, uMin, uMax, 8000, 1e-5f);
    CHECK(res.usedAdmm);
    CHECK(res.converged);
    for (int i = 0; i < box.vars(); i++) {
        CHECK_NEAR(loose.solution()[i], box.solution()[i], 3e-3);
    }
}

CHAD_TEST(mpc, stateCorridorHoldsOnAnIndependentForwardSimulation) {
    // The state row is a'G_k with the free response a'Phi_k x0 moved into the
    // bounds. Rolling the plant forward from Ad and Bd checks that condensing
    // against arithmetic that shares none of its code.
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    // Cross-track barely weighted, so nothing but the constraint keeps it in.
    const MpcWeights w{1.0f, 0.01f, 0.01f, 1.0f, 1.0f};
    const int horizon = 12;
    const float uMin[2] = {-2.0f, -5.0f};
    const float uMax[2] = {2.0f, 5.0f};
    const float x0[3] = {0.0f, 0.0f, 0.35f};
    const float corridor = 0.04f;

    CondensedMpc open;
    open.setup(plant, w, horizon);
    open.zero();
    open.solve(x0, uMin, uMax, 8000, 1e-8f);
    const float peakOpen = peakStateOverHorizon(plant, x0, open.solution(), horizon, 1);
    CHECK(peakOpen > 1.5f * corridor);

    CondensedMpc closed;
    closed.setup(plant, w, horizon);
    const float a[3] = {0.0f, 1.0f, 0.0f};
    CHECK(closed.addStateConstraint(a, -corridor, corridor));
    CHECK_EQ(closed.stateConstraints(), 1);
    closed.zero();
    const MpcResult res = closed.solve(x0, uMin, uMax, 20000, 1e-5f);
    CHECK(res.usedAdmm);
    CHECK(res.converged);
    CHECK_EQ(res.constraintRows, 3 * horizon);

    const float peakClosed = peakStateOverHorizon(plant, x0, closed.solution(), horizon, 1);
    CHECK(peakClosed <= corridor + 3e-3f);
}

CHAD_TEST(mpc, stateCorridorOverAStageRangeOnlyConstrainsThatRange) {
    // A row COUNT cannot catch an off-by-one -- [4,8], [5,9] and [6,10] all
    // produce five rows. Forward simulation can: with cross-track growing
    // monotonically here, the last stage inside the range sits ON the bound and
    // the first stage past it is outside, which pins both ends of the window.
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    const MpcWeights w{1.0f, 0.01f, 0.01f, 1.0f, 1.0f};
    const int horizon = 12;
    const float uMin[2] = {-2.0f, -5.0f};
    const float uMax[2] = {2.0f, 5.0f};
    const float x0[3] = {0.0f, 0.0f, 0.35f};
    const float corridor = 0.04f;
    const float a[3] = {0.0f, 1.0f, 0.0f};

    for (int first : {4, 5, 6}) {
        const int last = first + 4;
        CondensedMpc mpc;
        mpc.setup(plant, w, horizon);
        CHECK(mpc.addStateConstraint(a, -corridor, corridor, first, last));
        mpc.zero();
        const MpcResult res = mpc.solve(x0, uMin, uMax, 20000, 1e-5f);
        CHECK(res.converged);
        CHECK_EQ(res.constraintRows, 2 * horizon + (last - first + 1));

        const float* u = mpc.solution();
        // Every stage in the window holds...
        for (int k = first; k <= last; k++) {
            CHECK(stateAtStage(plant, x0, u, k, 1) <= corridor + 3e-3f);
        }
        // ...the last one is ON the bound, so the window really does end there,
        // and the very next stage is outside it. Both fail for a window shifted
        // by one in either direction.
        CHECK_NEAR(stateAtStage(plant, x0, u, last, 1), corridor, 2e-3);
        CHECK(stateAtStage(plant, x0, u, last + 1, 1) > corridor + 1e-4f);
        CHECK(stateAtStage(plant, x0, u, first - 1, 1) < corridor - 1e-3f);
    }
}

CHAD_TEST(mpc, stateConstraintBudgetIsBoundedAndInputsAreValidated) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 8);

    const float zero3[3] = {0.0f, 0.0f, 0.0f};
    CHECK(!mpc.addStateConstraint(zero3, -1.0f, 1.0f));
    const float good[3] = {0.0f, 1.0f, 0.0f};
    CHECK(!mpc.addStateConstraint(good, 1.0f, -1.0f));          // crossed bounds
    const float bad[3] = {0.0f, std::nanf(""), 0.0f};
    CHECK(!mpc.addStateConstraint(bad, -1.0f, 1.0f));
    CHECK_EQ(mpc.stateConstraints(), 0);

    for (int i = 0; i < MPC_MAX_STATE_CONSTRAINTS; i++) {
        CHECK(mpc.addStateConstraint(good, -1.0f, 1.0f));
    }
    CHECK(!mpc.addStateConstraint(good, -1.0f, 1.0f));
    CHECK_EQ(mpc.stateConstraints(), MPC_MAX_STATE_CONSTRAINTS);
}

CHAD_TEST(mpc, everyAcceptedStateConstraintGetsItsRows) {
    // The worst case the API can be asked for: the longest horizon, a rate limit
    // on both axes, and the whole advertised budget of state specs, each
    // spanning the whole horizon. Accepting a spec and then dropping its rows is
    // the failure this pins, because the solve then certifies the optimum of a
    // QP the caller did not ask for.
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    const int horizon = MPC_MAX_HORIZON;
    const MpcWeights w{1.0f, 0.01f, 0.01f, 1.0f, 1.0f};
    const float uMin[2] = {-2.0f, -5.0f};
    const float uMax[2] = {2.0f, 5.0f};
    const float x0[3] = {0.0f, 0.0f, 0.35f};
    const float anchor[2] = {0.0f, 0.0f};
    const float du[2] = {100.0f, 100.0f};      // rows present, never binding
    const float corridor = 0.04f;
    const float along[3] = {1.0f, 0.0f, 0.0f};
    const float head[3] = {0.0f, 0.0f, 1.0f};
    const float cross[3] = {0.0f, 1.0f, 0.0f};

    // Three harmless specs, then the one that matters -- so the corridor lands
    // last in the row order and is the first thing a row budget would lose.
    CondensedMpc mpc;
    mpc.setup(plant, w, horizon);
    mpc.setInputRateLimit(du);
    CHECK(mpc.addStateConstraint(along, -10.0f, 10.0f));
    CHECK(mpc.addStateConstraint(head, -10.0f, 10.0f));
    CHECK(mpc.addStateConstraint(along, -20.0f, 20.0f));
    CHECK(mpc.addStateConstraint(cross, -corridor, corridor));
    CHECK_EQ(mpc.stateConstraints(), MPC_MAX_STATE_CONSTRAINTS);
    CHECK_EQ(mpc.plannedRows(), 4 * horizon + MPC_MAX_STATE_CONSTRAINTS * horizon);
    CHECK(mpc.plannedRows() <= MPC_MAX_QP_ROWS);

    mpc.zero();
    mpc.setPreviousInput(anchor);
    const MpcResult res = mpc.solve(x0, uMin, uMax, 40000, 1e-4f);
    CHECK_EQ(res.constraintRows, mpc.plannedRows());
    CHECK_EQ(res.droppedRows, 0);
    CHECK(res.converged);
    CHECK(!res.primalInfeasible);
    CHECK(peakStateOverHorizon(plant, x0, mpc.solution(), horizon, 1)
          <= corridor + 3e-3f);

    // Without the fourth spec the corridor is wide open, so the check above is
    // about the fourth spec's rows and not about the first three.
    CondensedMpc three;
    three.setup(plant, w, horizon);
    three.setInputRateLimit(du);
    CHECK(three.addStateConstraint(along, -10.0f, 10.0f));
    CHECK(three.addStateConstraint(head, -10.0f, 10.0f));
    CHECK(three.addStateConstraint(along, -20.0f, 20.0f));
    three.zero();
    three.setPreviousInput(anchor);
    three.solve(x0, uMin, uMax, 40000, 1e-4f);
    CHECK(peakStateOverHorizon(plant, x0, three.solution(), horizon, 1) > 4.0f * corridor);
}

CHAD_TEST(mpc, jointlyEmptyConstraintsAreReportedNotHidden) {
    // Reachable from MpcPathFollower: a reference faster than the drivetrain
    // makes the input box entirely negative, while on the first tick the rate
    // anchor is still zero, so the head row wants u_0 near zero. Each row is
    // consistent on its own; together they are empty. The committed move parks
    // on the box edge, which is physically right -- but silence is not.
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 10);
    const float du[2] = {0.01f, 0.02f};
    mpc.setInputRateLimit(du);
    const float uMin[2] = {-3.6f, -5.0f};
    const float uMax[2] = {-0.4f, 5.0f};
    const float x0[3] = {0.2f, 0.1f, 0.05f};
    const float anchor[2] = {0.0f, 0.0f};

    mpc.certifyOptimality = false;      // the cheapest configuration there is
    mpc.zero();
    mpc.setPreviousInput(anchor);
    const MpcResult blind = mpc.solve(x0, uMin, uMax, 50, 1e-4f);
    CHECK(blind.primalInfeasible);
    CHECK(blind.primalResidual > 0.0f);       // measured even so, because it matters
    CHECK(!blind.converged);
    CHECK_NEAR(mpc.solution()[0], uMax[0], 1e-5);   // still deliverable

    // Widen the rate limit so the intersection is non-empty: same rows, no flag.
    const float wide[2] = {4.0f, 4.0f};
    mpc.setInputRateLimit(wide);
    mpc.certifyOptimality = true;
    mpc.admm.checkEvery = 1;
    mpc.zero();
    mpc.setPreviousInput(anchor);
    const MpcResult ok = mpc.solve(x0, uMin, uMax, 8000, 1e-4f);
    CHECK(!ok.primalInfeasible);
    CHECK(ok.converged);
}

CHAD_TEST(mpc, admmMultipliersSurviveARecondensing) {
    // Speed buckets re-condense mid-run, and with timeVaryingHorizon on that is
    // constant. Rebuilding C without carrying the multipliers across makes every
    // bucket edge a cold start -- exactly when the reference is moving fastest.
    const MpcWeights w = weights();
    const float uMin[2] = {-0.3f, -0.6f};
    const float uMax[2] = {0.3f, 0.6f};
    const float x0[3] = {0.25f, 0.35f, 0.12f};
    const float du[2] = {0.02f, 0.05f};
    const float anchor[2] = {0.0f, 0.0f};

    CondensedMpc mpc;
    mpc.setup(DrivetrainPlant::discretize(1.00f, 0.02f), w, 10);
    mpc.setInputRateLimit(du);
    mpc.admm.checkEvery = 1;
    mpc.zero();
    mpc.setPreviousInput(anchor);
    const MpcResult cold = mpc.solve(x0, uMin, uMax, 8000, 1e-5f);
    CHECK(cold.converged);
    const MpcResult warm = mpc.solve(x0, uMin, uMax, 8000, 1e-5f);
    CHECK(warm.converged);
    // Headroom for the comparisons below, stated against the warm price rather
    // than as an absolute count: tuning rhoScale from 1 to 3 took this cold
    // solve from 74 iterations to 19, so an absolute floor here was really
    // pinning the old rho.
    CHECK(cold.iterations > 8);
    CHECK(cold.iterations >= 3 * warm.iterations);

    // Identical plant and horizon: nothing about the QP changed, so a warm start
    // that survived costs what `warm` cost.
    mpc.setup(DrivetrainPlant::discretize(1.00f, 0.02f), w, 10);
    mpc.setInputRateLimit(du);
    const MpcResult same = mpc.solve(x0, uMin, uMax, 8000, 1e-5f);
    CHECK(same.converged);
    CHECK(same.iterations <= warm.iterations + 1);

    // A neighbouring speed bucket is a slightly different QP, so it costs more
    // than nothing -- but nothing like a cold start. Measured 10 against 19 at
    // rhoScale 3; it was 16 against 74 at rhoScale 1, and the margin shrank
    // because the cold solve got cheaper, not because the carry-over got worse.
    mpc.setup(DrivetrainPlant::discretize(1.05f, 0.02f), w, 10);
    mpc.setInputRateLimit(du);
    const MpcResult neighbour = mpc.solve(x0, uMin, uMax, 8000, 1e-5f);
    CHECK(neighbour.converged);
    CHECK(neighbour.iterations * 3 < cold.iterations * 2);
}

CHAD_TEST(mpc, clearConstraintsReturnsToTheBoxFastPath) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 8);
    const float du[2] = {0.02f, 0.05f};
    mpc.setInputRateLimit(du);
    CHECK(mpc.hasGeneralConstraints());

    const float uMin[2] = {-0.3f, -0.6f};
    const float uMax[2] = {0.3f, 0.6f};
    const float x0[3] = {0.2f, 0.3f, 0.1f};
    mpc.zero();
    CHECK(mpc.solve(x0, uMin, uMax, 2000, 1e-5f).usedAdmm);

    mpc.clearConstraints();
    CHECK(!mpc.hasGeneralConstraints());
    mpc.zero();
    const MpcResult res = mpc.solve(x0, uMin, uMax, 2000, 1e-7f);
    CHECK(!res.usedAdmm);
    CHECK(res.converged);
    CHECK(res.primalResidual < 0.0f);
}

CHAD_TEST(mpc, admmWarmStartCostsFewerIterations) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 10);
    const float du[2] = {0.02f, 0.05f};
    mpc.setInputRateLimit(du);
    mpc.admm.checkEvery = 1;

    const float uMin[2] = {-0.3f, -0.6f};
    const float uMax[2] = {0.3f, 0.6f};
    const float x0[3] = {0.25f, 0.35f, 0.12f};
    const float anchor[2] = {0.0f, 0.0f};

    mpc.zero();
    mpc.setPreviousInput(anchor);
    const MpcResult cold = mpc.solve(x0, uMin, uMax, 8000, 1e-5f);
    CHECK(cold.converged);
    CHECK(cold.iterations > 3);
    const MpcResult warm = mpc.solve(x0, uMin, uMax, 8000, 1e-5f);
    CHECK(warm.converged);
    CHECK(warm.iterations < cold.iterations);
}

CHAD_TEST(mpc, admmCapIsReportedWithoutClaimingSuccess) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.4f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 16);
    const float du[2] = {0.005f, 0.01f};
    mpc.setInputRateLimit(du);
    mpc.admm.checkEvery = 1;

    const float uMin[2] = {-1.0f, -2.0f};
    const float uMax[2] = {1.0f, 2.0f};
    const float x0[3] = {1.0f, 1.0f, 0.5f};
    mpc.zero();
    const MpcResult res = mpc.solve(x0, uMin, uMax, 2, 1e-9f);
    CHECK_EQ(res.iterations, 2);
    CHECK(!res.converged);
    CHECK(res.hitIterationCap);
    CHECK(res.primalResidual >= 0.0f);
    CHECK(res.dualResidual >= 0.0f);
    CHECK_NEAR(res.kktResidual, std::fmax(res.primalResidual, res.dualResidual), 1e-9);
}

CHAD_TEST(mpc, certificationOffMeansUnknownNotConverged) {
    // The step norm is not a certificate. With certification disabled the early
    // exit still happens -- that is what makes a fixed budget cheap -- but the
    // verdict must be "not known", never "converged".
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 10);
    mpc.certifyOptimality = false;

    const float uMin[2] = {-0.3f, -0.6f};
    const float uMax[2] = {0.3f, 0.6f};
    const float x0[3] = {0.2f, 0.25f, 0.1f};
    mpc.zero();
    const MpcResult early = mpc.solve(x0, uMin, uMax, 500, 1e-4f);
    CHECK(early.iterations < 500);          // the tolerance did stop it early
    CHECK(early.stepInf <= 1e-4f);
    CHECK(!early.converged);
    CHECK(early.kktResidual < 0.0f);

    const float du[2] = {0.02f, 0.05f};
    mpc.setInputRateLimit(du);
    mpc.zero();
    const MpcResult general = mpc.solve(x0, uMin, uMax, 50, 1e-4f);
    CHECK(general.usedAdmm);
    CHECK(!general.converged);
    // The primal residual is measured even with certification off: a general row
    // the iterate has not met is the one thing the output clamp cannot fix, so
    // it is worth a pass over the row support. The dual needs an H matvec and is
    // not, which is what keeps converged false.
    CHECK(general.primalResidual >= 0.0f);
    CHECK(general.dualResidual < 0.0f);
    CHECK(general.kktResidual < 0.0f);
}

CHAD_TEST(mpc, warmStartSurvivesARecondensingAtTheSameHorizon) {
    // Speed buckets change H mid-run. Throwing the input sequence away at every
    // bucket edge would defeat the point of a real-time iteration, and the
    // neighbouring bucket describes nearly the same QP.
    const MpcWeights w = weights();
    const float uMin[2] = {-0.3f, -0.6f};
    const float uMax[2] = {0.3f, 0.6f};
    const float x0[3] = {0.25f, 0.35f, 0.15f};

    CondensedMpc mpc;
    mpc.setup(DrivetrainPlant::discretize(1.00f, 0.02f), w, 10);
    mpc.zero();
    mpc.solve(x0, uMin, uMax, 20000, 1e-9f);
    float kept[MPC_MAX_VARS];
    for (int i = 0; i < mpc.vars(); i++) kept[i] = mpc.solution()[i];
    CHECK(std::fabs(kept[0]) > 1e-4f);

    mpc.setup(DrivetrainPlant::discretize(1.05f, 0.02f), w, 10);
    for (int i = 0; i < mpc.vars(); i++) CHECK_EQ(mpc.solution()[i], kept[i]);

    // A horizon change makes the old sequence meaningless, so that one clears.
    mpc.setup(DrivetrainPlant::discretize(1.05f, 0.02f), w, 6);
    for (int i = 0; i < mpc.vars(); i++) CHECK_EQ(mpc.solution()[i], 0.0f);
}

CHAD_TEST(mpc, constraintRowsAreRebuiltAgainstTheNewCondensing) {
    // A state row is a'G_k, so it has to change when G does. Same corridor, two
    // speeds, materially different plans.
    const MpcWeights w{1.0f, 0.01f, 0.01f, 1.0f, 1.0f};
    const float uMin[2] = {-2.0f, -5.0f};
    const float uMax[2] = {2.0f, 5.0f};
    const float x0[3] = {0.0f, 0.0f, 0.35f};
    const float a[3] = {0.0f, 1.0f, 0.0f};

    CondensedMpc mpc;
    mpc.setup(DrivetrainPlant::discretize(1.5f, 0.02f), w, 12);
    CHECK(mpc.addStateConstraint(a, -0.04f, 0.04f));
    mpc.zero();
    CHECK(mpc.solve(x0, uMin, uMax, 20000, 1e-5f).converged);
    float slow[2];
    mpc.firstMove(slow);

    const DrivetrainPlant fast = DrivetrainPlant::discretize(3.0f, 0.02f);
    mpc.setup(fast, w, 12);
    mpc.zero();
    const MpcResult res = mpc.solve(x0, uMin, uMax, 20000, 1e-5f);
    CHECK(res.converged);
    float quick[2];
    mpc.firstMove(quick);
    CHECK(std::fabs(quick[1] - slow[1]) > 1e-3f);
    // And the corridor still holds against the new plant, which it could not if
    // the rows had been left over from the old one.
    CHECK(peakStateOverHorizon(fast, x0, mpc.solution(), 12, 1) <= 0.04f + 3e-3f);
}

CHAD_TEST(mpc, nonFiniteStateGivesAFiniteBoxFeasibleInput) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.2f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 10);
    const float uMin[2] = {-0.3f, -0.6f};
    const float uMax[2] = {0.3f, 0.6f};

    // Box feasibility alone is the clamp on the way out, so pin the documented
    // behaviour instead: the dead axis is treated as ZERO error, which means
    // the answer must equal the answer for the same state with that axis zeroed
    // -- bit for bit, since it is the same arithmetic.
    const float dead[3] = {0.2f, std::nanf(""), 0.1f};
    const float zeroed[3] = {0.2f, 0.0f, 0.1f};
    mpc.zero();
    const MpcResult res = mpc.solve(dead, uMin, uMax, 200, 1e-6f);
    CHECK(res.stateNotFinite);
    float substituted[MPC_MAX_VARS];
    for (int i = 0; i < mpc.vars(); i++) substituted[i] = mpc.solution()[i];
    mpc.zero();
    const MpcResult clean = mpc.solve(zeroed, uMin, uMax, 200, 1e-6f);
    CHECK(!clean.stateNotFinite);
    CHECK(std::fabs(clean.kktResidual - res.kktResidual) < 1e-9f);
    for (int i = 0; i < mpc.vars(); i++) {
        CHECK_FINITE(substituted[i]);
        CHECK_EQ(substituted[i], mpc.solution()[i]);
        const float lo = (i % 2 == 0) ? uMin[0] : uMin[1];
        const float hi = (i % 2 == 0) ? uMax[0] : uMax[1];
        CHECK(substituted[i] >= lo - 1e-6f);
        CHECK(substituted[i] <= hi + 1e-6f);
    }
    // And the substitution is not vacuous: the live axes still steer.
    CHECK(std::fabs(substituted[0]) > 1e-4f);

    // An infinite bound must not become a NaN gradient either.
    const float inf = HUGE_VALF;
    const float wild[2] = {inf, inf};
    const float tame[3] = {0.2f, 0.3f, 0.1f};
    mpc.zero();
    const MpcResult res2 = mpc.solve(tame, uMin, wild, 200, 1e-6f);
    CHECK(res2.stateNotFinite);
    float u[2];
    mpc.firstMove(u);
    CHECK_FINITE(u[0]);
    CHECK_FINITE(u[1]);
}

CHAD_TEST(mpc, crossedInputBoxCollapsesInsteadOfDiverging) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    CondensedMpc mpc;
    mpc.setup(plant, weights(), 8);
    const float uMin[2] = {0.20f, -0.5f};    // above uMax on the linear axis
    const float uMax[2] = {-0.20f, 0.5f};
    const float x0[3] = {0.3f, 0.3f, 0.1f};
    mpc.zero();
    const MpcResult res = mpc.solve(x0, uMin, uMax, 500, 1e-6f);
    CHECK(res.stateNotFinite);               // flagged as a sanitised input
    float u[2];
    mpc.firstMove(u);
    CHECK_NEAR(u[0], 0.0f, 1e-6);            // the midpoint of the crossed pair
    CHECK_FINITE(u[1]);
}

CHAD_TEST(mpc, poisonedHessianFallsBackAndZeroes) {
    // A NaN reaching the weights must not become a NaN voltage. The Cholesky
    // refuses to factor, the general rows are dropped, the box fallback still
    // produces NaN from the poisoned H, and the output guard catches it.
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    const MpcWeights poisoned{1.0f, std::nanf(""), 1.0f, 1.0f, 1.0f};
    CondensedMpc mpc;
    mpc.setup(plant, poisoned, 8);
    const float du[2] = {0.02f, 0.05f};
    mpc.setInputRateLimit(du);

    const float uMin[2] = {-0.3f, -0.6f};
    const float uMax[2] = {0.3f, 0.6f};
    const float x0[3] = {0.2f, 0.3f, 0.1f};
    mpc.zero();
    const MpcResult res = mpc.solve(x0, uMin, uMax, 100, 1e-6f);
    CHECK(res.factorizationFailed);
    CHECK(res.solutionNotFinite);
    CHECK(!res.converged);
    float u[2];
    mpc.firstMove(u);
    CHECK_EQ(u[0], 0.0f);
    CHECK_EQ(u[1], 0.0f);
}

CHAD_TEST(mpc, shiftAdvancesThePlanByExactlyOneStage) {
    // The documented contract, tested directly rather than through an iteration
    // count: at the next tick the horizon has advanced by one, so u_1 is the new
    // u_0. Deleting shift() fails this; a shift by two blocks fails it too.
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.0f, 0.02f);
    const int horizon = 10;
    CondensedMpc mpc;
    mpc.setup(plant, weights(), horizon);
    const float uMin[2] = {-0.3f, -0.6f};
    const float uMax[2] = {0.3f, 0.6f};
    const float x0[3] = {0.30f, 0.40f, 0.18f};

    mpc.zero();
    CHECK(mpc.solve(x0, uMin, uMax, 20000, 1e-8f).converged);
    float before[MPC_MAX_VARS];
    for (int i = 0; i < mpc.vars(); i++) before[i] = mpc.solution()[i];
    // Not a constant sequence, or "shifted by one" says nothing.
    CHECK(std::fabs(before[0] - before[2]) > 1e-4f);

    mpc.shift();
    for (int i = 0; i + 2 < mpc.vars(); i++) CHECK_EQ(mpc.solution()[i], before[i + 2]);
    for (int i = mpc.vars() - 2; i < mpc.vars(); i++) {
        CHECK_EQ(mpc.solution()[i], before[i]);   // the tail repeats
    }
}

CHAD_TEST(mpc, shiftedWarmStartBeatsAnUnshiftedOneUnderModelConsistentMotion) {
    // Against COLD, any warm start wins, so that comparison proves nothing
    // about the shift. Against the same warm start left in place, it only wins
    // when the premise holds: the state advances exactly as the prediction
    // model says AND the terminal cost is the infinite-horizon cost-to-go, which
    // is what makes the optimal sequence time-consistent. Both are what
    // MpcPathFollower arranges.
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.2f, 0.02f);
    const MpcWeights w = weights();
    const float Q[3][3] = {{w.qx, 0.0f, 0.0f}, {0.0f, w.qy, 0.0f}, {0.0f, 0.0f, w.qTheta}};
    const float R[2][2] = {{w.rv, 0.0f}, {0.0f, w.rw}};
    const DareSolution<3, 2> dare = solveDare<3, 2>(plant.Ad, plant.Bd, Q, R);
    CHECK(dare.converged);

    const float uMin[2] = {-0.30f, -0.60f};
    const float uMax[2] = {0.30f, 0.60f};
    const float du[2] = {0.02f, 0.05f};

    int total[2] = {0, 0};
    for (int shifted = 0; shifted < 2; shifted++) {
        CondensedMpc mpc;
        mpc.setup(plant, w, 10, dare.P);
        mpc.setInputRateLimit(du);
        mpc.admm.checkEvery = 1;
        float x0[3] = {0.30f, 0.40f, 0.15f};
        float anchor[2] = {0.0f, 0.0f};
        mpc.zero();
        for (int t = 0; t < 30; t++) {
            if (t > 0 && shifted) mpc.shift();
            mpc.setPreviousInput(anchor);
            const MpcResult res = mpc.solve(x0, uMin, uMax, 20000, 1e-5f);
            CHECK(res.converged);
            total[shifted] += res.iterations;
            float u[2];
            mpc.firstMove(u);
            float nx[3];
            for (int r = 0; r < 3; r++) {
                nx[r] = plant.Ad[r][0] * x0[0] + plant.Ad[r][1] * x0[1] +
                        plant.Ad[r][2] * x0[2] + plant.Bd[r][0] * u[0] +
                        plant.Bd[r][1] * u[1];
            }
            for (int r = 0; r < 3; r++) x0[r] = nx[r];
            anchor[0] = u[0];
            anchor[1] = u[1];
        }
    }
    CHECK(total[1] < total[0]);
}

CHAD_TEST(mpc, zeroSpeedCostToGoIsNotUsableAsATerminalCost) {
    // At v = 0 the cross-track state is unreachable and A[1][1] = 1, so the
    // DARE has no stabilising solution. Value iteration does not notice: it
    // adds qy to P[1][1] on every sweep and hands the result out anyway, so the
    // "cost-to-go" it reports is its iteration budget in disguise. MpcPathFollower
    // used to feed exactly that to the MPC as a terminal cost near a stop.
    const DrivetrainPlant rest = DrivetrainPlant::discretize(0.0f, 0.02f);
    const MpcWeights w = weights();
    const LqrSolution few = solveDiscreteLqr(rest, w, 50, 1e-9f);
    const LqrSolution many = solveDiscreteLqr(rest, w, 400, 1e-9f);
    CHECK(!few.converged);
    CHECK(!many.converged);
    CHECK(many.P[1][1] > 5.0f * few.P[1][1]);

    const float Q[3][3] = {{w.qx, 0.0f, 0.0f}, {0.0f, w.qy, 0.0f}, {0.0f, 0.0f, w.qTheta}};
    const float R[2][2] = {{w.rv, 0.0f}, {0.0f, w.rw}};
    const DareSolution<3, 2> dead = solveDare<3, 2>(rest.Ad, rest.Bd, Q, R);
    CHECK(!dead.converged);

    const DrivetrainPlant moving = DrivetrainPlant::discretize(0.5f, 0.02f);
    const DareSolution<3, 2> alive = solveDare<3, 2>(moving.Ad, moving.Bd, Q, R);
    CHECK(alive.converged);
    CHECK(alive.residual < 1e-2f);
}

// ===========================================================================
// ScheduledLqr's speed grid spans [0, vMax], and at v = 0 cross-track is
// uncontrollable: A[1][2] = v*dt and Bd[1][1] = v*dt^2/2 both vanish, so the
// DARE has no stabilising solution there. Interpolating through that point
// corrupts the schedule at every low speed -- and not only the cost-to-go the
// MPC uses as a terminal cost, but the GAIN, which is what actually steers.
//
// The reference in each case is a direct solve at the queried speed, which
// shares no code with the interpolation being tested.
// ===========================================================================
CHAD_TEST(mpc, scheduledLqrExcludesSpeedsWithNoStabilisingSolution) {
    const MpcWeights w{1.0f, 40.0f, 4.0f, 1.0f, 0.25f};
    const float dt = 0.01f;
    const float vMax = 1.887f;

    // The premise: the solve at v = 0 does not converge, and says so.
    const LqrSolution zero = solveDiscreteLqr(DrivetrainPlant::discretize(0.0f, dt), w);
    CHECK(!zero.converged);
    // ...while one grid step up it does. Without both halves this test would
    // pass on a schedule that excluded everything.
    const LqrSolution step =
        solveDiscreteLqr(DrivetrainPlant::discretize(vMax / 8.0f, dt), w);
    CHECK(step.converged);
    // The magnitude of what excluding it avoids.
    CHECK(zero.P[1][1] > 1e6f * step.P[1][1]);
    // And the gain there is genuinely zero, which is why interpolating from it
    // drags the low-speed gains toward zero rather than toward infinity. Zero
    // is the CORRECT gain for an uncontrollable axis, so it cannot be used as a
    // marker for the failed solve.
    CHECK_EQ(zero.K[1][1], 0.0f);

    ScheduledLqr sch;
    sch.build(w, vMax, dt, 9);
    CHECK(sch.built());
    CHECK_EQ(sch.points(), 9);
    CHECK_EQ(sch.usablePoints(), 8);
    CHECK_NEAR(sch.vMin(), vMax / 8.0f, 1e-6);

    // The cross-track gain must track the direct solve across the whole usable
    // range, including well below the first grid point where the old schedule
    // was blending in the v = 0 zero.
    float worstGainRel = 0.0f;
    float atV = -1.0f;
    for (int i = 0; i <= 40; i++) {
        const float v = vMax * static_cast<float>(i) / 40.0f;
        const LqrSolution ex = solveDiscreteLqr(DrivetrainPlant::discretize(v, dt), w);
        if (!ex.converged) continue;
        float K[2][3];
        CHECK(sch.gainAt(v, K));
        const float rel = std::fabs(K[1][1] - ex.K[1][1]) / std::fabs(ex.K[1][1]);
        if (rel > worstGainRel) { worstGainRel = rel; atV = v; }
    }
    std::printf("[sched] usable=%d vMin=%.4f worst |dK11|/K11 = %.4f at v=%.4f\n",
                sch.usablePoints(), static_cast<double>(sch.vMin()),
                static_cast<double>(worstGainRel), static_cast<double>(atV));
    // Measured 0.003. It was 0.58 -- the schedule under-corrected cross-track by
    // 58% at 0.1 m/s -- while the v = 0 point was in the interpolation.
    CHECK(worstGainRel < 0.02f);

    // The cost-to-go is no longer astronomical anywhere. The bound is stated
    // against the solve at the lowest usable speed, not a literal: it is the
    // largest value any interpolation of the surviving points can produce.
    const LqrSolution lowest =
        solveDiscreteLqr(DrivetrainPlant::discretize(sch.vMin(), dt), w);
    CHECK(lowest.converged);
    for (int i = 0; i <= 40; i++) {
        const float v = vMax * static_cast<float>(i) / 40.0f;
        float P[3][3];
        CHECK(sch.costToGoAt(v, P));
        CHECK_FINITE(P[1][1]);
        CHECK(P[1][1] <= lowest.P[1][1] * 1.001f);
        CHECK(P[1][1] > 0.0f);
    }

    // Below vMin every lookup is the same solution, so a stopped robot gets the
    // slowest gain the plant admits rather than a blend with a divergent one.
    float k0[2][3], kLow[2][3];
    CHECK(sch.gainAt(0.0f, k0));
    CHECK(sch.gainAt(0.5f * sch.vMin(), kLow));
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 3; j++) CHECK_EQ(k0[i][j], kLow[i][j]);
}
