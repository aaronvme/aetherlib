#include "chad/movement_control/rti.hpp"

#include <cmath>

#include "chad/movement_control/mpc_lqr.hpp"
#include "chad_test.hpp"

using namespace chad;

namespace {

MpcWeights weights() { return {1.0f, 1.0f, 2.0f, 0.6f, 0.4f}; }

const float kBoxMin[2] = {-0.30f, -0.60f};
const float kBoxMax[2] = {0.30f, 0.60f};

CondensedMpc makeMpc(int horizon = 10, float v = 1.2f) {
    CondensedMpc mpc;
    mpc.setup(DrivetrainPlant::discretize(v, 0.02f), weights(), horizon);
    return mpc;
}

/// The same QP solved to convergence, for comparison against the budgeted one.
void convergedFirstMove(int horizon, float v, const float x0[3], float out[2]) {
    CondensedMpc mpc = makeMpc(horizon, v);
    mpc.zero();
    mpc.solve(x0, kBoxMin, kBoxMax, 20000, 1e-9f);
    mpc.firstMove(out);
}

}  // namespace

CHAD_TEST(rti, unboundInstanceIsSafe) {
    RealTimeIteration rti;
    const float x0[3] = {1.0f, 1.0f, 1.0f};
    float u[2] = {9.0f, 9.0f};
    const MpcResult res = rti.step(x0, kBoxMin, kBoxMax, u);
    CHECK_EQ(u[0], 0.0f);
    CHECK_EQ(u[1], 0.0f);
    CHECK_EQ(res.iterations, 0);
}

CHAD_TEST(rti, spendsExactlyItsBudget) {
    CondensedMpc mpc = makeMpc();
    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 3;
    rti.config.tol = 0.0f;

    const float x0[3] = {0.4f, 0.5f, 0.2f};
    float u[2];
    for (int tick = 0; tick < 5; tick++) {
        const MpcResult res = rti.step(x0, kBoxMin, kBoxMax, u);
        CHECK_EQ(res.iterations, 3);
    }
}

CHAD_TEST(rti, nonPositiveBudgetStillTakesOneIteration) {
    CondensedMpc mpc = makeMpc();
    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 0;
    const float x0[3] = {0.2f, 0.2f, 0.1f};
    float u[2];
    CHECK_EQ(rti.step(x0, kBoxMin, kBoxMax, u).iterations, 1);
}

CHAD_TEST(rti, convergesOverTicksOnAHeldState) {
    CondensedMpc mpc = makeMpc();
    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 1;
    rti.config.tol = 0.0f;
    rti.config.shiftWarmStart = false;

    const float x0[3] = {0.35f, 0.45f, 0.18f};
    float reference[2];
    convergedFirstMove(10, 1.2f, x0, reference);

    float u[2] = {0.0f, 0.0f};
    float firstError = 0.0f;
    for (int tick = 0; tick < 400; tick++) {
        rti.step(x0, kBoxMin, kBoxMax, u);
        if (tick == 0) firstError = std::fabs(u[0] - reference[0]);
    }
    const float finalError = std::fabs(u[0] - reference[0]);
    CHECK(finalError < firstError);
    CHECK(finalError < 5e-3f);
}

CHAD_TEST(rti, firstTickStartsFromZeroNotStaleState) {
    CondensedMpc mpc = makeMpc();
    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 1;

    const float far[3] = {2.0f, 2.0f, 1.0f};
    float u[2];
    for (int i = 0; i < 50; i++) rti.step(far, kBoxMin, kBoxMax, u);
    const float saturated = u[0];
    CHECK(std::fabs(saturated) > 1e-4f);

    // A fresh bind must not inherit the previous run's solution.
    rti.bind(&mpc);
    const float origin[3] = {0.0f, 0.0f, 0.0f};
    rti.step(origin, kBoxMin, kBoxMax, u);
    CHECK_NEAR(u[0], 0.0f, 1e-6);
    CHECK_NEAR(u[1], 0.0f, 1e-6);
}

CHAD_TEST(rti, resetClearsTheWarmStart) {
    // `|u| <= |warm|` is satisfied by a reset() that does nothing, because the
    // iterate approaches the optimum from below here. The claim that bites is an
    // equality: after reset the next tick must reproduce a FRESH solver exactly,
    // and must differ from carrying on.
    const float x0[3] = {0.5f, 0.5f, 0.25f};

    CondensedMpc mpc = makeMpc();
    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 2;
    float u[2];
    for (int i = 0; i < 30; i++) rti.step(x0, kBoxMin, kBoxMax, u);
    const float warm[2] = {u[0], u[1]};

    CondensedMpc fresh = makeMpc();
    RealTimeIteration freshRti(&fresh);
    freshRti.config.iterationsPerTick = 2;
    float uFresh[2];
    freshRti.step(x0, kBoxMin, kBoxMax, uFresh);

    rti.reset();
    rti.step(x0, kBoxMin, kBoxMax, u);
    CHECK_NEAR(u[0], uFresh[0], 1e-7);
    CHECK_NEAR(u[1], uFresh[1], 1e-7);
    // On the axis that separates them. Two iterations from zero already lands
    // axis 0 within 4e-5 of where thirty warm ticks put it, so asserting there
    // cannot distinguish a working reset from no reset at all.
    CHECK(std::fabs(u[1] - warm[1]) > 1e-3f);
    CHECK(std::fabs(warm[1]) > 1e-3f);
}

CHAD_TEST(rti, resetClearsTheAdmmSlackAndMultipliers) {
    // The case above runs on the box fast path, which never touches AdmmQp, so
    // it cannot see whether reset() reaches the ADMM slack w and duals y. A rate
    // limit far too loose to bind forces the general path while leaving u_0 free
    // of the rate anchor -- which reset() deliberately does NOT clear, being
    // caller data. Carrying w and y across the reset moves u by 5e-2 here.
    const float x0[3] = {0.5f, 0.5f, 0.25f};
    const float du[2] = {5.0f, 10.0f};

    CondensedMpc mpc = makeMpc();
    mpc.setInputRateLimit(du);
    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 2;
    float u[2];
    for (int i = 0; i < 30; i++) rti.step(x0, kBoxMin, kBoxMax, u);
    const float warm[2] = {u[0], u[1]};

    CondensedMpc fresh = makeMpc();
    fresh.setInputRateLimit(du);
    RealTimeIteration freshRti(&fresh);
    freshRti.config.iterationsPerTick = 2;
    float uFresh[2];
    freshRti.step(x0, kBoxMin, kBoxMax, uFresh);

    rti.reset();
    rti.step(x0, kBoxMin, kBoxMax, u);
    CHECK(mpc.constraintRows() > 0);      // really on the general path
    CHECK_NEAR(u[0], uFresh[0], 1e-7);
    CHECK_NEAR(u[1], uFresh[1], 1e-7);
    CHECK(std::fabs(u[0] - warm[0]) > 1e-3f);
    CHECK(std::fabs(u[1] - warm[1]) > 1e-3f);
}

/// The old version of this walked the state by a scalar decay and asserted only
/// that u stayed inside the box, which the unconditional output clamp guarantees
/// whatever shift() does. What shiftWarmStart is FOR is reducing the distance
/// between the committed move and the move a converged solve would have made, so
/// that is what is measured -- and only under the premise that makes shifting
/// correct: the state advances by the prediction model, and the terminal cost is
/// the infinite-horizon cost-to-go.
CHAD_TEST(rti, shiftedWarmStartBeatsHoldingWhenTheTerminalCostIsTheCostToGo) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.2f, 0.02f);
    const LqrSolution lqr = solveDiscreteLqr(plant, weights());
    CHECK(lqr.converged);
    const float a[3] = {0.0f, 1.0f, 0.0f};

    double err[2] = {0.0, 0.0};
    for (int mode = 0; mode < 2; mode++) {
        CondensedMpc mpc;
        mpc.setup(plant, weights(), 10, lqr.P);
        CHECK(mpc.addStateConstraint(a, -0.05f, 0.05f));
        mpc.admm.checkEvery = 1;
        RealTimeIteration rti(&mpc);
        rti.config.iterationsPerTick = 1;
        rti.config.tol = 0.0f;
        rti.config.shiftWarmStart = (mode == 1);

        float x0[3] = {0.05f, 0.06f, 0.03f};
        float u[2];
        for (int tick = 0; tick < 40; tick++) {
            rti.step(x0, kBoxMin, kBoxMax, u);
            // The move a solve to convergence at this same state would commit.
            CondensedMpc ref;
            ref.setup(plant, weights(), 10, lqr.P);
            CHECK(ref.addStateConstraint(a, -0.05f, 0.05f));
            ref.admm.checkEvery = 1;
            ref.zero();
            ref.solve(x0, kBoxMin, kBoxMax, 40000, 1e-6f);
            err[mode] += std::fmax(std::fabs(u[0] - ref.solution()[0]),
                                   std::fabs(u[1] - ref.solution()[1]));
            float nx[3];
            for (int r = 0; r < 3; r++) {
                nx[r] = plant.Ad[r][0] * x0[0] + plant.Ad[r][1] * x0[1] +
                        plant.Ad[r][2] * x0[2] + plant.Bd[r][0] * u[0] +
                        plant.Bd[r][1] * u[1];
            }
            for (int r = 0; r < 3; r++) x0[r] = nx[r];
        }
    }
    // Measured 0.08x. A shift map that scrambled the multipliers would not beat
    // leaving the iterate alone, let alone beat it by an order of magnitude.
    CHECK(err[1] < 0.5 * err[0]);
    CHECK(err[0] > 1e-3);      // the unshifted run is genuinely off, not both zero
}

CHAD_TEST(rti, respectsTheBoxEveryTick) {
    CondensedMpc mpc = makeMpc(6, 1.8f);
    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 1;

    const float tight[2] = {-0.02f, -0.05f};
    const float tightMax[2] = {0.02f, 0.05f};
    const float x0[3] = {5.0f, 5.0f, 2.0f};
    float u[2];
    for (int tick = 0; tick < 100; tick++) {
        rti.step(x0, tight, tightMax, u);
        CHECK(u[0] >= tight[0] - 1e-6f && u[0] <= tightMax[0] + 1e-6f);
        CHECK(u[1] >= tight[1] - 1e-6f && u[1] <= tightMax[1] + 1e-6f);
    }
}

CHAD_TEST(rti, certificationIsOffByDefaultAndPlumbedThrough) {
    CondensedMpc mpc = makeMpc();
    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 1;

    const float x0[3] = {0.3f, 0.3f, 0.1f};
    float u[2];
    const MpcResult cheap = rti.step(x0, kBoxMin, kBoxMax, u);
    CHECK(cheap.kktResidual < 0.0f);

    rti.config.certifyOptimality = true;
    const MpcResult certified = rti.step(x0, kBoxMin, kBoxMax, u);
    CHECK(certified.kktResidual >= 0.0f);
}

CHAD_TEST(rti, earlyExitToleranceStopsSpendingTheBudget) {
    CondensedMpc mpc = makeMpc();
    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 200;
    rti.config.tol = 1e-5f;
    rti.config.shiftWarmStart = false;

    const float x0[3] = {0.25f, 0.30f, 0.12f};
    float u[2];
    const MpcResult first = rti.step(x0, kBoxMin, kBoxMax, u);
    const MpcResult second = rti.step(x0, kBoxMin, kBoxMax, u);
    CHECK(second.iterations <= first.iterations);
    CHECK(second.iterations < 200);
}

CHAD_TEST(rti, rateLimitedTicksNeverJerkEvenOnAOneIterationBudget) {
    // One iteration of ADMM per tick will not certify anything, so the rate
    // bound on the COMMITTED move cannot rest on convergence. It is enforced on
    // the way out, which is the only version of the guarantee worth having on a
    // drivetrain: the point of the limit is that no tick ever slips a wheel.
    CondensedMpc mpc = makeMpc(10, 1.4f);
    const float du[2] = {0.01f, 0.02f};
    mpc.setInputRateLimit(du);

    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 1;
    rti.config.tol = 0.0f;
    rti.config.trackPreviousInput = true;

    const float x0[3] = {2.0f, 2.0f, 1.0f};   // far outside the box, on purpose
    float prev[2] = {0.0f, 0.0f};
    float u[2];
    bool moved = false;
    for (int tick = 0; tick < 120; tick++) {
        const MpcResult res = rti.step(x0, kBoxMin, kBoxMax, u);
        CHECK(res.usedAdmm);
        CHECK_FINITE(u[0]);
        CHECK_FINITE(u[1]);
        for (int a = 0; a < 2; a++) {
            CHECK(std::fabs(u[a] - prev[a]) <= du[a] + 1e-5f);
            CHECK(u[a] >= kBoxMin[a] - 1e-6f);
            CHECK(u[a] <= kBoxMax[a] + 1e-6f);
            prev[a] = u[a];
        }
        if (std::fabs(u[0]) > 1e-4f) moved = true;
    }
    // A limit that froze the controller at zero would satisfy the bound too.
    CHECK(moved);
    CHECK(std::fabs(prev[0]) > 10.0f * du[0]);
}

CHAD_TEST(rti, aBindingRateLimitHoldsFromTheFirstTick) {
    // A tight rate limit off a zero anchor saturates immediately, so the very
    // first committed move is already the converged one. That is the guarantee
    // worth having -- the limit binds even when the solve is budget-capped --
    // and it is why this test cannot also assert that the error shrinks.
    CondensedMpc mpc = makeMpc(10, 1.2f);
    const float du[2] = {0.05f, 0.10f};
    mpc.setInputRateLimit(du);
    mpc.admm.checkEvery = 1;

    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 1;
    rti.config.tol = 0.0f;
    rti.config.shiftWarmStart = false;
    rti.config.trackPreviousInput = false;

    const float x0[3] = {0.35f, 0.45f, 0.18f};

    CondensedMpc reference = makeMpc(10, 1.2f);
    reference.setInputRateLimit(du);
    reference.admm.checkEvery = 1;
    reference.zero();
    CHECK(reference.solve(x0, kBoxMin, kBoxMax, 20000, 1e-5f).converged);
    float target[2];
    reference.firstMove(target);

    float u[2] = {0.0f, 0.0f};
    for (int tick = 0; tick < 600; tick++) {
        rti.step(x0, kBoxMin, kBoxMax, u);
        CHECK(std::fabs(u[0]) <= du[0] + 1e-5f);
        CHECK(std::fabs(u[1]) <= du[1] + 1e-5f);
    }
    CHECK_NEAR(u[0], target[0], 2e-3);
    CHECK_NEAR(u[1], target[1], 2e-3);
}

CHAD_TEST(rti, generalConstraintsConvergeOverTicksOnAHeldState) {
    // Rate limit loose enough not to bind, so one iteration per tick really does
    // start away from the answer and has to walk to it.
    CondensedMpc mpc = makeMpc(10, 1.2f);
    const float du[2] = {10.0f, 10.0f};
    mpc.setInputRateLimit(du);
    mpc.admm.checkEvery = 1;

    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 1;
    rti.config.tol = 0.0f;
    rti.config.shiftWarmStart = false;
    rti.config.trackPreviousInput = false;

    const float x0[3] = {0.35f, 0.45f, 0.18f};

    CondensedMpc reference = makeMpc(10, 1.2f);
    reference.setInputRateLimit(du);
    reference.admm.checkEvery = 1;
    reference.zero();
    CHECK(reference.solve(x0, kBoxMin, kBoxMax, 20000, 1e-5f).converged);
    float target[2];
    reference.firstMove(target);

    float u[2] = {0.0f, 0.0f};
    float firstError = 0.0f;
    for (int tick = 0; tick < 600; tick++) {
        rti.step(x0, kBoxMin, kBoxMax, u);
        if (tick == 0) firstError = std::fabs(u[0] - target[0]);
    }
    const float finalError = std::fabs(u[0] - target[0]);
    CHECK(firstError > 1e-4f);
    CHECK(finalError < firstError);
    CHECK(finalError < 2e-3f);
}

CHAD_TEST(rti, previousInputTrackingCanBeTurnedOff) {
    CondensedMpc mpc = makeMpc(8, 1.0f);
    const float du[2] = {0.01f, 0.02f};
    mpc.setInputRateLimit(du);

    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 40;
    rti.config.trackPreviousInput = false;

    // With the anchor pinned at zero the committed move can never leave
    // +/- du, however many ticks pass.
    const float x0[3] = {2.0f, 2.0f, 1.0f};
    float u[2];
    for (int tick = 0; tick < 40; tick++) {
        rti.step(x0, kBoxMin, kBoxMax, u);
        CHECK(std::fabs(u[0]) <= du[0] + 1e-5f);
        CHECK(std::fabs(u[1]) <= du[1] + 1e-5f);
    }

    // Tracking on, the same run walks out to the box instead.
    rti.reset();
    rti.config.trackPreviousInput = true;
    for (int tick = 0; tick < 200; tick++) rti.step(x0, kBoxMin, kBoxMax, u);
    CHECK(std::fabs(u[0]) > 5.0f * du[0]);
}

CHAD_TEST(rti, budgetIsStillExactOnTheGeneralPath) {
    CondensedMpc mpc = makeMpc();
    const float du[2] = {0.02f, 0.05f};
    mpc.setInputRateLimit(du);
    mpc.admm.checkEvery = 1;

    RealTimeIteration rti(&mpc);
    rti.config.iterationsPerTick = 3;
    rti.config.tol = 0.0f;

    const float x0[3] = {0.4f, 0.5f, 0.2f};
    float u[2];
    for (int tick = 0; tick < 5; tick++) {
        const MpcResult res = rti.step(x0, kBoxMin, kBoxMax, u);
        CHECK(res.usedAdmm);
        CHECK(res.iterations <= 3);
        CHECK(res.constraintRows == 4 * mpc.horizon());
    }
}
