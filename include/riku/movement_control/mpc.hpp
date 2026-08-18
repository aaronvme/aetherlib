/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include "riku/movement_control/mpc_plant.hpp"
#include "riku/movement_control/mpc_lqr.hpp"

namespace riku {

// Box-constrained linear MPC over DrivetrainPlant. The states are condensed out,
// so the decision variable is the input sequence alone:
//
//     min 1/2 U' H U + (F x0)' U   s.t.  uMin <= u_k <= uMax
//
// H and F depend only on reference speed, which is why setup() is the expensive
// half and solve() touches nothing but preallocated members.
//
// The point of the box is that an LQR cannot respect a saturation limit -- it
// emits the unconstrained optimum and something downstream clips it, which is a
// different and generally worse trajectory than planning inside the limit.
//
// Give it the DARE solution P from lqr.hpp as the terminal cost and, with no
// constraint active, the first move equals the LQR move at any horizon. That
// equivalence is the correctness gate in sim/test/mpc.mjs.

constexpr int MPC_MAX_HORIZON = 20;
constexpr int MPC_MAX_VARS = 2 * MPC_MAX_HORIZON;

enum class MpcSolver {
    /// Projected gradient with Nesterov momentum (FISTA). Fixed step 1/L,
    /// L = lambda_max(H). One iteration is one H matvec.
    FastGradient,
    /// One projected Gauss-Seidel sweep, i.e. coordinate-wise exact
    /// minimisation with clamping. Same flop count per sweep as the gradient
    /// step, strictly better contraction on a diagonally dominant H.
    GaussSeidel,
};

struct MpcResult {
    int iterations{0};
    float stepInf{0.0f};      // ||U_k - U_{k-1}||_inf on the last iteration
    bool converged{false};
    int activeBounds{0};      // decision variables sitting on a bound
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

    /// Solve to `maxIters` or until ||dU||_inf <= tol. Warm starts from
    /// whatever is currently in the solution buffer.
    MpcResult solve(const float x0[3], const float uMin[2], const float uMax[2],
                    int maxIters, float tol);

    /// Shift the solution one step forward, repeating the last move. The
    /// standard real-time-iteration warm start: at the next tick the horizon
    /// has advanced by one, so u_1 is the new u_0.
    void shift();

    void zero();

    const float* solution() const { return mU; }
    void firstMove(float u[2]) const { u[0] = mU[0]; u[1] = mU[1]; }

    int horizon() const { return mN; }
    int vars() const { return 2 * mN; }
    float lipschitz() const { return mL; }
    int powerIterations() const { return mPowerIters; }
    MpcSolver solver{MpcSolver::FastGradient};

    /// Analytic flop counts, for cost projection onto hardware that cannot be
    /// profiled directly. Multiply-and-add counted as two flops.
    long flopsPerIteration() const;
    long flopsSetup() const;

private:
    int mN{0};
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
};

}  // namespace riku
