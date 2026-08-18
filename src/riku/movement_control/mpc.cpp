/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "riku/movement_control/mpc.hpp"

#include <cmath>

#include "riku/util/util.hpp"

namespace riku {

void CondensedMpc::setup(const DrivetrainPlant& plant, const MpcWeights& w, int horizon,
                         const float terminalP[3][3], const float inputR[2][2]) {
    mN = clamp(horizon, 1, MPC_MAX_HORIZON);
    const int nv = 2 * mN;

    // Phi_k = Ad^k, G[k][j] = Ad^(k-1-j) Bd for j < k. Both closed form for
    // this plant; see DrivetrainPlant.
    for (int k = 1; k <= mN; k++) {
        float Ak[3][3];
        plant.powerA(k, Ak);
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) mPhi[3 * (k - 1) + r][c] = Ak[r][c];

        for (int j = 0; j < nv; j++)
            for (int r = 0; r < 3; r++) mG[3 * (k - 1) + r][j] = 0.0f;

        for (int j = 0; j < k; j++) {
            float AB[3][2];
            plant.powerAtimesB(k - 1 - j, AB);
            for (int r = 0; r < 3; r++) {
                mG[3 * (k - 1) + r][2 * j + 0] = AB[r][0];
                mG[3 * (k - 1) + r][2 * j + 1] = AB[r][1];
            }
        }
    }

    // Qbar is diagonal: Q on stages 1..N-1, terminalP (or Q) on stage N.
    const float q[3] = {w.qx, w.qy, w.qTheta};

    // H = 2 (G' Qbar G + Rbar). Qbar diagonal on the first N-1 blocks keeps
    // that part a weighted dot product; only the terminal block is full.
    for (int i = 0; i < nv; i++) {
        for (int j = i; j < nv; j++) {
            float s = 0.0f;
            for (int k = 0; k < mN - 1; k++) {
                const int b = 3 * k;
                s += q[0] * mG[b + 0][i] * mG[b + 0][j] +
                     q[1] * mG[b + 1][i] * mG[b + 1][j] +
                     q[2] * mG[b + 2][i] * mG[b + 2][j];
            }
            const int bt = 3 * (mN - 1);
            if (terminalP != nullptr) {
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        s += mG[bt + r][i] * terminalP[r][c] * mG[bt + c][j];
            } else {
                s += q[0] * mG[bt + 0][i] * mG[bt + 0][j] +
                     q[1] * mG[bt + 1][i] * mG[bt + 1][j] +
                     q[2] * mG[bt + 2][i] * mG[bt + 2][j];
            }
            // Input weight, block diagonal in time. Off-diagonal within a
            // block is nonzero once the input has been transformed.
            if (i / 2 == j / 2) {
                s += (inputR != nullptr) ? inputR[i % 2][j % 2]
                                         : ((i == j) ? ((i % 2 == 0) ? w.rv : w.rw) : 0.0f);
            }
            mH[i][j] = 2.0f * s;
            mH[j][i] = mH[i][j];
        }
    }

    // F = 2 G' Qbar Phi.
    for (int i = 0; i < nv; i++) {
        for (int c = 0; c < 3; c++) {
            float s = 0.0f;
            for (int k = 0; k < mN - 1; k++) {
                const int b = 3 * k;
                s += q[0] * mG[b + 0][i] * mPhi[b + 0][c] +
                     q[1] * mG[b + 1][i] * mPhi[b + 1][c] +
                     q[2] * mG[b + 2][i] * mPhi[b + 2][c];
            }
            const int bt = 3 * (mN - 1);
            if (terminalP != nullptr) {
                for (int r = 0; r < 3; r++)
                    for (int cc = 0; cc < 3; cc++)
                        s += mG[bt + r][i] * terminalP[r][cc] * mPhi[bt + cc][c];
            } else {
                s += q[0] * mG[bt + 0][i] * mPhi[bt + 0][c] +
                     q[1] * mG[bt + 1][i] * mPhi[bt + 1][c] +
                     q[2] * mG[bt + 2][i] * mPhi[bt + 2][c];
            }
            mF[i][c] = 2.0f * s;
        }
    }

    for (int i = 0; i < nv; i++) {
        mDiagInv[i] = (mH[i][i] > 1e-20f) ? 1.0f / mH[i][i] : 0.0f;
    }

    // lambda_max(H) sets the fast-gradient step to 1/L, the largest step that
    // still descends. Gauss-Seidel does not use it, and the power iteration is
    // the single most expensive thing in setup(), so skip it entirely there.
    mPowerIters = 0;
    float lam = 0.0f;
    if (solver == MpcSolver::FastGradient) {
        float v[MPC_MAX_VARS], t[MPC_MAX_VARS];
        for (int i = 0; i < nv; i++) v[i] = 1.0f;
        for (int it = 0; it < 30; it++) {
            for (int i = 0; i < nv; i++) {
                float s = 0.0f;
                for (int j = 0; j < nv; j++) s += mH[i][j] * v[j];
                t[i] = s;
            }
            float nrm = 0.0f;
            for (int i = 0; i < nv; i++) nrm += t[i] * t[i];
            nrm = std::sqrt(nrm);
            if (nrm < 1e-20f) break;
            // Rayleigh quotient on the normalised iterate. H is symmetric, so
            // v'Hv / v'v converges quadratically in the eigenvalue.
            float ray = 0.0f;
            for (int i = 0; i < nv; i++) {
                v[i] = t[i] / nrm;
                ray += v[i] * t[i];
            }
            mPowerIters = it + 1;
            const bool done = std::fabs(ray - lam) < 1e-4f * (std::fabs(ray) + 1e-9f);
            lam = ray;
            if (done) break;
        }
    } else {
        // Row-sum bound, a rigorous upper bound on lambda_max for a symmetric
        // matrix. Costs one pass and is only used if the solver is switched.
        for (int i = 0; i < nv; i++) {
            float row = 0.0f;
            for (int j = 0; j < nv; j++) row += std::fabs(mH[i][j]);
            if (row > lam) lam = row;
        }
    }
    mL = (lam > 1e-12f) ? lam : 1.0f;
    mStep = 1.0f / mL;

    zero();
}

void CondensedMpc::zero() {
    for (int i = 0; i < MPC_MAX_VARS; i++) mU[i] = mY[i] = mUprev[i] = 0.0f;
}

void CondensedMpc::shift() {
    const int nv = 2 * mN;
    for (int i = 0; i + 2 < nv; i++) mU[i] = mU[i + 2];
    // Last block repeats, which is the best available guess for the new tail.
    for (int i = 0; i < nv; i++) mY[i] = mU[i];
}

MpcResult CondensedMpc::solve(const float x0[3], const float uMin[2], const float uMax[2],
                              int maxIters, float tol) {
    MpcResult res;
    if (mN == 0) return res;
    const int nv = 2 * mN;

    for (int i = 0; i < nv; i++) {
        mLin[i] = mF[i][0] * x0[0] + mF[i][1] * x0[1] + mF[i][2] * x0[2];
    }

    // Warm start must itself be feasible, or the first momentum extrapolation
    // can leave the box and never come back within a single iteration.
    for (int i = 0; i < nv; i++) {
        const float lo = (i % 2 == 0) ? uMin[0] : uMin[1];
        const float hi = (i % 2 == 0) ? uMax[0] : uMax[1];
        mU[i] = clamp(mU[i], lo, hi);
        mY[i] = mU[i];
    }

    if (solver == MpcSolver::GaussSeidel) {
        for (int it = 0; it < maxIters; it++) {
            float stepInf = 0.0f;
            for (int i = 0; i < nv; i++) {
                const float lo = (i % 2 == 0) ? uMin[0] : uMin[1];
                const float hi = (i % 2 == 0) ? uMax[0] : uMax[1];
                float s = mLin[i];
                for (int j = 0; j < nv; j++) s += mH[i][j] * mU[j];
                // Exact coordinate minimiser: u_i -= (grad_i) / H_ii.
                const float ui = clamp(mU[i] - s * mDiagInv[i], lo, hi);
                const float d = std::fabs(ui - mU[i]);
                if (d > stepInf) stepInf = d;
                mU[i] = ui;
            }
            res.iterations = it + 1;
            res.stepInf = stepInf;
            if (stepInf <= tol) { res.converged = true; break; }
        }
    } else {
        float theta = 1.0f;
        for (int i = 0; i < nv; i++) mUprev[i] = mU[i];

        for (int it = 0; it < maxIters; it++) {
            for (int i = 0; i < nv; i++) {
                float s = mLin[i];
                for (int j = 0; j < nv; j++) s += mH[i][j] * mY[j];
                mGrad[i] = s;
            }
            float stepInf = 0.0f;
            for (int i = 0; i < nv; i++) {
                const float lo = (i % 2 == 0) ? uMin[0] : uMin[1];
                const float hi = (i % 2 == 0) ? uMax[0] : uMax[1];
                const float ui = clamp(mY[i] - mStep * mGrad[i], lo, hi);
                const float d = std::fabs(ui - mU[i]);
                if (d > stepInf) stepInf = d;
                mUprev[i] = mU[i];
                mU[i] = ui;
            }
            const float thetaNext = 0.5f * (1.0f + std::sqrt(1.0f + 4.0f * theta * theta));
            const float beta = (theta - 1.0f) / thetaNext;
            theta = thetaNext;
            for (int i = 0; i < nv; i++) mY[i] = mU[i] + beta * (mU[i] - mUprev[i]);

            res.iterations = it + 1;
            res.stepInf = stepInf;
            if (stepInf <= tol) { res.converged = true; break; }
        }
    }

    for (int i = 0; i < nv; i++) {
        const float lo = (i % 2 == 0) ? uMin[0] : uMin[1];
        const float hi = (i % 2 == 0) ? uMax[0] : uMax[1];
        if (mU[i] <= lo + 1e-7f || mU[i] >= hi - 1e-7f) res.activeBounds++;
    }
    return res;
}

long CondensedMpc::flopsPerIteration() const {
    const long nv = 2 * mN;
    if (solver == MpcSolver::GaussSeidel) {
        // Per coordinate: one length-nv dot product (2*nv), plus the update.
        return nv * (2 * nv + 4);
    }
    // H matvec (2*nv*nv) + gradient step (4*nv) + momentum (6*nv).
    return 2 * nv * nv + 10 * nv;
}

long CondensedMpc::flopsSetup() const {
    const long nv = 2 * mN;
    // H: nv*(nv+1)/2 entries, each a weighted dot over N stage blocks.
    const long h = (nv * (nv + 1) / 2) * static_cast<long>(mN) * 8;
    const long f = nv * 3 * static_cast<long>(mN) * 8;
    // Step-size estimate: one matvec plus two length-nv reductions per sweep.
    const long pi = static_cast<long>(mPowerIters) * (2 * nv * nv + 6 * nv);
    return h + f + pi;
}

}  // namespace riku
