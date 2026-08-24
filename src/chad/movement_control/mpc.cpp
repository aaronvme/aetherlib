/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "chad/movement_control/mpc.hpp"

#include "chad/movement_control/riccati.hpp"

#include <cmath>

#include "chad/util/util.hpp"

namespace chad {

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

    assemble(w, terminalP, inputR);
}

/// Stage-varying condensing: Phi_k = A_{k-1}...A_0 and G[k][j] = A_{k-1}...A_{j+1} B_j,
/// both by forward recursion over the block rows. A single-speed horizon cannot
/// represent a reference that is braking into the end of a path, which is
/// exactly where the prediction matters most.
void CondensedMpc::setupVarying(const DrivetrainPlant* stages, int count,
                                const MpcWeights& w, const float terminalP[3][3],
                                const float inputR[2][2]) {
    if (stages == nullptr || count <= 0) return;
    mN = clamp(count, 1, MPC_MAX_HORIZON);
    const int nv = 2 * mN;

    for (int k = 0; k < mN; k++) {
        const DrivetrainPlant& stage = stages[k];
        const int row = 3 * k;

        for (int j = 0; j < nv; j++)
            for (int r = 0; r < 3; r++) mG[row + r][j] = 0.0f;

        if (k == 0) {
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++) mPhi[r][c] = stage.Ad[r][c];
        } else {
            const int prev = 3 * (k - 1);
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    float sum = 0.0f;
                    for (int m = 0; m < 3; m++) sum += stage.Ad[r][m] * mPhi[prev + m][c];
                    mPhi[row + r][c] = sum;
                }
            }
            for (int j = 0; j < 2 * k; j++) {
                for (int r = 0; r < 3; r++) {
                    float sum = 0.0f;
                    for (int m = 0; m < 3; m++) sum += stage.Ad[r][m] * mG[prev + m][j];
                    mG[row + r][j] = sum;
                }
            }
        }

        for (int r = 0; r < 3; r++) {
            mG[row + r][2 * k + 0] = stage.Bd[r][0];
            mG[row + r][2 * k + 1] = stage.Bd[r][1];
        }
    }

    assemble(w, terminalP, inputR);
}

void CondensedMpc::assemble(const MpcWeights& w, const float terminalP[3][3],
                            const float inputR[2][2]) {
    const int nv = 2 * mN;

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

    // The fast-gradient step is 1/L with L = lambda_max(H). Power iteration
    // approaches lambda_max from BELOW, so using it raw gives a step slightly
    // too long to guarantee descent; the margin and the Gershgorin row-sum
    // bound above it keep the iteration safe. Setup runs once per speed bucket,
    // not per tick, so this is computed for both solvers to keep the
    // convergence measure comparable between them.
    mPowerIters = 0;
    // Leading nv x nv only: mH is declared at capacity and the rows past nv are
    // stale from an earlier, longer horizon.
    float lam = linalg::spectralRadiusLeading<MPC_MAX_VARS>(mH, nv, 30, 1e-4f,
                                                            &mPowerIters);

    float rowSum = 0.0f;
    for (int i = 0; i < nv; i++) {
        float row = 0.0f;
        for (int j = 0; j < nv; j++) row += std::fabs(mH[i][j]);
        if (row > rowSum) rowSum = row;
    }

    lam *= 1.05f;
    if (rowSum > 1e-12f && lam > rowSum) lam = rowSum;
    mL = (lam > 1e-12f) ? lam : 1.0f;
    mStep = 1.0f / mL;

    // Re-condensing changes H, so the factorization and the state rows are
    // stale, but the warm start is not: consecutive speed buckets describe
    // nearly the same QP and the previous input sequence is still the best
    // available guess. It only stops meaning anything when the horizon length
    // itself changes.
    if (nv != mAssembledVars) {
        mAssembledVars = nv;
        zero();
    } else {
        for (int i = 0; i < nv; i++) mY[i] = mU[i];
    }
    mQpDirty = true;
}

float CondensedMpc::stationarityInf(const float uMin[2], const float uMax[2]) const {
    const int nv = 2 * mN;
    float worst = 0.0f;
    for (int i = 0; i < nv; i++) {
        float g = mLin[i];
        for (int j = 0; j < nv; j++) g += mH[i][j] * mU[j];
        const float lo = (i % 2 == 0) ? uMin[0] : uMin[1];
        const float hi = (i % 2 == 0) ? uMax[0] : uMax[1];
        const float moved = std::fabs(clamp(mU[i] - mStep * g, lo, hi) - mU[i]);
        if (moved > worst) worst = moved;
    }
    return worst;
}

float CondensedMpc::objective() const {
    const int nv = 2 * mN;
    float quad = 0.0f, lin = 0.0f;
    for (int i = 0; i < nv; i++) {
        float row = 0.0f;
        for (int j = 0; j < nv; j++) row += mH[i][j] * mU[j];
        quad += mU[i] * row;
        lin += mLin[i] * mU[i];
    }
    return 0.5f * quad + lin;
}

void CondensedMpc::zero() {
    for (int i = 0; i < MPC_MAX_VARS; i++) mU[i] = mY[i] = mUprev[i] = 0.0f;
    mQp.resetState();
}

void CondensedMpc::shift() {
    const int nv = 2 * mN;
    for (int i = 0; i + 2 < nv; i++) mU[i] = mU[i + 2];
    // Last block repeats, which is the best available guess for the new tail.
    for (int i = 0; i < nv; i++) mY[i] = mU[i];
    if (mQp.rows() > 0) {
        // shiftRows derives the slack from z, so z has to hold the shifted
        // primal first. A pending rebuild does not waste this: the multipliers
        // are carried across it by row identity.
        float* z = mQp.z();
        const int n = (nv < mQp.vars()) ? nv : mQp.vars();
        for (int i = 0; i < n; i++) z[i] = mU[i];
        mQp.shiftRows(mRowShiftFrom);
    }
}

void CondensedMpc::clearConstraints() {
    mRateActive = false;
    mRateMax[0] = mRateMax[1] = 0.0f;
    mRateAnchor[0] = mRateAnchor[1] = 0.0f;
    for (int i = 0; i < MPC_MAX_VARS; i++) mRefDelta[i] = 0.0f;
    mStateCount = 0;
    mQpDirty = true;
}

void CondensedMpc::setInputRateLimit(const float duMax[2]) {
    const float d0 = (std::isfinite(duMax[0]) && duMax[0] > 0.0f) ? duMax[0] : 0.0f;
    const float d1 = (std::isfinite(duMax[1]) && duMax[1] > 0.0f) ? duMax[1] : 0.0f;
    const bool active = (d0 > 0.0f) || (d1 > 0.0f);
    if (active == mRateActive && d0 == mRateMax[0] && d1 == mRateMax[1]) return;
    mRateMax[0] = d0;
    mRateMax[1] = d1;
    mRateActive = active;
    mQpDirty = true;
}

void CondensedMpc::setPreviousInput(const float uPrev[2]) {
    for (int a = 0; a < 2; a++) {
        mRateAnchor[a] = std::isfinite(uPrev[a]) ? uPrev[a] : 0.0f;
    }
}

void CondensedMpc::setReferenceDeltas(const float* d, int count) {
    const int n = (d == nullptr) ? 0 : clamp(count, 0, MPC_MAX_VARS);
    for (int i = 0; i < n; i++) mRefDelta[i] = std::isfinite(d[i]) ? d[i] : 0.0f;
    for (int i = n; i < MPC_MAX_VARS; i++) mRefDelta[i] = 0.0f;
}

int CondensedMpc::rowsForSpec(const StateSpec& s) const {
    if (mN <= 0) return 0;
    const int first = clamp(s.first, 1, mN);
    const int last = clamp((s.last <= 0) ? mN : s.last, first, mN);
    return last - first + 1;
}

int CondensedMpc::plannedRows() const {
    if (mN <= 0) return 0;
    int rows = 2 * mN;   // box
    if (mRateActive) {
        rows += mN * (((mRateMax[0] > 0.0f) ? 1 : 0) + ((mRateMax[1] > 0.0f) ? 1 : 0));
    }
    for (int s = 0; s < mStateCount; s++) rows += rowsForSpec(mStateSpec[s]);
    return rows;
}

bool CondensedMpc::addStateConstraint(const float a[3], float lo, float hi,
                                      int firstStage, int lastStage) {
    if (mStateCount >= MPC_MAX_STATE_CONSTRAINTS) return false;
    if (!std::isfinite(lo) || !std::isfinite(hi) || lo > hi) return false;
    float mag = 0.0f;
    for (int i = 0; i < 3; i++) {
        if (!std::isfinite(a[i])) return false;
        mag += std::fabs(a[i]);
    }
    if (!(mag > 0.0f)) return false;

    StateSpec candidate;
    for (int i = 0; i < 3; i++) candidate.a[i] = a[i];
    candidate.lo = lo;
    candidate.hi = hi;
    candidate.first = firstStage;
    candidate.last = lastStage;
    // Budget the ROWS, not the spec count: returning true and then losing the
    // rows in rebuildQp() hands the caller a constraint it does not have.
    if (plannedRows() + rowsForSpec(candidate) > MPC_MAX_QP_ROWS) return false;

    mStateSpec[mStateCount++] = candidate;
    mQpDirty = true;
    return true;
}

int CondensedMpc::rowKey(unsigned char kind, unsigned char axis, short stage) {
    const RowKind k = static_cast<RowKind>(kind);
    const int block = (k == RowKind::Box) ? 0 : ((k == RowKind::State) ? 2 : 1);
    const int group = (k == RowKind::State) ? axis : 0;
    const int sub = (k == RowKind::State) ? 0 : axis;
    return ((block * 8 + group) * (MPC_MAX_HORIZON + 2) + stage) * 4 + sub;
}

void CondensedMpc::rebuildQp() {
    mQpDirty = false;
    mDroppedRows = 0;
    const int nv = 2 * mN;

    // A re-condensing changes C, but not what each row MEANS. Carrying the
    // multipliers over by identity is the difference between a warm start and a
    // cold one at every speed-bucket edge, which on a real trajectory is
    // constantly. commit() zeroes them, so they are saved first and replayed
    // after. Also picks up whatever shift() just did, so the shift is not lost.
    const int savedRows = mQp.rows();
    {
        const float* w = mQp.slack();
        const float* y = mQp.dual();
        for (int r = 0; r < savedRows; r++) {
            mSavedKey[r] = rowKey(mRowKind[r], mRowAxis[r], mRowStage[r]);
            mSavedW[r] = w[r];
            mSavedY[r] = y[r];
        }
    }

    mQp.begin(nv);
    if (nv == 0) return;

    for (int i = 0; i < nv; i++) {
        const int r = mQp.addUnitRow(i, 1.0f);
        if (r < 0) { mDroppedRows++; continue; }
        mRowKind[r] = static_cast<unsigned char>(RowKind::Box);
        mRowAxis[r] = static_cast<unsigned char>(i % 2);
        mRowStage[r] = static_cast<short>(i / 2);
        mRowLo[r] = mRowHi[r] = 0.0f;
        for (int c = 0; c < 3; c++) mRowPhi[r][c] = 0.0f;
        mRowShiftFrom[r] = (i / 2 + 1 < mN) ? static_cast<short>(r + 2) : -1;
    }

    if (mRateActive) {
        const int stride = ((mRateMax[0] > 0.0f) ? 1 : 0) + ((mRateMax[1] > 0.0f) ? 1 : 0);
        for (int k = 0; k < mN; k++) {
            for (int a = 0; a < 2; a++) {
                if (!(mRateMax[a] > 0.0f)) continue;
                const int r = (k == 0) ? mQp.addUnitRow(2 * k + a, 1.0f)
                                       : mQp.addPairRow(2 * k + a, 2 * (k - 1) + a);
                if (r < 0) { mDroppedRows++; continue; }
                mRowKind[r] = static_cast<unsigned char>((k == 0) ? RowKind::RateHead
                                                                  : RowKind::Rate);
                mRowAxis[r] = static_cast<unsigned char>(a);
                mRowStage[r] = static_cast<short>(k);
                mRowLo[r] = mRowHi[r] = 0.0f;
                for (int c = 0; c < 3; c++) mRowPhi[r][c] = 0.0f;
                // Stage 0's row is u_0 against a committed command, not a
                // difference of two decision variables, so a later stage's
                // multiplier does not transfer into it.
                mRowShiftFrom[r] = (k > 0 && k + 1 < mN) ? static_cast<short>(r + stride) : -1;
            }
        }
    }

    for (int s = 0; s < mStateCount; s++) {
        const StateSpec& spec = mStateSpec[s];
        const int first = clamp(spec.first, 1, mN);
        const int last = clamp((spec.last <= 0) ? mN : spec.last, first, mN);
        int prev = -1;
        for (int k = first; k <= last; k++) {
            const int b = 3 * (k - 1);
            float peak = 0.0f;
            bool finite = true;
            for (int j = 0; j < nv; j++) {
                mRowScratch[j] = spec.a[0] * mG[b + 0][j] + spec.a[1] * mG[b + 1][j] +
                                 spec.a[2] * mG[b + 2][j];
                // Tracked separately from the peak: NaN > peak is false, so a
                // NaN row would otherwise pass for an all-zero one.
                if (!std::isfinite(mRowScratch[j])) { finite = false; continue; }
                const float m = std::fabs(mRowScratch[j]);
                if (m > peak) peak = m;
            }
            const int r = mQp.addRow(mRowScratch);
            if (r < 0) {
                // An all-zero row constrains nothing and is not a loss. Any
                // other refusal is a constraint the caller asked for and did
                // not get, and must not pass for a solved problem.
                if (peak > 0.0f || !finite) mDroppedRows++;
                continue;
            }
            mRowKind[r] = static_cast<unsigned char>(RowKind::State);
            // The spec index, not an axis: two specs can constrain the same
            // stage, and rowKey() needs to tell those rows apart.
            mRowAxis[r] = static_cast<unsigned char>(s);
            mRowStage[r] = static_cast<short>(k);
            mRowLo[r] = spec.lo;
            mRowHi[r] = spec.hi;
            for (int c = 0; c < 3; c++) {
                mRowPhi[r][c] = spec.a[0] * mPhi[b + 0][c] + spec.a[1] * mPhi[b + 1][c] +
                                spec.a[2] * mPhi[b + 2][c];
            }
            mRowShiftFrom[r] = -1;
            if (prev >= 0) mRowShiftFrom[prev] = static_cast<short>(r);
            prev = r;
        }
    }

    // Replay by identity. Both sequences are generated in rowKey order, so one
    // forward scan matches them; a row with no counterpart keeps commit()'s zero.
    {
        float* w = mQp.slack();
        float* y = mQp.dual();
        const int rows = mQp.rows();
        int src = 0;
        for (int r = 0; r < rows; r++) {
            const int key = rowKey(mRowKind[r], mRowAxis[r], mRowStage[r]);
            while (src < savedRows && mSavedKey[src] < key) src++;
            if (src < savedRows && mSavedKey[src] == key) {
                w[r] = mSavedW[src];
                y[r] = mSavedY[src];
                src++;
            }
        }
    }

    float rho = admm.rho;
    if (!(rho > 0.0f) || !std::isfinite(rho)) {
        const float scale = (std::isfinite(admm.rhoScale) && admm.rhoScale > 0.0f)
                                ? admm.rhoScale : 1.0f;
        rho = scale * mQp.suggestRho(mH);
    }

    // State rows are penalised at their own multiple of rho. Armed here rather
    // than in solve(): it changes the KKT matrix, so it belongs with the
    // factorization and must not be touched per tick.
    const float sw = (std::isfinite(admm.rhoStateWeight) && admm.rhoStateWeight > 0.0f)
                         ? admm.rhoStateWeight : 1.0f;
    if (sw != 1.0f) {
        const int rows = mQp.rows();
        for (int r = 0; r < rows; r++) {
            mRowRho[r] = (static_cast<RowKind>(mRowKind[r]) == RowKind::State) ? sw : 1.0f;
        }
        mQp.setRowRhoWeight(mRowRho, rows);
    } else {
        mQp.setRowRhoWeight(nullptr, 0);
    }
    mQp.factorize(mH, rho, admm.sigma);
}

void CondensedMpc::fillQpBounds(const float x0[3], const float uMin[2],
                                const float uMax[2]) {
    float* lo = mQp.lower();
    float* hi = mQp.upper();
    const int rows = mQp.rows();
    for (int r = 0; r < rows; r++) {
        const int a = mRowAxis[r];
        switch (static_cast<RowKind>(mRowKind[r])) {
            case RowKind::Box:
                lo[r] = uMin[a];
                hi[r] = uMax[a];
                break;
            case RowKind::RateHead:
                lo[r] = mRateAnchor[a] - mRateMax[a];
                hi[r] = mRateAnchor[a] + mRateMax[a];
                break;
            case RowKind::Rate: {
                const float d = mRefDelta[2 * mRowStage[r] + a];
                lo[r] = -mRateMax[a] - d;
                hi[r] = mRateMax[a] - d;
                break;
            }
            case RowKind::State: {
                const float free = mRowPhi[r][0] * x0[0] + mRowPhi[r][1] * x0[1] +
                                   mRowPhi[r][2] * x0[2];
                lo[r] = mRowLo[r] - free;
                hi[r] = mRowHi[r] - free;
                break;
            }
        }
    }
}

namespace {
constexpr float kBoundEpsilon = 1e-7f;
constexpr float kHugeBound = 1e18f;
}  // namespace

MpcResult CondensedMpc::solveBox(const float uMin[2], const float uMax[2],
                                 int maxIters, float tol) {
    MpcResult res;
    const int nv = 2 * mN;
    const int cap = (maxIters > 0) ? maxIters : 1;

    // Warm start must itself be feasible, or the first momentum extrapolation
    // can leave the box and never come back within a single iteration.
    for (int i = 0; i < nv; i++) {
        const float lo = (i % 2 == 0) ? uMin[0] : uMin[1];
        const float hi = (i % 2 == 0) ? uMax[0] : uMax[1];
        mU[i] = clamp(mU[i], lo, hi);
        mY[i] = mU[i];
    }

    // Held only while it describes the CURRENT iterate: an exit at the cap must
    // not report a residual measured several iterations ago.
    float resid = -1.0f;

    if (solver == MpcSolver::GaussSeidel) {
        for (int it = 0; it < cap; it++) {
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
            if (stepInf <= tol) {
                if (!certifyOptimality) break;
                resid = stationarityInf(uMin, uMax);
                if (resid <= tol) break;
                resid = -1.0f;
            }
        }
    } else {
        float theta = 1.0f;
        for (int i = 0; i < nv; i++) mUprev[i] = mU[i];

        for (int it = 0; it < cap; it++) {
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

            // Gradient-based adaptive restart: once the momentum direction
            // starts opposing descent the iteration is ringing rather than
            // converging, and dropping theta back to 1 beats riding it out.
            float align = 0.0f;
            for (int i = 0; i < nv; i++) align += mGrad[i] * (mU[i] - mUprev[i]);
            if (align > 0.0f) {
                theta = 1.0f;
                res.restarts++;
                for (int i = 0; i < nv; i++) mY[i] = mU[i];
            } else {
                const float thetaNext =
                    0.5f * (1.0f + std::sqrt(1.0f + 4.0f * theta * theta));
                const float beta = (theta - 1.0f) / thetaNext;
                theta = thetaNext;
                for (int i = 0; i < nv; i++) mY[i] = mU[i] + beta * (mU[i] - mUprev[i]);
            }

            res.iterations = it + 1;
            res.stepInf = stepInf;
            if (stepInf <= tol) {
                if (!certifyOptimality) break;
                resid = stationarityInf(uMin, uMax);
                if (resid <= tol) break;
                resid = -1.0f;
            }
        }
    }

    if (certifyOptimality) {
        if (resid < 0.0f) resid = stationarityInf(uMin, uMax);
        res.kktResidual = resid;
        res.converged = resid <= tol;
    }
    res.hitIterationCap = (res.iterations >= cap) && !res.converged;
    return res;
}

MpcResult CondensedMpc::solveAdmm(const float x0[3], const float uMin[2],
                                  const float uMax[2], int maxIters, float tol) {
    if (mQpDirty) rebuildQp();

    if (!mQp.factorized()) {
        // No usable factor means the general rows cannot be honoured at all.
        // Emitting a box-feasible input and saying so beats emitting garbage.
        MpcResult res = solveBox(uMin, uMax, maxIters, tol);
        res.factorizationFailed = true;
        res.usedAdmm = true;
        res.constraintRows = mQp.rows();
        res.droppedRows = mDroppedRows;
        return res;
    }

    fillQpBounds(x0, uMin, uMax);

    const int nv = 2 * mN;
    float* z = mQp.z();
    for (int i = 0; i < nv; i++) z[i] = mU[i];

    QpTolerance qt;
    qt.primal = tol;
    qt.dualAbs = (admm.epsDualAbs > 0.0f) ? admm.epsDualAbs : tol;
    qt.dualRel = (std::isfinite(admm.epsDualRel) && admm.epsDualRel > 0.0f)
                     ? admm.epsDualRel : 0.0f;

    const int check = certifyOptimality ? ((admm.checkEvery > 0) ? admm.checkEvery : 1) : 0;
    const QpResult q = mQp.solve(mLin, maxIters, qt, check, admm.alpha);
    for (int i = 0; i < nv; i++) mU[i] = z[i];

    MpcResult res;
    res.usedAdmm = true;
    res.stepInf = -1.0f;
    res.iterations = q.iterations;
    res.primalResidual = q.primalResidual;
    res.dualResidual = q.dualResidual;
    res.converged = q.converged;
    res.hitIterationCap = q.hitIterationCap;
    res.relaxedRows = q.relaxedRows;
    res.activeRows = q.activeRows;
    res.constraintRows = mQp.rows();
    res.rho = q.rho;
    res.primalInfeasible = q.primalInfeasible;
    res.droppedRows = mDroppedRows;
    res.worstViolation = q.worstViolation;

    // Attribution only. Naming the KIND of row the mixed-units maximum came
    // from is the whole value of the number: a box row is in input units and
    // gets clamped downstream, a state row is in the spec's units and does not.
    {
        const int wr = q.worstViolationRow;
        if (wr >= 0 && wr < mQp.rows() &&
            static_cast<RowKind>(mRowKind[wr]) == RowKind::State) {
            res.worstViolationSpec = mRowAxis[wr];
        }
    }
    if (mDroppedRows > 0) res.converged = false;
    if (q.primalResidual >= 0.0f && q.dualResidual >= 0.0f) {
        res.kktResidual = (q.primalResidual > q.dualResidual) ? q.primalResidual
                                                             : q.dualResidual;
    }
    return res;
}

void CondensedMpc::measureStateViolation(const float x0[3], MpcResult& res) const {
    const int rows = mQp.rows();
    for (int r = 0; r < rows; r++) {
        if (static_cast<RowKind>(mRowKind[r]) != RowKind::State) continue;
        const int s = mRowAxis[r];
        if (s < 0 || s >= MPC_MAX_STATE_CONSTRAINTS) continue;
        const float* a = mQp.rowOf(r);
        float acc = 0.0f;
        for (int j = mQp.rowBegin(r); j < mQp.rowEnd(r); j++) acc += a[j] * mU[j];
        // rowOf() is normalised to unit inf-norm; the spec's bounds are not.
        const float pred = acc / mQp.rowScale(r) +
                           mRowPhi[r][0] * x0[0] + mRowPhi[r][1] * x0[1] +
                           mRowPhi[r][2] * x0[2];
        const float d = (mRowLo[r] - pred > pred - mRowHi[r]) ? mRowLo[r] - pred
                                                              : pred - mRowHi[r];
        if (d > res.stateViolation[s]) res.stateViolation[s] = d;
    }
}

MpcResult CondensedMpc::solve(const float x0[3], const float uMin[2], const float uMax[2],
                              int maxIters, float tol) {
    if (mN == 0) return MpcResult{};
    const int nv = 2 * mN;

    // A dead sensor must not steer. Substituting zero error means the solve
    // decays the correction toward the reference instead of jumping on a NaN,
    // and the flag lets the caller notice.
    bool clean = true;
    float x[3];
    for (int i = 0; i < 3; i++) {
        x[i] = x0[i];
        if (!std::isfinite(x[i])) {
            x[i] = 0.0f;
            clean = false;
        }
    }
    float lo2[2], hi2[2];
    for (int a = 0; a < 2; a++) {
        float lo = uMin[a], hi = uMax[a];
        if (!std::isfinite(lo)) { lo = -kHugeBound; clean = false; }
        if (!std::isfinite(hi)) { hi = kHugeBound; clean = false; }
        if (lo > hi) {
            const float mid = 0.5f * (lo + hi);
            lo = hi = mid;
            clean = false;
        }
        lo2[a] = lo;
        hi2[a] = hi;
    }

    for (int i = 0; i < nv; i++) {
        mLin[i] = mF[i][0] * x[0] + mF[i][1] * x[1] + mF[i][2] * x[2];
    }

    const bool general = hasGeneralConstraints() || solver == MpcSolver::Admm;
    MpcResult res = general ? solveAdmm(x, lo2, hi2, maxIters, tol)
                            : solveBox(lo2, hi2, maxIters, tol);
    res.stateNotFinite = !clean;

    for (int i = 0; i < nv; i++) {
        if (std::isfinite(mU[i])) continue;
        zero();
        res.solutionNotFinite = true;
        res.converged = false;
        res.kktResidual = -1.0f;
        break;
    }

    // The rate limit exists to stop a command that slips the wheels, and a
    // capped ADMM solve does not guarantee it. Only the head of the horizon is
    // ever committed, so clamping that one block is enough, and a converged
    // solve already satisfies it -- this moves nothing then.
    if (mRateActive) {
        for (int a = 0; a < 2 && a < nv; a++) {
            if (!(mRateMax[a] > 0.0f)) continue;
            mU[a] = clamp(mU[a], mRateAnchor[a] - mRateMax[a], mRateAnchor[a] + mRateMax[a]);
        }
    }

    // The box is the hardware limit, so it wins over everything above: a capped
    // or uncertified general solve still returns a deliverable input. Past the
    // primal tolerance this moves nothing.
    for (int i = 0; i < nv; i++) {
        const float lo = (i % 2 == 0) ? lo2[0] : lo2[1];
        const float hi = (i % 2 == 0) ? hi2[0] : hi2[1];
        mU[i] = clamp(mU[i], lo, hi);
        if (mU[i] <= lo + kBoundEpsilon || mU[i] >= hi - kBoundEpsilon) res.activeBounds++;
    }

    // After the clamps, so it scores the plan that is actually returned. The
    // clamps move the input a long way on a diverging iterate, and a corridor
    // number taken before them describes a sequence nobody executes.
    if (mStateCount > 0 && res.usedAdmm) measureStateViolation(x, res);
    return res;
}

long CondensedMpc::flopsPerIteration() const {
    const long nv = 2 * mN;
    if (solver == MpcSolver::Admm || hasGeneralConstraints()) {
        const long rows = mQp.rows();
        const long nnz = mQp.nonzeros();
        // Two triangular solves (nv^2 each, counted as multiply-add), the rhs
        // and C z~ passes over the stored support, and the relaxation and dual
        // updates.
        return 2 * nv * nv + 4 * nnz + 6 * nv + 8 * rows;
    }
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
    long qp = 0;
    if (mQp.rows() > 0) {
        // C'C accumulation over the stored support, then a dense Cholesky.
        qp = 2L * mQp.nonzeros() * mQp.nonzeros() / (mQp.rows() > 0 ? mQp.rows() : 1) +
             nv * nv * nv / 3;
    }
    return h + f + pi + qp;
}

}  // namespace chad
