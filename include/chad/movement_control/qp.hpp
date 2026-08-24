/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include <cmath>
#include <limits>

namespace chad {

struct QpResult {
    int iterations{0};
    /// max_r |C_r z - w_r| / scale_r -- ADMM's SPLITTING inconsistency, not a
    /// constraint violation, and a per-row maximum rather than one physical
    /// quantity: each term is in its own row's units, so a problem mixing a
    /// box on m/s with a corridor on metres reports the larger of two numbers
    /// that cannot be compared. `worstPrimalRow` says which row it came from,
    /// which is the only way to know what unit the number is in.
    ///
    /// It bounds the violation from above -- w is always inside [l, u] -- but
    /// the bound is on the ATTAINING row, so it says nothing about any other.
    /// Use worstViolation, or a specific row's entry in rowViolations(), when
    /// the question is how far outside a particular constraint the iterate is.
    ///
    /// Always measured after a solve on a factorized problem: one pass over the
    /// row support, not the dual's dense H matvec. Negative only when no solve
    /// ran at all.
    float primalResidual{-1.0f};
    /// max_r dist(C_r z, [l_r, u_r]) / scale_r: the actual violation, zero when
    /// the iterate is feasible. Same mixed-units caveat as primalResidual --
    /// `worstViolationRow` attributes it. Measured against the bounds the solve
    /// used, so a relaxed row is scored against its collapsed midpoint.
    float worstViolation{0.0f};
    /// Row attaining primalResidual / worstViolation, or -1 when there were no
    /// rows. Zero is a valid row index, so -1 is the only safe "none".
    int worstPrimalRow{-1};
    int worstViolationRow{-1};
    /// ||H z + q + C' y||_inf. Needs an H matvec, so it is measured only when
    /// checkEvery > 0. Negative when it was not.
    float dualResidual{-1.0f};
    /// True only when both residuals were evaluated and both passed.
    bool converged{false};
    /// The iteration cap was reached. Lets a caller separate "spent the budget"
    /// from "exited early without certifying".
    bool hitIterationCap{false};
    bool factorizationFailed{false};
    float rho{0.0f};
    int activeRows{0};
    /// Rows whose bounds crossed and were collapsed to their midpoint.
    int relaxedRows{0};
    /// max(||Hz||_inf, ||C'y||_inf, ||q||_inf) at the reported dual residual.
    /// The scale the relative dual tolerance is measured against. Negative when
    /// the residuals were never evaluated.
    float dualScale{-1.0f};
    /// ||Cz||_inf in row units: the scale the relative PRIMAL tolerance is
    /// measured against.
    float primalScale{0.0f};
    /// (sum_j |C_rj|) * ||z||_inf / scale_r for the row that attained
    /// primalResidual: the magnitude of the TERMS its dot product sums, which
    /// is what bounds the float32 cancellation error in it.
    ///
    /// primalScale is the magnitude of the RESULT. Using that as a noise floor
    /// understates it by exactly however much the sum cancelled, and a row held
    /// near zero by a tight bound while the iterate is large cancels by three
    /// orders of magnitude -- which is enough to certify a feasible problem
    /// infeasible. Measured, not argued: see the guard test in mpc_qp_test.cpp.
    float primalTermScale{0.0f};
    /// The feasible set was found to be empty. Two sources, both cheap and both
    /// only SUFFICIENT: rows on a single variable whose intervals do not
    /// intersect, and a primal residual stuck above tolerance while the dual has
    /// converged. Nothing here builds an OSQP-style infeasibility certificate --
    /// that needs the dual iterate's direction and is out of scope. A false here
    /// means "not detected", not "feasible" -- and the second source is not
    /// evaluated at all when QpTolerance::primal is zero, since there is then no
    /// threshold for the residual to be stuck above.
    bool primalInfeasible{false};
};

/// Exit criterion. The primal residual is in the caller's own row units, so it
/// stays absolute -- a corridor tolerance is a physical quantity. The dual is in
/// gradient units, where float32 cancellation in a dense H matvec floors the
/// achievable residual around 1e-6 whatever the budget, so it carries OSQP's
/// relative term and cannot be chased below the noise.
struct QpTolerance {
    float primal{1e-4f};
    float dualAbs{1e-4f};
    /// eps_dual = dualAbs + dualRel * max(||Hz||_inf, ||C'y||_inf, ||q||_inf).
    float dualRel{1e-3f};
};

// ADMM for  min 1/2 z'Hz + q'z  s.t.  l <= C z <= u, in OSQP's splitting: the
// constraint becomes C z = w with multiplier y, and w carries the projection
// onto [l, u]. Each iteration solves
//
//     (H + sigma I + rho C'C) z~ = sigma z - q + C'(rho w - y)
//
// which is symmetric positive definite for sigma > 0 and PSD H. Its matrix
// depends on H, C, rho and sigma and on nothing that moves tick to tick, so it
// is Cholesky-factored once in factorize() and only back-substituted per
// iteration. rho is deliberately not adapted inside the loop: that would force
// a refactorization, which is the cost this structure exists to avoid.
//
// Rows are stored dense but carry their support [begin, end), because the rows
// a condensed MPC generates are one or two nonzeros wide in a decision vector
// forty long -- the matvecs are the inner loop and the span makes them cheap.
//
// Rows are also normalised to unit infinity-norm on the way in, which is the
// one piece of equilibration that pays for itself here: it stops a state row
// whose entries are O(dt^2) from being invisible next to a unit box row. The
// scale is remembered, so the reported primal residual is in the caller's own
// units, not scaled ones.
//
// H is held by POINTER and must outlive the solver. Nothing here allocates.
template <int MaxVars, int MaxRows>
class AdmmQp {
public:
    static constexpr int MAX_VARS = MaxVars;
    static constexpr int MAX_ROWS = MaxRows;

    /// Start a fresh constraint matrix over `n` variables. Drops the factor.
    void begin(int n) {
        mN = (n < 0) ? 0 : ((n > MaxVars) ? MaxVars : n);
        mRows = 0;
        mNnz = 0;
        mFactored = false;
        mH = nullptr;
        // Row identities change with the row set, so a weight indexed by row
        // must not survive one. The caller re-arms it after rebuilding.
        if (mHaveRowRho) setRowRhoWeight(nullptr, 0);
    }

    /// Append a dense row of length vars(). Returns its index, or -1 if the row
    /// budget is spent or the row is empty / non-finite.
    int addRow(const float* a) {
        if (mRows >= MaxRows || mN == 0) return -1;
        const int r = mRows;
        float peak = 0.0f;
        for (int j = 0; j < mN; j++) {
            const float v = a[j];
            mC[r][j] = v;
            // Tested per entry, not against the peak: NaN > peak is false, so a
            // NaN never becomes the peak and a peak test never sees it.
            if (!std::isfinite(v)) return -1;
            const float m = std::fabs(v);
            if (m > peak) peak = m;
        }
        if (!(peak > 0.0f)) return -1;
        int lo = 0, hi = mN;
        while (lo < hi && mC[r][lo] == 0.0f) lo++;
        while (hi > lo && mC[r][hi - 1] == 0.0f) hi--;
        return commit(lo, hi, 1.0f / peak);
    }

    /// Append `coeff * e_col`.
    int addUnitRow(int col, float coeff = 1.0f) {
        if (mRows >= MaxRows || col < 0 || col >= mN) return -1;
        const float m = std::fabs(coeff);
        if (!(m > 0.0f) || !std::isfinite(m)) return -1;
        const int r = mRows;
        for (int j = 0; j < mN; j++) mC[r][j] = 0.0f;
        mC[r][col] = coeff;
        return commit(col, col + 1, 1.0f / m);
    }

    /// Append `e_plus - e_minus`, the difference row a rate limit needs.
    int addPairRow(int plus, int minus) {
        if (mRows >= MaxRows) return -1;
        if (plus < 0 || plus >= mN || minus < 0 || minus >= mN || plus == minus) return -1;
        const int r = mRows;
        for (int j = 0; j < mN; j++) mC[r][j] = 0.0f;
        mC[r][plus] = 1.0f;
        mC[r][minus] = -1.0f;
        const int lo = (plus < minus) ? plus : minus;
        const int hi = ((plus > minus) ? plus : minus) + 1;
        return commit(lo, hi, 1.0f);
    }

    /// rho balancing mean(diag H) against mean(diag C'C), so neither term of
    /// the KKT matrix disappears into the other's rounding. Call after the rows
    /// are in place.
    float suggestRho(const float (*H)[MaxVars]) const {
        if (mN == 0 || mRows == 0 || H == nullptr) return 1.0f;
        float hs = 0.0f;
        for (int i = 0; i < mN; i++) hs += std::fabs(H[i][i]);
        hs /= static_cast<float>(mN);
        float cs = 0.0f;
        for (int r = 0; r < mRows; r++) {
            for (int j = mBegin[r]; j < mEnd[r]; j++) cs += mC[r][j] * mC[r][j];
        }
        cs /= static_cast<float>(mN);
        if (!(hs > 1e-20f) || !(cs > 1e-20f)) return 1.0f;
        const float rho = hs / cs;
        return std::isfinite(rho) ? rho : 1.0f;
    }

    /// Per-row weight on rho, in row order, taking effect at the next
    /// factorize(). A non-positive or non-finite entry, or a short count, means
    /// 1 for that row. Pass null to clear.
    ///
    /// This is a fixed WEIGHTING, not adaptation: rho is still constant for the
    /// life of a factorization, so nothing here costs a refactorization. It
    /// exists because one rho cannot suit both row shapes a condensed MPC
    /// generates -- a rate limit is a well-conditioned difference operator that
    /// wants rho large, while a corridor contributes rows that are nearly
    /// collinear from one stage to the next and whose C'C therefore wrecks the
    /// KKT conditioning when rho is large. Measured on both, separately.
    void setRowRhoWeight(const float* weight, int count) {
        mHaveRowRho = (weight != nullptr && count > 0);
        for (int r = 0; r < MaxRows; r++) {
            float w = 1.0f;
            if (mHaveRowRho && r < count && weight[r] > 0.0f &&
                std::isfinite(weight[r])) {
                w = weight[r];
            }
            mRhoWeight[r] = w;
        }
    }

    /// Factor H + sigma I + C' diag(rho_r) C. `H` is retained by pointer. Bumps
    /// sigma and retries if a pivot dies, and reports failure rather than
    /// emitting a factor that would produce garbage.
    bool factorize(const float (*H)[MaxVars], float rho, float sigma) {
        mFactored = false;
        if (H == nullptr || mN == 0) return false;
        if (!(rho > 0.0f) || !std::isfinite(rho)) return false;
        mH = H;
        mRho = rho;
        for (int r = 0; r < mRows; r++) {
            const float rr = mHaveRowRho ? rho * mRhoWeight[r] : rho;
            mRhoRow[r] = (rr > 0.0f && std::isfinite(rr)) ? rr : rho;
            mInvRhoRow[r] = 1.0f / mRhoRow[r];
        }

        float s = (sigma > 0.0f && std::isfinite(sigma)) ? sigma : 1e-6f;
        for (int attempt = 0; attempt < 4; attempt++) {
            for (int i = 0; i < mN; i++) {
                for (int j = 0; j <= i; j++) {
                    // Symmetrised on the way in: an asymmetric H makes the
                    // factor solve a different problem than the residual test.
                    mK[i][j] = 0.5f * (mH[i][j] + mH[j][i]);
                }
                mK[i][i] += s;
            }
            for (int r = 0; r < mRows; r++) {
                const int b = mBegin[r], e = mEnd[r];
                for (int i = b; i < e; i++) {
                    const float ci = mC[r][i];
                    if (ci == 0.0f) continue;
                    const float w = mRhoRow[r] * ci;
                    for (int j = b; j <= i; j++) mK[i][j] += w * mC[r][j];
                }
            }
            for (int i = 0; i < mN; i++) mPivotRef[i] = mK[i][i];
            if (cholesky()) {
                mSigma = s;
                mFactored = true;
                return true;
            }
            s *= 100.0f;
        }
        return false;
    }

    /// Refactor with the stored H and sigma at a new rho.
    bool refactorize(float rho) { return factorize(mH, rho, mSigma); }

    /// Absolute-only form, kept for callers that predate the relative dual
    /// term. Equivalent to dualRel = 0.
    QpResult solve(const float* q, int maxIters, float epsPrimal, float epsDual,
                   int checkEvery, float alpha = 1.6f) {
        return solve(q, maxIters, QpTolerance{.primal = epsPrimal, .dualAbs = epsDual, .dualRel = 0.0f}, checkEvery, alpha);
    }

    QpResult solve(const float* q, int maxIters, const QpTolerance& tol,
                   int checkEvery, float alpha = 1.6f) {
        QpResult res;
        res.rho = mRho;
        if (!mFactored) {
            res.factorizationFailed = true;
            return res;
        }
        if (mN == 0) {
            res.converged = true;
            res.primalResidual = 0.0f;
            res.dualResidual = 0.0f;
            return res;
        }

        const float a = (std::isfinite(alpha) && alpha > 0.0f && alpha < 2.0f) ? alpha : 1.6f;
        const float oneMinusA = 1.0f - a;
        const int check = (checkEvery > 0) ? checkEvery : 0;
        const int cap = (maxIters > 0) ? maxIters : 1;

        // Scaled bounds, and a crossed pair is collapsed rather than left to
        // make the problem infeasible: ADMM does not converge on an infeasible
        // QP, and the caller needs a finite answer regardless.
        for (int r = 0; r < mRows; r++) {
            float lo = mScale[r] * mLower[r];
            float hi = mScale[r] * mUpper[r];
            if (!std::isfinite(lo)) lo = -kHuge;
            if (!std::isfinite(hi)) hi = kHuge;
            if (lo > hi) {
                const float mid = 0.5f * (lo + hi);
                lo = hi = mid;
                res.relaxedRows++;
            }
            mLoS[r] = lo;
            mHiS[r] = hi;
            // The warm-started slack has to sit inside the CURRENT bounds or
            // the first dual update is computed against a stale feasible set.
            mW[r] = (mW[r] < lo) ? lo : ((mW[r] > hi) ? hi : mW[r]);
        }

        // Rows supported on a single variable must intersect. Two rows that are
        // each individually consistent -- the input box and an input-rate bound
        // off a stale anchor, say -- can still be jointly empty, and ADMM does
        // not report that: it parks at a fixed point with a standing primal
        // residual. Detected only, never repaired; which constraint should give
        // way is the caller's call, not the solver's.
        for (int i = 0; i < mN; i++) {
            mVarLo[i] = -kHuge;
            mVarHi[i] = kHuge;
        }
        for (int r = 0; r < mRows; r++) {
            if (mEnd[r] - mBegin[r] != 1) continue;
            const int j = mBegin[r];
            const float c = mC[r][j];
            if (c == 0.0f) continue;
            float lo = mLoS[r] / c, hi = mHiS[r] / c;
            if (c < 0.0f) { const float t = lo; lo = hi; hi = t; }
            if (lo > mVarLo[j]) mVarLo[j] = lo;
            if (hi < mVarHi[j]) mVarHi[j] = hi;
        }
        for (int i = 0; i < mN; i++) {
            const float slop =
                1e-6f * (1.0f + std::fabs(mVarLo[i]) + std::fabs(mVarHi[i]));
            if (mVarLo[i] > mVarHi[i] + slop) {
                res.primalInfeasible = true;
                break;
            }
        }

        float prevPrimal = -1.0f;
        int stalled = 0;

        for (int it = 0; it < cap; it++) {
            for (int i = 0; i < mN; i++) mRhs[i] = mSigma * mZ[i] - q[i];
            for (int r = 0; r < mRows; r++) {
                const float c = mRhoRow[r] * mW[r] - mY[r];
                if (c == 0.0f) continue;
                for (int j = mBegin[r]; j < mEnd[r]; j++) mRhs[j] += mC[r][j] * c;
            }
            backSolve(mRhs, mZt);

            for (int r = 0; r < mRows; r++) {
                float s = 0.0f;
                for (int j = mBegin[r]; j < mEnd[r]; j++) s += mC[r][j] * mZt[j];
                mWt[r] = s;
            }

            for (int i = 0; i < mN; i++) mZ[i] = a * mZt[i] + oneMinusA * mZ[i];
            for (int r = 0; r < mRows; r++) {
                const float relaxed = a * mWt[r] + oneMinusA * mW[r];
                const float t = relaxed + mY[r] * mInvRhoRow[r];
                const float wn = (t < mLoS[r]) ? mLoS[r] : ((t > mHiS[r]) ? mHiS[r] : t);
                mY[r] += mRhoRow[r] * (relaxed - wn);
                mW[r] = wn;
            }

            res.iterations = it + 1;
            const bool last = (it + 1 == cap);
            if (check > 0 && (((it + 1) % check) == 0 || last)) {
                residuals(q, res);
                const float epsDual = tol.dualAbs + tol.dualRel * res.dualScale;
                if (res.primalResidual <= tol.primal && res.dualResidual <= epsDual) {
                    res.converged = true;
                    break;
                }
                // A dual that has converged while the primal will not move is
                // the standard cheap signature of an empty feasible set. The
                // A residual sitting at the float32 cancellation floor of a
                // dense matvec is SOLVED, not stuck, and the two are otherwise
                // indistinguishable: dual converged, primal not moving. That is
                // what made this certify feasible problems as infeasible, and
                // more often the longer it ran -- 0/400 ticks at 16 iterations,
                // 296/400 at 160. The tolerance cannot be relied on to be above
                // the floor, because the RTI path deliberately passes zero.
                //
                // Scaled by the magnitude of the TERMS the attaining row sums,
                // not of their sum: a row pinned near zero by a tight bound
                // while the iterate is large cancels by orders of magnitude,
                // and ||Cz|| then understates its own noise floor by exactly
                // that factor. Measured: at ||Cz|| this certified two feasible
                // problems infeasible, one of them at every tolerance tried.
                //
                // Kept out of the convergence test on purpose: as a tolerance
                // this moves the solution, and at OSQP's relative default it
                // misses an active-set reference by more than the reference
                // checks allow.
                const float primalFloor =
                    8.0f * std::numeric_limits<float>::epsilon() * res.primalTermScale;
                if (res.dualResidual <= epsDual && prevPrimal >= 0.0f &&
                    res.primalResidual > primalFloor &&
                    res.primalResidual >= prevPrimal * (1.0f - 1e-3f)) {
                    if (++stalled >= 4) res.primalInfeasible = true;
                } else {
                    stalled = 0;
                }
                prevPrimal = res.primalResidual;
            }
        }

        // A caller that turned certification off still has to be told when a
        // general row is not satisfied -- the box can be clamped on the way out,
        // a corridor on a predicted state cannot. This is 2*nnz, not the dual's
        // 2*n^2 matvec, so it is affordable on a one-iteration budget.
        if (check == 0) res.primalResidual = primalInf(&res);

        res.hitIterationCap = (res.iterations >= cap) && !res.converged;
        res.activeRows = 0;
        for (int r = 0; r < mRows; r++) {
            if (mW[r] <= mLoS[r] + kActive || mW[r] >= mHiS[r] - kActive) res.activeRows++;
        }
        return res;
    }

    /// Residuals at the current iterate, without taking a step. Reported in the
    /// caller's units: the primal is divided back through the row scale, and the
    /// dual needs no unscaling because scale * C_r * (y_r / scale) is C_r y_r.
    void residuals(const float* q, QpResult& res) {
        const float pr = primalInf(&res);
        float hz = 0.0f, qn = 0.0f;
        for (int i = 0; i < mN; i++) {
            float s = 0.0f;
            for (int j = 0; j < mN; j++) s += mH[i][j] * mZ[j];
            mDual[i] = s + q[i];
            mCty[i] = 0.0f;
            const float a = std::fabs(s), b = std::fabs(q[i]);
            if (a > hz) hz = a;
            if (b > qn) qn = b;
        }
        for (int r = 0; r < mRows; r++) {
            const float yr = mY[r];
            if (yr == 0.0f) continue;
            for (int j = mBegin[r]; j < mEnd[r]; j++) mCty[j] += mC[r][j] * yr;
        }
        float dr = 0.0f, cy = 0.0f;
        for (int i = 0; i < mN; i++) {
            const float d = std::fabs(mDual[i] + mCty[i]);
            if (d > dr) dr = d;
            const float c = std::fabs(mCty[i]);
            if (c > cy) cy = c;
        }
        float scale = (hz > qn) ? hz : qn;
        if (cy > scale) scale = cy;
        res.primalResidual = pr;
        res.dualResidual = std::isfinite(dr) ? dr : kHuge;
        res.dualScale = std::isfinite(scale) ? scale : kHuge;
    }

    void resetState() {
        for (int i = 0; i < MaxVars; i++) mZ[i] = 0.0f;
        for (int r = 0; r < MaxRows; r++) mW[r] = mY[r] = 0.0f;
    }

    /// Warm start for a horizon that has advanced one step. Each row's
    /// MULTIPLIER moves from `from[r]` (`< 0` keeps the current value; requires
    /// from[r] > r, which is what a shift toward later stages gives), and the
    /// slack is then re-derived as C z rather than shifted alongside.
    ///
    /// Call it with the shifted primal already in z(). Deriving w from z is what
    /// OSQP's warm start does, and it matters: the rows with no counterpart
    /// under `from` -- the head of a rate limit, the tail of any stage range --
    /// keep a slack that belongs to the OLD primal, and an inconsistent
    /// (z, w, y) triple costs more iterations than the shift buys.
    void shiftRows(const short* from) {
        for (int r = 0; r < mRows; r++) {
            const int src = from[r];
            if (src > r && src < mRows) mY[r] = mY[src];
        }
        for (int r = 0; r < mRows; r++) {
            float s = 0.0f;
            for (int j = mBegin[r]; j < mEnd[r]; j++) s += mC[r][j] * mZ[j];
            mW[r] = s;
        }
    }

    int vars() const { return mN; }
    int rows() const { return mRows; }
    int nonzeros() const { return mNnz; }
    bool factorized() const { return mFactored; }
    float rho() const { return mRho; }
    /// The rho a given row is actually penalised at, after its weight.
    float rhoOfRow(int r) const { return mRhoRow[r]; }
    float sigma() const { return mSigma; }

    float* z() { return mZ; }
    const float* z() const { return mZ; }
    float* slack() { return mW; }
    float* dual() { return mY; }
    /// Bounds in the caller's units; written before each solve.
    float* lower() { return mLower; }
    float* upper() { return mUpper; }
    /// Row of C AFTER normalisation, for tests that need to see it.
    const float* rowOf(int r) const { return mC[r]; }
    float rowScale(int r) const { return mScale[r]; }
    /// Stored support [begin, end) of a row, so a caller re-evaluating one row
    /// pays its nonzeros rather than the whole decision vector.
    int rowBegin(int r) const { return mBegin[r]; }
    int rowEnd(int r) const { return mEnd[r]; }
    /// Per-row dist(C_r z, [l_r, u_r]) in that row's own units, from the last
    /// residual pass. Lets a caller that knows what its rows MEAN reduce the
    /// mixed-units maximum to a number with a single unit.
    const float* rowViolations() const { return mViol; }

private:
    static constexpr float kHuge = 1e18f;
    static constexpr float kActive = 1e-7f;

    /// One pass over the row support producing, all in the caller's units:
    /// ||Cz - w||_inf, ||Cz||_inf, and each row's distance to its own [l, u].
    /// The violation is a compare and a subtract per row on top of the matvec
    /// that has to happen anyway, which is why it is not a second pass.
    float primalInf(QpResult* res) {
        float zInf = 0.0f;
        for (int i = 0; i < mN; i++) {
            const float m = std::fabs(mZ[i]);
            if (m > zInf) zInf = m;
        }
        float pr = 0.0f, sc = 0.0f, vi = 0.0f, ts = 0.0f;
        int prRow = -1, viRow = -1;
        for (int r = 0; r < mRows; r++) {
            float s = 0.0f;
            for (int j = mBegin[r]; j < mEnd[r]; j++) s += mC[r][j] * mZ[j];
            const float inv = 1.0f / mScale[r];
            const float d = std::fabs(s - mW[r]) * inv;
            if (d > pr) { pr = d; prRow = r; ts = mRowAbs[r] * zInf * inv; }
            const float m = std::fabs(s) * inv;
            if (m > sc) sc = m;
            const float over = (mLoS[r] - s > s - mHiS[r]) ? mLoS[r] - s : s - mHiS[r];
            const float v = (over > 0.0f) ? over * inv : 0.0f;
            mViol[r] = v;
            if (v > vi) { vi = v; viRow = r; }
        }
        if (res != nullptr) {
            res->primalScale = std::isfinite(sc) ? sc : kHuge;
            res->primalTermScale = std::isfinite(ts) ? ts : kHuge;
            res->worstViolation = std::isfinite(vi) ? vi : kHuge;
            res->worstPrimalRow = prRow;
            res->worstViolationRow = viRow;
        }
        return std::isfinite(pr) ? pr : kHuge;
    }

    int commit(int lo, int hi, float scale) {
        const int r = mRows;
        mBegin[r] = lo;
        mEnd[r] = hi;
        mScale[r] = scale;
        if (scale != 1.0f) {
            for (int j = lo; j < hi; j++) mC[r][j] *= scale;
        }
        float abs1 = 0.0f;
        for (int j = lo; j < hi; j++) abs1 += std::fabs(mC[r][j]);
        mRowAbs[r] = abs1;
        mLower[r] = -kHuge;
        mUpper[r] = kHuge;
        mLoS[r] = -kHuge;
        mHiS[r] = kHuge;
        mViol[r] = 0.0f;
        mW[r] = 0.0f;
        mY[r] = 0.0f;
        mNnz += hi - lo;
        mRows++;
        mFactored = false;
        return r;
    }

    /// In-place lower Cholesky of mK. A pivot that has lost all of its original
    /// magnitude means the matrix is no longer positive definite in float, and
    /// continuing would emit a factor that solves nothing.
    bool cholesky() {
        for (int i = 0; i < mN; i++) {
            float d = mK[i][i];
            for (int k = 0; k < i; k++) d -= mK[i][k] * mK[i][k];
            if (!(d > 1e-10f * mPivotRef[i]) || !std::isfinite(d)) return false;
            const float li = std::sqrt(d);
            mK[i][i] = li;
            const float inv = 1.0f / li;
            for (int j = i + 1; j < mN; j++) {
                float s = mK[j][i];
                for (int k = 0; k < i; k++) s -= mK[j][k] * mK[i][k];
                mK[j][i] = s * inv;
            }
        }
        return true;
    }

    void backSolve(const float* rhs, float* out) const {
        for (int i = 0; i < mN; i++) {
            float s = rhs[i];
            for (int k = 0; k < i; k++) s -= mK[i][k] * out[k];
            out[i] = s / mK[i][i];
        }
        for (int i = mN - 1; i >= 0; i--) {
            float s = out[i];
            for (int k = i + 1; k < mN; k++) s -= mK[k][i] * out[k];
            out[i] = s / mK[i][i];
        }
    }

    const float (*mH)[MaxVars]{nullptr};
    int mN{0};
    int mRows{0};
    int mNnz{0};
    bool mFactored{false};
    float mRho{1.0f};
    float mSigma{1e-6f};

    float mC[MaxRows][MaxVars]{};
    int mBegin[MaxRows]{};
    int mEnd[MaxRows]{};
    float mScale[MaxRows]{};
    float mRowAbs[MaxRows]{};
    float mRhoWeight[MaxRows]{};
    float mRhoRow[MaxRows]{};
    float mInvRhoRow[MaxRows]{};
    bool mHaveRowRho{false};

    float mK[MaxVars][MaxVars]{};
    float mPivotRef[MaxVars]{};

    float mZ[MaxVars]{};
    float mZt[MaxVars]{};
    float mRhs[MaxVars]{};
    float mDual[MaxVars]{};
    float mCty[MaxVars]{};
    float mVarLo[MaxVars]{};
    float mVarHi[MaxVars]{};

    float mW[MaxRows]{};
    float mWt[MaxRows]{};
    float mY[MaxRows]{};
    float mLower[MaxRows]{};
    float mUpper[MaxRows]{};
    float mLoS[MaxRows]{};
    float mHiS[MaxRows]{};
    float mViol[MaxRows]{};
};

}  // namespace chad
