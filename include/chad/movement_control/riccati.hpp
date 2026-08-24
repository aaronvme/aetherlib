#pragma once

#include <cmath>

namespace chad {

/// Fixed-size dense linear algebra for the small state-space solves. Sizes are
/// compile-time, nothing allocates, and every routine is safe to call from a
/// control task.
namespace linalg {

template <int N, int M, int K>
void multiply(const float a[N][K], const float b[K][M], float out[N][M]) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < M; j++) {
            float s = 0.0f;
            for (int k = 0; k < K; k++) s += a[i][k] * b[k][j];
            out[i][j] = s;
        }
    }
}

template <int N, int M, int K>
void multiplyTransposedA(const float a[K][N], const float b[K][M], float out[N][M]) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < M; j++) {
            float s = 0.0f;
            for (int k = 0; k < K; k++) s += a[k][i] * b[k][j];
            out[i][j] = s;
        }
    }
}

template <int N, int M, int K>
void multiplyTransposedB(const float a[N][K], const float b[M][K], float out[N][M]) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < M; j++) {
            float s = 0.0f;
            for (int k = 0; k < K; k++) s += a[i][k] * b[j][k];
            out[i][j] = s;
        }
    }
}

template <int N>
void identity(float out[N][N]) {
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) out[i][j] = (i == j) ? 1.0f : 0.0f;
}

template <int N, int M>
void copy(const float in[N][M], float out[N][M]) {
    for (int i = 0; i < N; i++)
        for (int j = 0; j < M; j++) out[i][j] = in[i][j];
}

/// Largest absolute entry, used as the convergence measure on matrix iterations.
template <int N, int M>
float maxAbs(const float a[N][M]) {
    float m = 0.0f;
    for (int i = 0; i < N; i++)
        for (int j = 0; j < M; j++) {
            const float v = std::fabs(a[i][j]);
            if (v > m) m = v;
        }
    return m;
}

/// Force exact symmetry. Riccati iterates are symmetric in exact arithmetic;
/// in float they drift, and an asymmetric cost-to-go breaks the gain solve.
template <int N>
void symmetrise(float a[N][N]) {
    for (int i = 0; i < N; i++)
        for (int j = i + 1; j < N; j++) {
            const float m = 0.5f * (a[i][j] + a[j][i]);
            a[i][j] = m;
            a[j][i] = m;
        }
}

/// Gauss-Jordan inverse with partial pivoting. Returns false and leaves `out`
/// as the identity if the matrix is singular to working precision.
template <int N>
bool inverse(const float in[N][N], float out[N][N]) {
    float a[N][2 * N];
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            a[i][j] = in[i][j];
            a[i][N + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (int col = 0; col < N; col++) {
        int pivot = col;
        float best = std::fabs(a[col][col]);
        for (int r = col + 1; r < N; r++) {
            const float v = std::fabs(a[r][col]);
            if (v > best) {
                best = v;
                pivot = r;
            }
        }
        if (!(best > 1e-20f)) {
            identity<N>(out);
            return false;
        }
        if (pivot != col) {
            for (int j = 0; j < 2 * N; j++) {
                const float t = a[col][j];
                a[col][j] = a[pivot][j];
                a[pivot][j] = t;
            }
        }
        const float inv = 1.0f / a[col][col];
        for (int j = 0; j < 2 * N; j++) a[col][j] *= inv;
        for (int r = 0; r < N; r++) {
            if (r == col) continue;
            const float f = a[r][col];
            if (f == 0.0f) continue;
            for (int j = 0; j < 2 * N; j++) a[r][j] -= f * a[col][j];
        }
    }

    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) out[i][j] = a[i][N + j];
    return true;
}

/// Spectral radius by power iteration on A^k, enough to test closed-loop
/// stability without an eigensolver.
template <int N>
float spectralRadius(const float a[N][N], int iterations = 200) {
    float v[N];
    for (int i = 0; i < N; i++) v[i] = 1.0f / static_cast<float>(N);
    float rho = 0.0f;
    for (int it = 0; it < iterations; it++) {
        float t[N];
        for (int i = 0; i < N; i++) {
            float s = 0.0f;
            for (int j = 0; j < N; j++) s += a[i][j] * v[j];
            t[i] = s;
        }
        float norm = 0.0f;
        for (int i = 0; i < N; i++) norm += t[i] * t[i];
        norm = std::sqrt(norm);
        if (!(norm > 1e-30f)) return 0.0f;
        for (int i = 0; i < N; i++) v[i] = t[i] / norm;
        rho = norm;
    }
    return rho;
}

/// Spectral radius of the LEADING n x n block of a matrix declared MaxN x MaxN.
/// A fixed-capacity buffer whose live size varies per problem cannot use
/// spectralRadius<N>: that folds in the stale rows past n. Stops early once the
/// estimate stops moving, since the caller pays for this at setup. No
/// allocation -- the scratch is two stack vectors of the declared capacity.
template <int MaxN>
float spectralRadiusLeading(const float a[MaxN][MaxN], int n, int maxIterations = 30,
                            float tol = 1e-4f, int* iterationsOut = nullptr) {
    if (iterationsOut != nullptr) *iterationsOut = 0;
    if (n <= 0 || n > MaxN) return 0.0f;

    float v[MaxN], t[MaxN];
    for (int i = 0; i < n; i++) v[i] = 1.0f;
    float lam = 0.0f;
    for (int it = 0; it < maxIterations; it++) {
        for (int i = 0; i < n; i++) {
            float s = 0.0f;
            for (int j = 0; j < n; j++) s += a[i][j] * v[j];
            t[i] = s;
        }
        float nrm = 0.0f;
        for (int i = 0; i < n; i++) nrm += t[i] * t[i];
        nrm = std::sqrt(nrm);
        if (nrm < 1e-20f) break;
        for (int i = 0; i < n; i++) v[i] = t[i] / nrm;
        if (iterationsOut != nullptr) *iterationsOut = it + 1;
        const bool done = std::fabs(nrm - lam) < tol * (std::fabs(nrm) + 1e-9f);
        lam = nrm;
        if (done) break;
    }
    return lam;
}

}  // namespace linalg

template <int N, int M>
struct DareSolution {
    float P[N][N]{};    // cost-to-go, also an MPC terminal cost
    float K[M][N]{};    // u = -K x
    int iterations{0};
    bool converged{false};
    /// max |A'PA - P - A'PB(R + B'PB)^-1 B'PA + Q|. The residual of the
    /// equation being solved, not the change between iterates: an iteration
    /// that has stopped moving has not necessarily solved anything.
    float residual{0.0f};
};

/// Gain for u = -Kx given a cost-to-go P.
template <int N, int M>
void dareGain(const float A[N][N], const float B[N][M], const float R[M][M],
              const float P[N][N], float K[M][N]) {
    float bt_p[M][N];
    linalg::multiplyTransposedA<M, N, N>(B, P, bt_p);

    float m[M][M];
    linalg::multiply<M, M, N>(bt_p, B, m);
    for (int i = 0; i < M; i++)
        for (int j = 0; j < M; j++) m[i][j] += R[i][j];
    linalg::symmetrise<M>(m);

    float minv[M][M];
    linalg::inverse<M>(m, minv);

    float bt_p_a[M][N];
    linalg::multiply<M, N, N>(bt_p, A, bt_p_a);
    linalg::multiply<M, N, M>(minv, bt_p_a, K);
}

template <int N, int M>
float dareResidual(const float A[N][N], const float B[N][M], const float Q[N][N],
                   const float R[M][M], const float P[N][N]) {
    float K[M][N];
    dareGain<N, M>(A, B, R, P, K);

    float p_a[N][N];
    linalg::multiply<N, N, N>(P, A, p_a);
    float at_p_a[N][N];
    linalg::multiplyTransposedA<N, N, N>(A, p_a, at_p_a);

    float bt_p_a[M][N];
    {
        float bt_p[M][N];
        linalg::multiplyTransposedA<M, N, N>(B, P, bt_p);
        linalg::multiply<M, N, N>(bt_p, A, bt_p_a);
    }

    float correction[N][N];
    linalg::multiplyTransposedA<N, N, M>(bt_p_a, K, correction);

    float res[N][N];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            res[i][j] = at_p_a[i][j] - P[i][j] - correction[i][j] + Q[i][j];
    return linalg::maxAbs<N, N>(res);
}

/// Solve the discrete algebraic Riccati equation by structure-preserving
/// doubling. Each step squares the error, so a plant that value iteration needs
/// hundreds of sweeps for converges here in under ten — which is what makes it
/// affordable to re-solve on a robot instead of interpolating a table.
///
///     A_{k+1} = A_k W_k^-1 A_k
///     G_{k+1} = G_k + A_k W_k^-1 G_k A_k'
///     P_{k+1} = P_k + A_k' P_k W_k^-1 A_k,     W_k = I + G_k P_k
///
/// with G_0 = B R^-1 B', P_0 = Q. R must be positive definite.
template <int N, int M>
DareSolution<N, M> solveDare(const float A[N][N], const float B[N][M],
                             const float Q[N][N], const float R[M][M],
                             int maxIterations = 40, float tol = 1e-7f) {
    DareSolution<N, M> out;

    float rInv[M][M];
    const bool rOk = linalg::inverse<M>(R, rInv);

    float ak[N][N];
    linalg::copy<N, N>(A, ak);

    float gk[N][N];
    if (rOk) {
        float b_rinv[N][M];
        linalg::multiply<N, M, M>(B, rInv, b_rinv);
        linalg::multiplyTransposedB<N, N, M>(b_rinv, B, gk);
    } else {
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++) gk[i][j] = 0.0f;
    }
    linalg::symmetrise<N>(gk);

    float pk[N][N];
    linalg::copy<N, N>(Q, pk);
    linalg::symmetrise<N>(pk);

    for (int it = 0; it < maxIterations; it++) {
        float w[N][N];
        linalg::multiply<N, N, N>(gk, pk, w);
        for (int i = 0; i < N; i++) w[i][i] += 1.0f;

        float wInv[N][N];
        if (!linalg::inverse<N>(w, wInv)) break;

        float wInvA[N][N];
        linalg::multiply<N, N, N>(wInv, ak, wInvA);

        float aNext[N][N];
        linalg::multiply<N, N, N>(ak, wInvA, aNext);

        float gStep[N][N];
        {
            float gkAkT[N][N];
            linalg::multiplyTransposedB<N, N, N>(gk, ak, gkAkT);
            float wInvGkAkT[N][N];
            linalg::multiply<N, N, N>(wInv, gkAkT, wInvGkAkT);
            linalg::multiply<N, N, N>(ak, wInvGkAkT, gStep);
        }

        float pStep[N][N];
        {
            float pkWInvA[N][N];
            linalg::multiply<N, N, N>(pk, wInvA, pkWInvA);
            linalg::multiplyTransposedA<N, N, N>(ak, pkWInvA, pStep);
        }

        // A state the input cannot reach — cross-track at zero reference speed
        // is the one that matters here — has no stabilising solution, and the
        // iterate grows without bound. Stop on the last finite iterate rather
        // than handing back an infinite gain.
        if (!std::isfinite(linalg::maxAbs<N, N>(pStep)) ||
            !std::isfinite(linalg::maxAbs<N, N>(gStep))) {
            break;
        }

        float delta = 0.0f;
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++) {
                const float d = std::fabs(pStep[i][j]);
                if (d > delta) delta = d;
                pk[i][j] += pStep[i][j];
                gk[i][j] += gStep[i][j];
            }
        linalg::symmetrise<N>(pk);
        linalg::symmetrise<N>(gk);
        linalg::copy<N, N>(aNext, ak);

        out.iterations = it + 1;
        const float scale = linalg::maxAbs<N, N>(pk);
        if (!std::isfinite(scale) || scale > 1e18f) break;
        if (delta <= tol * (scale + 1.0f)) {
            out.converged = true;
            break;
        }
    }

    linalg::copy<N, N>(pk, out.P);
    dareGain<N, M>(A, B, R, out.P, out.K);
    out.residual = dareResidual<N, M>(A, B, Q, R, out.P);
    return out;
}

/// Closed-loop A - BK, for stability checks.
template <int N, int M>
void closedLoop(const float A[N][N], const float B[N][M], const float K[M][N],
                float out[N][N]) {
    float bk[N][N];
    linalg::multiply<N, N, M>(B, K, bk);
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) out[i][j] = A[i][j] - bk[i][j];
}

}  // namespace chad
