#include "riku/movement_control/lqr.hpp"

#include <algorithm>
#include <cmath>

namespace riku {

namespace {

constexpr int N = DrivetrainErrorPlant::N;
constexpr int M = DrivetrainErrorPlant::M;
constexpr float kVMin = 2.0f;

void mul(const float* A, int ar, int ac, const float* B, int bc, float* out) {
    for (int i = 0; i < ar; i++)
        for (int j = 0; j < bc; j++) {
            float s = 0.0f;
            for (int k = 0; k < ac; k++) s += A[i * ac + k] * B[k * bc + j];
            out[i * bc + j] = s;
        }
}

void mulT(const float* A, int ar, int ac, const float* B, int bc, float* out) {
    for (int i = 0; i < ac; i++)
        for (int j = 0; j < bc; j++) {
            float s = 0.0f;
            for (int k = 0; k < ar; k++) s += A[k * ac + i] * B[k * bc + j];
            out[i * bc + j] = s;
        }
}

/// One backward step: K from P, then P <- Q + A'P(A - BK), symmetrised.
/// Returns false if S = R + B'PB went singular, which would otherwise hand back
/// a silent zero gain.
bool riccatiStep(const float* A, const float* B, const float* Q, const float* R,
                 float* P, float* K) {
    float PA[N * N], PB[N * M], S[M * M], T[M * N];
    mul(P, N, N, A, N, PA);
    mul(P, N, N, B, M, PB);
    mulT(B, N, M, PB, M, S);
    S[0] += R[0];
    S[3] += R[1];
    mulT(B, N, M, PA, N, T);

    const float det = S[0] * S[3] - S[1] * S[2];
    if (std::fabs(det) < 1e-12f) return false;
    const float Si[M * M] = {S[3] / det, -S[1] / det, -S[2] / det, S[0] / det};
    mul(Si, M, M, T, N, K);

    float BK[N * N], AmBK[N * N], PAmBK[N * N], Pn[N * N];
    mul(B, N, M, K, N, BK);
    for (int i = 0; i < N * N; i++) AmBK[i] = A[i] - BK[i];
    mul(P, N, N, AmBK, N, PAmBK);
    mulT(A, N, N, PAmBK, N, Pn);
    for (int i = 0; i < N; i++) Pn[i * N + i] += Q[i];
    for (int i = 0; i < N * N; i++) {
        P[i] = 0.5f * (Pn[i] + Pn[(i % N) * N + (i / N)]);
    }
    return true;
}

void costs(const LqrWeights& w, float* Q, float* R) {
    Q[0] = w.qx * w.qx; Q[1] = w.qy * w.qy; Q[2] = w.qtheta * w.qtheta;
    // R strictly positive, or S = R + B'PB can go singular.
    const float rv = std::max(w.rv, 1e-3f), rw = std::max(w.romega, 1e-3f);
    R[0] = rv * rv; R[1] = rw * rw;
}

/// Error dynamics with the reference yaw rate kept. omega = 0 is exactly the
/// plant DrivetrainErrorPlant assumes. A is no longer nilpotent once omega is
/// nonzero, so the ZOH uses a truncated series -- at a 10 ms tick and |omega|
/// under ~6 rad/s the dropped term is O(1e-4) of Ad.
void discretiseWithOmega(float v, float omega, float dt, float* Ad, float* Bd) {
    const float Ac[N * N] = {0.0f, omega, 0.0f,
                             -omega, 0.0f, v,
                             0.0f, 0.0f, 0.0f};
    float A2[N * N];
    mul(Ac, N, N, Ac, N, A2);
    for (int i = 0; i < N * N; i++) {
        Ad[i] = Ac[i] * dt + 0.5f * A2[i] * dt * dt;
    }
    for (int i = 0; i < N; i++) Ad[i * N + i] += 1.0f;

    // Bd = (I dt + Ac dt^2/2 + Ac^2 dt^3/6) B, with B = [1 0; 0 0; 0 1].
    float G[N * N];
    for (int i = 0; i < N * N; i++) {
        G[i] = 0.5f * Ac[i] * dt * dt + A2[i] * dt * dt * dt / 6.0f;
    }
    for (int i = 0; i < N; i++) G[i * N + i] += dt;
    for (int i = 0; i < N; i++) {
        Bd[i * M + 0] = G[i * N + 0];
        Bd[i * M + 1] = G[i * N + 2];
    }
}

}  // namespace

LqrGain solveLqr(float v, float dt, const LqrWeights& w, int maxIters, float tol,
                 float* pOut) {
    float A[N * N], B[N * M];
    DrivetrainErrorPlant::discretise(v, dt, A, B);

    float Q[N], R[M];
    costs(w, Q, R);

    float P[N * N] = {Q[0], 0, 0, 0, Q[1], 0, 0, 0, Q[2]};
    LqrGain out;

    for (int iter = 0; iter < maxIters; iter++) {
        float prev[N * N], K[M * N];
        for (int i = 0; i < N * N; i++) prev[i] = P[i];
        if (!riccatiStep(A, B, Q, R, P, K)) break;

        float delta = 0.0f;
        for (int i = 0; i < N * N; i++) delta = std::max(delta, std::fabs(P[i] - prev[i]));
        for (int i = 0; i < M * N; i++) out.k[i] = K[i];
        if (delta < tol) break;
    }
    if (pOut != nullptr) for (int i = 0; i < N * N; i++) pOut[i] = P[i];
    return out;
}

void solveTvlqr(const float* v, const float* omega, int n, float dt,
                const LqrWeights& w, LqrGain* out) {
    if (n <= 0) return;
    float Q[N], R[M];
    costs(w, Q, R);

    // Terminal cost is the infinite-horizon cost-to-go at the last commandable
    // speed, NOT Q. Seeding with Q makes the last ~10 gains decay towards zero
    // -- on a 2 s path that is a fifth of the run with no authority left to
    // close out heading, which measured as multi-second timeouts.
    float P[N * N];
    solveLqr(std::max(std::fabs(v[n - 1]), kVMin), dt, w, 600, 1e-5f, P);

    for (int i = n - 1; i >= 0; i--) {
        const float vi = std::max(std::fabs(v[i]), kVMin);
        float A[N * N], B[N * M], K[M * N];
        if (omega != nullptr) {
            discretiseWithOmega(vi, omega[i], dt, A, B);
        } else {
            DrivetrainErrorPlant::discretise(vi, dt, A, B);
        }
        if (!riccatiStep(A, B, Q, R, P, K)) {
            out[i] = (i + 1 < n) ? out[i + 1] : LqrGain{};
            continue;
        }
        for (int k = 0; k < M * N; k++) out[i].k[k] = K[k];
    }
}

void LqrGainSchedule::build(const LqrWeights& w, float dt, float vMax, int points) {
    mVMax = std::max(vMax, 1.0f);
    points = std::max(points, 2);
    mGains.clear();
    mGains.reserve(points);
    // Grid starts above zero. At v = 0 cross-track is uncontrollable, so the
    // Riccati recursion has no finite solution and the cross-track gain steps
    // from 0 to 2.4 within the first 2 in/s -- a node there makes interpolation
    // wrong by 2.2 over the first cell.
    for (int i = 0; i < points; i++) {
        const float f = static_cast<float>(i) / (points - 1);
        mGains.push_back(solveLqr(kVMin + (mVMax - kVMin) * f, dt, w));
    }
}

LqrGain LqrGainSchedule::at(float v) const {
    if (mGains.empty()) return {};
    if (!std::isfinite(v)) return mGains.front();
    // clamp() propagates NaN, so a zero span would hand NaN gains to the caller.
    const float span = std::max(mVMax - kVMin, 1e-3f);
    const float pos = clamp((std::fabs(v) - kVMin) / span, 0.0f, 1.0f);
    const float t = pos * static_cast<float>(mGains.size() - 1);
    const int i = static_cast<int>(t);
    const int j = std::min<int>(i + 1, static_cast<int>(mGains.size()) - 1);
    const float f = t - static_cast<float>(i);
    LqrGain out;
    for (int k = 0; k < 6; k++) out.k[k] = mGains[i].k[k] * (1.0f - f) + mGains[j].k[k] * f;
    return out;
}

}  // namespace riku
