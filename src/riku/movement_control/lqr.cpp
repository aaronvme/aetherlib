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

}  // namespace

LqrGain solveLqr(float v, float dt, const LqrWeights& w, int maxIters, float tol) {
    float A[N * N], B[N * M];
    DrivetrainErrorPlant::discretise(v, dt, A, B);

    const float Q[N] = {w.qx * w.qx, w.qy * w.qy, w.qtheta * w.qtheta};
    // R strictly positive, or S = R + B'PB can go singular and the solve returns
    // a silent zero gain.
    const float rv = std::max(w.rv, 1e-3f), rw = std::max(w.romega, 1e-3f);
    const float R[M] = {rv * rv, rw * rw};

    float P[N * N] = {Q[0], 0, 0, 0, Q[1], 0, 0, 0, Q[2]};
    LqrGain out;

    for (int iter = 0; iter < maxIters; iter++) {
        float PA[N * N], PB[N * M], S[M * M], T[M * N];
        mul(P, N, N, A, N, PA);
        mul(P, N, N, B, M, PB);
        mulT(B, N, M, PB, M, S);
        S[0] += R[0];
        S[3] += R[1];
        mulT(B, N, M, PA, N, T);

        const float det = S[0] * S[3] - S[1] * S[2];
        if (std::fabs(det) < 1e-12f) break;
        const float Si[M * M] = {S[3] / det, -S[1] / det, -S[2] / det, S[0] / det};

        float K[M * N];
        mul(Si, M, M, T, N, K);

        // P+ = A'P(A - BK) + Q
        float BK[N * N], AmBK[N * N], PAmBK[N * N], Pn[N * N];
        mul(B, N, M, K, N, BK);
        for (int i = 0; i < N * N; i++) AmBK[i] = A[i] - BK[i];
        mul(P, N, N, AmBK, N, PAmBK);
        mulT(A, N, N, PAmBK, N, Pn);
        for (int i = 0; i < N; i++) Pn[i * N + i] += Q[i];

        float delta = 0.0f;
        for (int i = 0; i < N * N; i++) {
            const float sym = 0.5f * (Pn[i] + Pn[(i % N) * N + (i / N)]);
            delta = std::max(delta, std::fabs(sym - P[i]));
            P[i] = sym;
        }
        for (int i = 0; i < M * N; i++) out.k[i] = K[i];
        if (delta < tol) break;
    }
    return out;
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
