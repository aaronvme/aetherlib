/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "chad/movement_control/mpc_lqr.hpp"

#include <cmath>

#include "chad/util/util.hpp"

namespace chad {
namespace {

constexpr int NS = DrivetrainPlant::N_STATES;
constexpr int NI = DrivetrainPlant::N_INPUTS;

/// out = A^T * P * A  (3x3)
void atpa(const float A[NS][NS], const float P[NS][NS], float out[NS][NS]) {
    float t[NS][NS];
    for (int i = 0; i < NS; i++)
        for (int j = 0; j < NS; j++) {
            float s = 0.0f;
            for (int k = 0; k < NS; k++) s += P[i][k] * A[k][j];
            t[i][j] = s;
        }
    for (int i = 0; i < NS; i++)
        for (int j = 0; j < NS; j++) {
            float s = 0.0f;
            for (int k = 0; k < NS; k++) s += A[k][i] * t[k][j];
            out[i][j] = s;
        }
}

/// out = B^T * P * A  (2x3)
void btpa(const float B[NS][NI], const float P[NS][NS], const float A[NS][NS],
          float out[NI][NS]) {
    float t[NI][NS];
    for (int i = 0; i < NI; i++)
        for (int j = 0; j < NS; j++) {
            float s = 0.0f;
            for (int k = 0; k < NS; k++) s += B[k][i] * P[k][j];
            t[i][j] = s;
        }
    for (int i = 0; i < NI; i++)
        for (int j = 0; j < NS; j++) {
            float s = 0.0f;
            for (int k = 0; k < NS; k++) s += t[i][k] * A[k][j];
            out[i][j] = s;
        }
}

/// out = R + B^T * P * B  (2x2)
void rbtpb(const float R[NI][NI], const float B[NS][NI], const float P[NS][NS],
           float out[NI][NI]) {
    float t[NI][NS];
    for (int i = 0; i < NI; i++)
        for (int j = 0; j < NS; j++) {
            float s = 0.0f;
            for (int k = 0; k < NS; k++) s += B[k][i] * P[k][j];
            t[i][j] = s;
        }
    for (int i = 0; i < NI; i++)
        for (int j = 0; j < NI; j++) {
            float s = 0.0f;
            for (int k = 0; k < NS; k++) s += t[i][k] * B[k][j];
            out[i][j] = R[i][j] + s;
        }
}

/// Closed-form 2x2 inverse. Falls back to a diagonal pseudo-inverse if the
/// determinant collapses, which happens when v_ref == 0 leaves the angular
/// channel uncoupled from the position states.
void inv2(const float M[NI][NI], float out[NI][NI]) {
    const float det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
    if (std::fabs(det) < 1e-12f) {
        out[0][0] = (std::fabs(M[0][0]) > 1e-12f) ? 1.0f / M[0][0] : 0.0f;
        out[1][1] = (std::fabs(M[1][1]) > 1e-12f) ? 1.0f / M[1][1] : 0.0f;
        out[0][1] = out[1][0] = 0.0f;
        return;
    }
    const float inv = 1.0f / det;
    out[0][0] =  M[1][1] * inv;
    out[0][1] = -M[0][1] * inv;
    out[1][0] = -M[1][0] * inv;
    out[1][1] =  M[0][0] * inv;
}

}  // namespace

LqrSolution solveDiscreteLqr(const DrivetrainPlant& plant, const MpcWeights& w,
                             int maxIterations, float tol) {
    LqrSolution out;

    const float Q[NS][NS] = {{w.qx, 0.0f, 0.0f}, {0.0f, w.qy, 0.0f}, {0.0f, 0.0f, w.qTheta}};
    const float R[NI][NI] = {{w.rv, 0.0f}, {0.0f, w.rw}};

    float P[NS][NS];
    for (int i = 0; i < NS; i++)
        for (int j = 0; j < NS; j++) P[i][j] = Q[i][j];

    float M[NI][NI], Minv[NI][NI], BPA[NI][NS], APA[NS][NS], Pn[NS][NS];

    for (int it = 0; it < maxIterations; it++) {
        rbtpb(R, plant.Bd, P, M);
        inv2(M, Minv);
        btpa(plant.Bd, P, plant.Ad, BPA);
        atpa(plant.Ad, P, APA);

        // K = M^-1 * B^T P A
        for (int i = 0; i < NI; i++)
            for (int j = 0; j < NS; j++) {
                float s = 0.0f;
                for (int k = 0; k < NI; k++) s += Minv[i][k] * BPA[k][j];
                out.K[i][j] = s;
            }

        // P+ = Q + A^T P A - (B^T P A)^T K
        float delta = 0.0f, mag = 0.0f;
        for (int i = 0; i < NS; i++)
            for (int j = 0; j < NS; j++) {
                float s = 0.0f;
                for (int k = 0; k < NI; k++) s += BPA[k][i] * out.K[k][j];
                Pn[i][j] = Q[i][j] + APA[i][j] - s;
                delta += std::fabs(Pn[i][j] - P[i][j]);
                mag += std::fabs(Pn[i][j]);
            }

        for (int i = 0; i < NS; i++)
            for (int j = 0; j < NS; j++) P[i][j] = 0.5f * (Pn[i][j] + Pn[j][i]);

        out.iterations = it + 1;
        if (delta <= tol * (mag + 1e-9f)) {
            out.converged = true;
            break;
        }
    }

    for (int i = 0; i < NS; i++)
        for (int j = 0; j < NS; j++) out.P[i][j] = P[i][j];

    return out;
}

// ============================================================================
// ScheduledLqr
// ============================================================================

void ScheduledLqr::build(const MpcWeights& w, float vMax, float dt, int points) {
    mCount = clamp(points, 2, MAX_POINTS);
    mVMax = (vMax > 1e-6f) ? vMax : 1.0f;

    for (int i = 0; i < mCount; i++) {
        const float v = mVMax * static_cast<float>(i) / static_cast<float>(mCount - 1);
        mSpeeds[i] = v;
        mSolutions[i] = solveDiscreteLqr(DrivetrainPlant::discretize(v, dt), w);
    }
}

namespace {
/// Bracketing index and blend fraction for a speed inside the schedule.
void locate(const float* speeds, int count, float v, int& i0, int& i1, float& f) {
    const float av = std::fabs(v);
    if (av <= speeds[0]) { i0 = i1 = 0; f = 0.0f; return; }
    if (av >= speeds[count - 1]) { i0 = i1 = count - 1; f = 0.0f; return; }
    int i = 1;
    while (i < count - 1 && speeds[i] < av) i++;
    i0 = i - 1;
    i1 = i;
    const float span = speeds[i1] - speeds[i0];
    f = (span > 1e-9f) ? (av - speeds[i0]) / span : 0.0f;
}
}  // namespace

bool ScheduledLqr::gainAt(float v, float K[2][3]) const {
    if (mCount == 0) return false;
    int i0, i1; float f;
    locate(mSpeeds, mCount, v, i0, i1, f);
    for (int i = 0; i < NI; i++)
        for (int j = 0; j < NS; j++)
            K[i][j] = mSolutions[i0].K[i][j] +
                      (mSolutions[i1].K[i][j] - mSolutions[i0].K[i][j]) * f;
    return true;
}

bool ScheduledLqr::costToGoAt(float v, float P[3][3]) const {
    if (mCount == 0) return false;
    int i0, i1; float f;
    locate(mSpeeds, mCount, v, i0, i1, f);
    for (int i = 0; i < NS; i++)
        for (int j = 0; j < NS; j++)
            P[i][j] = mSolutions[i0].P[i][j] +
                      (mSolutions[i1].P[i][j] - mSolutions[i0].P[i][j]) * f;
    return true;
}

void ScheduledLqr::control(float v, const float x[3], float u[2]) const {
    float K[2][3];
    u[0] = u[1] = 0.0f;
    if (!gainAt(v, K)) return;
    for (int i = 0; i < NI; i++) {
        float s = 0.0f;
        for (int j = 0; j < NS; j++) s += K[i][j] * x[j];
        u[i] = -s;
    }
}

}  // namespace chad
