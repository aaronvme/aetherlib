/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "chad/movement_control/mpc_lqr.hpp"

#include "chad/movement_control/riccati.hpp"

#include <cmath>

#include "chad/util/util.hpp"

namespace chad {
namespace {

constexpr int NS = DrivetrainPlant::N_STATES;
constexpr int NI = DrivetrainPlant::N_INPUTS;

}  // namespace

LqrSolution solveDiscreteLqr(const DrivetrainPlant& plant, const MpcWeights& w,
                             int maxIterations, float tol) {
    const float Q[NS][NS] = {{w.qx, 0.0f, 0.0f}, {0.0f, w.qy, 0.0f}, {0.0f, 0.0f, w.qTheta}};
    const float R[NI][NI] = {{w.rv, 0.0f}, {0.0f, w.rw}};

    const DareSolution<NS, NI> sol =
        solveDare<NS, NI>(plant.Ad, plant.Bd, Q, R, maxIterations, tol);

    LqrSolution out;
    for (int i = 0; i < NI; i++)
        for (int j = 0; j < NS; j++) out.K[i][j] = sol.K[i][j];
    for (int i = 0; i < NS; i++)
        for (int j = 0; j < NS; j++) out.P[i][j] = sol.P[i][j];
    out.iterations = sol.iterations;
    out.converged = sol.converged;
    out.residual = sol.residual;
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

    // Lowest speed with a stabilising solution. The scan stops at the first
    // converged point rather than taking the last non-converged one, because
    // convergence is monotone in speed here and a gap would mean something else
    // is wrong. mCount - 1 is the floor: if nothing converged, keep one point
    // rather than an empty schedule, so a lookup still returns a defined gain.
    mFirst = mCount - 1;
    for (int i = 0; i < mCount; i++) {
        if (mSolutions[i].converged) { mFirst = i; break; }
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
    locate(mSpeeds + mFirst, mCount - mFirst, v, i0, i1, f);
    i0 += mFirst;
    i1 += mFirst;
    for (int i = 0; i < NI; i++)
        for (int j = 0; j < NS; j++)
            K[i][j] = mSolutions[i0].K[i][j] +
                      (mSolutions[i1].K[i][j] - mSolutions[i0].K[i][j]) * f;
    return true;
}

bool ScheduledLqr::costToGoAt(float v, float P[3][3]) const {
    if (mCount == 0) return false;
    int i0, i1; float f;
    locate(mSpeeds + mFirst, mCount - mFirst, v, i0, i1, f);
    i0 += mFirst;
    i1 += mFirst;
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
