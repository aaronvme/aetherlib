/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include "chad/movement_control/mpc_plant.hpp"

namespace chad {

// Infinite-horizon LQR on DrivetrainPlant, by value iteration on fixed-size
// arrays: no Eigen, no heap, closed-form 2x2 inverse, a few hundred flops per
// solve. LTVPathFollower does the same job with Eigen dynamic matrices at every
// tick; this exists so that cost can be measured against something a brain
// would want to run.
//
// The gain is for u = -K x. With x = pose - reference that is the same as
// u = +K e for e = reference - pose.

struct MpcWeights {
    float qx{1.0f};
    float qy{1.0f};
    float qTheta{1.0f};
    float rv{1.0f};
    float rw{1.0f};
};

struct LqrSolution {
    float K[2][3]{};       // u = -K x
    float P[3][3]{};       // cost-to-go, also the MPC terminal cost
    int iterations{0};
    bool converged{false};
};

/// Value-iterate the DARE. `tol` is a relative change threshold on P.
LqrSolution solveDiscreteLqr(const DrivetrainPlant& plant, const MpcWeights& w,
                             int maxIterations = 500, float tol = 1e-7f);

// ============================================================================
// ScheduledLqr — gains precomputed over reference speed
// ============================================================================
//
// The plant depends on nothing but reference speed, so the whole controller is
// a one-dimensional table. Build it once, then a tick is a table lookup, a
// lerp and a 2x3 matvec.
// ============================================================================

class ScheduledLqr {
public:
    static constexpr int MAX_POINTS = 16;

    /// Build `points` gains spanning [0, vMax] (length units per second).
    void build(const MpcWeights& w, float vMax, float dt, int points = 9);

    /// Interpolated gain at reference speed v. Returns false if not built.
    bool gainAt(float v, float K[2][3]) const;

    /// Interpolated cost-to-go at v, for use as an MPC terminal cost.
    bool costToGoAt(float v, float P[3][3]) const;

    /// u = -K(v) * x, written into u[2].
    void control(float v, const float x[3], float u[2]) const;

    bool built() const { return mCount > 0; }
    int points() const { return mCount; }
    float vMax() const { return mVMax; }

private:
    LqrSolution mSolutions[MAX_POINTS];
    float mSpeeds[MAX_POINTS]{};
    int mCount{0};
    float mVMax{0.0f};
};

}  // namespace chad
