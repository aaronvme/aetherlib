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
    /// Residual of the equation solved, not the change between iterates.
    float residual{0.0f};
};

/// Solve the DARE by structure-preserving doubling (riccati.hpp::solveDare).
/// `tol` bounds the RESIDUAL of the equation, which is a stronger statement than
/// the relative change between iterates this used to report -- an iteration that
/// has stopped moving has not necessarily solved anything. Doubling squares the
/// error each step, so the default cap is 40 rather than 500.
LqrSolution solveDiscreteLqr(const DrivetrainPlant& plant, const MpcWeights& w,
                             int maxIterations = 40, float tol = 1e-7f);

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
    ///
    /// Grid points where the DARE has no stabilising solution are EXCLUDED from
    /// the interpolation, and a lookup below the lowest surviving speed returns
    /// that speed's solution. Cross-track is uncontrollable at v = 0 -- A[1][2]
    /// is v*dt and Bd row 1 is v*dt^2/2, so both vanish -- and the solve there
    /// returns P[1][1] = 4.4e13 against 1.3e5 one grid step up. Interpolating
    /// through it wrecks both outputs, not just the cost-to-go: measured, the
    /// blended P[1][1] was wrong by 3e8 at 0.01 m/s and K[1][1] was 5.24
    /// against an exact 12.38 at 0.1 m/s, i.e. the schedule under-corrected
    /// cross-track by 58% wherever the robot was slow.
    ///
    /// Dropping non-converged points rather than flooring at a constant speed
    /// is the same fix lqr.cpp gets from kVMin = 2 in/s, without the constant:
    /// the condition is a property of the plant, and solveDare already reports
    /// it. Worst relative error in P over [0.05, vMax] falls from 1.9e9 to
    /// 0.72. A hard floor at 2 in/s measures 0.82 and at 4 in/s 0.49, so the
    /// residual error is grid RESOLUTION -- P[1][1] goes as 1/v and the grid is
    /// uniform -- and not the floor.
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
    /// Points actually interpolated, and the lowest speed among them. Below
    /// that speed every lookup returns the same solution.
    int usablePoints() const { return mCount - mFirst; }
    float vMin() const { return (mCount > 0) ? mSpeeds[mFirst] : 0.0f; }

private:
    LqrSolution mSolutions[MAX_POINTS];
    float mSpeeds[MAX_POINTS]{};
    int mCount{0};
    /// First grid index whose DARE converged. Everything below it is a plant
    /// with no stabilising solution and is not interpolated through.
    int mFirst{0};
    float mVMax{0.0f};
};

}  // namespace chad
