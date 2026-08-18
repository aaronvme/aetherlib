/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

namespace riku {

// Pose-error plant in the condensed form MPC needs. drivetrain_plant.hpp holds
// the same discretisation for the plain LQR; this one keeps A^k and A^j B as
// methods, because condensing the QP asks for those powers directly and they
// are closed form here -- A is nilpotent, so A^k differs from A only in the
// cross-track term scaling by k.
//
//     A(v) = [0 0 0; 0 0 v; 0 0 0]    B = [1 0; 0 0; 0 1]

struct DrivetrainPlant {
    static constexpr int N_STATES = 3;
    static constexpr int N_INPUTS = 2;

    float Ad[N_STATES][N_STATES];
    float Bd[N_STATES][N_INPUTS];
    float vRef{0.0f};
    float dt{0.01f};

    /// Exact ZOH discretisation at reference speed `v` (same length unit per
    /// second that the state uses) and step `dt` seconds.
    static DrivetrainPlant discretize(float v, float dt) {
        DrivetrainPlant p;
        p.vRef = v;
        p.dt = dt;

        p.Ad[0][0] = 1.0f; p.Ad[0][1] = 0.0f; p.Ad[0][2] = 0.0f;
        p.Ad[1][0] = 0.0f; p.Ad[1][1] = 1.0f; p.Ad[1][2] = v * dt;
        p.Ad[2][0] = 0.0f; p.Ad[2][1] = 0.0f; p.Ad[2][2] = 1.0f;

        p.Bd[0][0] = dt;   p.Bd[0][1] = 0.0f;
        p.Bd[1][0] = 0.0f; p.Bd[1][1] = 0.5f * v * dt * dt;
        p.Bd[2][0] = 0.0f; p.Bd[2][1] = dt;

        return p;
    }

    /// Ad^k, also closed form: (I + A*dt)^k == I + k*A*dt.
    void powerA(int k, float out[N_STATES][N_STATES]) const {
        out[0][0] = 1.0f; out[0][1] = 0.0f; out[0][2] = 0.0f;
        out[1][0] = 0.0f; out[1][1] = 1.0f; out[1][2] = static_cast<float>(k) * vRef * dt;
        out[2][0] = 0.0f; out[2][1] = 0.0f; out[2][2] = 1.0f;
    }

    /// Ad^j * Bd. Written from the stored Bd rather than the closed form so it
    /// stays correct after withWheelInputs() changes what the input means.
    void powerAtimesB(int j, float out[N_STATES][N_INPUTS]) const {
        const float f = static_cast<float>(j) * vRef * dt;
        for (int r = 0; r < N_STATES; r++)
            for (int c = 0; c < N_INPUTS; c++) out[r][c] = Bd[r][c];
        out[1][0] += f * Bd[2][0];
        out[1][1] += f * Bd[2][1];
    }

    /// Re-express the input as per-side wheel speed deviations (uL, uR) rather
    /// than (dv, domega), by post-multiplying Bd with
    ///
    ///     T = [ 0.5   0.5 ]     (dv, domega) = T (uL, uR)
    ///         [-1/b   1/b ]
    ///
    /// The point is the CONSTRAINT set. In twist coordinates the drivetrain's
    /// real limit |v| + |omega|*b/2 <= vmax is a diamond, and a box has to be
    /// inscribed in it -- which forces an arbitrary split of the budget between
    /// speed and turn rate, and a controller that cannot trade one for the
    /// other. In wheel coordinates the same limit IS a box, exactly, with no
    /// conservatism and nothing to tune.
    ///
    /// @param trackWidthM  wheel-to-wheel distance in the state's length unit
    DrivetrainPlant withWheelInputs(float trackWidthM) const {
        DrivetrainPlant p = *this;
        const float invB = 1.0f / trackWidthM;
        for (int r = 0; r < N_STATES; r++) {
            const float b0 = Bd[r][0], b1 = Bd[r][1];
            p.Bd[r][0] = 0.5f * b0 - invB * b1;
            p.Bd[r][1] = 0.5f * b0 + invB * b1;
        }
        return p;
    }

};

}  // namespace riku
