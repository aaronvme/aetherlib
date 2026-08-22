/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include "chad/movement_control/mpc.hpp"

namespace chad {

// Spends a fixed solver budget per tick instead of solving to convergence.
//
// Consecutive QPs are nearly identical because the plant moves slowly next to
// the control rate, so the previous tick's solution is already a good warm
// start and the loop converges over time rather than within a tick. That buys a
// per-tick cost you can budget against a hard 10 ms deadline, at the price of
// never being exactly optimal. Whether that price is measurable is an empirical
// question -- see sim/test/mpc.mjs.

class RealTimeIteration {
public:
    struct Config {
        /// Iterations per control tick. 1 is the RTI scheme proper.
        int iterationsPerTick{1};
        /// Early-exit threshold on ||dU||_inf. Zero spends the whole budget,
        /// which is what makes the tick cost constant.
        float tol{0.0f};
        bool shiftWarmStart{true};
    };

    RealTimeIteration() = default;
    explicit RealTimeIteration(CondensedMpc* mpc) : mMpc(mpc) {}

    void bind(CondensedMpc* mpc) { mMpc = mpc; mFirst = true; }
    void reset() { mFirst = true; if (mMpc != nullptr) mMpc->zero(); }

    Config config;

    /// One control tick. Writes the first move into u[2].
    MpcResult step(const float x0[3], const float uMin[2], const float uMax[2], float u[2]) {
        MpcResult res;
        if (mMpc == nullptr) { u[0] = u[1] = 0.0f; return res; }

        if (mFirst) {
            mMpc->zero();
            mFirst = false;
        } else if (config.shiftWarmStart) {
            mMpc->shift();
        }

        res = mMpc->solve(x0, uMin, uMax, (config.iterationsPerTick > 0)
                                              ? config.iterationsPerTick : 1,
                          config.tol);
        mMpc->firstMove(u);
        return res;
    }

private:
    CondensedMpc* mMpc{nullptr};
    bool mFirst{true};
};

}  // namespace chad
