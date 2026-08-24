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
        /// Iterations per control tick. 1 is the RTI scheme proper, and is
        /// fine while the box fast path is in use. It is NOT safe on the ADMM
        /// path: one iteration there can leave the committed move a full
        /// rate-limit width from the optimum -- measured |u0 - u0*| = 0.300
        /// against a 0.30 rad/s limit -- and the closed loop diverged in 24 of
        /// 108 runs at one iteration against 0 of 108 at two. Two iterations
        /// also cut the error to 0.055 and four to 8.3e-4. MpcPathFollower
        /// therefore defaults to 2.
        int iterationsPerTick{1};
        /// Early-exit threshold on ||dU||_inf. Zero spends the whole budget,
        /// which is what makes the tick cost constant.
        float tol{0.0f};
        /// Move the plan one stage forward between ticks. Correct exactly when
        /// the terminal cost is the infinite-horizon cost-to-go, because only
        /// then is the optimal sequence time-consistent and u_1 really the new
        /// u_0. The distinction is measurable, not stylistic: with lqr.hpp's P
        /// as the terminal cost the shifted sequence is closer to the next
        /// tick's optimum than the unshifted one is (6.2e-3 against 9.3e-3,
        /// inf-norm), and with the terminal stage merely weighted by Q the
        /// ordering flips (3.1e-3 against 5.5e-4). Against a converged solve at
        /// a one-iteration budget that is 0.83x the committed-move error on the
        /// box path and 0.08x with a cross-track corridor when P is supplied,
        /// and 3.0x WORSE on the box path when it is not.
        ///
        /// So: pass P and leave this on -- MpcPathFollower does both. Weight the
        /// terminal stage by Q and turn it off.
        bool shiftWarmStart{true};
        /// Off by default, because a constant tick cost is what this scheme
        /// sells: the DUAL residual is an H matvec, about what one ADMM
        /// iteration costs, so on a one-iteration budget it doubles the tick.
        /// With it off the result reports converged = false and a negative
        /// dualResidual, because nothing certified it.
        ///
        /// It does not hide a corridor violation. MpcResult::primalResidual is
        /// measured either way -- it is one pass over the row support, a third
        /// of an iteration at the largest configuration here -- and it is the
        /// number that says how far outside, in the row's own units, because a
        /// state corridor cannot be clamped on the way out the way the box and
        /// the head of a rate limit are. Turn this on only when `converged`
        /// itself has to mean something.
        bool certifyOptimality{false};
        /// Feed each tick's first move back as the anchor for an input-rate
        /// limit, so the limit reads as "the correction may not jerk". A caller
        /// that wants the limit on the TOTAL command owns the anchor itself and
        /// turns this off.
        bool trackPreviousInput{true};
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

        mMpc->certifyOptimality = config.certifyOptimality;

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
        if (config.trackPreviousInput) mMpc->setPreviousInput(u);
        return res;
    }

private:
    CondensedMpc* mMpc{nullptr};
    bool mFirst{true};
};

}  // namespace chad
