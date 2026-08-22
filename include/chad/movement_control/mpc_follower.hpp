/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include <atomic>
#include <vector>

#include "pros/rtos.hpp"

#include "chad/chassis.hpp"
#include "chad/drivetrain.hpp"
#include "chad/movement_control/feedforward.hpp"
#include "chad/movement_control/mpc_lqr.hpp"
#include "chad/movement_control/mpc.hpp"
#include "chad/movement_control/rti.hpp"

namespace chad {

// One follower, three inner controllers. The outer loop -- reference advance,
// error rotation, feedforward, desaturation -- is identical across all three
// modes, and only the two lines turning a pose error into a twist correction
// differ. That is the only way a comparison between LQR and MPC measures the
// controllers rather than the harness around them.
//
// The reference advances on the clock, one trajectory sample per tick, so
// nothing reads the robot's progress and a zero starting velocity cannot
// deadlock the run. It also means a geometric path fed in raw will race the
// robot: sample the trajectory in TIME at dt.
//
// Trajectory units are metres and radians, matching what kV is fitted in.
// Chassis poses arrive in inches and are converted on the way in.

struct TrajectorySample {
    float x{0.0f};        // metres
    float y{0.0f};
    float theta{0.0f};    // standard math radians
    float v{0.0f};        // m/s
    float w{0.0f};        // rad/s
};

class MpcPathFollower {
public:
    enum class Mode {
        /// Speed-scheduled LQR, gains precomputed once. No input constraint.
        ScheduledLqr,
        /// Riccati solved from scratch every tick. Same control law as
        /// ScheduledLqr; exists to price the schedule.
        RiccatiPerTick,
        /// Condensed MPC solved to convergence every tick.
        MpcConverged,
        /// Condensed MPC, fixed iteration budget, shifted warm start.
        MpcRti,
    };

    struct Config {
        Mode mode{Mode::MpcRti};
        int horizon{10};
        int maxIters{400};          // converged budget
        int rtiIters{1};
        float tol{1e-6f};           // on ||dU||_inf, in m/s and rad/s
        MpcSolver solver{MpcSolver::FastGradient};
        MpcWeights weights{};
        float dt{0.01f};

        /// Respect the input box. False makes MPC unconstrained, which is the
        /// configuration where its first move must equal the LQR's.
        bool constrainInputs{true};

        /// Per-side wheel speed the drivetrain can actually deliver, m/s. The
        /// input box is an exact inner approximation of |v| + |w|*b/2 <= this,
        /// so no admissible (v, w) can ask a wheel for more than the hardware
        /// has.
        float wheelSpeedLimit{1.945f};

        /// How the SLACK in the wheel-speed budget is split between the two
        /// axes: 1.0 gives it all to linear, 0.0 all to angular. The reference
        /// twist itself always keeps its own share, so the box always contains
        /// the reference whenever the reference is deliverable at all.
        float slackToLinear{0.5f};

        /// Put the decision variables in WHEEL coordinates (per-side speed
        /// deviation) instead of twist coordinates. The drivetrain's real limit
        /// is then an exact box and slackToLinear stops mattering: the solver
        /// gets to trade speed against turn rate itself. Costs nothing extra per
        /// tick -- it is a different Bd and a non-diagonal input weight, both
        /// folded into the condensing.
        bool wheelSpaceInputs{false};

        /// Clip each drive side independently instead of letting
        /// FeedforwardController scale both by a common factor. Independent
        /// clipping is the naive way to enforce a voltage ceiling and it
        /// DISTORTS the commanded curvature; the common-factor version does not.
        /// Which one is downstream turns out to decide whether respecting the
        /// limit inside the controller is worth anything.
        bool independentClip{false};

        /// Voltage ceiling for the feedforward stage. Lowering it below 12 is
        /// how the saturation experiment starves the controller.
        float maxVoltage{12.0f};

        /// Reference-speed quantisation for the MPC condensing. The condensed
        /// matrices depend only on v, so they are rebuilt when v crosses a
        /// bucket edge rather than every tick.
        int speedBuckets{16};

        FeedforwardGains gains{};

        /// Optional real-time (not virtual-time) microsecond clock. When set,
        /// the follower charges solver wall time to the stats. Null disables
        /// timing entirely.
        double (*microsClock)(){nullptr};

        bool stopAtEnd{true};
    };

    struct Stats {
        int ticks{0};
        int solverIterations{0};
        int maxIterations{0};
        int rebuilds{0};
        int convergedTicks{0};

        double solveMicros{0.0};      // total, solver only
        double worstSolveMicros{0.0};
        double worstTickMicros{0.0};  // includes a condensing rebuild
        double setupMicros{0.0};

        // Cross-track is e_y against the reference sample, metres.
        double sumAbsCross{0.0};
        float peakAbsCross{0.0f};
        double sumAbsHeading{0.0};
        float endError{0.0f};
        float endHeading{0.0f};

        int satTicks{0};              // ticks whose demand exceeded the box
        int activeBoundTicks{0};
        float peakWheelDemand{0.0f};  // m/s, worst per-side demand before clip
        float peakClipRatio{0.0f};    // demanded / deliverable, worst tick
    };

    MpcPathFollower(Chassis& chassis, Drivetrain& drivetrain);

    /// Start a run on its own task. Returns immediately.
    void follow(const std::vector<TrajectorySample>& traj, const Config& cfg);

    void cancel();
    bool isRunning() const { return mRunning.load(); }
    const Stats& stats() const { return mStats; }
    const Config& config() const { return mConfig; }

    /// Single-tick control law, exposed so it can be exercised without a robot.
    /// Returns the twist correction for a pose error, in m/s and rad/s.
    void controlOnce(const float x0[3], float vRef, float wRef, float u[2]);

private:
    static void trampoline(void* self);
    void run();
    void rebuildFor(float vRef);
    void inputBox(float vRef, float wRef, float uMin[2], float uMax[2]) const;
    /// Map a solved input back to a twist correction (identity unless the
    /// decision variables are in wheel coordinates).
    void toTwist(float u[2]) const;

    Chassis& mChassis;
    Drivetrain& mDrivetrain;

    ScheduledLqr mSchedule;
    CondensedMpc mMpc;
    RealTimeIteration mRti;

    Config mConfig;
    Stats mStats;
    std::vector<TrajectorySample> mTraj;

    int mBucket{-1};
    std::atomic<bool> mRunning{false};
    std::atomic<bool> mCancel{false};
    pros::Task* mTask{nullptr};
};

}  // namespace chad
