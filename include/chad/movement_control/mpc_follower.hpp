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
#include "chad/movement_control/riccati.hpp"
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
        /// Converged-path cap, and the deadline knob on that path: the worst
        /// tick IS this number whenever any tick fails to certify, because those
        /// ticks spend all of it.
        ///
        /// 400 because a TIGHT rate limit needs it. A 0.05 rad/s per-tick limit
        /// against a slewing reference takes 344 iterations on its worst tick,
        /// and capping below that does not merely lose certification -- it
        /// leaves the interior rate rows violated, which is a wheel-slip
        /// constraint and cannot be clamped on the way out the way the head can.
        /// Measured, worst interior plan rate against a 0.05 limit: cap 25 gives
        /// 4.07x, 50 gives 1.93x, 100 gives 1.13x, 150 gives 1.02x, 400 gives
        /// 1.000x with nothing at the cap.
        ///
        /// 100 was tried and is wrong. It is enough for a LOOSE rate limit --
        /// (2.0 m/s, 0.30 rad/s) certifies inside 68 iterations and closed-loop
        /// cost there is identical to five figures at every cap from 25 to
        /// 1600 -- and generalising from that to the default was the error. The
        /// cost of 400 is bounded and affordable: the worst constrained tick
        /// measured is 4.22 ms of a 10 ms budget, projected from wasm flop
        /// counts rather than measured on a brain.
        int maxIters{400};
        /// Iterations per tick on the RTI path. 2, not 1, because 1 is not safe
        /// once a general row is present: over 108 runs spanning nine initial
        /// conditions and four reference slews, a rate limit of (2.0 m/s, 0.30
        /// rad/s) per tick diverged in 24 of them at one iteration and 0 of 108
        /// at two. On the box fast path 1 is fine and the second iteration is
        /// one H matvec, so paying it unconditionally is the cheap side of the
        /// trade. See RealTimeIteration::Config::iterationsPerTick.
        int rtiIters{2};
        /// PRIMAL solver tolerance: input units (m/s, rad/s) on the box path,
        /// each row's own units on the general path. The dual criterion is
        /// separate and relative -- see CondensedMpc::AdmmConfig -- because a
        /// dense float32 H matvec floors the dual residual near 1e-6 whatever
        /// the budget. 1e-4 is already three orders below what a V5 motor's
        /// velocity quantisation can resolve.
        float tol{1e-4f};
        MpcSolver solver{MpcSolver::FastGradient};
        MpcWeights weights{};
        float dt{0.01f};

        /// Respect the input box. False makes MPC unconstrained, which is the
        /// configuration where its first move must equal the LQR's.
        bool constrainInputs{true};

        /// Per-side wheel speed the drivetrain can actually deliver, m/s. The
        /// input box is an exact inner approximation of |v| + |w|*b/2 <= this,
        /// so no admissible (v, w) can ask a wheel for more than the hardware
        /// has. Zero derives it from the drivetrain, which is the honest source.
        float wheelSpeedLimit{0.0f};

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

        /// Reference-speed quantisation for the MPC condensing, rebuilt when a
        /// bucket edge is crossed rather than every tick.
        int speedBuckets{16};

        /// Condense the horizon at the speed each of its own stages will run at,
        /// instead of one speed for all of them. A single-speed horizon cannot
        /// see the reference braking into the end of a path, which is where the
        /// prediction is worth the most. Rebuilds are keyed on the first and
        /// last speed in the horizon, so the cost stays bucketed.
        bool timeVaryingHorizon{true};

        /// Per-tick change limit on each decision variable, in that variable's
        /// own units: (m/s, rad/s) in twist space, or per-side m/s when
        /// wheelSpaceInputs is set. Non-positive on an axis leaves that axis
        /// alone; both non-positive is the default and keeps the box-only fast
        /// path. This is the constraint that stops a solution which is
        /// admissible on paper from slipping the wheels.
        float inputRateLimit[2]{0.0f, 0.0f};

        /// Rate-limit the TOTAL command (reference + correction) rather than
        /// the correction alone. The former is the physical slip limit; the
        /// latter is what you want when the reference is already smooth.
        bool rateLimitTotalCommand{true};

        /// Corridor on the PREDICTED cross-track error, metres. Non-positive
        /// disables it.
        ///
        /// A row on the predicted state, so the solver plans inside it -- but
        /// only to the extent the solve converges. Unlike the input box and the
        /// head of the rate limit, a corridor cannot be clamped on the way out:
        /// it constrains a state the controller reaches later, not the number
        /// being committed now. At rtiIters = 1 it is therefore closer to a
        /// heavy penalty than to a constraint -- measured 37% outside a 0.04 m
        /// corridor at 1 iteration, 17% at 4, 1% at 20.
        ///
        /// Stats::worstCrossTrackViolation is how far outside, in metres, and is
        /// measured whether or not certifyOptimality is set. NOT
        /// worstPrimalResidual, which is a maximum over rows in different units
        /// and on a corridor this solve cannot satisfy is attained on an input
        /// box row in m/s -- measured 137 against a real 0.072 m, a factor of
        /// 1900. Spend more iterations if the metres matter.
        ///
        /// Sizing: cross-track authority over the horizon is small. At v = 1.4
        /// m/s, dt = 0.01 and N = 10 the whole input box moves the predicted
        /// cross-track by about 0.02 m, so a corridor is almost always either
        /// slack or unreachable, with only a ~2 cm window either side of the
        /// current error where it is an active constraint. Measured: over a
        /// 75-problem grid at N = 10 a fixed 0.06 m corridor was active in
        /// exactly 0 of them.
        ///
        /// It must be LOOSER than the excursion the rest of the configuration
        /// already forces, or the problem is infeasible and the committed input
        /// comes off a diverging iterate. Measured: theta0 = 0.35 rad with a
        /// 0.10 rad/s/tick rate limit cannot hold better than 0.0468 m, and
        /// asking for 0.04 m took closed-loop peak cross-track to 0.514 m --
        /// nine times the 0.0575 m of no constraint at all. An infeasible
        /// corridor also burns the whole iteration cap every tick, which is the
        /// only configuration measured that threatens a 10 ms V5 budget. Note
        /// Stats::infeasibleTicks will not warn you: it is a sufficient test
        /// only, and on this path worstCrossTrackViolation is the signal.
        float crossTrackCorridor{0.0f};

        /// Corridor on the predicted heading error, radians. Non-positive off.
        /// Same fixed-budget caveat as crossTrackCorridor.
        float headingCorridor{0.0f};

        /// ADMM penalty; zero derives it from H and C'C at each rebuild. rho is
        /// fixed for the life of a factorization by construction. The two scales
        /// default to what CondensedMpc::AdmmConfig measured -- see there for
        /// the sweep they came from.
        float admmRho{0.0f};
        float admmRhoScale{3.0f};
        float admmRhoStateWeight{0.3f};
        /// Iterations between residual evaluations on the general path.
        int admmCheckEvery{4};

        /// Advance the warm start one stage per tick. On is right here because
        /// condense() feeds the DARE cost-to-go as the terminal cost, which is
        /// what makes the optimal sequence time-consistent. See
        /// RealTimeIteration::Config::shiftWarmStart.
        bool shiftWarmStart{true};

        /// Certify the solve against the DUAL residual as well, so
        /// convergedTicks means something. The corridor violations --
        /// Stats::worstCrossTrackViolation and worstHeadingViolation -- are
        /// populated either way. On, because the extra cost is about one solver
        /// iteration per tick and knowing whether the plan was optimal is worth
        /// that.
        bool certifyOptimality{true};

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

        int admmTicks{0};             // ticks that went through the general path
        int constraintRows{0};        // rows in C on the last rebuild
        int factorizationFailures{0};
        int relaxedRowTicks{0};       // ticks with a bound pair collapsed
        int infeasibleTicks{0};       // ticks whose constraint set was empty
        int droppedRowTicks{0};       // ticks missing a requested constraint row
        bool constraintsRejected{false};   // a configured corridor did not fit
        /// Worst ||C U - w||_inf over the ticks that MEASURED one, i.e. ADMM's
        /// splitting inconsistency. A MAXIMUM OVER ROWS IN DIFFERENT UNITS --
        /// input units on the box rows, input units per tick on the rate rows,
        /// metres on a cross-track corridor -- so it carries the unit of
        /// whichever row was worst and is not a distance. Read it as a solver
        /// health number: large means the iterate is far from consistent, which
        /// on this path means an infeasible constraint set. For "how far outside
        /// the corridor" use worstCrossTrackViolation.
        ///
        /// Negative when no tick measured one -- certification off, or the box
        /// fast path, which has no rows. Not zero: zero would read as a perfect
        /// solve.
        float worstPrimalResidual{-1.0f};
        float worstDualResidual{-1.0f};
        /// Worst distance the PLAN fell outside crossTrackCorridor, in metres,
        /// and outside headingCorridor, in radians. Measured on the input
        /// sequence as committed, after the rate and box clamps, so it describes
        /// the plan the robot executed. Zero means held, or not configured --
        /// safe, because it is a distance and zero is the good case.
        float worstCrossTrackViolation{0.0f};
        float worstHeadingViolation{0.0f};

        int satTicks{0};              // ticks whose demand exceeded the box
        int activeBoundTicks{0};
        float peakWheelDemand{0.0f};  // m/s, worst per-side demand before clip
        float peakClipRatio{0.0f};    // demanded / deliverable, worst tick
    };

    MpcPathFollower(Chassis& chassis, Drivetrain& drivetrain);
    ~MpcPathFollower();

    MpcPathFollower(const MpcPathFollower&) = delete;
    MpcPathFollower& operator=(const MpcPathFollower&) = delete;

    /// Start a run on its own task. Returns immediately. A previous run is
    /// cancelled and its task reclaimed first, so a multi-motion auton does not
    /// accumulate one task per motion.
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
    /// Free the task wrapper. Only safe once mRunning has gone false.
    void reclaimTask();
    void rebuildFor(float vRef);
    /// Condense the horizon starting at trajectory index `i`, one stage per step.
    void rebuildForHorizon(int i);
    /// Terminal cost and input weight shared by both condensing paths.
    void condense(const DrivetrainPlant* stages, int count, float terminalSpeed);
    /// DARE cost-to-go at `speed`, or false when there is no solution there.
    bool terminalCost(float speed, float P[3][3]) const;
    /// DARE gain at `speed`, by doubling. Returns the solve for its stats.
    DareSolution<3, 2> riccatiGain(float speed, float K[2][3]) const;
    void inputBox(float vRef, float wRef, float uMin[2], float uMax[2]) const;
    /// Push the configured general constraints onto the solver. Called from
    /// condense(), because the state rows are condensed against G.
    void applyConstraints();
    /// Rate-limit anchor and per-stage reference deltas for the horizon
    /// starting at trajectory index `i`. Negative `i` means no trajectory.
    void feedRateLimit(int i, float vRef, float wRef);
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
    int mCrossSpec{-1};                  // spec index of each corridor, or -1
    int mHeadSpec{-1};
    float mLastCommand[2]{0.0f, 0.0f};   // twist actually commanded last tick
    float mLastInput[2]{0.0f, 0.0f};     // last solved input, decision units
    bool mHaveLast{false};
    std::atomic<bool> mRunning{false};
    std::atomic<bool> mCancel{false};
    pros::Task* mTask{nullptr};
};

}  // namespace chad
