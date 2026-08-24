/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "chad/movement_control/mpc_follower.hpp"

#include <cmath>

#include "chad/movement_control/riccati.hpp"
#include "chad/util/util.hpp"

namespace chad {

MpcPathFollower::MpcPathFollower(Chassis& chassis, Drivetrain& drivetrain)
    : mChassis(chassis), mDrivetrain(drivetrain) {}

MpcPathFollower::~MpcPathFollower() {
    cancel();
    while (mRunning.load()) pros::delay(5);
    reclaimTask();
}

/// Only the wrapper is ours. run() returns when the trajectory ends, and the
/// FreeRTOS port sets a task's return address to task_clean_up, which is
/// task_delete(NULL) -- so the task has already deleted itself by the time we
/// get here and remove() would be a use-after-free on a freed TCB. Call only
/// after joining on mRunning.
void MpcPathFollower::reclaimTask() {
    delete mTask;
    mTask = nullptr;
}

void MpcPathFollower::follow(const std::vector<TrajectorySample>& traj, const Config& cfg) {
    if (mRunning.load()) {
        cancel();
        while (mRunning.load()) pros::delay(5);
    }
    reclaimTask();

    mConfig = cfg;
    mTraj = traj;              // one copy, before the loop starts
    mStats = Stats{};
    mBucket = -1;
    mHaveLast = false;
    mLastCommand[0] = mLastCommand[1] = 0.0f;
    mLastInput[0] = mLastInput[1] = 0.0f;
    mCancel = false;
    mRunning = true;

    if (!(mConfig.wheelSpeedLimit > 1e-6f)) {
        mConfig.wheelSpeedLimit = mDrivetrain.maxWheelSpeedInPerSec() * INCH_TO_METER;
    }

    const double t0 = (mConfig.microsClock != nullptr) ? mConfig.microsClock() : 0.0;
    mSchedule.build(mConfig.weights, mConfig.wheelSpeedLimit, mConfig.dt, 9);
    if (mConfig.microsClock != nullptr) {
        mStats.setupMicros = mConfig.microsClock() - t0;
    }

    mRti.bind(&mMpc);
    mRti.config.iterationsPerTick = mConfig.rtiIters;
    mRti.config.tol = 0.0f;      // spend the budget; that is the point of RTI
    mRti.config.shiftWarmStart = mConfig.shiftWarmStart;
    mRti.config.certifyOptimality = mConfig.certifyOptimality;
    mRti.config.trackPreviousInput = false;   // feedRateLimit owns the anchor
    mMpc.certifyOptimality = mConfig.certifyOptimality;

    mTask = new pros::Task(trampoline, this, "MpcFollower");
    pros::delay(10);
}

void MpcPathFollower::cancel() { mCancel = true; }

void MpcPathFollower::trampoline(void* self) {
    static_cast<MpcPathFollower*>(self)->run();
}

void MpcPathFollower::rebuildFor(float vRef) {
    const float span = (mConfig.wheelSpeedLimit > 1e-6f) ? mConfig.wheelSpeedLimit : 1.0f;
    const int buckets = clamp(mConfig.speedBuckets, 1, 64);
    int b = static_cast<int>(std::fabs(vRef) / span * static_cast<float>(buckets));
    b = clamp(b, 0, buckets - 1);
    if (b == mBucket) return;
    mBucket = b;

    const float vBucket = span * (static_cast<float>(b) + 0.5f) / static_cast<float>(buckets);
    const DrivetrainPlant plant = DrivetrainPlant::discretize(vBucket, mConfig.dt);
    condense(&plant, 1, vBucket);
}

void MpcPathFollower::rebuildForHorizon(int i) {
    const int n = static_cast<int>(mTraj.size());
    if (n == 0) return;

    const int horizon = clamp(mConfig.horizon, 1, MPC_MAX_HORIZON);
    const float span = (mConfig.wheelSpeedLimit > 1e-6f) ? mConfig.wheelSpeedLimit : 1.0f;
    const int buckets = clamp(mConfig.speedBuckets, 1, 64);

    auto speedAt = [&](int k) {
        const int idx = clamp(i + k, 0, n - 1);
        return std::fabs(mTraj[idx].v);
    };
    auto bucketOf = [&](float v) {
        int b = static_cast<int>(v / span * static_cast<float>(buckets));
        return clamp(b, 0, buckets - 1);
    };

    // Keyed on both ends of the horizon: one speed cannot distinguish cruising
    // from braking, and braking is the case this exists for.
    const int key = bucketOf(speedAt(0)) * 64 + bucketOf(speedAt(horizon - 1));
    if (key == mBucket) return;
    mBucket = key;

    DrivetrainPlant stages[MPC_MAX_HORIZON];
    for (int k = 0; k < horizon; k++) {
        stages[k] = DrivetrainPlant::discretize(speedAt(k), mConfig.dt);
    }
    condense(stages, horizon, speedAt(horizon - 1));
}

/// Terminal cost from the DARE at the speed the horizon ends at. Gated on the
/// solve actually converging, because cross-track is uncontrollable at zero
/// reference speed with A[1][1] = 1: there is no stabilising solution there, and
/// a value iteration run against it just accumulates qy into P[1][1] until its
/// budget runs out. Weighting the terminal state by Q is the honest fallback.
bool MpcPathFollower::terminalCost(float speed, float P[3][3]) const {
    const MpcWeights& w = mConfig.weights;
    const float Q[3][3] = {{w.qx, 0.0f, 0.0f}, {0.0f, w.qy, 0.0f}, {0.0f, 0.0f, w.qTheta}};
    const float R[2][2] = {{w.rv, 0.0f}, {0.0f, w.rw}};
    const DrivetrainPlant p = DrivetrainPlant::discretize(std::fabs(speed), mConfig.dt);
    const DareSolution<3, 2> s = solveDare<3, 2>(p.Ad, p.Bd, Q, R);
    if (!s.converged) return false;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) P[i][j] = s.P[i][j];
    return true;
}

void MpcPathFollower::condense(const DrivetrainPlant* stages, int count,
                               float terminalSpeed) {
    float P[3][3];
    const bool haveP = terminalCost(terminalSpeed, P);
    mMpc.solver = mConfig.solver;

    const bool varying = count > 1;
    if (mConfig.wheelSpaceInputs) {
        const float bM = mDrivetrain.trackWidth * INCH_TO_METER;
        // R' = T' R T, so the cost is byte-for-byte the same objective as the
        // twist-space formulation -- only the coordinates and therefore the
        // shape of the feasible set change.
        const float invB = 1.0f / bM;
        const float d = 0.25f * mConfig.weights.rv + invB * invB * mConfig.weights.rw;
        const float o = 0.25f * mConfig.weights.rv - invB * invB * mConfig.weights.rw;
        const float R2[2][2] = {{d, o}, {o, d}};

        if (varying) {
            DrivetrainPlant wheelStages[MPC_MAX_HORIZON];
            for (int k = 0; k < count; k++) wheelStages[k] = stages[k].withWheelInputs(bM);
            mMpc.setupVarying(wheelStages, count, mConfig.weights, haveP ? P : nullptr, R2);
        } else {
            mMpc.setup(stages[0].withWheelInputs(bM), mConfig.weights, mConfig.horizon,
                       haveP ? P : nullptr, R2);
        }
    } else if (varying) {
        mMpc.setupVarying(stages, count, mConfig.weights, haveP ? P : nullptr);
    } else {
        mMpc.setup(stages[0], mConfig.weights, mConfig.horizon, haveP ? P : nullptr);
    }
    mMpc.admm.rho = mConfig.admmRho;
    mMpc.admm.rhoScale = mConfig.admmRhoScale;
    mMpc.admm.rhoStateWeight = mConfig.admmRhoStateWeight;
    mMpc.admm.checkEvery = mConfig.admmCheckEvery;
    applyConstraints();
    mStats.rebuilds++;
}

/// Per-tick DARE by structure-preserving doubling. This mode exists to price
/// solving the Riccati equation on the robot instead of interpolating a table,
/// so it has to use the solver you would actually ship -- value iteration needs
/// hundreds of sweeps for the same answer and prices the wrong thing.
DareSolution<3, 2> MpcPathFollower::riccatiGain(float speed, float K[2][3]) const {
    const MpcWeights& w = mConfig.weights;
    const float Q[3][3] = {{w.qx, 0.0f, 0.0f}, {0.0f, w.qy, 0.0f}, {0.0f, 0.0f, w.qTheta}};
    const float R[2][2] = {{w.rv, 0.0f}, {0.0f, w.rw}};
    const DrivetrainPlant p = DrivetrainPlant::discretize(speed, mConfig.dt);
    const DareSolution<3, 2> s = solveDare<3, 2>(p.Ad, p.Bd, Q, R);
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 3; j++) K[i][j] = s.K[i][j];
    return s;
}

void MpcPathFollower::applyConstraints() {
    mMpc.clearConstraints();
    if (mConfig.inputRateLimit[0] > 0.0f || mConfig.inputRateLimit[1] > 0.0f) {
        mMpc.setInputRateLimit(mConfig.inputRateLimit);
    }
    // The return value matters: a spec is refused when its rows will not fit,
    // and a corridor the caller believes it configured is worth reporting.
    // Which spec index each corridor landed on, so its violation can be read
    // back in the unit it was asked for. Not assumed: heading is spec 0 when
    // cross-track is off.
    mCrossSpec = -1;
    mHeadSpec = -1;
    if (mConfig.crossTrackCorridor > 0.0f) {
        const float a[3] = {0.0f, 1.0f, 0.0f};
        const int idx = mMpc.stateConstraints();
        if (mMpc.addStateConstraint(a, -mConfig.crossTrackCorridor,
                                    mConfig.crossTrackCorridor)) {
            mCrossSpec = idx;
        } else {
            mStats.constraintsRejected = true;
        }
    }
    if (mConfig.headingCorridor > 0.0f) {
        const float a[3] = {0.0f, 0.0f, 1.0f};
        const int idx = mMpc.stateConstraints();
        if (mMpc.addStateConstraint(a, -mConfig.headingCorridor,
                                    mConfig.headingCorridor)) {
            mHeadSpec = idx;
        } else {
            mStats.constraintsRejected = true;
        }
    }
}

void MpcPathFollower::feedRateLimit(int i, float vRef, float wRef) {
    if (!(mConfig.inputRateLimit[0] > 0.0f || mConfig.inputRateLimit[1] > 0.0f)) return;

    const int horizon = clamp(mConfig.horizon, 1, MPC_MAX_HORIZON);
    const float halfTrack = 0.5f * mDrivetrain.trackWidth * INCH_TO_METER;
    float anchor[2] = {0.0f, 0.0f};
    float delta[MPC_MAX_VARS]{};

    if (!mConfig.rateLimitTotalCommand) {
        // The reference is assumed smooth, so only the correction is bounded
        // and the per-stage reference steps are irrelevant.
        if (mHaveLast) { anchor[0] = mLastInput[0]; anchor[1] = mLastInput[1]; }
        mMpc.setPreviousInput(anchor);
        mMpc.setReferenceDeltas(nullptr, 0);
        return;
    }

    if (mHaveLast) {
        anchor[0] = mLastCommand[0] - vRef;
        anchor[1] = mLastCommand[1] - wRef;
    }
    const int n = static_cast<int>(mTraj.size());
    if (i >= 0 && n > 0) {
        for (int k = 1; k < horizon; k++) {
            const TrajectorySample& a = mTraj[clamp(i + k - 1, 0, n - 1)];
            const TrajectorySample& b = mTraj[clamp(i + k, 0, n - 1)];
            delta[2 * k + 0] = b.v - a.v;
            delta[2 * k + 1] = b.w - a.w;
        }
    }

    if (mConfig.wheelSpaceInputs) {
        const float aL = anchor[0] - anchor[1] * halfTrack;
        const float aR = anchor[0] + anchor[1] * halfTrack;
        anchor[0] = aL;
        anchor[1] = aR;
        for (int k = 1; k < horizon; k++) {
            const float dv = delta[2 * k + 0], dw = delta[2 * k + 1];
            delta[2 * k + 0] = dv - dw * halfTrack;
            delta[2 * k + 1] = dv + dw * halfTrack;
        }
    }

    mMpc.setPreviousInput(anchor);
    mMpc.setReferenceDeltas(delta, 2 * horizon);
}

void MpcPathFollower::inputBox(float vRef, float wRef, float uMin[2], float uMax[2]) const {
    if (!mConfig.constrainInputs) {
        uMin[0] = uMin[1] = -1e6f;
        uMax[0] = uMax[1] = 1e6f;
        return;
    }
    const float limit = mConfig.wheelSpeedLimit;
    const float halfTrack = 0.5f * mDrivetrain.trackWidth * INCH_TO_METER;

    if (mConfig.wheelSpaceInputs) {
        // Exact: each wheel independently within +/- limit, and the decision
        // variable IS that wheel's deviation from its reference speed.
        const float vL = vRef - wRef * halfTrack;
        const float vR = vRef + wRef * halfTrack;
        uMin[0] = -limit - vL; uMax[0] = limit - vL;
        uMin[1] = -limit - vR; uMax[1] = limit - vR;
        return;
    }

    // The true constraint is a diamond: |v| + |w|*halfTrack <= limit. A box has
    // to be inscribed in it, and the only defensible way to split the budget is
    // around what the reference already needs -- otherwise a reference running
    // near top speed leaves the angular axis nothing and the controller cannot
    // steer at all.
    const float vNeed = std::fabs(vRef);
    const float wNeed = std::fabs(wRef) * halfTrack;
    const float need = vNeed + wNeed;

    float vBudget, wBudgetSpeed;
    if (need >= limit) {
        // The reference is not deliverable. Shrink both axes proportionally;
        // u = 0 then sits outside the box, which is correct -- the controller is
        // forced to a command the drivetrain can actually produce.
        const float k = (need > 1e-6f) ? limit / need : 0.0f;
        vBudget = vNeed * k;
        wBudgetSpeed = wNeed * k;
    } else {
        const float slack = limit - need;
        const float a = clamp(mConfig.slackToLinear, 0.0f, 1.0f);
        vBudget = vNeed + a * slack;
        wBudgetSpeed = wNeed + (1.0f - a) * slack;
    }
    const float wBudget = (halfTrack > 1e-6f) ? wBudgetSpeed / halfTrack : 0.0f;

    uMin[0] = -vBudget - vRef;
    uMax[0] =  vBudget - vRef;
    uMin[1] = -wBudget - wRef;
    uMax[1] =  wBudget - wRef;
}

void MpcPathFollower::toTwist(float u[2]) const {
    if (!mConfig.wheelSpaceInputs) return;
    const float bM = mDrivetrain.trackWidth * INCH_TO_METER;
    const float uL = u[0], uR = u[1];
    u[0] = 0.5f * (uL + uR);
    u[1] = (uR - uL) / bM;
}

void MpcPathFollower::controlOnce(const float x0[3], float vRef, float wRef, float u[2]) {
    float uMin[2], uMax[2];
    inputBox(vRef, wRef, uMin, uMax);

    switch (mConfig.mode) {
        case Mode::ScheduledLqr:
            if (!mSchedule.built()) mSchedule.build(mConfig.weights, mConfig.wheelSpeedLimit,
                                                    mConfig.dt, 9);
            mSchedule.control(vRef, x0, u);
            break;

        case Mode::RiccatiPerTick: {
            float K[2][3];
            riccatiGain(std::fabs(vRef), K);
            for (int i = 0; i < 2; i++) {
                u[i] = -(K[i][0] * x0[0] + K[i][1] * x0[1] + K[i][2] * x0[2]);
            }
            break;
        }

        case Mode::MpcConverged:
            rebuildFor(vRef);
            mMpc.zero();
            feedRateLimit(-1, vRef, wRef);
            mMpc.solve(x0, uMin, uMax, mConfig.maxIters, mConfig.tol);
            mMpc.firstMove(u);
            toTwist(u);
            break;

        case Mode::MpcRti:
            rebuildFor(vRef);
            feedRateLimit(-1, vRef, wRef);
            mRti.step(x0, uMin, uMax, u);
            toTwist(u);
            break;
    }
}

void MpcPathFollower::run() {
    const int n = static_cast<int>(mTraj.size());
    if (n == 0) {
        mRunning = false;
        return;
    }

    FeedforwardController ff(mConfig.gains, mDrivetrain.trackWidth * INCH_TO_METER);
    // With independent clipping the follower does the limiting itself, so the
    // controller's own desaturation has to be taken out of the way first.
    ff.setMaxVoltage(mConfig.independentClip ? 1.0e6f : mConfig.maxVoltage);
    ff.reset();

    const float circumferenceM = mDrivetrain.wheelDiameter * PI * INCH_TO_METER;
    const float rpmToMps = circumferenceM * mDrivetrain.externalGearRatio() / 60.0f;
    const float halfTrack = 0.5f * mDrivetrain.trackWidth * INCH_TO_METER;

    if (mConfig.mode == Mode::MpcRti) mRti.reset();

    std::uint32_t tick = pros::millis();
    const std::uint32_t period = static_cast<std::uint32_t>(mConfig.dt * 1000.0f + 0.5f);

    for (int i = 0; i < n && !mCancel.load(); i++) {
        const double tickStart = (mConfig.microsClock != nullptr) ? mConfig.microsClock() : 0.0;

        const TrajectorySample& ref = mTraj[i];
        const Pose p = mChassis.getPose();
        const float px = p.x * INCH_TO_METER;
        const float py = p.y * INCH_TO_METER;

        // State is (pose - reference) rotated into the robot frame: along-track,
        // cross-track, heading. That is exactly DrivetrainPlant's x.
        const float dx = px - ref.x;
        const float dy = py - ref.y;
        const float c = std::cos(p.theta);
        const float s = std::sin(p.theta);
        float x0[3];
        x0[0] =  c * dx + s * dy;
        x0[1] = -s * dx + c * dy;
        x0[2] = wrapRad(p.theta - ref.theta);

        float uMin[2], uMax[2];
        inputBox(ref.v, ref.w, uMin, uMax);

        float u[2] = {0.0f, 0.0f};
        MpcResult r;
        const int rebuildsBefore = mStats.rebuilds;

        const double solveStart = (mConfig.microsClock != nullptr) ? mConfig.microsClock() : 0.0;
        switch (mConfig.mode) {
            case Mode::ScheduledLqr:
                mSchedule.control(std::fabs(ref.v), x0, u);
                break;
            case Mode::RiccatiPerTick: {
                float K[2][3];
                const DareSolution<3, 2> sol = riccatiGain(std::fabs(ref.v), K);
                for (int k = 0; k < 2; k++) {
                    u[k] = -(K[k][0] * x0[0] + K[k][1] * x0[1] + K[k][2] * x0[2]);
                }
                r.iterations = sol.iterations;
                r.converged = sol.converged;
                break;
            }
            case Mode::MpcConverged:
                if (mConfig.timeVaryingHorizon) rebuildForHorizon(i);
                else rebuildFor(std::fabs(ref.v));
                mMpc.zero();
                feedRateLimit(i, ref.v, ref.w);
                r = mMpc.solve(x0, uMin, uMax, mConfig.maxIters, mConfig.tol);
                mMpc.firstMove(u);
                mLastInput[0] = u[0];
                mLastInput[1] = u[1];
                toTwist(u);
                break;
            case Mode::MpcRti:
                if (mConfig.timeVaryingHorizon) rebuildForHorizon(i);
                else rebuildFor(std::fabs(ref.v));
                feedRateLimit(i, ref.v, ref.w);
                r = mRti.step(x0, uMin, uMax, u);
                mLastInput[0] = u[0];
                mLastInput[1] = u[1];
                toTwist(u);
                break;
        }
        if (mConfig.microsClock != nullptr) {
            const double dtus = mConfig.microsClock() - solveStart;
            mStats.solveMicros += dtus;
            if (mStats.rebuilds == rebuildsBefore && dtus > mStats.worstSolveMicros) {
                mStats.worstSolveMicros = dtus;
            }
        }

        mStats.solverIterations += r.iterations;
        if (r.iterations > mStats.maxIterations) mStats.maxIterations = r.iterations;
        if (r.converged) mStats.convergedTicks++;
        if (r.activeBounds > 0) mStats.activeBoundTicks++;
        if (r.usedAdmm) mStats.admmTicks++;
        if (r.factorizationFailed) mStats.factorizationFailures++;
        if (r.relaxedRows > 0) mStats.relaxedRowTicks++;
        if (r.primalInfeasible) mStats.infeasibleTicks++;
        if (r.droppedRows > 0) mStats.droppedRowTicks++;
        if (r.constraintRows > mStats.constraintRows) mStats.constraintRows = r.constraintRows;
        // A negative residual means NOT MEASURED. Folding it into a worst-case
        // maximum would report a run that checked nothing as a perfect solve.
        if (r.primalResidual >= 0.0f && r.primalResidual > mStats.worstPrimalResidual) {
            mStats.worstPrimalResidual = r.primalResidual;
        }
        // Per corridor, in the unit that corridor was configured in.
        if (mCrossSpec >= 0 && r.stateViolation[mCrossSpec] >
                                   mStats.worstCrossTrackViolation) {
            mStats.worstCrossTrackViolation = r.stateViolation[mCrossSpec];
        }
        if (mHeadSpec >= 0 && r.stateViolation[mHeadSpec] >
                                  mStats.worstHeadingViolation) {
            mStats.worstHeadingViolation = r.stateViolation[mHeadSpec];
        }
        if (r.dualResidual >= 0.0f && r.dualResidual > mStats.worstDualResidual) {
            mStats.worstDualResidual = r.dualResidual;
        }

        const float vCmd = ref.v + u[0];
        const float wCmd = ref.w + u[1];
        mLastCommand[0] = vCmd;
        mLastCommand[1] = wCmd;
        mHaveLast = true;

        // What the twist asks of each wheel, before the voltage stage clips it.
        const float leftDemand = vCmd - wCmd * halfTrack;
        const float rightDemand = vCmd + wCmd * halfTrack;
        const float demand = std::fmax(std::fabs(leftDemand), std::fabs(rightDemand));
        if (demand > mStats.peakWheelDemand) mStats.peakWheelDemand = demand;
        if (demand > mConfig.wheelSpeedLimit + 1e-4f) {
            mStats.satTicks++;
            const float ratio = demand / mConfig.wheelSpeedLimit;
            if (ratio > mStats.peakClipRatio) mStats.peakClipRatio = ratio;
        }

        float leftMeas = 0.0f, rightMeas = 0.0f;
        if (mDrivetrain.leftMotors != nullptr) {
            leftMeas = mDrivetrain.leftMotors->get_actual_velocity() * rpmToMps;
        }
        if (mDrivetrain.rightMotors != nullptr) {
            rightMeas = mDrivetrain.rightMotors->get_actual_velocity() * rpmToMps;
        }

        DrivetrainVoltages out = ff.update(vCmd, wCmd, leftMeas, rightMeas, mConfig.dt);
        if (mConfig.independentClip) {
            out.left = clamp(out.left, -mConfig.maxVoltage, mConfig.maxVoltage);
            out.right = clamp(out.right, -mConfig.maxVoltage, mConfig.maxVoltage);
        }
        if (mDrivetrain.leftMotors != nullptr) {
            mDrivetrain.leftMotors->move_voltage(out.leftMillivolts());
        }
        if (mDrivetrain.rightMotors != nullptr) {
            mDrivetrain.rightMotors->move_voltage(out.rightMillivolts());
        }

        const float absCross = std::fabs(x0[1]);
        mStats.sumAbsCross += absCross;
        if (absCross > mStats.peakAbsCross) mStats.peakAbsCross = absCross;
        mStats.sumAbsHeading += std::fabs(x0[2]);
        mStats.ticks++;

        if (mConfig.microsClock != nullptr) {
            const double tickUs = mConfig.microsClock() - tickStart;
            if (tickUs > mStats.worstTickMicros) mStats.worstTickMicros = tickUs;
        }

        pros::Task::delay_until(&tick, period);
    }

    if (mConfig.stopAtEnd) {
        if (mDrivetrain.leftMotors != nullptr) mDrivetrain.leftMotors->brake();
        if (mDrivetrain.rightMotors != nullptr) mDrivetrain.rightMotors->brake();
    }

    const Pose end = mChassis.getPose();
    const TrajectorySample& last = mTraj[n - 1];
    mStats.endError = std::hypot(end.x * INCH_TO_METER - last.x,
                                 end.y * INCH_TO_METER - last.y);
    mStats.endHeading = wrapRad(end.theta - last.theta);

    mRunning = false;
}

}  // namespace chad
