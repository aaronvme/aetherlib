/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "chad/movement_control/mpc_follower.hpp"

#include <cmath>

#include "chad/util/util.hpp"

namespace chad {

MpcPathFollower::MpcPathFollower(Chassis& chassis, Drivetrain& drivetrain)
    : mChassis(chassis), mDrivetrain(drivetrain) {}

void MpcPathFollower::follow(const std::vector<TrajectorySample>& traj, const Config& cfg) {
    if (mRunning.load()) {
        cancel();
        while (mRunning.load()) pros::delay(5);
    }

    mConfig = cfg;
    mTraj = traj;              // one copy, before the loop starts
    mStats = Stats{};
    mBucket = -1;
    mCancel = false;
    mRunning = true;

    const double t0 = (mConfig.microsClock != nullptr) ? mConfig.microsClock() : 0.0;
    mSchedule.build(mConfig.weights, mConfig.wheelSpeedLimit, mConfig.dt, 9);
    if (mConfig.microsClock != nullptr) {
        mStats.setupMicros = mConfig.microsClock() - t0;
    }

    mRti.bind(&mMpc);
    mRti.config.iterationsPerTick = mConfig.rtiIters;
    mRti.config.tol = 0.0f;      // spend the budget; that is the point of RTI
    mRti.config.shiftWarmStart = true;

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
    float P[3][3];
    const bool haveP = mSchedule.costToGoAt(vBucket, P);
    mMpc.solver = mConfig.solver;

    const DrivetrainPlant plant = DrivetrainPlant::discretize(vBucket, mConfig.dt);
    if (mConfig.wheelSpaceInputs) {
        const float bM = mDrivetrain.trackWidth * INCH_TO_METER;
        // R' = T' R T, so the cost is byte-for-byte the same objective as the
        // twist-space formulation -- only the coordinates and therefore the
        // shape of the feasible set change.
        const float invB = 1.0f / bM;
        const float d = 0.25f * mConfig.weights.rv + invB * invB * mConfig.weights.rw;
        const float o = 0.25f * mConfig.weights.rv - invB * invB * mConfig.weights.rw;
        const float R2[2][2] = {{d, o}, {o, d}};
        mMpc.setup(plant.withWheelInputs(bM), mConfig.weights, mConfig.horizon,
                   haveP ? P : nullptr, R2);
    } else {
        mMpc.setup(plant, mConfig.weights, mConfig.horizon, haveP ? P : nullptr);
    }
    mStats.rebuilds++;
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
            const LqrSolution s =
                solveDiscreteLqr(DrivetrainPlant::discretize(std::fabs(vRef), mConfig.dt),
                                 mConfig.weights);
            for (int i = 0; i < 2; i++) {
                u[i] = -(s.K[i][0] * x0[0] + s.K[i][1] * x0[1] + s.K[i][2] * x0[2]);
            }
            break;
        }

        case Mode::MpcConverged:
            rebuildFor(vRef);
            mMpc.zero();
            mMpc.solve(x0, uMin, uMax, mConfig.maxIters, mConfig.tol);
            mMpc.firstMove(u);
            toTwist(u);
            break;

        case Mode::MpcRti:
            rebuildFor(vRef);
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
                const LqrSolution sol = solveDiscreteLqr(
                    DrivetrainPlant::discretize(std::fabs(ref.v), mConfig.dt), mConfig.weights);
                for (int k = 0; k < 2; k++) {
                    u[k] = -(sol.K[k][0] * x0[0] + sol.K[k][1] * x0[1] + sol.K[k][2] * x0[2]);
                }
                r.iterations = sol.iterations;
                r.converged = sol.converged;
                break;
            }
            case Mode::MpcConverged:
                rebuildFor(std::fabs(ref.v));
                mMpc.zero();
                r = mMpc.solve(x0, uMin, uMax, mConfig.maxIters, mConfig.tol);
                mMpc.firstMove(u);
                toTwist(u);
                break;
            case Mode::MpcRti:
                rebuildFor(std::fabs(ref.v));
                r = mRti.step(x0, uMin, uMax, u);
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

        const float vCmd = ref.v + u[0];
        const float wCmd = ref.w + u[1];

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
