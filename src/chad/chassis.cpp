// Ported from FBLib - https://github.com/jiazegao/FBLib
#include "chad/chassis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "chad/util/fasttrig.hpp"
#include "pros/motors.hpp"

namespace chad {

Chassis::Chassis(Drivetrain& drivetrain, const OdomSensors& sensors,
                 const ChassisConfig& config)
    : mDrivetrain(drivetrain),
      mConfig(config),
      mOdom(sensors),
      mRcl(mOdom, config.rclConfig),
      mMcl(mOdom, config.distanceSensors, Pose{}, config.mclConfig),
      mEkf(mOdom, config.distanceSensors, Pose{}, config.ekfConfig),
      mLateralPID(config.lateralGains, config.lateralWindupRange, config.lateralFlipReset),
      mAngularPID(config.angularGains, config.angularWindupRange, config.angularFlipReset),
      mRamseteFf(FeedforwardGains{}, drivetrain.trackWidth * INCH_TO_METER),
      mThrottleCurve(config.throttleCurve),
      mSteerCurve(config.steerCurve)
{
    mLateralPID.setDerivativeFilter(config.lateralDerivativeFilter);
    mAngularPID.setDerivativeFilter(config.angularDerivativeFilter);

    // Apply user-configured sensor mount offsets (robot-specific)
    mMcl.setSensorMounts(config.sensorMounts);
    mEkf.setSensorMounts(config.sensorMounts);
    mEkf.attachMcl(&mMcl);

    // Auto-configure RCL sensors from config arrays.
    // Null distance sensor entries are automatically disabled;
    // sensorCount is set to the count of non-null entries.
    mRcl.configureSensors(config.distanceSensors, config.sensorMounts);

    // Start the persistent motion task (created once, reused for all async motions)
    ensureMotionTask();

    // Start the background odometry task — the SOLE caller of mOdom.update().
    // Centralizing updates in one task fixes two concurrency bugs that arise
    // now that odometry is read from several tasks:
    //   1. Data race: the motion loop, MCL, and RCL used to each call update()
    //      concurrently, corrupting mPose/mPrev* (unserialized read-modify-write).
    //   2. Missed deltas: every update() call consumes the sensor baseline, so
    //      MCL's per-tick getLastDelta() lost the motion consumed by other
    //      callers' ticks — particles systematically fell behind the robot.
    // It also keeps the pose fresh during driver control, when no motion
    // command is running (previously the pose froze outside of motions).
    mOdom.startBackgroundOdom();
}

Chassis::~Chassis() {
    // executeCurrentMotion()'s loop only watches mMotionCancelled, so cancel
    // first and wait for it to unwind. Removing the task mid-motion skips
    // endMotion() and the drive holds its last voltage forever.
    mMotionCancelled = true;
    mTaskShouldStop = true;
    if (mMotionTask != nullptr) {
        mMotionTask->notify();       // wake from notify_take so it can exit
        for (int i = 0; i < 100 && mMotionRunning; ++i) pros::delay(5);
        mMotionTask->remove();
        delete mMotionTask;
        mMotionTask = nullptr;
    }
    mDrivetrain.setVoltage(0, 0);

    // Stop the tracking worker tasks BEFORE the odometry task — they call into
    // mOdom every tick, so they must be gone before OdomTracking is torn down.
    mEkf.stopTracking();
    mMcl.stopTracking();
    mRcl.stopTracking();
    mOdom.stopBackgroundOdom();
}

void Chassis::calibrate() {
    mOdom.calibrate();
}

void Chassis::setPose(const Pose& pose) {
    // Dry-run: only re-baseline the SIMULATION. The preview must be
    // side-effect-free — auton functions call setPose() at their start, and
    // mutating real odometry/MCL/RCL here would teleport the robot's actual
    // localization every time a preview is generated.
    if (mDryRun) {
        mDryRunStartPose = pose;
        mDryRunPose = pose;
        mDryRunPath.clear();
        mDryRunPath.push_back(pose);
        return;
    }
    mOdom.setPose(pose);
    // Propagate to MCL so particles are reinitialized around the new pose.
    // Without this, MCL particles stay at the old pose when the user changes
    // the chassis pose (e.g. during competition_initialize).
    mMcl.setPose(pose);
    mRcl.setRclPose(pose);
    mEkf.setPose(pose);
}

void Chassis::setPosition(float x, float y) {
    if (mDryRun) {   // side-effect-free in dry-run — adjust the sim pose only
        mDryRunPose.x = x;
        mDryRunPose.y = y;
        mDryRunPath.push_back(mDryRunPose);
        return;
    }
    Pose current = mOdom.getPose();
    Pose newPose{x, y, current.theta};
    mOdom.setPose(newPose);
    // Propagate to tracking systems so MCL particles stay centered and RCL
    // estimate stays synchronized.
    mMcl.setPose(newPose);
    mRcl.setRclPose(newPose);
    mEkf.setPose(newPose);
}

void Chassis::setHeading(float thetaDeg) {
    if (mDryRun) {   // side-effect-free in dry-run — adjust the sim pose only
        mDryRunPose.theta = vexToStdRad(thetaDeg);
        mDryRunPath.push_back(mDryRunPose);
        return;
    }
    Pose current = mOdom.getPose();
    Pose newPose{current.x, current.y, vexToStdRad(thetaDeg)};
    mOdom.setPose(newPose);
    // Propagate to tracking systems — same reason as setPosition().
    mMcl.setPose(newPose);
    mRcl.setRclPose(newPose);
    mEkf.setPose(newPose);
}

Pose Chassis::getPose() const {
    return mOdom.getPose();
}

void Chassis::tank(float left, float right) {
    if (mDryRun) return;
    if (!isSettled()) return;   // yield to running OR queued async motion
    float l = mThrottleCurve.apply(left);
    float r = mThrottleCurve.apply(right);
    mDrivetrain.setVoltage(l, r);
}

void Chassis::arcade(float throttle, float steer) {
    if (mDryRun) return;
    if (!isSettled()) return;   // yield to running OR queued async motion
    float t = mThrottleCurve.apply(throttle);
    float s = mSteerCurve.apply(steer);

    float left  = t + s;
    float right = t - s;

    // Normalize to [-127, 127]
    float maxVal = std::max(std::fabs(left), std::fabs(right));
    if (maxVal > 127.0f) {
        left  = left  * 127.0f / maxVal;
        right = right * 127.0f / maxVal;
    }

    mDrivetrain.setVoltage(left, right);
}

void Chassis::curvature(float throttle, float steer) {
    // Curvature drive: inside wheel slows proportionally during turns
    if (mDryRun) return;
    if (!isSettled()) return;   // yield to running OR queued async motion
    float t = mThrottleCurve.apply(throttle);
    float s = mSteerCurve.apply(steer);

    float left, right;
    if (std::fabs(t) < 5.0f) {
        // Stationary turn
        left = s;
        right = -s;
    } else {
        // Moving turn: reduce inside wheel speed
        float turnScale = std::fabs(s) / 127.0f;
        if (s > 0) {
            // Turning right: slow down right side
            left = t;
            right = t * (1.0f - turnScale * 0.7f);
        } else {
            // Turning left: slow down left side
            left = t * (1.0f - turnScale * 0.7f);
            right = t;
        }
    }

    float maxVal = std::max(std::fabs(left), std::fabs(right));
    if (maxVal > 127.0f) {
        left  = left  * 127.0f / maxVal;
        right = right * 127.0f / maxVal;
    }

    mDrivetrain.setVoltage(left, right);
}

void Chassis::startMotion() {
    mMotionRunning = true;
}

void Chassis::endMotion(bool brake) {
    if (brake) mDrivetrain.setVoltage(0, 0);
    mMotionRunning = false;
    mLateralPID.reset();
    mAngularPID.reset();
}

void Chassis::runMotionTask() {
    while (!mTaskShouldStop) {
        // Block until work arrives (FreeRTOS task notification)
        pros::Task::notify_take(true, TIMEOUT_MAX);
        if (mTaskShouldStop) break;

        // Drain all queued motions. This task is the SOLE executor of motions,
        // so nothing else touches mCurrentMotion / the PIDs / the motors.
        uint32_t drainWaitMs = 0;
        while (true) {
            MotionRequest req;
            bool hasWork = false;
            mQueueMutex.lock();
            if (!mMotionQueue.empty()) {
                req = mMotionQueue.front();
                mMotionQueue.pop();
                hasWork = true;
            }
            mQueueMutex.unlock();

            if (!hasWork) {
                // A motion that chained out left the drive running so the next
                // one could inherit its speed. A blocking caller cannot observe
                // the completion for a poll period and then still has to return
                // through enqueueMotion, so zeroing the instant the queue drains
                // made earlyExitRange a hard stop on the default API. Wait a
                // bounded moment for the follow-up -- and stop if none comes,
                // because a drained queue must never leave voltage on.
                if (drainAction(mLastOutcome.load(), drainWaitMs) == DrainAction::Wait) {
                    pros::delay(5);
                    drainWaitMs += 5;
                    continue;
                }
                mDrivetrain.setVoltage(0, 0);
                break;
            }
            drainWaitMs = 0;

            // If cancel() was called, skip execution but still publish the
            // sequence id so any blocking caller waiting on it is released.
            if (!mMotionCancelled) {
                mCurrentMotion = req;
                executeCurrentMotion();
            } else {
                mLastOutcome.store(MotionOutcome::Cancelled);
            }
            mCompletedSeq.store(publishCompleted(mCompletedSeq.load(), req.seq));
        }
    }
}

void Chassis::emitTwist(float v, float w, float maxOut, bool velocityController,
                        float vLeftMeas, float vRightMeas, float tickDt,
                        float& outLeft, float& outRight) {
    const float freeSpeed = mDrivetrain.maxWheelSpeedInPerSec();
    const DifferentialKinematics kin = mDrivetrain.kinematics();
    const float outLimit = freeSpeed * (maxOut / 127.0f);

    WheelSpeeds wheels = kin.toWheelSpeeds({v, 0.0f, w});
    const float scale = wheels.desaturate(outLimit);
    v *= scale;
    w *= scale;

    if (velocityController) {
        const DrivetrainVoltages out =
            mRamseteFf.update(v * INCH_TO_METER, w, vLeftMeas * INCH_TO_METER,
                              vRightMeas * INCH_TO_METER, tickDt);
        outLeft = clamp(out.leftPower(), -maxOut, maxOut);
        outRight = clamp(out.rightPower(), -maxOut, maxOut);
        return;
    }

    const float toMotor = (freeSpeed > 1e-3f) ? (127.0f / freeSpeed) : 0.0f;
    outLeft = clamp(wheels.left * toMotor, -maxOut, maxOut);
    outRight = clamp(wheels.right * toMotor, -maxOut, maxOut);
}

void Chassis::executeCurrentMotion() {
    auto& req = mCurrentMotion;

    // Timestamp when execution actually begins (not when enqueued)
    req.startTime = pros::millis();

    // Dry-run: continue from the previous simulated motion's end pose so a
    // multi-step auton chains into one continuous path
    if (mDryRun) {
        if (mDryRunPath.empty()) {
            mDryRunPose = mOdom.getPose();
            mDryRunPath.push_back(mDryRunPose);
        }
        mDryRunStartPose = mDryRunPose;
    }

    // Precompute max linear speed for dry-run integration
    // v_max = rpm / 60 * wheel_diameter * PI  (inches/second)
    float maxSpeedInPerSec = mDrivetrain.rpm / 60.0f *
        mDrivetrain.wheelDiameter * PI;

    // Resolved ONCE here, from the pose the preceding motion left behind, so a
    // queued Distance still aims correctly after a turn. Recomputing it per tick
    // from the live pose moves the target with the robot and the error never
    // shrinks. The background odometry task keeps the pose fresh.
    StraightTarget straightTarget{};
    if (req.type == MotionType::Distance) {
        straightTarget = resolveStraightTarget(mDryRun ? mDryRunPose : mOdom.getPose(),
                                               req.targetDistance,
                                               req.moveDistParams.forwards);
    }

    // — RAMSETE: precompute path arc-length + a velocity profile ONCE —
    // ramseteCumLen[i] is the arc length from path[0] to path[i] (inches).
    // ramseteProfile gives the desired forward speed (in/s) as a function of
    // distance travelled along the path, so the robot accelerates off the line
    // and decelerates into the endpoint instead of slamming full speed.
    std::vector<float> ramseteCumLen;
    std::vector<ProfilePoint> ramseteProfile;
    float ramsetePeakPos = 0.0f;   // arc length at which the profile peaks
    if (req.type == MotionType::Trajectory && req.trajParams.useVelocityController) {
        mRamseteFf.setGains(req.trajParams.ffGains);
        mRamseteFf.setMaxVoltage(12.0f * req.trajParams.maxSpeed / 127.0f);
        mRamseteFf.setTrackWidth(mDrivetrain.trackWidth * INCH_TO_METER);
        mRamseteFf.reset();
    }

    if (req.type == MotionType::RAMSETE) {
        const auto& path = req.ramsetePath;
        if (req.ramseteParams.useVelocityController) {
            mRamseteFf.setGains(req.ramseteParams.ffGains);
            // Desaturate against the authority the caller actually allowed. Left
            // at 12 V the controller scales the pair against full scale and the
            // per-side clamp below then truncates one side alone, which is the
            // curvature loss the pair-scaling exists to prevent.
            mRamseteFf.setMaxVoltage(12.0f * req.ramseteParams.maxSpeed / 127.0f);
            mRamseteFf.setTrackWidth(mDrivetrain.trackWidth * INCH_TO_METER);
            mRamseteFf.reset();
        }
        ramseteCumLen.resize(path.size(), 0.0f);
        for (size_t i = 1; i < path.size(); ++i) {
            ramseteCumLen[i] = ramseteCumLen[i - 1] +
                distance(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y);
        }
        float totalLen = ramseteCumLen.empty() ? 0.0f : ramseteCumLen.back();
        // Peak profile speed (in/s), scaled by the caller's maxSpeed fraction.
        float cruiseInPerSec = maxSpeedInPerSec * (req.ramseteParams.maxSpeed / 127.0f);
        const float maxAccel = req.ramseteParams.maxAccel;
        const float latAccel = req.ramseteParams.maxLateralAccel;

        if (req.ramseteParams.useVelocityProfile && totalLen > 1e-3f &&
            cruiseInPerSec > 1e-3f && maxAccel > 1e-3f) {

            if (latAccel <= 1e-3f || path.size() < 3) {
                // Cornering limit disabled: plain trapezoid over arc length.
                ramseteProfile = generateTrapezoidal(totalLen, cruiseInPerSec, maxAccel);
            } else {
                // Curvature-limited profile, sampled at the path points: cap each
                // point at the speed its bend can hold, then sweep both ways so the
                // acceleration limit holds too -- braking starts before the corner,
                // not at it. Both ends pinned to rest.
                const size_t n = path.size();
                std::vector<float> v(n, cruiseInPerSec);

                for (size_t i = 1; i + 1 < n; ++i) {
                    // Same curvature estimate the law uses for w_d, so the profile and
                    // the feedforward cannot disagree about the path.
                    float segDist = distance(path[i - 1].x, path[i - 1].y,
                                             path[i + 1].x, path[i + 1].y);
                    if (segDist <= 1e-3f) continue;
                    float curvature = std::fabs(
                        angleDiffRad(path[i - 1].theta, path[i + 1].theta)) / segDist;
                    if (curvature > 1e-5f) {
                        v[i] = std::min(v[i], std::sqrt(latAccel / curvature));
                    }
                }
                v.front() = 0.0f;
                v.back() = 0.0f;

                for (size_t i = 1; i < n; ++i) {           // accelerate forwards
                    float ds = ramseteCumLen[i] - ramseteCumLen[i - 1];
                    v[i] = std::min(v[i], std::sqrt(v[i - 1] * v[i - 1] + 2.0f * maxAccel * ds));
                }
                for (size_t i = n - 1; i > 0; --i) {       // brake backwards
                    float ds = ramseteCumLen[i] - ramseteCumLen[i - 1];
                    v[i - 1] = std::min(v[i - 1], std::sqrt(v[i] * v[i] + 2.0f * maxAccel * ds));
                }

                ramseteProfile.resize(n);
                for (size_t i = 0; i < n; ++i) {
                    ramseteProfile[i].position = ramseteCumLen[i];
                    ramseteProfile[i].velocity = v[i];
                }
            }

            float best = -1.0f;
            for (const auto& pt : ramseteProfile) {
                if (pt.velocity > best) { best = pt.velocity; ramsetePeakPos = pt.position; }
            }
        }
    }

    mMotionCancelled = false;
    startMotion();

    // Dry-run runs offline (no wall-clock waits); time advances in simulated 10 ms steps
    uint32_t dryRunSteps = 0;

    SettleSpec spec = settleSpecFor(req, settleConfig());
    // Dry-run integrates a synthetic pose and never reads a wheel, so every
    // sample would look like a stopped drivetrain.
    if (mDryRun) spec.stallSpeed = 0.0f;
    SettleMonitor monitor{spec};
    monitor.reset(0);              // elapsed, not wall clock: dry-run shares it
    RateEstimator errorRate;
    // The floor's job is to stop a motion creeping before it arrives. Left live
    // afterwards it kicks the robot back out of tolerance and the pair limit
    // cycles around the target, so it goes off for good on first arrival.
    bool floorRetired = false;
    MotionOutcome outcome = MotionOutcome::Cancelled;
    // Not the previous motion's value for the whole of this one.
    mLastOutcome.store(MotionOutcome::None);

    BearingHold pointHold;
    TurnLatch turnLatch;

    // Zeroed per motion, so a queued one cannot inherit the ramp.
    float prevLateralOut = 0.0f;

    uint32_t prevTick = pros::millis();

    while (mMotionRunning && !mMotionCancelled) {
        const uint32_t nowTick = pros::millis();
        const uint32_t tickMs = nowTick - prevTick;
        float tickDt = tickMs / 1000.0f;
        if (mDryRun || tickDt <= 0.0f || tickDt > 0.1f) tickDt = 0.01f;

        const uint32_t elapsed = mDryRun ? (dryRunSteps * 10u)
                                         : (pros::millis() - req.startTime);
        prevTick = nowTick;
        // Odometry freshness is provided by the background odometry task — do
        // NOT call mOdom.update() here (it would race and consume deltas).
        Pose pose = mDryRun ? mDryRunPose : mOdom.getPose();

        float lateralOut = 0.0f, angularOut = 0.0f;
        float customLeft = 0.0f, customRight = 0.0f;

        // Output floors, in motor units. A tuned loop goes quiet before it
        // arrives; the floor is what stops a motion creeping out its timeout.
        float latFloor = 0.0f, angFloor = 0.0f;
        float settleError = 0.0f;      // in whichever unit this motion settles on
        // What the speed test differentiates. Usually the settle error, but a
        // motion may gate its settle error on a reference clock and a synthetic
        // step differentiates into a speed the robot does not have.
        float measuredError = 0.0f;
        bool measuredErrorSet = false;
        bool degenerate = false;       // the request itself is unusable
        bool twistOutput = false;      // RAMSETE / Trajectory write per-side directly
        bool swingOutput = false;
        // The motion's own primary target, met on its own terms: stopping here
        // is an arrival with a residual, not a stall.
        bool atTarget = false;
        // A reference that is legitimately stopped mid-motion cannot end the
        // motion and is not a stall either.
        bool referenceDone = true;
        float latLimit = 127.0f;       // max lateral output (±)
        float angLimit = 127.0f;       // max angular output (±)

        // One sweep of the motors a tick, shared by the followers and the rest
        // test. Four separate per-side calls allocated and freed four vectors on
        // every control cycle.
        const WheelReading wheels =
            mDryRun ? WheelReading{} : mDrivetrain.wheelVelocitiesLive();
        const WheelSpeeds wheelVel = wheels.speeds;

        switch (req.type) {
        case MotionType::Distance: {
            latLimit = req.moveDistParams.maxSpeed;
            const MotionDemand d = straightDemand(pose, straightTarget);
            lateralOut = mLateralPID.update(d.lateral);
            angularOut = mAngularPID.update(d.angular);
            settleError = d.settleError;
            latFloor = req.moveDistParams.minSpeed;
            break;
        }
        case MotionType::Point: {
            latLimit = req.movePointParams.maxSpeed;
            const MotionDemand d =
                pointDemand(pose, req.targetX, req.targetY,
                            req.movePointParams.forwards,
                            req.movePointParams.bearingHoldDist, pointHold);
            lateralOut = mLateralPID.update(d.lateral);
            angularOut = mAngularPID.update(d.angular);
            settleError = d.settleError;
            latFloor = req.movePointParams.minSpeed;
            break;
        }
        case MotionType::Heading: {
            angLimit = req.turnHeadingParams.maxSpeed;
            const float rawErrRad = angleDiffRad(pose.theta, req.targetHeadingRad);
            angularOut = mAngularPID.update(
                resolvedTurnError(rawErrRad, req.turnHeadingParams.direction,
                                  degToRad(req.turnHeadingParams.targetTolerance),
                                  turnLatch));
            settleError = rawErrRad;
            angFloor = req.turnHeadingParams.minSpeed;
            break;
        }
        case MotionType::TurnPoint: {
            angLimit = req.turnPointParams.maxSpeed;
            const float rawErrRad = headingToPointOffset(
                pose, req.targetX, req.targetY, req.turnPointParams.offset);
            angularOut = mAngularPID.update(
                resolvedTurnError(rawErrRad, req.turnPointParams.direction,
                                  degToRad(req.turnPointParams.targetTolerance),
                                  turnLatch));
            settleError = rawErrRad;
            angFloor = req.turnPointParams.minSpeed;
            break;
        }
        case MotionType::Arc: {
            latLimit = req.arcParams.maxSpeed;
            const MotionDemand d = arcDemand(pose, req.targetX, req.targetY,
                                             req.targetRadius,
                                             req.arcParams.forwards);
            lateralOut = mLateralPID.update(d.lateral);
            angularOut = mAngularPID.update(d.angular);
            settleError = d.settleError;
            latFloor = req.arcParams.minSpeed;
            break;
        }
        case MotionType::Boomerang: {
            latLimit = req.boomerangParams.maxSpeed;
            const MotionDemand d =
                boomerangDemand(pose, req.targetX, req.targetY, req.targetHeadingRad,
                                req.boomerangParams);
            lateralOut = mLateralPID.update(d.lateral) * d.lateralScale;
            angularOut = mAngularPID.update(d.angular);
            settleError = d.settleError;
            atTarget = d.atTarget;
            latFloor = req.boomerangParams.minSpeed;
            break;
        }
        case MotionType::RAMSETE: {
            // Standard unicycle tracking law (WPILib / De Luca convention):
            //   k  = 2ζ·√(ω_d² + b·v_d²)
            //   v  = v_d·cos(e_θ) + k·e_x                          [in/s]
            //   ω  = ω_d + b·v_d·sinc(e_θ)·e_y + k·e_θ             [rad/s]
            //
            // Everything is in real units: v/v_d in inches/second, ω/ω_d in
            // radians/second.  v_d comes from a velocity profile over the path
            // (accel/decel), NOT a constant.  Outputs are converted to per-side
            // wheel speeds and emitted directly (no lateral/angular mixing), so
            // the sign convention is unambiguous.
            //
            // `b` is taken in the conventional meter-based parameterization
            // (b≈2.0, ζ≈0.7 are good defaults) and scaled to inches here.

            const auto& path = req.ramsetePath;
            if (path.size() < 2) {
                degenerate = true;
                break;
            }

            float distToEnd = distanceToPoint(pose, path.back().x, path.back().y);

            // — Reference pose: where the robot should BE, i.e. its projection
            //   onto the path. Offsetting it forward leaves a standing e_x, and
            //   k·e_x then adds speed on top of every point of the profile. —
            int closestIdx = closestPathIndex(path, pose);
            int targetIdx = closestIdx;
            const Pose& target = path[targetIdx];

            // — Desired forward speed v_d (in/s) from the velocity profile,
            //   sampled at the robot's arc-length progress along the path. —
            float progress = (closestIdx >= 0 &&
                              closestIdx < static_cast<int>(ramseteCumLen.size()))
                             ? ramseteCumLen[closestIdx] : 0.0f;
            float v_d = maxSpeedInPerSec * (req.ramseteParams.maxSpeed / 127.0f);
            if (!ramseteProfile.empty()) {
                // profile positions are monotonic — take the first sample at or
                // past our progress (its velocity is the target speed there).
                v_d = ramseteProfile.back().velocity;
                for (const auto& pt : ramseteProfile) {
                    if (pt.position >= progress) { v_d = pt.velocity; break; }
                }
            }

            // The profile reads 0 at both ends and the whole law scales with
            // v_d, so without a floor a path never starts from its own first
            // point. Only floor before the peak — past it the decel ramp has
            // to be allowed to reach zero or the robot creeps off the end.
            if (progress < ramsetePeakPos) {
                v_d = std::max(v_d, maxSpeedInPerSec *
                                    (req.ramseteParams.minSpeed / 127.0f));
            }

            // — Reference angular velocity ω_d = v_d · curvature (rad/s) —
            float w_d = 0.0f;
            if (targetIdx > 0 && targetIdx < static_cast<int>(path.size()) - 1) {
                const Pose& prev = path[targetIdx - 1];
                const Pose& next = path[targetIdx + 1];
                float segDist = distanceToPoint(prev, next.x, next.y);
                if (segDist > 1e-3f) {
                    w_d = v_d * angleDiffRad(prev.theta, next.theta) / segDist;
                }
            }

            // — Pose error in the robot frame —
            float dx = target.x - pose.x;
            float dy = target.y - pose.y;
            float e_x =  dx * std::cos(pose.theta) + dy * std::sin(pose.theta);
            float e_y = -dx * std::sin(pose.theta) + dy * std::cos(pose.theta);
            float e_theta = angleDiffRad(pose.theta, target.theta);

            // — RAMSETE gains + law (b scaled meters→inches) —
            float b = req.ramseteParams.b * (INCH_TO_METER * INCH_TO_METER);
            float zeta = req.ramseteParams.zeta;
            float k = 2.0f * zeta * std::sqrt(w_d * w_d + b * v_d * v_d);

            // sinc(e_θ) = sin(e_θ)/e_θ, → 1 as e_θ → 0 (avoids 0/0)
            float sinc = (std::fabs(e_theta) < 1e-4f)
                         ? 1.0f : std::sin(e_theta) / e_theta;

            float v = v_d * std::cos(e_theta) + k * e_x;             // in/s
            float w = w_d + b * v_d * sinc * e_y + k * e_theta;      // rad/s

            const float maxOut = req.ramseteParams.maxSpeed;

            emitTwist(v, w, maxOut, req.ramseteParams.useVelocityController,
                      wheelVel.left, wheelVel.right, tickDt, customLeft, customRight);
            twistOutput = true;

            settleError = dualSettleError(
                distToEnd, req.ramseteParams.targetTolerance,
                angleDiffRad(pose.theta, path.back().theta),
                degToRad(req.ramseteParams.headingTolerance));
            // Position alone: settleError also demands the final tangent, which
            // ramsete.hpp records as 5-12 deg off with no authority left to fix
            // it. Stopped and inside the position tolerance, the path arrived.
            // Arming the rest test on "the reference wants motion" instead was
            // what disarmed it in the one case it exists for -- an overshoot,
            // where the profile also reads zero.
            atTarget = distToEnd < req.ramseteParams.targetTolerance;
            break;
        }
        case MotionType::Trajectory: {
            if (req.trajectory.isEmpty()) {
                degenerate = true;
                break;
            }

            const TrajectoryFollowParams& tp = req.trajParams;
            const float timeScale = (tp.timeScale > 1e-3f) ? tp.timeScale : 1.0f;
            const float refTime = (static_cast<float>(elapsed) * 0.001f) / timeScale;
            const TrajectoryState ref = req.trajectory.sample(refTime);
            const Pose finalPose = req.trajectory.states().back().pose;

            // Stretching the reference in time slows its speeds by the same
            // factor; curvature is geometric and does not scale.
            const float vRef = ref.velocity / timeScale;
            const float wRef = vRef * ref.curvature;

            const bool refDone = refTime >= req.trajectory.totalTime();
            // The along-axis error, which is the only part the hold can close.
            float holdAlongErr = 0.0f;

            if (refDone && tp.holdAtEnd) {
                // Every RAMSETE term is multiplied by the reference velocity and
                // a trajectory ends at rest, so k and both outputs are
                // identically zero: holding could only delay the exit, never
                // correct anything. Settle the way a straight drive does.
                // Projecting onto the final heading axis deliberately leaves
                // perpendicular error alone -- correcting that from rest needs a
                // turn-drive-turn, which is not something to do behind the
                // caller's back at the end of a path.
                const MotionDemand d = straightDemand(
                    pose, StraightTarget{finalPose.x, finalPose.y, finalPose.theta});
                lateralOut = mLateralPID.update(d.lateral);
                angularOut = mAngularPID.update(d.angular);
                latLimit = tp.maxSpeed;
                angLimit = tp.maxSpeed;
                holdAlongErr = d.lateral;
            } else {
                const Pose err = ref.pose.relativeTo(pose);
                const float bIn = tp.b * (INCH_TO_METER * INCH_TO_METER);
                const float k =
                    2.0f * tp.zeta * std::sqrt(wRef * wRef + bIn * vRef * vRef);
                const float v = vRef * std::cos(err.theta) + k * err.x;
                const float w =
                    wRef + bIn * vRef * sinc(err.theta) * err.y + k * err.theta;

                emitTwist(v, w, tp.maxSpeed, tp.useVelocityController, wheelVel.left,
                          wheelVel.right, tickDt, customLeft, customRight);
                twistOutput = true;
            }

            // The clock owns the motion until the reference runs out, and a
            // reference that is legitimately stopped mid-trajectory is not a
            // stall. Both are `referenceDone`, not a fake error: the old
            // targetTolerance+1 step differentiated into 20 in/s off a
            // stationary robot and blocked the speed test it fed.
            const float posErr = distanceToPoint(pose, finalPose.x, finalPose.y);
            const float combined = dualSettleError(
                posErr, tp.targetTolerance,
                angleDiffRad(pose.theta, finalPose.theta),
                degToRad(tp.headingTolerance));
            settleError = tp.holdAtEnd ? combined : 0.0f;
            measuredError = combined;
            measuredErrorSet = true;
            // Judged on the error the hold can actually close, not the Euclidean
            // one. A purely perpendicular residual leaves straightDemand's
            // projection at zero, so the pose never moves and a posErr test can
            // never be met: the motion reported Stalled where the same physical
            // outcome with holdAtEnd off reported Arrived. settleError keeps the
            // full error, so the residual is still recorded honestly.
            atTarget = refDone && (!tp.holdAtEnd ||
                                   std::fabs(holdAlongErr) <= tp.targetTolerance);
            referenceDone = refDone;
            break;
        }
        case MotionType::Swing: {
            angLimit = req.swingParams.maxSpeed;
            const float rawErrRad = angleDiffRad(pose.theta, req.targetHeadingRad);
            angularOut = mAngularPID.update(
                resolvedTurnError(rawErrRad, req.swingParams.direction,
                                  degToRad(req.swingParams.targetTolerance),
                                  turnLatch));
            // The mix happens after the limit and the floor, so a swing gets the
            // same authority bounds as any other turn.
            swingOutput = true;
            settleError = rawErrRad;
            angFloor = req.swingParams.minSpeed;
            break;
        }
        default:
            break;
        }

        if (degenerate) {
            outcome = MotionOutcome::Invalid;
            break;
        }

        lateralOut = clamp(lateralOut, -latLimit, latLimit);
        angularOut = clamp(angularOut, -angLimit, angLimit);

        // Lateral only, so twist motions are unaffected by it.
        if (mConfig.lateralSlewPerSec > 0.0f) {
            const float step = mConfig.lateralSlewPerSec * tickDt;
            lateralOut = clamp(lateralOut, prevLateralOut - step,
                               prevLateralOut + step);
        }
        prevLateralOut = lateralOut;

        // The rest signal comes off the wheels, not the pose: a robot rotating
        // in place makes no headway toward a point yet is plainly not stalled.
        // A turn gets its turn rate, in the radians its threshold is now in.
        // With every motor silent the sweep reports zero, which is what a
        // stopped drivetrain reports too. That is not a rest measurement, so
        // hand the monitor a non-finite one: it declines to call that a stall.
        const float driveSpeed =
            (mDryRun || wheels.valid())
                ? restSpeed(req.type, wheelVel.left, wheelVel.right,
                            mDrivetrain.trackWidth)
                : std::numeric_limits<float>::quiet_NaN();
        const float rateInput = measuredErrorSet ? measuredError : settleError;
        const SettleState state = monitor.update(
            elapsed, SettleInput{settleError, errorRate.update(rateInput, tickDt),
                                 driveSpeed, atTarget, referenceDone});
        floorRetired = floorRetired ||
                       monitor.inToleranceMs(elapsed) >= kFloorRetireMs;

        // Below the floor the loop is asking for a crawl. Push it back up to
        // something the drivetrain can actually turn a wheel with, but only
        // before it has ever arrived -- forcing output afterwards kicks it back
        // out of tolerance and the pair limit cycles. minSpeed overrides the PID
        // and the slew; it does NOT override maxSpeed, which is why the floor is
        // applied inside the limit rather than after it.
        if (!twistOutput && !floorRetired) {
            lateralOut = applyOutputFloor(lateralOut, latFloor, latLimit);
            angularOut = applyOutputFloor(angularOut, angFloor, angLimit);
        }

        SidePair out;
        if (twistOutput) out = {customLeft, customRight};
        else if (swingOutput) out = swingMix(angularOut, req.swingSide);
        else out = tankMix(lateralOut, angularOut);

        if (!mDryRun) {
            mDrivetrain.setVoltage(out.left, out.right);
        } else {
            // Dry-run: integrate the motor outputs into a synthetic pose.
            float leftFrac  = out.left / 127.0f;
            float rightFrac = out.right / 127.0f;

            float linearVel  = (leftFrac + rightFrac) * 0.5f * maxSpeedInPerSec;
            float angularVel = (rightFrac - leftFrac) * maxSpeedInPerSec /
                mDrivetrain.trackWidth;

            const float dt = 0.01f;
            mDryRunPose = mDryRunPose.exp({linearVel * dt, 0.0f, angularVel * dt});
            mDryRunPose.theta = wrapRad(mDryRunPose.theta);

            // Cap recording at the preview draw limit (4096, about 41 s of sim
            // time) to bound worst-case path memory at ~48 KB.
            if (mDryRunPath.size() < 4096) {
                mDryRunPath.push_back(mDryRunPose);
            }
        }

        if (mTelemetry && !mDryRun) {
            mTelemetry({static_cast<int>(req.type),
                        pose.x, pose.y, stdRadToVexDeg(pose.theta),
                        req.targetX, req.targetY,
                        lateralOut, angularOut, monitor.inTolerance()});
        }

        // The output above is written before the exit test on purpose: a motion
        // that chains out leaves that voltage on for the next one.
        if (state != SettleState::Running) {
            outcome = outcomeFor(state);
            break;
        }

        if (mDryRun) {
            if (++dryRunSteps > 6000u) {   // cap: 60 s of simulated time
                outcome = MotionOutcome::TimedOut;
                break;
            }
        } else {
            pros::delay(10);
        }
    }

    mLastOutcome.store(outcome);
    endMotion(brakeOnEnd(outcome));
}

SettleConfig Chassis::settleConfig() const {
    return {mConfig.settleTimeMs, mConfig.settleSpeedIn, mConfig.settleSpeedDeg,
            mConfig.stallSpeedIn, mConfig.stallSpeedDeg, mConfig.stallTimeMs,
            mConfig.stallGraceMs};
}

namespace {
/// Negative means unspecified. Resolved here so the control loop never sees the
/// sentinel.
float orConfig(float v, float fallback) { return v >= 0.0f ? v : fallback; }
}  // namespace

void Chassis::moveDistance(float target, float timeout, const MoveDistanceParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Distance;
    req.moveDistParams = params;
    req.moveDistParams.targetTolerance =
        orConfig(params.targetTolerance, mConfig.targetToleranceIn);
    // Negative means backwards, not forwards to a target behind you -- that is
    // a 180 degree heading error.
    if (target < 0.0f) {
        req.moveDistParams.forwards = !req.moveDistParams.forwards;
        target = -target;
    }
    req.targetDistance = target;     // target X/Y computed at exec time
    req.timeout = timeout;

    enqueueMotion(req, async);
}

void Chassis::moveToPoint(float x, float y, float timeout, const MoveToPointParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Point;
    req.targetX = x;
    req.targetY = y;
    req.movePointParams = params;
    req.movePointParams.targetTolerance =
        orConfig(params.targetTolerance, mConfig.targetToleranceIn);
    req.movePointParams.bearingHoldDist =
        orConfig(params.bearingHoldDist, mConfig.bearingHoldDist);
    req.timeout = timeout;

    enqueueMotion(req, async);
}

void Chassis::turnToHeading(float thetaDeg, float timeout, const TurntoHeadingParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Heading;
    req.targetHeadingRad = vexToStdRad(thetaDeg);
    req.turnHeadingParams = params;
    req.turnHeadingParams.targetTolerance =
        orConfig(params.targetTolerance, mConfig.targetToleranceDeg);
    req.timeout = timeout;

    enqueueMotion(req, async);
}

void Chassis::turnToPoint(float x, float y, float timeout, const TurnToPointParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::TurnPoint;
    req.targetX = x;
    req.targetY = y;
    req.turnPointParams = params;
    req.turnPointParams.targetTolerance =
        orConfig(params.targetTolerance, mConfig.targetToleranceDeg);
    req.timeout = timeout;

    enqueueMotion(req, async);
}

void Chassis::moveArc(float x, float y, float radius, float timeout, const ArcParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Arc;
    req.targetX = x;
    req.targetY = y;
    req.targetRadius = radius;
    req.arcParams = params;
    req.timeout = timeout;

    enqueueMotion(req, async);
}

void Chassis::moveBoomerang(float x, float y, float thetaDeg, float timeout, const BoomerangParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Boomerang;
    req.targetX = x;
    req.targetY = y;
    req.targetHeadingRad = vexToStdRad(thetaDeg);
    req.boomerangParams = params;
    req.timeout = timeout;

    enqueueMotion(req, async);
}

void Chassis::moveRAMSETE(const std::vector<Pose>& path, float timeout, const RAMSETEParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::RAMSETE;
    req.ramsetePath = path;
    req.ramseteParams = params;
    req.ramseteParams.targetTolerance =
        orConfig(params.targetTolerance, mConfig.targetToleranceIn);
    req.timeout = timeout;
    // Mirrored, never edited: the exit test reads these and nothing else writes
    // them, so the two cannot drift apart.
    req.followerParams = {req.ramseteParams.targetTolerance,
                          req.ramseteParams.stallSpeed,
                          req.ramseteParams.settleTimeMs, true};

    enqueueMotion(req, async);
}

void Chassis::followTrajectory(const Trajectory& trajectory, float timeout,
                              const TrajectoryFollowParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Trajectory;
    req.trajParams = params;
    req.trajectory = trajectory;
    req.timeout = timeout;
    req.followerParams = {params.targetTolerance, params.stallSpeed,
                          params.settleTimeMs, params.holdAtEnd};
    if (!trajectory.isEmpty()) {
        const Pose& last = trajectory.states().back().pose;
        req.targetX = last.x;
        req.targetY = last.y;
        req.targetHeadingRad = last.theta;
    }
    enqueueMotion(req, async);
}

void Chassis::swingToHeading(float thetaDeg, SwingSide side, float timeout, const SwingParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Swing;
    req.targetHeadingRad = vexToStdRad(thetaDeg);
    req.swingSide = side;
    req.swingParams = params;
    req.swingParams.targetTolerance =
        orConfig(params.targetTolerance, mConfig.targetToleranceDeg);
    req.timeout = timeout;

    enqueueMotion(req, async);
}

void Chassis::enqueueMotion(MotionRequest& req, bool async) {
    ensureMotionTask();

    // Serialize enqueues: wait until no motion is queued so we keep the
    // "at most one pending" invariant and callers get sequential ordering.
    mQueueMutex.lock();
    while (!mMotionQueue.empty()) {
        mQueueMutex.unlock();
        if (mMotionCancelled) return;   // don't stack behind a cancel-in-progress
        pros::delay(5);
        mQueueMutex.lock();
    }
    req.seq = ++mMotionSeqCounter;
    mMotionQueue.push(req);
    mQueueMutex.unlock();
    mMotionTask->notify();

    if (!async) {
        // Block until the motion task finishes THIS motion (by sequence id).
        // A plain "queue empty && !running" check can return in the gap between
        // the task popping the request and marking it running, so we wait on a
        // completion counter that only advances after the motion actually ends.
        uint32_t mySeq = req.seq;
        while (!sequenceSettled(mCompletedSeq.load(), mySeq)) {
            if (mMotionCancelled) break;
            pros::delay(5);
        }
    }
}

MotionOutcome Chassis::lastOutcome() const {
    return mLastOutcome.load();
}

bool Chassis::lastMotionArrived() const {
    const MotionOutcome o = mLastOutcome.load();
    return o == MotionOutcome::Arrived || o == MotionOutcome::ChainedOut;
}

bool Chassis::isSettled() const {
    // Settled once the task has completed every enqueued motion. Using the
    // sequence ids also covers a motion that is queued but not yet running.
    return sequenceSettled(mCompletedSeq.load(), mMotionSeqCounter.load());
}

void Chassis::cancelMotion() {
    mMotionCancelled = true;          // signal current motion loop to stop

    // Clear all pending queued motions
    mQueueMutex.lock();
    const bool dropped = !mMotionQueue.empty();
    while (!mMotionQueue.empty()) mMotionQueue.pop();
    mQueueMutex.unlock();

    // Wake the persistent task so it sees the cancel
    if (mMotionTask != nullptr) {
        mMotionTask->notify();
    }

    // Wait for the current motion to actually stop
    while (mMotionRunning) {
        pros::delay(5);
    }

    // A motion dropped from the queue never ran, so it has no outcome of its
    // own; without this the caller it releases reads the PREVIOUS motion's.
    // Stored before the sequence id, or the caller can be woken first.
    if (dropped) mLastOutcome.store(MotionOutcome::Cancelled);

    // Release any blocking caller: every motion enqueued so far is now
    // resolved (either it ran, or it was dropped from the queue above).
    mCompletedSeq.store(publishCompleted(mCompletedSeq.load(),
                                         mMotionSeqCounter.load()));

    // Reset cancel flag — without this, subsequent async motions are
    // silently discarded because runMotionTask() checks mMotionCancelled
    // after dequeue and skips execution if it's still true.
    mMotionCancelled = false;

    // Safety: ensure motors are off (defense in depth)
    mDrivetrain.setVoltage(0, 0);
}

void Chassis::waitUntilSettled() {
    // Wait until the motion task has completed every enqueued motion. Comparing
    // sequence ids (rather than "queue empty && !running") closes the gap where
    // a just-enqueued motion is popped but not yet marked running.
    while (!sequenceSettled(mCompletedSeq.load(), mMotionSeqCounter.load())) {
        pros::delay(10);
    }
}

void Chassis::waitUntilDist(float dist) {
    // In dry-run the real pose never moves — waiting would hang the preview.
    if (mDryRun) return;
    Pose start = mOdom.getPose();
    // The background odometry task keeps the pose fresh — no update() here
    // (calling it from this task too would race and consume MCL's deltas).
    while (distanceToPoint(mOdom.getPose(), start.x, start.y) < dist) {
        // A motion that times out, stalls against a wall, or is cancelled may
        // never cover `dist`, and this is a straight-line displacement so a
        // path that curves back cannot reach it either. Without this the
        // calling routine blocks for the rest of the match.
        if (isSettled()) return;
        pros::delay(10);
    }
}

void Chassis::ensureMotionTask() {
    if (mMotionTask == nullptr) {
        mTaskShouldStop = false;
        mMotionTask = new pros::Task([this]() { runMotionTask(); });
    }
}

OdomTracking& Chassis::odom() { return mOdom; }
RclTracking& Chassis::rcl()  { return mRcl; }
MclTracking& Chassis::mcl()  { return mMcl; }
EkfTracking& Chassis::ekf()  { return mEkf; }

void Chassis::startTracking() {
    if (mConfig.useRclTracking) mRcl.startTracking();
    if (mConfig.useMclTracking) mMcl.startTracking();
    if (mConfig.useEkfTracking) {
        // One owner for the fused pose. Left on, MCL and RCL would keep lerping
        // odometry toward their own estimates underneath the filter that is
        // already fusing them, and the jitter the EKF absorbs comes straight
        // back. The EKF reads the same distance sensors RCL does, so RCL has
        // nothing to add here beyond a second opinion in the diagnostics.
        mMcl.setAutoSync(false);
        mRcl.setAutoSync(false);
        mEkf.setPose(mOdom.getPose());
        mEkf.startTracking();
    }
}

void Chassis::stopTracking() {
    mEkf.stopTracking();
    mRcl.stopTracking();
    mMcl.stopTracking();
}

void Chassis::setLateralGains(const PIDGains& gains) {
    mLateralPID.setGains(gains);
}

void Chassis::setAngularGains(const PIDGains& gains) {
    mAngularPID.setGains(gains);
}

void Chassis::setThrottleCurve(const DriveCurve& curve) {
    mThrottleCurve = curve;
}

void Chassis::setSteerCurve(const DriveCurve& curve) {
    mSteerCurve = curve;
}

void Chassis::setPidTuning(const ChassisConfig& config) {
    mConfig.lateralGains = config.lateralGains;
    mConfig.angularGains = config.angularGains;
    mConfig.lateralWindupRange = config.lateralWindupRange;
    mConfig.angularWindupRange = config.angularWindupRange;
    mConfig.lateralDerivativeFilter = config.lateralDerivativeFilter;
    mConfig.angularDerivativeFilter = config.angularDerivativeFilter;
    mConfig.settleTimeMs = config.settleTimeMs;
    // The rest of the exit test moves with the gains: a loop retuned to a new kD
    // settles at a different speed, and stalls at a different threshold.
    mConfig.settleSpeedIn = config.settleSpeedIn;
    mConfig.settleSpeedDeg = config.settleSpeedDeg;
    mConfig.stallSpeedIn = config.stallSpeedIn;
    mConfig.stallSpeedDeg = config.stallSpeedDeg;
    mConfig.stallTimeMs = config.stallTimeMs;
    mConfig.stallGraceMs = config.stallGraceMs;

    mLateralPID.setGains(config.lateralGains);
    mAngularPID.setGains(config.angularGains);
    mLateralPID.setWindupRange(config.lateralWindupRange);
    mAngularPID.setWindupRange(config.angularWindupRange);
    mLateralPID.setDerivativeFilter(config.lateralDerivativeFilter);
    mAngularPID.setDerivativeFilter(config.angularDerivativeFilter);
}

void Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    mDrivetrain.setBrakeMode(mode);
}

void Chassis::setDistanceSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors) {
    mConfig.distanceSensors = sensors;
    mMcl.setDistanceSensors(sensors);
    mRcl.configureSensors(sensors, mConfig.sensorMounts);
}

void Chassis::setTelemetry(std::function<void(const ChassisSample&)> callback) {
    mTelemetry = std::move(callback);
}

void Chassis::setDryRun(bool enabled) {
    mDryRun = enabled;

    // The offline dry-run loop doesn't consume wall-clock time, so the PIDs
    // must use the simulated 10 ms step — otherwise dt ≈ 0 kills the I/D terms
    // and the previewed motion is under-damped compared to the real loop.
    mLateralPID.setFixedDt(enabled ? 0.01f : 0.0f);
    mAngularPID.setFixedDt(enabled ? 0.01f : 0.0f);

    if (enabled) {
        mDryRunPath.clear();
        mDryRunStartPose = mOdom.getPose();
        // Seed the synthetic pose too. executeCurrentMotion() only seeds it
        // when mDryRunPath is empty, which it no longer is after the push
        // below, so leaving it stale previews a path from the wrong origin.
        mDryRunPose = mDryRunStartPose;
        mDryRunPath.push_back(mDryRunStartPose);
    }
}

bool Chassis::isDryRun() const {
    return mDryRun;
}

const std::vector<Pose>& Chassis::dryRunPath() const {
    return mDryRunPath;
}

void Chassis::resetDryRunPath() {
    mDryRunPath.clear();
    mDryRunPath.shrink_to_fit();   // release the heap between dry runs
}

}  // namespace chad
