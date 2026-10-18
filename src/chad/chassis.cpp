#include "chad/chassis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "chad/movement_control/spline.hpp"
#include "chad/util/fasttrig.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"

namespace chad {
namespace {
// moveToPoint, inside the bearing-hold radius. Swept on sim/test/near_target.mjs.
// The bearing latches once facing within 25 deg; until then the robot turns in
// place. 20 re-turns too far on a body that pivots off its tracking centre,
// 35 leaves more cross-track and chatter.
constexpr float kPointLatchCos = 0.906f;        // cos 25 deg
// Along-track settling needs the heading within 15 deg of the held bearing.
constexpr float kPointAlignedCos = 0.966f;      // cos 15 deg
// Stall exit: both sides under 1 in/s, inside 1.5 in along-track, after the
// drive has once exceeded 2 in/s. kP 9.5 drops under a typical drive's static
// friction near 1 in out; 0.75 in times out on that plant.
constexpr float kPointStallRadiusIn = 1.5f;
constexpr float kPointStoppedInPerSec = 1.0f;
constexpr float kPointMovingInPerSec = 2.0f;
}

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

    mMcl.setSensorMounts(config.sensorMounts);
    mEkf.setSensorMounts(config.sensorMounts);
    mEkf.attachMcl(&mMcl);
    mEkf.attachRcl(&mRcl);

    mRcl.configureSensors(config.distanceSensors, config.sensorMounts);

    ensureMotionTask();

    mOdom.startBackgroundOdom();
}

Chassis::~Chassis() {
    mTaskShouldStop = true;
    abortMotion();
    if (mMotionTask != nullptr) {
        for (int i = 0; i < 100 && mMotionRunning; ++i) pros::delay(5);
        mMotionTask->remove();
        delete mMotionTask;
        mMotionTask = nullptr;
    }
    mDrivetrain.setVoltage(0, 0);

    mEkf.stopTracking();
    mMcl.stopTracking();
    mRcl.stopTracking();
    mOdom.stopBackgroundOdom();
}

void Chassis::calibrate() {
    mOdom.calibrate();
}

void Chassis::setMirror(FieldMirror mirror) { mMirror = mirror; }

FieldMirror Chassis::mirror() const { return mMirror; }

TurnDirection Chassis::mirrorTurn(TurnDirection dir) const {
    if (!mirrorFlipsHandedness(mMirror)) return dir;
    if (dir == TurnDirection::CW)  return TurnDirection::CCW;
    if (dir == TurnDirection::CCW) return TurnDirection::CW;
    return dir;
}

// Every motion funnels through enqueueMotion, so this is the one place that
// has to know the shape of each request.
void Chassis::applyMirror(MotionRequest& req) const {
    if (mMirror == FieldMirror::None) return;

    switch (req.type) {
        case MotionType::Point:
            mirrorPointInPlace(mMirror, req.targetX, req.targetY);
            break;
        case MotionType::TurnPoint:
            mirrorPointInPlace(mMirror, req.targetX, req.targetY);
            req.turnPointParams.direction = mirrorTurn(req.turnPointParams.direction);
            if (mirrorFlipsHandedness(mMirror)) req.turnPointParams.offset = -req.turnPointParams.offset;
            break;
        case MotionType::Heading:
            req.targetHeadingRad = mirrorHeadingRad(mMirror, req.targetHeadingRad);
            req.turnHeadingParams.direction = mirrorTurn(req.turnHeadingParams.direction);
            break;
        case MotionType::Arc:
            mirrorPointInPlace(mMirror, req.targetX, req.targetY);
            if (mirrorFlipsHandedness(mMirror)) req.targetRadius = -req.targetRadius;
            break;
        case MotionType::Boomerang:
            mirrorPointInPlace(mMirror, req.targetX, req.targetY);
            req.targetHeadingRad = mirrorHeadingRad(mMirror, req.targetHeadingRad);
            break;
        case MotionType::RAMSETE:
            for (Pose& p : req.ramsetePath) p = mirrorPose(mMirror, p);
            break;
        case MotionType::Swing:
            req.targetHeadingRad = mirrorHeadingRad(mMirror, req.targetHeadingRad);
            req.swingParams.direction = mirrorTurn(req.swingParams.direction);
            if (mirrorFlipsHandedness(mMirror)) {
                req.swingSide = (req.swingSide == SwingSide::Left) ? SwingSide::Right
                                                                   : SwingSide::Left;
            }
            break;
        // A distance is relative to wherever the robot already points.
        case MotionType::Distance:
        case MotionType::None:
            break;
    }
}

void Chassis::setPose(const Pose& raw) {
    setPoseFieldFrame(mirrorPose(mMirror, raw));
}

void Chassis::setPoseFieldFrame(const Pose& pose) {
    if (mDryRun) {
        mDryRunStartPose = pose;
        mDryRunPose = pose;
        mDryRunPath.clear();
        mDryRunPath.push_back(pose);
        return;
    }
    resetPose(pose);
}

void Chassis::resetPose(const Pose& pose) {
    // An RCL cycle that lands between the odom write and its reset measures
    // against the old pose and syncs odom back toward it. Resetting on both
    // sides discards whatever such a cycle held.
    mRcl.hardReset(pose);
    mOdom.setPose(pose);
    mRcl.hardReset(pose);

    // A particle resample is milliseconds on the Brain; skip filters that are
    // neither configured nor running.
    if (mConfig.useMclTracking || mMcl.isTracking()) mMcl.setPose(pose);
    if (mConfig.useEkfTracking || mEkf.isTracking()) mEkf.setPose(pose);
}

void Chassis::setPosition(float x, float y) {
    mirrorPointInPlace(mMirror, x, y);
    if (mDryRun) {
        mDryRunPose.x = x;
        mDryRunPose.y = y;
        mDryRunPath.push_back(mDryRunPose);
        return;
    }
    Pose current = mOdom.getPose();
    resetPose({x, y, current.theta});
}

void Chassis::setHeading(float thetaDeg) {
    const float theta = mirrorHeadingRad(mMirror, vexToStdRad(thetaDeg));
    if (mDryRun) {
        mDryRunPose.theta = theta;
        mDryRunPath.push_back(mDryRunPose);
        return;
    }
    Pose current = mOdom.getPose();
    resetPose({current.x, current.y, theta});
}

Pose Chassis::getPose() const {
    return mirrorPose(mMirror, mOdom.getPose());
}

void Chassis::tank(float left, float right) {
    if (mDryRun) return;
    if (!isSettled()) return;
    float l = mThrottleCurve.apply(left);
    float r = mThrottleCurve.apply(right);
    mDrivetrain.setVoltage(l, r);
}

void Chassis::arcade(float throttle, float steer) {
    if (mDryRun) return;
    if (!isSettled()) return;
    float t = mThrottleCurve.apply(throttle);
    float s = mSteerCurve.apply(steer);

    float left  = t + s;
    float right = t - s;

    float maxVal = std::max(std::fabs(left), std::fabs(right));
    if (maxVal > 127.0f) {
        left  = left  * 127.0f / maxVal;
        right = right * 127.0f / maxVal;
    }

    mDrivetrain.setVoltage(left, right);
}

void Chassis::curvature(float throttle, float steer) {
    if (mDryRun) return;
    if (!isSettled()) return;
    float t = mThrottleCurve.apply(throttle);
    float s = mSteerCurve.apply(steer);

    float left, right;
    if (std::fabs(t) < 5.0f) {
        left = s;
        right = -s;
    } else {
        float turnScale = std::fabs(s) / 127.0f;
        if (s > 0) {
            left = t;
            right = t * (1.0f - turnScale * 0.7f);
        } else {
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

float Chassis::computeLateralError(float targetX, float targetY) const {
    Pose current = mOdom.getPose();
    return distanceToPoint(current, targetX, targetY);
}

float Chassis::computeAngularError(float targetHeadingRad) const {
    Pose current = mOdom.getPose();
    return angleDiffRad(current.theta, targetHeadingRad);
}

float Chassis::computeAngularErrorToPoint(float targetX, float targetY) const {
    Pose current = mOdom.getPose();
    float bearingRad = bearingToPoint(current, targetX, targetY);
    return angleDiffRad(current.theta, bearingRad);
}

void Chassis::startMotion() {
    mMotionRunning = true;
}

void Chassis::endMotion(uint32_t seq) {
    // Released by a cancelMotion that stopped waiting: the drive is the
    // caller's now, and a 0 V here would override it.
    if (!mMotionSlot.completed(seq)) mDrivetrain.setVoltage(0, 0);
    mMotionRunning = false;

    mLateralPID.reset();
    mAngularPID.reset();
}

void Chassis::runMotionTask() {
    while (!mTaskShouldStop) {
        pros::Task::notify_take(true, TIMEOUT_MAX);
        if (mTaskShouldStop) break;

        while (true) {
            mQueueMutex.lock();
            const bool hasWork = mMotionSlot.pop(mCurrentMotion, mCurrentSeq);
            mQueueMutex.unlock();

            if (!hasWork) break;

            if (!mTaskShouldStop) executeCurrentMotion();

            mQueueMutex.lock();
            mMotionSlot.finish(mCurrentSeq);
            mQueueMutex.unlock();
        }
    }
}

void Chassis::executeCurrentMotion() {
    auto& req = mCurrentMotion;
    const uint32_t seq = mCurrentSeq;
    if (mMotionSlot.cancelled(seq)) return;

    req.startTime = pros::millis();

    if (mDryRun) {
        if (mDryRunPath.empty()) {
            mDryRunPose = mOdom.getPose();
            mDryRunPath.push_back(mDryRunPose);
        }
        mDryRunStartPose = mDryRunPose;
    }

    float maxSpeedInPerSec = mDrivetrain.rpm / 60.0f *
        mDrivetrain.wheelDiameter * PI;

    float distTargetX = 0.0f, distTargetY = 0.0f;

    float distAxisRad = 0.0f;
    if (req.type == MotionType::Distance) {
        Pose startPose = mDryRun ? mDryRunPose : mOdom.getPose();
        float dir = req.moveDistParams.forwards ? 1.0f : -1.0f;
        distAxisRad = startPose.theta;
        distTargetX = startPose.x + req.targetDistance * dir * std::cos(startPose.theta);
        distTargetY = startPose.y + req.targetDistance * dir * std::sin(startPose.theta);
    }

    std::vector<float> ramseteCumLen;
    std::vector<ProfilePoint> ramseteProfile;
    float ramsetePeakPos = 0.0f;
    if (req.type == MotionType::RAMSETE) {
        const auto& path = req.ramsetePath;
        if (req.ramseteParams.useVelocityController) {
            mRamseteFf.setGains(req.ramseteParams.ffGains);

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

        float cruiseInPerSec = maxSpeedInPerSec * (req.ramseteParams.maxSpeed / 127.0f);
        const float maxAccel = req.ramseteParams.maxAccel;
        const float latAccel = req.ramseteParams.maxLateralAccel;

        if (req.ramseteParams.useVelocityProfile && totalLen > 1e-3f &&
            cruiseInPerSec > 1e-3f && maxAccel > 1e-3f) {
            if (latAccel <= 1e-3f || path.size() < 3) {
                ramseteProfile = generateTrapezoidal(totalLen, cruiseInPerSec, maxAccel);
            } else {
                const size_t n = path.size();
                std::vector<float> v(n, cruiseInPerSec);

                for (size_t i = 1; i + 1 < n; ++i) {
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

                for (size_t i = 1; i < n; ++i) {
                    float ds = ramseteCumLen[i] - ramseteCumLen[i - 1];
                    v[i] = std::min(v[i], std::sqrt(v[i - 1] * v[i - 1] + 2.0f * maxAccel * ds));
                }
                for (size_t i = n - 1; i > 0; --i) {
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

    startMotion();

    uint32_t dryRunSteps = 0;

    bool settleStarted = false;
    uint32_t settleStartElapsed = 0;

    bool bearingHeld = false;
    float heldBearing = 0.0f;

    // moveToPoint: turning in place to face the target before holding its
    // bearing; whether the drive has moved yet, so a robot that has not
    // started is never "stalled".
    bool pointAligning = false;
    bool pointMoved = false;

    bool approachSet = false;
    float approachX = 0.0f, approachY = 0.0f;

    float prevLateralOut = 0.0f;

    Drivetrain::SideHold swingHold;

    uint32_t prevTick = pros::millis();
    uint32_t wakeTick = prevTick;
    // PROS deletes the competition task on a mode change without unwinding,
    // so whoever issued this motion may no longer exist to cancel it.
    const uint8_t compStatus = pros::competition::get_status();

    while (mMotionRunning && !mMotionSlot.cancelled(seq)) {
        if (pros::competition::get_status() != compStatus) break;

        const uint32_t nowTick = pros::millis();
        const uint32_t tickMs = nowTick - prevTick;
        float tickDt = tickMs / 1000.0f;
        if (mDryRun || tickDt <= 0.0f || tickDt > 0.1f) tickDt = 0.01f;
        prevTick = nowTick;

        Pose pose = mDryRun ? mDryRunPose : mOdom.getPose();

        float lateralOut = 0.0f, angularOut = 0.0f;
        float customLeft = 0.0f, customRight = 0.0f;
        bool settled = false;
        bool exitNow = false;
        bool useCustomOutput = false;
        float latLimit = 127.0f;
        float angLimit = 127.0f;
        // Speed floors and the early-exit band, filled in per motion type.
        // remainingErr is inches, or degrees for turns; left negative,
        // the early exit never fires.
        float latFloor = 0.0f;
        float angFloor = 0.0f;
        float remainingErr = -1.0f;
        float earlyExit = 0.0f;

        switch (req.type) {
        case MotionType::Distance: {
            latLimit = req.moveDistParams.maxSpeed;
            latFloor = req.moveDistParams.minSpeed;
            earlyExit = req.moveDistParams.earlyExitRange;

            float targetX = distTargetX;
            float targetY = distTargetY;

            const float along = (targetX - pose.x) * std::cos(distAxisRad) +
                                (targetY - pose.y) * std::sin(distAxisRad);
            lateralOut = mLateralPID.update(along);

            angularOut = mAngularPID.update(angleDiffRad(pose.theta, distAxisRad));
            remainingErr = std::fabs(along);
            settled = remainingErr < req.moveDistParams.targetTolerance;
            break;
        }
        case MotionType::Point: {
            latLimit = req.movePointParams.maxSpeed;
            latFloor = req.movePointParams.minSpeed;
            earlyExit = req.movePointParams.earlyExitRange;
            const float distErr = distanceToPoint(pose, req.targetX, req.targetY);
            const float liveBearing = bearingToPoint(pose, req.targetX, req.targetY);

            const float along =
                distErr * std::cos(angleDiffRad(pose.theta, liveBearing));
            lateralOut = mLateralPID.update(along);

            // Inside the hold radius the live bearing swings faster than the
            // robot can turn, so it is held. Not before the robot faces it: a
            // bearing held while turning leaves the cross-track of the turn,
            // and driving while turning there orbits a body that pivots off
            // its tracking centre. Facing it can swing the robot back out of
            // the radius; it then drives in on the live bearing. A chained
            // move (minSpeed) keeps its speed and holds on entry instead.
            float steerBearing = liveBearing;
            if (!bearingHeld &&
                (distErr < req.movePointParams.bearingHoldDist || pointAligning)) {
                const float faceLive = req.movePointParams.forwards
                                           ? liveBearing : wrapRad(liveBearing + PI);
                pointAligning = latFloor <= 0.0f &&
                    std::cos(angleDiffRad(pose.theta, faceLive)) <= kPointLatchCos;
                if (!pointAligning && distErr < req.movePointParams.bearingHoldDist) {
                    heldBearing = liveBearing;
                    bearingHeld = true;
                }
            }
            if (bearingHeld) steerBearing = heldBearing;
            if (pointAligning) {
                lateralOut = 0.0f;
                latFloor = 0.0f;
            }

            if (!req.movePointParams.forwards) {
                steerBearing = wrapRad(steerBearing + PI);
            }
            const float headingErr = angleDiffRad(pose.theta, steerBearing);
            angularOut = mAngularPID.update(headingErr);
            remainingErr = distErr;

            // A held bearing cannot remove cross-track error: once held and
            // aligned with it, the along-track error decides.
            const float tol = req.movePointParams.targetTolerance;
            const float absAlong = std::fabs(along);
            settled = distErr < tol ||
                      (bearingHeld && absAlong < tol &&
                       std::cos(headingErr) > kPointAlignedCos);

            // Stopped short of tolerance: the output is under the drive's
            // static friction and waiting out the timeout gains nothing.
            // Wheel speed, not pose deltas, which RCL/EKF corrections jolt.
            if (!mDryRun && (!pointMoved || bearingHeld)) {
                const float wheel =
                    std::max(std::fabs(mDrivetrain.leftVelocityInPerSec()),
                             std::fabs(mDrivetrain.rightVelocityInPerSec()));
                if (wheel > kPointMovingInPerSec) pointMoved = true;
                if (bearingHeld && pointMoved && wheel < kPointStoppedInPerSec &&
                    absAlong < kPointStallRadiusIn) {
                    settled = true;
                }
            }

            // A chained move that passes beside its target never enters the
            // early-exit radius; it exits on crossing the line through the
            // target square to the approach instead.
            const float toX = req.targetX - pose.x;
            const float toY = req.targetY - pose.y;
            if (!approachSet && distErr > req.movePointParams.targetTolerance) {
                approachX = toX;
                approachY = toY;
                approachSet = true;
            }
            if (latFloor != 0.0f && toX * approachX + toY * approachY < 0.0f) {
                exitNow = true;
            }
            break;
        }
        case MotionType::Heading: {
            angLimit = req.turnHeadingParams.maxSpeed;
            angFloor = req.turnHeadingParams.minSpeed;
            earlyExit = req.turnHeadingParams.earlyExitRange;
            float rawErrRad = angleDiffRad(pose.theta, req.targetHeadingRad);

            float headingErrRad = resolveTurnError(rawErrRad, req.turnHeadingParams.direction);

            float tolRad = degToRad(req.turnHeadingParams.targetTolerance);
            if (std::fabs(rawErrRad) < tolRad) headingErrRad = rawErrRad;
            angularOut = mAngularPID.update(headingErrRad);
            lateralOut = 0;
            remainingErr = radToDeg(std::fabs(headingErrRad));
            settled = std::fabs(rawErrRad) < tolRad;
            break;
        }
        case MotionType::TurnPoint: {
            angLimit = req.turnPointParams.maxSpeed;
            angFloor = req.turnPointParams.minSpeed;
            earlyExit = req.turnPointParams.earlyExitRange;
            const float faceOffset = req.turnPointParams.offset +
                (req.turnPointParams.forwards ? 0.0f : 180.0f);
            float rawErrRad = headingToPointOffset(pose, req.targetX, req.targetY,
                                                   faceOffset);
            float headingErrRad = resolveTurnError(rawErrRad, req.turnPointParams.direction);
            float tolRad = degToRad(req.turnPointParams.targetTolerance);
            if (std::fabs(rawErrRad) < tolRad) headingErrRad = rawErrRad;
            angularOut = mAngularPID.update(headingErrRad);
            lateralOut = 0;
            // Degrees, and the way the turn is forced to go: a CW turn that
            // starts just past the target still has most of a circle left.
            remainingErr = radToDeg(std::fabs(headingErrRad));
            settled = std::fabs(rawErrRad) < tolRad;
            break;
        }
        case MotionType::Arc: {
            latLimit = req.arcParams.maxSpeed;
            latFloor = req.arcParams.minSpeed;

            float dx = req.targetX - pose.x;
            float dy = req.targetY - pose.y;
            float chordDist = std::sqrt(dx * dx + dy * dy);

            float chordBearingRad = std::atan2(dy, dx);

            float radius = std::fabs(req.targetRadius);

            const float chordSign =
                (std::cos(angleDiffRad(pose.theta, chordBearingRad)) >= 0.0f)
                    ? 1.0f : -1.0f;

            const float targetHeadingRad =
                (radius < 1e-6f)
                    ? chordBearingRad
                    : arcStartHeading(pose, req.targetX, req.targetY, req.targetRadius);

            lateralOut = mLateralPID.update(chordSign * chordDist);
            angularOut = mAngularPID.update(angleDiffRad(pose.theta, targetHeadingRad));
            remainingErr = chordDist;
            settled = chordDist < req.arcParams.targetTolerance;
            break;
        }
        case MotionType::Boomerang: {
            latLimit = req.boomerangParams.maxSpeed;

            float dx = req.targetX - pose.x;
            float dy = req.targetY - pose.y;
            float distToTarget = std::sqrt(dx * dx + dy * dy);

            float lead = req.boomerangParams.lead;
            float leadDecay = req.boomerangParams.leadDecay;

            float clampedLead = lead * distToTarget;
            if (clampedLead > 24.0f) clampedLead = 24.0f;

            // Backing in, the robot travels opposite the heading it ends on,
            // so the carrot leads along the travel direction and steering
            // points the back at it. The final heading is the same either way.
            const bool forwards = req.boomerangParams.forwards;
            const float travelRad = forwards ? req.targetHeadingRad
                                             : wrapRad(req.targetHeadingRad + PI);

            float carrotX = req.targetX - clampedLead * std::cos(travelRad);
            float carrotY = req.targetY - clampedLead * std::sin(travelRad);

            float t = (lead > 1e-6f)
                      ? std::min(distToTarget / (lead * 12.0f), 1.0f)
                      : 1.0f;
            carrotX = req.targetX + (carrotX - req.targetX) * (1.0f - t * leadDecay);
            carrotY = req.targetY + (carrotY - req.targetY) * (1.0f - t * leadDecay);

            float dCarrotX = carrotX - pose.x;
            float dCarrotY = carrotY - pose.y;
            float distToCarrot = std::sqrt(dCarrotX * dCarrotX + dCarrotY * dCarrotY);
            float carrotBearingRad = std::atan2(dCarrotY, dCarrotX);

            const float along =
                distToCarrot * std::cos(angleDiffRad(pose.theta, carrotBearingRad));
            lateralOut = mLateralPID.update(along);

            if (bearingHeld) {
                carrotBearingRad = heldBearing;
            } else if (distToCarrot < mConfig.boomerangBearingHoldDist) {
                heldBearing = carrotBearingRad;
                bearingHeld = true;
            }
            float headingToCarrotRad = angleDiffRad(
                pose.theta, forwards ? carrotBearingRad : wrapRad(carrotBearingRad + PI));
            float headingToFinalRad  = angleDiffRad(pose.theta, req.targetHeadingRad);

            float headingWeight = std::min(distToTarget / 12.0f, 1.0f);
            float blendedHeadingErrRad = headingToCarrotRad * headingWeight +
                headingToFinalRad * (1.0f - headingWeight);

            angularOut = mAngularPID.update(blendedHeadingErrRad);

            float speedScale = std::min(distToTarget / 6.0f, 1.0f);
            if (req.boomerangParams.maxSpeed > 1e-6f) {
                const float floorScale =
                    req.boomerangParams.minSpeed / req.boomerangParams.maxSpeed;
                if (speedScale < floorScale) speedScale = floorScale;
            }
            lateralOut *= speedScale;

            bool distSettled = distToTarget < req.boomerangParams.targetTolerance;
            bool headingSettled = std::fabs(headingToFinalRad) < degToRad(req.boomerangParams.headingTolerance);
            settled = distSettled && headingSettled;

            break;
        }
        case MotionType::RAMSETE: {
            const auto& path = req.ramsetePath;
            if (path.size() < 2) {
                exitNow = true;
                break;
            }

            float distToEnd = distanceToPoint(pose, path.back().x, path.back().y);

            int closestIdx = closestPathIndex(path, pose);
            int targetIdx = closestIdx;
            const Pose& target = path[targetIdx];

            float progress = (closestIdx >= 0 &&
                              closestIdx < static_cast<int>(ramseteCumLen.size()))
                             ? ramseteCumLen[closestIdx] : 0.0f;
            float v_d = maxSpeedInPerSec * (req.ramseteParams.maxSpeed / 127.0f);
            if (!ramseteProfile.empty()) {
                v_d = ramseteProfile.back().velocity;
                for (const auto& pt : ramseteProfile) {
                    if (pt.position >= progress) { v_d = pt.velocity; break; }
                }
            }

            if (progress < ramsetePeakPos) {
                v_d = std::max(v_d, maxSpeedInPerSec *
                                    (req.ramseteParams.minSpeed / 127.0f));
            }

            if (!req.ramseteParams.forwards) v_d = -v_d;

            float w_d = 0.0f;
            if (targetIdx > 0 && targetIdx < static_cast<int>(path.size()) - 1) {
                const Pose& prev = path[targetIdx - 1];
                const Pose& next = path[targetIdx + 1];
                float segDist = distanceToPoint(prev, next.x, next.y);
                if (segDist > 1e-3f) {
                    // Speed, not velocity: the robot covers the path start to end
                    // either way, so its heading rate keeps the curve's sign.
                    w_d = std::fabs(v_d) * angleDiffRad(prev.theta, next.theta)
                          / segDist;
                }
            }

            float dx = target.x - pose.x;
            float dy = target.y - pose.y;
            float e_x =  dx * std::cos(pose.theta) + dy * std::sin(pose.theta);
            float e_y = -dx * std::sin(pose.theta) + dy * std::cos(pose.theta);
            float e_theta = angleDiffRad(pose.theta, target.theta);

            float b = req.ramseteParams.b * (INCH_TO_METER * INCH_TO_METER);
            float zeta = req.ramseteParams.zeta;
            float k = 2.0f * zeta * std::sqrt(w_d * w_d + b * v_d * v_d);

            float sinc = (std::fabs(e_theta) < 1e-4f)
                         ? 1.0f : std::sin(e_theta) / e_theta;

            float v = v_d * std::cos(e_theta) + k * e_x;
            float w = w_d + b * v_d * sinc * e_y + k * e_theta;

            float halfTrack = mDrivetrain.trackWidth * 0.5f;
            float vLeft  = v - w * halfTrack;
            float vRight = v + w * halfTrack;
            float maxOut = req.ramseteParams.maxSpeed;

            const bool nearEnd = distToEnd < req.ramseteParams.targetTolerance;
            const bool wantSideVel = req.ramseteParams.useVelocityController ||
                                     (nearEnd && req.ramseteParams.stallSpeed > 0.0f);
            float vlMeas = 0.0f, vrMeas = 0.0f;
            if (wantSideVel) {
                vlMeas = mDrivetrain.leftVelocityInPerSec();
                vrMeas = mDrivetrain.rightVelocityInPerSec();
            }

            if (req.ramseteParams.useVelocityController) {
                const float outLimit = maxSpeedInPerSec * (maxOut / 127.0f);
                const float peak = std::max(std::fabs(vLeft), std::fabs(vRight));
                if (peak > outLimit && peak > 1e-6f) {
                    const float sc = outLimit / peak;
                    v *= sc; w *= sc;
                }
                const DrivetrainVoltages out = mRamseteFf.update(
                    v * INCH_TO_METER, w,
                    vlMeas * INCH_TO_METER, vrMeas * INCH_TO_METER, tickDt);
                customLeft  = clamp(out.leftPower(),  -maxOut, maxOut);
                customRight = clamp(out.rightPower(), -maxOut, maxOut);
            } else {
                float toMotor = (maxSpeedInPerSec > 1e-3f) ? (127.0f / maxSpeedInPerSec) : 0.0f;
                customLeft  = clamp(vLeft  * toMotor, -maxOut, maxOut);
                customRight = clamp(vRight * toMotor, -maxOut, maxOut);
            }
            useCustomOutput = true;

            {
                const bool headingOk =
                    std::fabs(e_theta) < degToRad(req.ramseteParams.headingTolerance);
                const bool stalled =
                    req.ramseteParams.stallSpeed > 0.0f &&
                    std::max(std::fabs(vlMeas), std::fabs(vrMeas)) <
                        req.ramseteParams.stallSpeed;
                settled = nearEnd && (headingOk || stalled);
            }

            break;
        }
        case MotionType::Swing: {
            angLimit = req.swingParams.maxSpeed;

            float rawErrRad = angleDiffRad(pose.theta, req.targetHeadingRad);
            float headingErrRad = resolveTurnError(rawErrRad, req.swingParams.direction);

            float tolRad = degToRad(req.swingParams.targetTolerance);
            if (std::fabs(rawErrRad) < tolRad) headingErrRad = rawErrRad;
            settled = std::fabs(rawErrRad) < tolRad;

            angularOut = clamp(mAngularPID.update(headingErrRad), -angLimit, angLimit);
            const float swingFloor = std::min(req.swingParams.minSpeed, angLimit);
            if (!settled && swingFloor > 0.0f && std::fabs(angularOut) < swingFloor) {
                angularOut = (angularOut < 0.0f) ? -swingFloor : swingFloor;
            }
            lateralOut = 0;

            // The named side drives with its point-turn share; the other is
            // held below.
            if (req.swingSide == SwingSide::Left) {
                customLeft = clamp(-angularOut, -127.0f, 127.0f);
                customRight = 0;
            } else {
                customLeft = 0;
                customRight = clamp(angularOut, -127.0f, 127.0f);
            }
            useCustomOutput = true;

            break;
        }
        default:
            break;
        }

        lateralOut = clamp(lateralOut, -latLimit, latLimit);
        angularOut = clamp(angularOut, -angLimit, angLimit);

        // A floor on the magnitude, never the direction, and only while
        // there is error left -- held through the tolerance band it would
        // stop the motion settling. Before the slew, so the ramp still
        // shapes the way up to it.
        latFloor = std::min(latFloor, latLimit);
        angFloor = std::min(angFloor, angLimit);
        if (!settled && latFloor > 0.0f && std::fabs(lateralOut) < latFloor) {
            lateralOut = (lateralOut < 0.0f) ? -latFloor : latFloor;
        }
        if (!settled && angFloor > 0.0f && std::fabs(angularOut) < angFloor) {
            angularOut = (angularOut < 0.0f) ? -angFloor : angFloor;
        }

        if (mConfig.lateralSlewPerSec > 0.0f) {
            const float step = mConfig.lateralSlewPerSec * tickDt;
            lateralOut = clamp(lateralOut, prevLateralOut - step,
                               prevLateralOut + step);
        }
        prevLateralOut = lateralOut;

        {
        float left, right;
        if (useCustomOutput) {
            left  = customLeft;
            right = customRight;
        } else {
            left  = clamp(lateralOut - angularOut, -127.0f, 127.0f);
            right = clamp(lateralOut + angularOut, -127.0f, 127.0f);
        }

        if (!mDryRun) {
            // The cancel may have landed during this tick's computation.
            if (mMotionSlot.cancelled(seq)) break;
            if (req.type == MotionType::Swing) {
                // 0 V alone lets a coasting side be dragged. The held side
                // gets HOLD once and no further commands.
                const bool leftDrives = req.swingSide == SwingSide::Left;
                if (swingHold.group == nullptr) swingHold = mDrivetrain.holdSide(!leftDrives);
                if (leftDrives) mDrivetrain.setLeftVoltage(left);
                else            mDrivetrain.setRightVoltage(right);
            } else {
                mDrivetrain.setVoltage(left, right);
            }
        } else {
            float leftFrac  = left / 127.0f;
            float rightFrac = right / 127.0f;

            float linearVel  = (leftFrac + rightFrac) * 0.5f * maxSpeedInPerSec;
            float angularVel = (rightFrac - leftFrac) * maxSpeedInPerSec /
                mDrivetrain.trackWidth;

            float dt = 0.01f;
            float dThetaRad = angularVel * dt;
            float halfDThetaRad = dThetaRad * 0.5f;

            float dX, dY;
            if (std::fabs(dThetaRad) < 1e-6f) {
                dX = linearVel * std::cos(mDryRunPose.theta) * dt;
                dY = linearVel * std::sin(mDryRunPose.theta) * dt;
            } else {
                float midHeadingRad = mDryRunPose.theta + halfDThetaRad;
                float arcDist = linearVel * dt;
                dX = arcDist * std::cos(midHeadingRad);
                dY = arcDist * std::sin(midHeadingRad);
            }

            mDryRunPose.x += dX;
            mDryRunPose.y += dY;
            mDryRunPose.theta += dThetaRad;
            mDryRunPose.theta = wrapRad(mDryRunPose.theta);

            if (mDryRunPath.size() < 4096) {
                mDryRunPath.push_back(mDryRunPose);
            }
        }
        }

        if (mTelemetry && !mDryRun) {
            mTelemetry({static_cast<int>(req.type),
                        pose.x, pose.y, stdRadToVexDeg(pose.theta),
                        req.targetX, req.targetY,
                        lateralOut, angularOut, settled});
        }

        uint32_t elapsed = mDryRun ? (dryRunSteps * 10u)
                                   : (pros::millis() - req.startTime);
        if (req.timeout > 0 && elapsed > req.timeout) {
            exitNow = true;
        }

        // Hand off to whatever is queued behind this while still carrying
        // speed, rather than converging on what was only ever a waypoint.
        if (earlyExit > 0.0f && remainingErr >= 0.0f && remainingErr < earlyExit) {
            exitNow = true;
        }

        if (exitNow) break;

        if (settled) {
            if (!settleStarted) {
                settleStarted = true;
                settleStartElapsed = elapsed;
            }
            const uint32_t hold = (req.type == MotionType::RAMSETE)
                                      ? req.ramseteParams.settleTimeMs
                                      : mConfig.settleTimeMs;
            if (elapsed - settleStartElapsed >= hold) break;
        } else {
            settleStarted = false;
        }

        if (mDryRun) {
            if (++dryRunSteps > 6000u) break;
        } else {
            // After an overrun, restart the period rather than running the
            // missed ticks back to back with no time between them.
            if (pros::millis() - wakeTick >= 10) wakeTick = pros::millis();
            pros::Task::delay_until(&wakeTick, 10);
        }
    }

    // Every exit path, a cancel included: only brake modes, no voltage.
    mDrivetrain.releaseSide(swingHold);
    endMotion(seq);
}

namespace {
float orConfig(float v, float fallback) { return v >= 0.0f ? v : fallback; }
}

void Chassis::moveDistance(float target, float timeout, const MoveDistanceParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Distance;
    req.moveDistParams = params;
    req.moveDistParams.targetTolerance =
        orConfig(params.targetTolerance, mConfig.targetToleranceIn);

    if (target < 0.0f) {
        req.moveDistParams.forwards = !req.moveDistParams.forwards;
        target = -target;
    }
    req.targetDistance = target;
    req.timeout = timeout;

    enqueueMotion(std::move(req), async);
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

    enqueueMotion(std::move(req), async);
}

void Chassis::turnToHeading(float thetaDeg, float timeout, const TurntoHeadingParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Heading;
    req.targetHeadingRad = vexToStdRad(thetaDeg);
    req.turnHeadingParams = params;
    req.turnHeadingParams.targetTolerance =
        orConfig(params.targetTolerance, mConfig.targetToleranceDeg);
    req.timeout = timeout;

    enqueueMotion(std::move(req), async);
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

    enqueueMotion(std::move(req), async);
}

void Chassis::moveArc(float x, float y, float radius, float timeout, const ArcParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Arc;
    req.targetX = x;
    req.targetY = y;
    req.targetRadius = radius;
    req.arcParams = params;
    req.arcParams.targetTolerance =
        orConfig(params.targetTolerance, mConfig.targetToleranceIn);
    req.timeout = timeout;

    enqueueMotion(std::move(req), async);
}

void Chassis::moveBoomerang(float x, float y, float thetaDeg, float timeout, const BoomerangParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Boomerang;
    req.targetX = x;
    req.targetY = y;
    req.targetHeadingRad = vexToStdRad(thetaDeg);
    req.boomerangParams = params;
    req.boomerangParams.targetTolerance =
        orConfig(params.targetTolerance, mConfig.targetToleranceIn);
    req.timeout = timeout;

    enqueueMotion(std::move(req), async);
}

void Chassis::moveRAMSETE(const std::vector<Pose>& path, float timeout, const RAMSETEParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::RAMSETE;

    req.ramsetePath = (params.smoothSpacingIn > 0.0f)
                          ? splinePath(path, params.smoothSpacingIn)
                          : path;

    // After splining, not before: splinePath() derives every heading from the
    // forward tangent and would undo this.
    if (!params.forwards) {
        for (Pose& p : req.ramsetePath) p.theta = wrapRad(p.theta + PI);
    }
    req.ramseteParams = params;
    req.ramseteParams.targetTolerance =
        orConfig(params.targetTolerance, mConfig.targetToleranceIn);
    req.timeout = timeout;

    enqueueMotion(std::move(req), async);
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

    enqueueMotion(std::move(req), async);
}

void Chassis::enqueueMotion(MotionRequest&& req, bool async) {
    applyMirror(req);
    ensureMotionTask();

    // The lock is only ever held for a copy in or out: a competition task
    // deleted while holding it would leave it locked for good.
    const uint32_t cancelMark = mMotionSlot.cancelledThrough();
    mQueueMutex.lock();
    while (mMotionSlot.hasPending()) {
        mQueueMutex.unlock();
        if (mMotionSlot.cancelledThrough() != cancelMark) return;
        pros::delay(5);
        mQueueMutex.lock();
    }
    const uint32_t seq = mMotionSlot.push(std::move(req));
    mQueueMutex.unlock();
    mMotionTask->notify();

    if (!async) {
        while (!mMotionSlot.done(seq)) pros::delay(5);
    }
}

bool Chassis::isSettled() const {
    return mMotionSlot.settled();
}

void Chassis::abortMotion() {
    mQueueMutex.lock();
    mMotionSlot.cancelAll();
    mQueueMutex.unlock();

    if (mMotionTask != nullptr) {
        mMotionTask->notify();
    }
}

void Chassis::cancelMotion() {
    // A competition task deleted mid-enqueue leaves the lock held for good.
    // The cancel still lands through the atomics, and the motion task drops
    // the pending request itself.
    uint32_t through;
    if (mQueueMutex.take(20)) {
        through = mMotionSlot.cancelAll();
        mQueueMutex.give();
    } else {
        through = mMotionSlot.markCancelled();
    }
    if (mMotionTask != nullptr) mMotionTask->notify();

    // The loop sees the cancel within a tick. The bound is for a motion task
    // that cannot get there, and for the documented misuse of calling this
    // from the motion task itself.
    for (int i = 0; i < 20 && mMotionSlot.runningThrough(through); ++i) {
        pros::delay(5);
    }
    mMotionSlot.release(through);

    mDrivetrain.setVoltage(0, 0);
}

void Chassis::waitUntilSettled() {
    while (!mMotionSlot.settled()) {
        pros::delay(10);
    }
}

void Chassis::waitUntilDist(float dist) {
    if (mDryRun) return;
    Pose start = mOdom.getPose();

    while (distanceToPoint(mOdom.getPose(), start.x, start.y) < dist) {
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

    mLateralPID.setGains(config.lateralGains);
    mAngularPID.setGains(config.angularGains);
    mLateralPID.setWindupRange(config.lateralWindupRange);
    mAngularPID.setWindupRange(config.angularWindupRange);
    mLateralPID.setDerivativeFilter(config.lateralDerivativeFilter);
    mAngularPID.setDerivativeFilter(config.angularDerivativeFilter);
}

void Chassis::setSettleTolerances(float toleranceIn, float toleranceDeg,
                                  float bearingHoldDist) {
    if (toleranceIn >= 0.0f)    mConfig.targetToleranceIn = toleranceIn;
    if (toleranceDeg >= 0.0f)   mConfig.targetToleranceDeg = toleranceDeg;
    if (bearingHoldDist >= 0.0f) mConfig.bearingHoldDist = bearingHoldDist;
}

void Chassis::setLateralSlew(float unitsPerSec) {
    mConfig.lateralSlewPerSec = unitsPerSec;
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

    mLateralPID.setFixedDt(enabled ? 0.01f : 0.0f);
    mAngularPID.setFixedDt(enabled ? 0.01f : 0.0f);

    if (enabled) {
        mDryRunPath.clear();
        mDryRunStartPose = mOdom.getPose();

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
    mDryRunPath.shrink_to_fit();
}
}
