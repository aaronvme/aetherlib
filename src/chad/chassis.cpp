#include "chad/chassis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "chad/movement_control/spline.hpp"
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

    mMcl.setSensorMounts(config.sensorMounts);
    mEkf.setSensorMounts(config.sensorMounts);
    mEkf.attachMcl(&mMcl);

    mRcl.configureSensors(config.distanceSensors, config.sensorMounts);

    ensureMotionTask();

    mOdom.startBackgroundOdom();
}

Chassis::~Chassis() {
    mMotionCancelled = true;
    mTaskShouldStop = true;
    if (mMotionTask != nullptr) {
        mMotionTask->notify();
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

void Chassis::setPose(const Pose& pose) {
    if (mDryRun) {
        mDryRunStartPose = pose;
        mDryRunPose = pose;
        mDryRunPath.clear();
        mDryRunPath.push_back(pose);
        return;
    }
    mOdom.setPose(pose);

    mMcl.setPose(pose);
    mRcl.setRclPose(pose);
    mEkf.setPose(pose);
}

void Chassis::setPosition(float x, float y) {
    if (mDryRun) {
        mDryRunPose.x = x;
        mDryRunPose.y = y;
        mDryRunPath.push_back(mDryRunPose);
        return;
    }
    Pose current = mOdom.getPose();
    Pose newPose{x, y, current.theta};
    mOdom.setPose(newPose);

    mMcl.setPose(newPose);
    mRcl.setRclPose(newPose);
    mEkf.setPose(newPose);
}

void Chassis::setHeading(float thetaDeg) {
    if (mDryRun) {
        mDryRunPose.theta = vexToStdRad(thetaDeg);
        mDryRunPath.push_back(mDryRunPose);
        return;
    }
    Pose current = mOdom.getPose();
    Pose newPose{current.x, current.y, vexToStdRad(thetaDeg)};
    mOdom.setPose(newPose);

    mMcl.setPose(newPose);
    mRcl.setRclPose(newPose);
    mEkf.setPose(newPose);
}

Pose Chassis::getPose() const {
    return mOdom.getPose();
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

void Chassis::endMotion() {
    mDrivetrain.setVoltage(0, 0);
    mMotionRunning = false;

    mMotionCancelled = false;
    mLateralPID.reset();
    mAngularPID.reset();
}

void Chassis::runMotionTask() {
    while (!mTaskShouldStop) {
        pros::Task::notify_take(true, TIMEOUT_MAX);
        if (mTaskShouldStop) break;

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

            if (!hasWork) break;

            if (!mMotionCancelled && !mTaskShouldStop) {
                mCurrentMotion = req;
                executeCurrentMotion();
            }
            mCompletedSeq.store(req.seq);
        }
    }
}

void Chassis::executeCurrentMotion() {
    auto& req = mCurrentMotion;

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

    mMotionCancelled = false;
    startMotion();

    uint32_t dryRunSteps = 0;

    bool settleStarted = false;
    uint32_t settleStartElapsed = 0;

    bool bearingHeld = false;
    float heldBearing = 0.0f;

    float prevLateralOut = 0.0f;

    uint32_t prevTick = pros::millis();

    while (mMotionRunning && !mMotionCancelled) {
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

        switch (req.type) {
        case MotionType::Distance: {
            latLimit = req.moveDistParams.maxSpeed;

            float targetX = distTargetX;
            float targetY = distTargetY;

            const float along = (targetX - pose.x) * std::cos(distAxisRad) +
                                (targetY - pose.y) * std::sin(distAxisRad);
            lateralOut = mLateralPID.update(along);

            angularOut = mAngularPID.update(angleDiffRad(pose.theta, distAxisRad));
            settled = std::fabs(along) < req.moveDistParams.targetTolerance;
            break;
        }
        case MotionType::Point: {
            latLimit = req.movePointParams.maxSpeed;
            const float distErr = distanceToPoint(pose, req.targetX, req.targetY);
            const float liveBearing = bearingToPoint(pose, req.targetX, req.targetY);

            const float along =
                distErr * std::cos(angleDiffRad(pose.theta, liveBearing));
            lateralOut = mLateralPID.update(along);

            float steerBearing = liveBearing;
            if (bearingHeld) {
                steerBearing = heldBearing;
            } else if (distErr < req.movePointParams.bearingHoldDist) {
                heldBearing = liveBearing;
                bearingHeld = true;
                steerBearing = heldBearing;
            }

            if (!req.movePointParams.forwards) {
                steerBearing = wrapRad(steerBearing + PI);
            }
            angularOut = mAngularPID.update(angleDiffRad(pose.theta, steerBearing));
            settled = distErr < req.movePointParams.targetTolerance;
            break;
        }
        case MotionType::Heading: {
            angLimit = req.turnHeadingParams.maxSpeed;
            float rawErrRad = angleDiffRad(pose.theta, req.targetHeadingRad);

            float headingErrRad = resolveTurnError(rawErrRad, req.turnHeadingParams.direction);

            float tolRad = degToRad(req.turnHeadingParams.targetTolerance);
            if (std::fabs(rawErrRad) < tolRad) headingErrRad = rawErrRad;
            angularOut = mAngularPID.update(headingErrRad);
            lateralOut = 0;
            settled = std::fabs(rawErrRad) < tolRad;
            break;
        }
        case MotionType::TurnPoint: {
            angLimit = req.turnPointParams.maxSpeed;
            float rawErrRad = headingToPointOffset(pose, req.targetX, req.targetY,
                                                   req.turnPointParams.offset);
            float headingErrRad = resolveTurnError(rawErrRad, req.turnPointParams.direction);
            float tolRad = degToRad(req.turnPointParams.targetTolerance);
            if (std::fabs(rawErrRad) < tolRad) headingErrRad = rawErrRad;
            angularOut = mAngularPID.update(headingErrRad);
            lateralOut = 0;
            settled = std::fabs(rawErrRad) < tolRad;
            break;
        }
        case MotionType::Arc: {
            latLimit = req.arcParams.maxSpeed;

            float dx = req.targetX - pose.x;
            float dy = req.targetY - pose.y;
            float chordDist = std::sqrt(dx * dx + dy * dy);

            float chordBearingRad = std::atan2(dy, dx);

            float radius = std::fabs(req.targetRadius);

            const float chordSign =
                (std::cos(angleDiffRad(pose.theta, chordBearingRad)) >= 0.0f)
                    ? 1.0f : -1.0f;

            if (radius < 1e-6f) {
                lateralOut = mLateralPID.update(chordSign * chordDist);
                float headingErrRad = angleDiffRad(pose.theta, chordBearingRad);
                angularOut = mAngularPID.update(headingErrRad);
            } else {
                float targetHeadingRad =
                    arcStartHeading(pose, req.targetX, req.targetY, req.targetRadius);
                float headingErrRad = angleDiffRad(pose.theta, targetHeadingRad);

                lateralOut = mLateralPID.update(chordSign * chordDist);
                angularOut = mAngularPID.update(headingErrRad);
            }
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

            float carrotX = req.targetX - clampedLead * std::cos(req.targetHeadingRad);
            float carrotY = req.targetY - clampedLead * std::sin(req.targetHeadingRad);

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
            } else if (distToCarrot < mConfig.bearingHoldDist) {
                heldBearing = carrotBearingRad;
                bearingHeld = true;
            }
            float headingToCarrotRad = angleDiffRad(pose.theta, carrotBearingRad);
            float headingToFinalRad  = angleDiffRad(pose.theta, req.targetHeadingRad);

            float headingWeight = std::min(distToTarget / 12.0f, 1.0f);
            float blendedHeadingErrRad = headingToCarrotRad * headingWeight +
                headingToFinalRad * (1.0f - headingWeight);

            angularOut = mAngularPID.update(blendedHeadingErrRad);

            float speedScale = std::min(distToTarget / 6.0f, 1.0f);
            if (speedScale < req.boomerangParams.minSpeed / req.boomerangParams.maxSpeed) {
                speedScale = req.boomerangParams.minSpeed / req.boomerangParams.maxSpeed;
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

            float w_d = 0.0f;
            if (targetIdx > 0 && targetIdx < static_cast<int>(path.size()) - 1) {
                const Pose& prev = path[targetIdx - 1];
                const Pose& next = path[targetIdx + 1];
                float segDist = distanceToPoint(prev, next.x, next.y);
                if (segDist > 1e-3f) {
                    w_d = v_d * angleDiffRad(prev.theta, next.theta) / segDist;
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

            float headingErrRad = angleDiffRad(pose.theta, req.targetHeadingRad);

            TurnDirection dir = req.swingParams.direction;
            if (dir == TurnDirection::CW) {
                headingErrRad = -std::fabs(headingErrRad);
            } else if (dir == TurnDirection::CCW) {
                headingErrRad = std::fabs(headingErrRad);
            }

            angularOut = mAngularPID.update(headingErrRad);
            lateralOut = 0;

            if (req.swingSide == SwingSide::Left) {
                customLeft = 0;
                customRight = clamp(angularOut, -127.0f, 127.0f);
            } else {
                customLeft = clamp(-angularOut, -127.0f, 127.0f);
                customRight = 0;
            }
            useCustomOutput = true;

            settled = std::fabs(headingErrRad) < degToRad(req.swingParams.targetTolerance);
            break;
        }
        default:
            break;
        }

        lateralOut = clamp(lateralOut, -latLimit, latLimit);
        angularOut = clamp(angularOut, -angLimit, angLimit);

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
            mDrivetrain.setVoltage(left, right);
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
            pros::delay(10);
        }
    }

    endMotion();
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
    req.arcParams.targetTolerance =
        orConfig(params.targetTolerance, mConfig.targetToleranceIn);
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
    req.boomerangParams.targetTolerance =
        orConfig(params.targetTolerance, mConfig.targetToleranceIn);
    req.timeout = timeout;

    enqueueMotion(req, async);
}

void Chassis::moveRAMSETE(const std::vector<Pose>& path, float timeout, const RAMSETEParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::RAMSETE;

    req.ramsetePath = (params.smoothSpacingIn > 0.0f)
                          ? splinePath(path, params.smoothSpacingIn)
                          : path;
    req.ramseteParams = params;
    req.ramseteParams.targetTolerance =
        orConfig(params.targetTolerance, mConfig.targetToleranceIn);
    req.timeout = timeout;

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

    mQueueMutex.lock();
    while (!mMotionQueue.empty()) {
        mQueueMutex.unlock();
        if (mMotionCancelled) return;
        pros::delay(5);
        mQueueMutex.lock();
    }
    req.seq = ++mMotionSeqCounter;
    mMotionQueue.push(req);
    mQueueMutex.unlock();
    mMotionTask->notify();

    if (!async) {
        uint32_t mySeq = req.seq;
        while (static_cast<int32_t>(mCompletedSeq.load() - mySeq) < 0) {
            if (mMotionCancelled) break;
            pros::delay(5);
        }
    }
}

bool Chassis::isSettled() const {
    return static_cast<int32_t>(mCompletedSeq.load() - mMotionSeqCounter.load()) >= 0;
}

void Chassis::abortMotion() {
    mMotionCancelled = true;

    mQueueMutex.lock();
    while (!mMotionQueue.empty()) mMotionQueue.pop();
    mQueueMutex.unlock();

    if (mMotionTask != nullptr) {
        mMotionTask->notify();
    }
}

void Chassis::cancelMotion() {
    abortMotion();

    while (mMotionRunning) {
        pros::delay(5);
    }

    mCompletedSeq.store(mMotionSeqCounter.load());

    mMotionCancelled = false;

    mDrivetrain.setVoltage(0, 0);
}

void Chassis::waitUntilSettled() {
    while (static_cast<int32_t>(mCompletedSeq.load() - mMotionSeqCounter.load()) < 0) {
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
