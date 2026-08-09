// Ported from FBLib - https://github.com/jiazegao/FBLib
#include "riku/movement_control/feedforward.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "riku/util/util.hpp"

namespace riku {

FeedforwardController::FeedforwardController(const FeedforwardGains& gains, float trackWidth)
    : mGains(gains), mTrackWidth(trackWidth) {}

void FeedforwardController::reset() {
    mPrevLinearVel = 0.0f;
    mPrevAngularVel = 0.0f;
    mPrevLeftError = 0.0f;
    mPrevRightError = 0.0f;
    mLeftIntegral = 0.0f;
    mRightIntegral = 0.0f;
    mFirstUpdate = true;
}

void FeedforwardController::setGains(const FeedforwardGains& gains) {
    mGains = gains;
}

DrivetrainVoltages FeedforwardController::update(float targetLinearVel, float targetAngularVel,
                                                 float measuredLeftVel, float measuredRightVel,
                                                 float dt) {
    // A non-positive or non-finite dt would divide by zero in the acceleration
    // term. Fall back to the nominal 10 ms tick rather than emitting infinities
    // into the motors.
    if (!std::isfinite(dt) || dt <= 0.0f) dt = 0.01f;

    // ------------------------------------------------------------------
    // Feedforward acceleration — differentiate the commanded twist
    // ------------------------------------------------------------------
    float linearAccel = 0.0f;
    float angularAccel = 0.0f;
    if (!mFirstUpdate) {
        linearAccel = (targetLinearVel - mPrevLinearVel) / dt;
        angularAccel = (targetAngularVel - mPrevAngularVel) / dt;
    }
    mPrevLinearVel = targetLinearVel;
    mPrevAngularVel = targetAngularVel;
    mFirstUpdate = false;

    // ------------------------------------------------------------------
    // Twist -> per-side wheel velocities
    // ------------------------------------------------------------------
    const float halfTrack = mTrackWidth / 2.0f;
    const float leftTargetVel = targetLinearVel - targetAngularVel * halfTrack;
    const float rightTargetVel = targetLinearVel + targetAngularVel * halfTrack;

    // ------------------------------------------------------------------
    // Velocity error + integral accumulation
    // ------------------------------------------------------------------
    const float leftError = leftTargetVel - measuredLeftVel;
    const float rightError = rightTargetVel - measuredRightVel;

    // Flip-reset: when the error changes sign the drivetrain has crossed its
    // setpoint, so whatever the integrator accumulated on the way in is now
    // pushing the wrong direction. Dump it. Same idea as PID's flipReset.
    if ((leftError < 0.0f) != (mPrevLeftError < 0.0f)) mLeftIntegral = 0.0f;
    if ((rightError < 0.0f) != (mPrevRightError < 0.0f)) mRightIntegral = 0.0f;

    const bool gateOpen = (mGains.integralErrorGate <= 0.0f);
    if (gateOpen || std::fabs(leftError) < mGains.integralErrorGate) {
        mLeftIntegral += leftError * dt;
    }
    if (gateOpen || std::fabs(rightError) < mGains.integralErrorGate) {
        mRightIntegral += rightError * dt;
    }

    if (mGains.integralClamp > 0.0f) {
        mLeftIntegral = clamp(mLeftIntegral, -mGains.integralClamp, mGains.integralClamp);
        mRightIntegral = clamp(mRightIntegral, -mGains.integralClamp, mGains.integralClamp);
    }

    // ------------------------------------------------------------------
    // Model terms
    // ------------------------------------------------------------------
    // Linear acceleration pushes both sides the same way; angular
    // acceleration pushes them opposite ways. Same for static friction.
    const float kaLeft = (mGains.kaStraight * linearAccel) - (mGains.kaTurn * angularAccel);
    const float kaRight = (mGains.kaStraight * linearAccel) + (mGains.kaTurn * angularAccel);

    const float ksLeft = (mGains.ksStraight * sign(leftTargetVel)) -
                         (mGains.ksTurn * sign(targetAngularVel));
    const float ksRight = (mGains.ksStraight * sign(rightTargetVel)) +
                          (mGains.ksTurn * sign(targetAngularVel));

    float leftVoltage = (mGains.kV * leftTargetVel) + kaLeft + ksLeft +
                        (mGains.kP * leftError) + (mGains.kI * mLeftIntegral);
    float rightVoltage = (mGains.kV * rightTargetVel) + kaRight + ksRight +
                         (mGains.kP * rightError) + (mGains.kI * mRightIntegral);

    // ------------------------------------------------------------------
    // Desaturate
    // ------------------------------------------------------------------
    // Scaling both sides by a common factor preserves the ratio between them,
    // so a saturated command still traces the requested curvature — just
    // slower. Clipping each side independently would not.
    const float peak = std::fmax(std::fabs(leftVoltage), std::fabs(rightVoltage));
    if (peak > mMaxVoltage && peak > 0.0f) {
        const float scale = mMaxVoltage / peak;
        leftVoltage *= scale;
        rightVoltage *= scale;
    }

    mPrevLeftError = leftError;
    mPrevRightError = rightError;

    if (mLog) {
        mLog->row({targetLinearVel, targetAngularVel, leftTargetVel, rightTargetVel,
                   measuredLeftVel, measuredRightVel, linearAccel, angularAccel,
                   leftVoltage, rightVoltage, dt});
    }

    return {leftVoltage, rightVoltage};
}

void FeedforwardController::setLogging(bool enabled) {
    if (!enabled) {
        mLog.reset();
        return;
    }
    if (mLog) return;
    mLog = std::make_unique<Logger>(
        "ff", std::vector<std::string>{"vTarget", "wTarget", "vLeftTarget", "vRightTarget",
                                       "vLeftMeas", "vRightMeas", "aLinear", "aAngular",
                                       "voltLeft", "voltRight", "dt"});
    mLog->start();
}

}  // namespace riku
