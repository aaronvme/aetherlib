/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// Adapted for AaronLib: state members are zero-initialised (upstream left them
// uninitialised, so the first update() read garbage into the kA terms).

#include "AaronLib/Movement_Control/VelocityController.hpp"

#include <algorithm>
#include <cmath>

namespace AARONLIB {

int VoltageController::sign(double x) {
    return (x > 0) - (x < 0);
}

VoltageController::VoltageController(double kv, double kaStraight, double kaTurn,
                                     double ksStraight, double ksTurn, double kp,
                                     double ki, double integralThreshold,
                                     double trackWidth)
    : kV(kv), kaStraight(kaStraight), kaTurn(kaTurn), ksStraight(ksStraight),
      ksTurn(ksTurn), kP(kp), kI(ki), trackWidth(trackWidth),
      integralThreshold(integralThreshold) {}

DrivetrainVoltages VoltageController::update(double targetLinearVelocity,
                                             double targetAngularVelocity,
                                             double measuredLeftVelocity,
                                             double measuredRightVelocity) {
    double deltaW = (targetAngularVelocity - prevAngularVelocity) / 0.01;
    double deltaV = (targetLinearVelocity - prevLinearVelocity) / 0.01;

    prevAngularVelocity = targetAngularVelocity;
    prevLinearVelocity = targetLinearVelocity;

    double leftVelocity = targetLinearVelocity - targetAngularVelocity * (trackWidth / 2.0);
    double rightVelocity = targetLinearVelocity + targetAngularVelocity * (trackWidth / 2.0);

    double leftError = leftVelocity - measuredLeftVelocity;
    double rightError = rightVelocity - measuredRightVelocity;

    if ((leftError < 0) != (prevLeftError < 0)) {
        leftIntegral = 0;
    }
    if (std::abs(leftError) < integralThreshold) {
        leftIntegral += leftError * 0.01;
    }
    if ((rightError < 0) != (prevRightError < 0)) {
        rightIntegral = 0;
    }
    if (std::abs(rightError) < integralThreshold) {
        rightIntegral += rightError * 0.01;
    }

    double kaLeft = (kaStraight * deltaV) - (kaTurn * deltaW);
    double kaRight = (kaStraight * deltaV) + (kaTurn * deltaW);

    double ksLeft = (ksStraight * sign(leftVelocity)) - (ksTurn * sign(targetAngularVelocity));
    double ksRight = (ksStraight * sign(rightVelocity)) + (ksTurn * sign(targetAngularVelocity));

    double leftVoltage =
        (kV * leftVelocity) + (kaLeft) + (ksLeft) + (kP * leftError) + (kI * leftIntegral);
    double rightVoltage =
        (kV * rightVelocity) + (kaRight) + (ksRight) + (kP * rightError) + (kI * rightIntegral);

    // NOTE: kept from upstream verbatim. The outputs above are in VOLTS but this
    // divides by 12000 (millivolts), so the desaturation never triggers and the
    // caller's own +/-12 V clamp does the limiting instead. Changing the constant
    // to 12.0 would start proportionally desaturating and alter tuned behaviour,
    // so it is deliberately left alone.
    double ratio = std::max(std::fabs(leftVoltage), std::fabs(rightVoltage)) / 12000.0;
    if (ratio > 1) {
        leftVoltage /= ratio;
        rightVoltage /= ratio;
    }

    prevLeftError = leftError;
    prevRightError = rightError;
    return {static_cast<float>(leftVoltage), static_cast<float>(rightVoltage)};
}

}  // namespace AARONLIB
