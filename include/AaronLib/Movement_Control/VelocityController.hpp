/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// Adapted for AaronLib: state members are zero-initialised (upstream left them
// uninitialised, so the first update() read garbage into the kA terms).

#pragma once

namespace AARONLIB {

struct DrivetrainVoltages {
    double leftVoltage;
    double rightVoltage;
};

// Feedforward (kV/kA/kS) plus per-side PI on velocity error.
class VoltageController {
public:
    VoltageController(double kv, double kaStraight, double kaTurn, double ksStraight,
                      double ksTurn, double kp, double ki, double integralThreshold,
                      double trackWidth);

    DrivetrainVoltages update(double targetLinearVelocity, double targetAngularVelocity,
                              double measuredLeftVelocity, double measuredRightVelocity);

private:
    static int sign(double x);

    double kV;
    double kaStraight;
    double kaTurn;
    double ksStraight;
    double ksTurn;
    double kP;
    double kI;

    double trackWidth;
    double integralThreshold;

    double prevLinearVelocity{0.0};
    double prevAngularVelocity{0.0};
    double prevLeftError{0.0};
    double prevRightError{0.0};
    double leftIntegral{0.0};
    double rightIntegral{0.0};
};

struct VelocityControllerConfig {
    float kV;
    float KA_turn;
    float KA_straight;
    float KS_turn;
    float KS_straight;

    float KP_straight;
    float KI_straight;
};

}  // namespace AARONLIB
