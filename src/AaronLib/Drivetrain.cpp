// Ported from FBLib - https://github.com/jiazegao/FBLib
// Split out of FBLib's Chassis.cpp (adapted for AaronLib).

#include "AaronLib/Drivetrain.hpp"

#include <cmath>
#include <vector>

namespace AARONLIB {

Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
                       float trackWidth, float wheelDiameter, float rpm,
                       float horizontalDrift)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      rpm(rpm),
      horizontalDrift(horizontalDrift) {
    // Odometry's motor-encoder fallback assumes positions are in DEGREES.
    // Enforce it — a user who set rotations/counts elsewhere would silently
    // corrupt distance measurements by a factor of 360.
    if (leftMotors != nullptr) {
        leftMotors->set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    }
    if (rightMotors != nullptr) {
        rightMotors->set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    }
}

void Drivetrain::setLeftVoltage(float voltage) {
    if (leftMotors != nullptr) {
        leftMotors->move_voltage(static_cast<int32_t>(voltage * 94.488f));  // mV * 127 → 12000mV
    }
}

void Drivetrain::setRightVoltage(float voltage) {
    if (rightMotors != nullptr) {
        rightMotors->move_voltage(static_cast<int32_t>(voltage * 94.488f));
    }
}

void Drivetrain::setVoltage(float left, float right) {
    setLeftVoltage(left);
    setRightVoltage(right);
}

void Drivetrain::setBrakeMode(pros::motor_brake_mode_e mode) {
    if (leftMotors != nullptr) leftMotors->set_brake_mode(mode);
    if (rightMotors != nullptr) rightMotors->set_brake_mode(mode);
}

float Drivetrain::averagePositionDeg() const {
    if (leftMotors == nullptr && rightMotors == nullptr) return 0.0f;

    // A disconnected motor reports PROS_ERR (INT32_MAX ≈ 2.1e9 "degrees").
    // One bad reading would catapult the average — and odometry with it —
    // thousands of miles. Skip error readings; average the healthy motors.
    auto accumulate = [](const std::vector<double>& positions,
                         double& sum, int& count) {
        for (double pos : positions) {
            if (!std::isfinite(pos) || std::fabs(pos) >= 2147483647.0) continue;
            sum += pos;
            count++;
        }
    };

    double sum = 0.0;
    int count = 0;
    if (leftMotors != nullptr)  accumulate(leftMotors->get_position_all(),  sum, count);
    if (rightMotors != nullptr) accumulate(rightMotors->get_position_all(), sum, count);
    return (count > 0) ? static_cast<float>(sum / count) : 0.0f;
}

float Drivetrain::externalGearRatio() const {
    if (mExtGearRatio > 0.0f) return mExtGearRatio;   // cached

    // Resolve cartridge RPM from the configured gearset. get_gearing() can
    // return invalid before the motor responds (e.g. during boot) — in that
    // case return 1.0 WITHOUT caching so a later call can retry.
    pros::MotorGroup* group = (leftMotors != nullptr) ? leftMotors : rightMotors;
    if (group == nullptr || rpm <= 0.0f) return 1.0f;

    float cartridgeRpm;
    switch (group->get_gearing()) {
        case pros::MotorGears::red:   cartridgeRpm = 100.0f; break;
        case pros::MotorGears::green: cartridgeRpm = 200.0f; break;
        case pros::MotorGears::blue:  cartridgeRpm = 600.0f; break;
        default:                      return 1.0f;   // unknown — retry later
    }

    mExtGearRatio = rpm / cartridgeRpm;
    return mExtGearRatio;
}

// ============================================================================
// DriveCurve
// ============================================================================

float DriveCurve::apply(float input) const {
    if (std::fabs(input) < deadband) return 0.0f;

    float sign = (input > 0.0f) ? 1.0f : -1.0f;
    float absInput = std::fabs(input);

    // Remap [deadband, 127] → [0, 1]
    float normalized = (absInput - deadband) / (127.0f - deadband);

    // Apply curve: x^curve
    float curved = std::pow(normalized, curve);

    // Remap back: [minOutput, 127]
    return sign * (minOutput + curved * (127.0f - minOutput));
}
}  // namespace AARONLIB
