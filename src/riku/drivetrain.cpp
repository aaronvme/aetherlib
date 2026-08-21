// Ported from FBLib - https://github.com/jiazegao/FBLib
// Split out of FBLib's Chassis.cpp (adapted for riku).

#include "riku/drivetrain.hpp"

#include <cmath>
#include <vector>

namespace riku {

Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
                       float trackWidth, float wheelDiameter, float rpm,
                       float horizontalDrift,
                       pros::MotorGroup* leftSenseMotors,
                       pros::MotorGroup* rightSenseMotors)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      leftSenseMotors(leftSenseMotors),
      rightSenseMotors(rightSenseMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      rpm(rpm),
      horizontalDrift(horizontalDrift) {
    // Odometry's motor-encoder fallback assumes positions are in DEGREES.
    // Enforce it — a user who set rotations/counts elsewhere would silently
    // corrupt distance measurements by a factor of 360.
    for (pros::MotorGroup* g : {leftMotors, rightMotors,
                                leftSenseMotors, rightSenseMotors}) {
        if (g != nullptr) g->set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
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

float Drivetrain::groupVelocityInPerSec(const pros::MotorGroup* a,
                                       const pros::MotorGroup* b) const {
    if (a == nullptr && b == nullptr) return 0.0f;

    auto accumulate = [](const std::vector<double>& rpms, double& sum, int& count) {
        for (double rpm : rpms) {
            if (!std::isfinite(rpm) || std::fabs(rpm) >= 2147483647.0) continue;
            sum += rpm;
            count++;
        }
    };

    double sum = 0.0;
    int count = 0;
    if (a != nullptr) accumulate(a->get_actual_velocity_all(), sum, count);
    if (b != nullptr) accumulate(b->get_actual_velocity_all(), sum, count);
    if (count == 0) return 0.0f;

    const double motorRpm = sum / count;
    return static_cast<float>(motorRpm * externalGearRatio() / 60.0 * 3.14159265358979 * wheelDiameter);
}

float Drivetrain::averageVelocityInPerSec() const {
    return groupVelocityInPerSec(leftSense(), rightSense());
}

float Drivetrain::leftVelocityInPerSec() const {
    return groupVelocityInPerSec(leftSense(), nullptr);
}

float Drivetrain::rightVelocityInPerSec() const {
    return groupVelocityInPerSec(rightSense(), nullptr);
}

float Drivetrain::averagePositionDeg() const {
    if (leftSense() == nullptr && rightSense() == nullptr) return 0.0f;

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
    if (leftSense() != nullptr)  accumulate(leftSense()->get_position_all(),  sum, count);
    if (rightSense() != nullptr) accumulate(rightSense()->get_position_all(), sum, count);
    return (count > 0) ? static_cast<float>(sum / count) : 0.0f;
}

float Drivetrain::groupPositionDeg(const pros::MotorGroup* g) const {
    if (g == nullptr) return 0.0f;
    double sum = 0.0;
    int count = 0;
    for (double pos : g->get_position_all()) {
        if (!std::isfinite(pos) || std::fabs(pos) >= 2147483647.0) continue;
        sum += pos;
        count++;
    }
    return (count > 0) ? static_cast<float>(sum / count) : 0.0f;
}

float Drivetrain::leftPositionIn() const {
    return groupPositionDeg(leftSense()) / 360.0f * externalGearRatio() *
           3.14159265358979f * wheelDiameter;
}

float Drivetrain::rightPositionIn() const {
    return groupPositionDeg(rightSense()) / 360.0f * externalGearRatio() *
           3.14159265358979f * wheelDiameter;
}

float Drivetrain::externalGearRatio() const {
    if (mExtGearRatio > 0.0f) return mExtGearRatio;   // cached

    // Resolve cartridge RPM from the configured gearset. get_gearing() can
    // return invalid before the motor responds (e.g. during boot) — in that
    // case return 1.0 WITHOUT caching so a later call can retry.
    pros::MotorGroup* group = (leftSense() != nullptr) ? leftSense() : rightSense();
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
}  // namespace riku
