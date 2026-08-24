// Ported from FBLib - https://github.com/jiazegao/FBLib
// Split out of FBLib's Chassis.cpp (adapted for chad).

#include "chad/drivetrain.hpp"

#include <cmath>
#include <cstdint>

namespace chad {

Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
                       float trackWidth, float wheelDiameter, float rpm,
                       pros::MotorGroup* leftSenseMotors,
                       pros::MotorGroup* rightSenseMotors,
                       WheelType wheels, float gripLimitOverride)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      leftSenseMotors(leftSenseMotors),
      rightSenseMotors(rightSenseMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      rpm(rpm),
      wheels(wheels),
      gripLimitOverride(gripLimitOverride) {
    // Odometry's motor-encoder fallback assumes positions are in DEGREES.
    // Enforce it — a user who set rotations/counts elsewhere would silently
    // corrupt distance measurements by a factor of 360.
    for (pros::MotorGroup* g : {leftMotors, rightMotors,
                                leftSenseMotors, rightSenseMotors}) {
        if (g != nullptr) g->set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    }
}

float Drivetrain::maxWheelSpeedInPerSec() const {
    return (rpm / 60.0f) * PI * wheelDiameter;
}

DriveModel Drivetrain::model() const {
    return {trackWidth, wheelDiameter, maxWheelSpeedInPerSec(), wheels, gripLimitOverride};
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

// A disconnected motor reports PROS_ERR (INT32_MAX, 2.1e9 "degrees"). One bad
// reading would catapult the average -- and odometry with it -- thousands of
// miles, so skip it and average the healthy motors.
static bool readingUsable(double v) {
    return std::isfinite(v) && std::fabs(v) < 2147483647.0;
}

void Drivetrain::accumulateVelocity(const pros::MotorGroup* g, double& sum, int& count,
                                    int& faulted) {
    if (g == nullptr) return;
    const int n = g->size();
    for (int i = 0; i < n; ++i) {
        const double rpm = g->get_actual_velocity(static_cast<std::uint8_t>(i));
        if (!readingUsable(rpm)) { faulted++; continue; }
        sum += rpm;
        count++;
    }
}

void Drivetrain::accumulatePosition(const pros::MotorGroup* g, double& sum, int& count,
                                    int& faulted) {
    if (g == nullptr) return;
    const int n = g->size();
    for (int i = 0; i < n; ++i) {
        const double pos = g->get_position(static_cast<std::uint8_t>(i));
        if (!readingUsable(pos)) { faulted++; continue; }
        sum += pos;
        count++;
    }
}

float Drivetrain::velocityFromRpm(double motorRpm) const {
    return static_cast<float>(motorRpm * externalGearRatio() / 60.0 *
                              3.14159265358979 * wheelDiameter);
}

float Drivetrain::groupVelocityInPerSec(const pros::MotorGroup* a,
                                       const pros::MotorGroup* b) const {
    double sum = 0.0;
    int count = 0, faulted = 0;
    accumulateVelocity(a, sum, count, faulted);
    accumulateVelocity(b, sum, count, faulted);
    return (count > 0) ? velocityFromRpm(sum / count) : 0.0f;
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

WheelSpeeds Drivetrain::wheelVelocitiesInPerSec() const {
    return wheelVelocitiesLive().speeds;
}

WheelReading Drivetrain::wheelVelocitiesLive() const {
    WheelReading out;
    double sum = 0.0;
    int count = 0;
    accumulateVelocity(leftSense(), sum, count, out.faulted);
    out.speeds.left = (count > 0) ? velocityFromRpm(sum / count) : 0.0f;
    out.live += count;
    sum = 0.0;
    count = 0;
    accumulateVelocity(rightSense(), sum, count, out.faulted);
    out.speeds.right = (count > 0) ? velocityFromRpm(sum / count) : 0.0f;
    out.live += count;
    return out;
}

float Drivetrain::averagePositionDeg() const {
    return averagePositionDegLive().value;
}

EncoderAverage Drivetrain::averagePositionDegLive() const {
    EncoderAverage out;
    double sum = 0.0;
    int count = 0;
    accumulatePosition(leftSense(), sum, count, out.faulted);
    accumulatePosition(rightSense(), sum, count, out.faulted);
    out.live = count;
    out.value = (count > 0) ? static_cast<float>(sum / count) : 0.0f;
    return out;
}

EncoderAverage Drivetrain::groupPositionDegLive(const pros::MotorGroup* g) const {
    EncoderAverage out;
    double sum = 0.0;
    int count = 0;
    accumulatePosition(g, sum, count, out.faulted);
    out.live = count;
    out.value = (count > 0) ? static_cast<float>(sum / count) : 0.0f;
    return out;
}

EncoderAverage Drivetrain::groupPositionInLive(const pros::MotorGroup* g) const {
    EncoderAverage out = groupPositionDegLive(g);
    out.value = out.value / 360.0f * externalGearRatio() *
                3.14159265358979f * wheelDiameter;
    return out;
}

EncoderAverage Drivetrain::leftPositionInLive() const {
    return groupPositionInLive(leftSense());
}

EncoderAverage Drivetrain::rightPositionInLive() const {
    return groupPositionInLive(rightSense());
}

float Drivetrain::leftPositionIn() const {
    return leftPositionInLive().value;
}

float Drivetrain::rightPositionIn() const {
    return rightPositionInLive().value;
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
}  // namespace chad
