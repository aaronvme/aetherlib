#include "chad/drivetrain.hpp"

#include <cmath>
#include <cstdint>
#include <initializer_list>

namespace chad {
Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
                       float trackWidth, float wheelDiameter, float rpm,
                       pros::MotorGroup* leftSenseMotors,
                       pros::MotorGroup* rightSenseMotors)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      leftSenseMotors(leftSenseMotors),
      rightSenseMotors(rightSenseMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      rpm(rpm) {
    for (pros::MotorGroup* g : {leftMotors, rightMotors,
                                leftSenseMotors, rightSenseMotors}) {
        if (g != nullptr) g->set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    }
}

void Drivetrain::setLeftVoltage(float voltage) {
    if (leftMotors != nullptr) {
        leftMotors->move_voltage(static_cast<int32_t>(voltage * 94.488f));
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

namespace {
bool readable(double v) { return std::isfinite(v) && std::fabs(v) < 2147483647.0; }

void accumulatePositions(const pros::MotorGroup* g, double& sum, int& count) {
    if (g == nullptr) return;
    const int n = g->size();
    for (int i = 0; i < n; i++) {
        const double pos = g->get_position(static_cast<std::uint8_t>(i));
        if (!readable(pos)) continue;
        sum += pos;
        count++;
    }
}
}  // namespace

int Drivetrain::readPositionsDeg(const pros::MotorGroup* g, double* out, int max) {
    if (g == nullptr) return 0;
    int n = g->size();
    if (n > max) n = max;
    for (int i = 0; i < n; i++) out[i] = g->get_position(static_cast<std::uint8_t>(i));
    return (n > 0) ? n : 0;
}

float Drivetrain::groupVelocityInPerSec(const pros::MotorGroup* a,
                                       const pros::MotorGroup* b) const {
    double sum = 0.0;
    int count = 0;
    for (const pros::MotorGroup* g : {a, b}) {
        if (g == nullptr) continue;
        const int n = g->size();
        for (int i = 0; i < n; i++) {
            const double rpm = g->get_actual_velocity(static_cast<std::uint8_t>(i));
            if (!readable(rpm)) continue;
            sum += rpm;
            count++;
        }
    }
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
    double sum = 0.0;
    int count = 0;
    accumulatePositions(leftSense(), sum, count);
    accumulatePositions(rightSense(), sum, count);
    return (count > 0) ? static_cast<float>(sum / count) : 0.0f;
}

float Drivetrain::groupPositionDeg(const pros::MotorGroup* g) const {
    double sum = 0.0;
    int count = 0;
    accumulatePositions(g, sum, count);
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
    if (mExtGearRatio > 0.0f) return mExtGearRatio;

    pros::MotorGroup* group = (leftSense() != nullptr) ? leftSense() : rightSense();
    if (group == nullptr || rpm <= 0.0f) return 1.0f;

    float cartridgeRpm;
    switch (group->get_gearing()) {
        case pros::MotorGears::red:   cartridgeRpm = 100.0f; break;
        case pros::MotorGears::green: cartridgeRpm = 200.0f; break;
        case pros::MotorGears::blue:  cartridgeRpm = 600.0f; break;
        default:                      return 1.0f;
    }

    mExtGearRatio = rpm / cartridgeRpm;
    return mExtGearRatio;
}

float DriveCurve::apply(float input) const {
    if (std::fabs(input) < deadband) return 0.0f;

    float sign = (input > 0.0f) ? 1.0f : -1.0f;
    float absInput = std::fabs(input);

    float normalized = (absInput - deadband) / (127.0f - deadband);

    float curved = std::pow(normalized, curve);

    return sign * (minOutput + curved * (127.0f - minOutput));
}
}
