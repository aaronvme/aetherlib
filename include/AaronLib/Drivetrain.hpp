// Ported from FBLib - https://github.com/jiazegao/FBLib
// Split out of FBLib's Chassis.hpp (adapted for AaronLib) so odometry can
// depend on Drivetrain without pulling in Chassis.
#pragma once

#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

namespace AARONLIB {

// Hardware abstraction for the drive base.
class Drivetrain {
public:
    Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
               float trackWidth, float wheelDiameter, float rpm = 450.0f,
               float horizontalDrift = 2.0f);

    pros::MotorGroup* leftMotors{nullptr};
    pros::MotorGroup* rightMotors{nullptr};
    float trackWidth;         // inches, center-to-center
    float wheelDiameter;      // inches
    float rpm;                // drive RPM (not cartridge RPM)
    float horizontalDrift;

    // Voltage, -127 to 127
    void setLeftVoltage(float voltage);
    void setRightVoltage(float voltage);
    void setVoltage(float left, float right);

    void setBrakeMode(pros::motor_brake_mode_e mode);

    /// Average motor position (degrees). Ignores PROS_ERR from dead motors.
    float averagePositionDeg() const;

    /// Wheel revs per motor rev (wheel RPM / cartridge RPM). Cached after the
    /// first valid read; returns 1.0 while the gearset is still unknown.
    float externalGearRatio() const;

private:
    mutable float mExtGearRatio{0.0f};   // 0 = unresolved
};

// Controller output for both drive sides. Shared by every controller that
// produces drivetrain voltages (Feedforward, VoltageController).
struct DrivetrainVoltages {
    float left{0.0f};    // volts
    float right{0.0f};   // volts

    /// Millivolts, for pros::Motor::move_voltage().
    float leftMillivolts() const { return left * 1000.0f; }
    float rightMillivolts() const { return right * 1000.0f; }

    /// The -127..127 range used by Drivetrain::setVoltage().
    float leftPower(float maxVoltage = 12.0f) const {
        return (left / maxVoltage) * 127.0f;
    }
    float rightPower(float maxVoltage = 12.0f) const {
        return (right / maxVoltage) * 127.0f;
    }
};

// Joystick input curve for driver control.
struct DriveCurve {
    float deadband{15.0f};
    float minOutput{20.0f};
    float curve{1.0f};          // 1.0 linear, >1.0 exponential, <1.0 logarithmic

    float apply(float input) const;
};

}  // namespace AARONLIB
