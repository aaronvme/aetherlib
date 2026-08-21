// Ported from FBLib - https://github.com/jiazegao/FBLib
// Split out of FBLib's Chassis.hpp (adapted for riku) so odometry can
// depend on Drivetrain without pulling in Chassis.
#pragma once

#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

namespace riku {

// Hardware abstraction for the drive base.
class Drivetrain {
public:
    Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
               float trackWidth, float wheelDiameter, float rpm = 450.0f,
               float horizontalDrift = 2.0f,
               pros::MotorGroup* leftSenseMotors = nullptr,
               pros::MotorGroup* rightSenseMotors = nullptr);

    pros::MotorGroup* leftMotors{nullptr};
    pros::MotorGroup* rightMotors{nullptr};

    // Encoder source. Every motor here must carry the SAME cartridge: one
    // externalGearRatio() covers the whole group, so a mixed group would scale
    // some motors' degrees wrongly. Null = sense from the command groups.
    pros::MotorGroup* leftSenseMotors{nullptr};
    pros::MotorGroup* rightSenseMotors{nullptr};
    float trackWidth;         // inches, center-to-center
    float wheelDiameter;      // inches
    float rpm;                // drive RPM (not cartridge RPM)
    float horizontalDrift;

    // Voltage, -127 to 127
    void setLeftVoltage(float voltage);
    void setRightVoltage(float voltage);
    void setVoltage(float left, float right);

    void setBrakeMode(pros::motor_brake_mode_e mode);

    /// Average wheel speed (inches/sec), signed. Ignores PROS_ERR from dead
    /// motors, same as averagePositionDeg().
    float averageVelocityInPerSec() const;

    /// Per-side wheel speed (inches/sec), signed. A velocity controller needs
    /// the sides apart -- averaging them hides the turn.
    float leftVelocityInPerSec() const;
    float rightVelocityInPerSec() const;

    /// Per-side wheel travel (inches), signed. Characterisation needs the sides
    /// apart: their difference over a known rotation is the track width.
    float leftPositionIn() const;
    float rightPositionIn() const;

    /// Average motor position (degrees). Ignores PROS_ERR from dead motors.
    float averagePositionDeg() const;

    /// Wheel revs per motor rev (wheel RPM / cartridge RPM). Cached after the
    /// first valid read; returns 1.0 while the gearset is still unknown.
    float externalGearRatio() const;

    pros::MotorGroup* leftSense() const {
        return (leftSenseMotors != nullptr) ? leftSenseMotors : leftMotors;
    }
    pros::MotorGroup* rightSense() const {
        return (rightSenseMotors != nullptr) ? rightSenseMotors : rightMotors;
    }

private:
    float groupVelocityInPerSec(const pros::MotorGroup* a,
                                const pros::MotorGroup* b) const;
    float groupPositionDeg(const pros::MotorGroup* g) const;

    mutable float mExtGearRatio{0.0f};   // 0 = unresolved
};

// Controller output for both drive sides.
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

}  // namespace riku
