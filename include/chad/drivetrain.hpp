#pragma once

#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

namespace chad {
class Drivetrain {
public:
    Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
               float trackWidth, float wheelDiameter, float rpm = 450.0f,
               pros::MotorGroup* leftSenseMotors = nullptr,
               pros::MotorGroup* rightSenseMotors = nullptr);

    pros::MotorGroup* leftMotors{nullptr};
    pros::MotorGroup* rightMotors{nullptr};

    pros::MotorGroup* leftSenseMotors{nullptr};
    pros::MotorGroup* rightSenseMotors{nullptr};
    float trackWidth;
    float wheelDiameter;
    float rpm;

    void setLeftVoltage(float voltage);
    void setRightVoltage(float voltage);
    void setVoltage(float left, float right);

    void setBrakeMode(pros::motor_brake_mode_e mode);

    // One side's motors in HOLD, and the brake modes they had before.
    struct SideHold {
        static constexpr int kMaxMotors = 8;
        pros::MotorGroup* group{nullptr};
        int count{0};
        pros::MotorBrake modes[kMaxMotors]{};
    };

    // Every motor on one side to HOLD, stopped where it stands.
    SideHold holdSide(bool left);
    // Restores the modes holdSide() found, on motors still in HOLD.
    void releaseSide(SideHold& hold);

    float averageVelocityInPerSec() const;

    float leftVelocityInPerSec() const;
    float rightVelocityInPerSec() const;

    float leftPositionIn() const;
    float rightPositionIn() const;

    // Mean of the absolute positions. Differencing it is only safe while the
    // same motors stay readable; odometry integrates per-motor deltas instead.
    float averagePositionDeg() const;

    // Per-motor positions into out[0..max), PROS_ERR_F left in place for a
    // motor that cannot be read. Returns the count written. No allocation.
    static int readPositionsDeg(const pros::MotorGroup* g, double* out, int max);

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

    mutable float mExtGearRatio{0.0f};
};

struct DrivetrainVoltages {
    float left{0.0f};
    float right{0.0f};

    float leftMillivolts() const { return left * 1000.0f; }
    float rightMillivolts() const { return right * 1000.0f; }

    float leftPower(float maxVoltage = 12.0f) const {
        return (left / maxVoltage) * 127.0f;
    }
    float rightPower(float maxVoltage = 12.0f) const {
        return (right / maxVoltage) * 127.0f;
    }
};

struct DriveCurve {
    float deadband{15.0f};
    float minOutput{20.0f};
    float curve{1.0f};

    float apply(float input) const;
};
}
