// Ported from FBLib - https://github.com/jiazegao/FBLib
// Split out of FBLib's Chassis.hpp (adapted for chad) so odometry can
// depend on Drivetrain without pulling in Chassis.
#pragma once

#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

#include "chad/util/kinematics.hpp"

namespace chad {

/// An encoder average and the health of the group behind it. A scalar cannot
/// tell "every motor is dead" from "sitting still": the first tick of a total
/// outage reads as one big jump, which odometry's guard rejects, and every tick
/// after that reads a consistent zero -- so the pose freezes while the robot
/// drives, and nothing says why. A consumer that can see `live == 0` holds its
/// last good value and counts the fault instead.
struct EncoderAverage {
    float value{0.0f};
    int live{0};
    int faulted{0};
    bool valid() const { return live > 0; }
};

/// Both sides from one sweep, with the same health counts.
struct WheelReading {
    WheelSpeeds speeds{};
    int live{0};
    int faulted{0};
    bool valid() const { return live > 0; }
};

// Hardware abstraction for the drive base.
class Drivetrain {
public:
    /// @param wheels  what the wheels do when the chassis is pushed sideways.
    ///                Omni scrubs out of a corner; traction holds it. Every
    ///                curvature limit in the library is derived from this, so
    ///                it is worth being honest about.
    Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
               float trackWidth, float wheelDiameter, float rpm = 450.0f,
               pros::MotorGroup* leftSenseMotors = nullptr,
               pros::MotorGroup* rightSenseMotors = nullptr,
               WheelType wheels = WheelType::Omni,
               float gripLimitOverride = 0.0f);

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
    WheelType wheels{WheelType::Omni};
    float gripLimitOverride{0.0f};   // inches/s^2, 0 = derive from `wheels`

    /// Free wheel speed, inches/s, from rpm and wheel diameter.
    float maxWheelSpeedInPerSec() const;

    /// Physical envelope for the planners and controllers: what this base can
    /// deliver and what its wheels will hold.
    DriveModel model() const;

    DifferentialKinematics kinematics() const { return DifferentialKinematics(trackWidth); }

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

    /// Both sides from one sweep of the motors. The motion loop wants both every
    /// tick, and the per-side calls read the same hardware twice over.
    WheelSpeeds wheelVelocitiesInPerSec() const;

    /// The same sweep, with the health counts. A total outage reports zero
    /// speed, which a rest test cannot tell from a stopped drivetrain.
    WheelReading wheelVelocitiesLive() const;

    /// Per-side wheel travel (inches), signed. Characterisation needs the sides
    /// apart: their difference over a known rotation is the track width.
    float leftPositionIn() const;
    float rightPositionIn() const;

    /// Average motor position (degrees). Ignores PROS_ERR from dead motors.
    float averagePositionDeg() const;

    /// The same average, with the counts. Odometry's only wheel source on a base
    /// with no tracking wheels, so a silent zero here freezes the pose.
    EncoderAverage averagePositionDegLive() const;
    EncoderAverage leftPositionInLive() const;
    EncoderAverage rightPositionInLive() const;

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
    /// Sum and count of the healthy motor readings in a group. Indexed reads,
    /// not the _all() overloads, which return a std::vector by value: at 100 Hz
    /// with several callers a tick that allocated and freed on every control
    /// cycle, fragmenting the heap alongside MCL's particle buffers.
    static void accumulateVelocity(const pros::MotorGroup* g, double& sum, int& count,
                                   int& faulted);
    static void accumulatePosition(const pros::MotorGroup* g, double& sum, int& count,
                                   int& faulted);
    float velocityFromRpm(double motorRpm) const;
    float groupVelocityInPerSec(const pros::MotorGroup* a,
                                const pros::MotorGroup* b) const;
    EncoderAverage groupPositionDegLive(const pros::MotorGroup* g) const;
    EncoderAverage groupPositionInLive(const pros::MotorGroup* g) const;

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

}  // namespace chad
