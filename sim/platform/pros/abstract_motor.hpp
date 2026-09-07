// chad-sim PROS shim — pros/abstract_motor.hpp
#pragma once

#include <cstdint>

#include "error.h"

namespace pros {
inline namespace v5 {

enum class MotorBrake {
    coast = 0,
    brake = 1,
    hold  = 2,
    invalid = INT32_MAX
};

enum class MotorUnits {
    degrees   = 0,
    rotations = 1,
    counts    = 2,
    invalid   = INT32_MAX
};

enum class MotorGears {
    ratio_36_to_1 = 0, red   = ratio_36_to_1,   // 100 RPM
    ratio_18_to_1 = 1, green = ratio_18_to_1,   // 200 RPM
    ratio_6_to_1  = 2, blue  = ratio_6_to_1,    // 600 RPM
    invalid = INT32_MAX
};

using MotorGearset = MotorGears;

/// Cartridge free speed for a gearset, in motor-shaft RPM.
inline double gearsetRpm(MotorGears g) {
    switch (g) {
        case MotorGears::red:   return 100.0;
        case MotorGears::green: return 200.0;
        case MotorGears::blue:  return 600.0;
        default:                return 200.0;
    }
}

inline MotorGears gearsetFromRpm(int rpm) {
    if (rpm == 100) return MotorGears::red;
    if (rpm == 600) return MotorGears::blue;
    return MotorGears::green;
}

}  // namespace v5

// C-style enums, still used across chad (e.g. Drivetrain::setBrakeMode).
typedef enum motor_brake_mode_e {
    E_MOTOR_BRAKE_COAST = 0,
    E_MOTOR_BRAKE_BRAKE = 1,
    E_MOTOR_BRAKE_HOLD  = 2,
    E_MOTOR_BRAKE_INVALID = INT32_MAX
} motor_brake_mode_e_t;

typedef enum motor_encoder_units_e {
    E_MOTOR_ENCODER_DEGREES   = 0,
    E_MOTOR_ENCODER_ROTATIONS = 1,
    E_MOTOR_ENCODER_COUNTS    = 2,
    E_MOTOR_ENCODER_INVALID   = INT32_MAX
} motor_encoder_units_e_t;

typedef enum motor_gearset_e {
    E_MOTOR_GEARSET_36 = 0,
    E_MOTOR_GEAR_RED   = E_MOTOR_GEARSET_36,
    E_MOTOR_GEARSET_18 = 1,
    E_MOTOR_GEAR_GREEN = E_MOTOR_GEARSET_18,
    E_MOTOR_GEARSET_06 = 2,
    E_MOTOR_GEAR_BLUE  = E_MOTOR_GEARSET_06,
    E_MOTOR_GEARSET_INVALID = INT32_MAX
} motor_gearset_e_t;

#define MOTOR_BRAKE_COAST pros::E_MOTOR_BRAKE_COAST
#define MOTOR_BRAKE_BRAKE pros::E_MOTOR_BRAKE_BRAKE
#define MOTOR_BRAKE_HOLD pros::E_MOTOR_BRAKE_HOLD
#define MOTOR_ENCODER_DEGREES pros::E_MOTOR_ENCODER_DEGREES
#define MOTOR_ENCODER_ROTATIONS pros::E_MOTOR_ENCODER_ROTATIONS
#define MOTOR_ENCODER_COUNTS pros::E_MOTOR_ENCODER_COUNTS

}  // namespace pros
