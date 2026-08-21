// chad-sim PROS shim — pros/motors.hpp
#pragma once

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <vector>

#include "abstract_motor.hpp"
#include "../sim_runtime.hpp"

namespace pros {
inline namespace v5 {

class Motor {
public:
    explicit Motor(std::int8_t port,
                   MotorGears gearset = MotorGears::green,
                   MotorUnits units = MotorUnits::degrees)
        : mPort(port < 0 ? -port : port), mReversed(port < 0), mUnits(units) {
        auto& m = sim::bus().motors[idx()];
        m.present = true;
        m.reversed = mReversed;
        m.gearsetRpm = static_cast<int>(gearsetRpm(gearset));
        mGears = gearset;
    }

    std::int32_t move_voltage(std::int32_t mv) const {
        sim::bus().motors[idx()].commandedMv = clampMv(mv);
        return 1;
    }
    std::int32_t move(std::int32_t power) const {
        return move_voltage(static_cast<std::int32_t>(power * (12000.0 / 127.0)));
    }
    std::int32_t brake() const { return move_voltage(0); }

    /// Velocity control. The sim has no closed-loop motor controller, so this
    /// is mapped onto the equivalent open-loop voltage against free speed.
    std::int32_t move_velocity(std::int32_t rpm) const {
        const auto& m = sim::bus().motors[idx()];
        const double free = (m.gearsetRpm > 0) ? m.gearsetRpm : 200.0;
        return move_voltage(static_cast<std::int32_t>((rpm / free) * 12000.0));
    }
    std::int32_t move_absolute(double, std::int32_t) const { return 1; }
    std::int32_t move_relative(double, std::int32_t) const { return 1; }

    std::int32_t is_installed() const { return sim::bus().motors[idx()].present ? 1 : 0; }

    double get_position() const { return signedPos(); }
    double get_actual_velocity() const {
        const auto& m = sim::bus().motors[idx()];
        return m.reversed ? -m.velocityRpm : m.velocityRpm;
    }
    double get_temperature() const { return sim::bus().motors[idx()].temperatureC; }
    double get_current_draw() const { return sim::bus().motors[idx()].currentMa; }
    double get_power() const { return 0.0; }
    double get_efficiency() const { return 100.0; }
    double get_torque() const { return 0.0; }
    std::int32_t get_voltage() const {
        return static_cast<std::int32_t>(sim::bus().motors[idx()].commandedMv);
    }
    std::int32_t is_over_temp() const { return 0; }
    std::int32_t is_over_current() const { return 0; }

    std::int32_t tare_position() const {
        sim::bus().motors[idx()].positionDeg = 0.0;
        return 1;
    }
    std::int32_t set_brake_mode(MotorBrake mode) const {
        sim::bus().motors[idx()].brakeMode = static_cast<int>(mode);
        return 1;
    }
    std::int32_t set_brake_mode(motor_brake_mode_e_t mode) const {
        sim::bus().motors[idx()].brakeMode = static_cast<int>(mode);
        return 1;
    }
    std::int32_t set_encoder_units(MotorUnits units) const {
        const_cast<Motor*>(this)->mUnits = units;
        return 1;
    }
    std::int32_t set_encoder_units(motor_encoder_units_e_t) const { return 1; }
    std::int32_t set_reversed(bool reverse) const {
        sim::bus().motors[idx()].reversed = reverse;
        return 1;
    }
    std::int32_t set_gearing(MotorGears g) const {
        sim::bus().motors[idx()].gearsetRpm = static_cast<int>(gearsetRpm(g));
        const_cast<Motor*>(this)->mGears = g;
        return 1;
    }

    MotorGears get_gearing() const { return mGears; }
    std::uint8_t get_port() const { return mPort; }

    static std::vector<Motor> get_all_devices() { return {}; }

private:
    int idx() const { return mPort; }
    double signedPos() const {
        const auto& m = sim::bus().motors[idx()];
        return m.reversed ? -m.positionDeg : m.positionDeg;
    }
    static double clampMv(double mv) {
        if (mv > 12000.0) return 12000.0;
        if (mv < -12000.0) return -12000.0;
        return mv;
    }

    std::uint8_t mPort{1};
    bool mReversed{false};
    MotorUnits mUnits{MotorUnits::degrees};
    MotorGears mGears{MotorGears::green};
};

}  // namespace v5
}  // namespace pros
