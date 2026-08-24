// chad-sim PROS shim — pros/motor_group.hpp
#pragma once

#include <cstdint>
#include <initializer_list>
#include <vector>

#include "motors.hpp"

namespace pros {
inline namespace v5 {

class MotorGroup {
public:
    MotorGroup(const std::initializer_list<std::int8_t> ports,
               MotorGears gearset = MotorGears::green,
               MotorUnits units = MotorUnits::degrees)
        : mGears(gearset), mUnits(units) {
        for (std::int8_t p : ports) registerPort(p, gearset);
    }

    MotorGroup(const std::vector<std::int8_t>& ports,
               MotorGears gearset = MotorGears::green,
               MotorUnits units = MotorUnits::degrees)
        : mGears(gearset), mUnits(units) {
        for (std::int8_t p : ports) registerPort(p, gearset);
    }

    // --- commands -----------------------------------------------------------

    std::int32_t move_voltage(const std::int32_t mv) const {
        const double cmd = clampMv(mv);
        for (std::size_t i = 0; i < mPorts.size(); i++) {
            sim::bus().motors[mPorts[i]].commandedMv = mReversed[i] ? -cmd : cmd;
        }
        return 1;
    }
    std::int32_t move(const std::int32_t power) const {
        return move_voltage(static_cast<std::int32_t>(power * (12000.0 / 127.0)));
    }
    std::int32_t brake() const { return move_voltage(0); }

    // --- telemetry ----------------------------------------------------------

    std::vector<double> get_position_all(void) const {
        std::vector<double> out;
        out.reserve(mPorts.size());
        for (std::size_t i = 0; i < mPorts.size(); i++) {
            const auto& m = sim::bus().motors[mPorts[i]];
            out.push_back(m.reversed ? -m.positionDeg : m.positionDeg);
        }
        return out;
    }

    std::vector<double> get_actual_velocity_all(void) const {
        std::vector<double> out;
        out.reserve(mPorts.size());
        for (std::size_t i = 0; i < mPorts.size(); i++) {
            const auto& m = sim::bus().motors[mPorts[i]];
            out.push_back(m.reversed ? -m.velocityRpm : m.velocityRpm);
        }
        return out;
    }

    std::vector<double> get_temperature_all(void) const {
        std::vector<double> out;
        for (std::size_t i = 0; i < mPorts.size(); i++) {
            out.push_back(sim::bus().motors[mPorts[i]].temperatureC);
        }
        return out;
    }

    std::vector<double> get_current_draw_all(void) const {
        std::vector<double> out;
        for (std::size_t i = 0; i < mPorts.size(); i++) {
            out.push_back(sim::bus().motors[mPorts[i]].currentMa);
        }
        return out;
    }

    double get_position(const std::uint8_t index = 0) const {
        if (index >= mPorts.size()) return PROS_ERR_F;
        const auto& m = sim::bus().motors[mPorts[index]];
        return m.reversed ? -m.positionDeg : m.positionDeg;
    }

    double get_actual_velocity(const std::uint8_t index = 0) const {
        if (index >= mPorts.size()) return PROS_ERR_F;
        const auto& m = sim::bus().motors[mPorts[index]];
        return m.reversed ? -m.velocityRpm : m.velocityRpm;
    }

    double get_temperature(const std::uint8_t index = 0) const {
        if (index >= mPorts.size()) return PROS_ERR_F;
        return sim::bus().motors[mPorts[index]].temperatureC;
    }

    double get_current_draw(const std::uint8_t index = 0) const {
        if (index >= mPorts.size()) return PROS_ERR_F;
        return sim::bus().motors[mPorts[index]].currentMa;
    }

    std::int32_t move_velocity(const std::int32_t rpm) const {
        const double free = gearsetRpm(mGears);
        return move_voltage(static_cast<std::int32_t>((rpm / free) * 12000.0));
    }

    std::int32_t is_installed(const std::uint8_t index = 0) const {
        if (index >= mPorts.size()) return 0;
        return sim::bus().motors[mPorts[index]].present ? 1 : 0;
    }

    // --- configuration ------------------------------------------------------

    std::int32_t set_brake_mode(MotorBrake mode) const { return setBrake(static_cast<int>(mode)); }
    std::int32_t set_brake_mode(motor_brake_mode_e_t mode) const { return setBrake(static_cast<int>(mode)); }
    std::int32_t set_brake_mode_all(MotorBrake mode) const { return setBrake(static_cast<int>(mode)); }
    std::int32_t set_brake_mode_all(motor_brake_mode_e_t mode) const { return setBrake(static_cast<int>(mode)); }

    std::int32_t set_encoder_units_all(MotorUnits) const { return 1; }
    std::int32_t set_encoder_units_all(motor_encoder_units_e_t) const { return 1; }
    std::int32_t set_encoder_units(MotorUnits) const { return 1; }

    std::int32_t tare_position_all(void) const {
        for (std::size_t i = 0; i < mPorts.size(); i++) sim::bus().motors[mPorts[i]].positionDeg = 0.0;
        return 1;
    }
    std::int32_t tare_position(void) const { return tare_position_all(); }

    /// Read from the port, not from the group: a real motor always reports its
    /// own cartridge, so a group spanning two cartridges reports two answers.
    MotorGears get_gearing(const std::uint8_t index = 0) const {
        if (index >= mPorts.size()) return MotorGears::invalid;
        return gearsetFromRpm(sim::bus().motors[mPorts[index]].gearsetRpm);
    }

    std::int32_t set_gearing_all(MotorGears g) const {
        const_cast<MotorGroup*>(this)->mGears = g;
        for (std::size_t i = 0; i < mPorts.size(); i++) {
            sim::bus().motors[mPorts[i]].gearsetRpm = static_cast<int>(gearsetRpm(g));
        }
        return 1;
    }

    std::int8_t size(void) const { return static_cast<std::int8_t>(mPorts.size()); }

    const std::vector<std::uint8_t>& ports() const { return mPorts; }

private:
    /// A group built with `invalid` gearset must not touch the port's cartridge:
    /// config.cpp declares the blue and green sense groups first and then a
    /// combined command group with `invalid`, and overwriting from that group
    /// would leave every drive port claiming the same cartridge -- hiding the
    /// mixed-cartridge mistake Drivetrain's header warns about.
    void registerPort(std::int8_t p, MotorGears gearset) {
        const int abs = (p < 0) ? -p : p;
        mPorts.push_back(static_cast<std::uint8_t>(abs));
        mReversed.push_back(p < 0);
        auto& m = sim::bus().motors[abs];
        m.present = true;
        m.reversed = (p < 0);
        if (gearset != MotorGears::invalid) {
            m.gearsetRpm = static_cast<int>(gearsetRpm(gearset));
        }
    }

    std::int32_t setBrake(int mode) const {
        for (std::size_t i = 0; i < mPorts.size(); i++) sim::bus().motors[mPorts[i]].brakeMode = mode;
        return 1;
    }
    static double clampMv(double mv) {
        if (mv > 12000.0) return 12000.0;
        if (mv < -12000.0) return -12000.0;
        return mv;
    }

    std::vector<std::uint8_t> mPorts;
    std::vector<bool> mReversed;
    MotorGears mGears{MotorGears::green};
    MotorUnits mUnits{MotorUnits::degrees};
};

}  // namespace v5
}  // namespace pros
