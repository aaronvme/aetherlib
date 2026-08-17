// riku-sim PROS shim — pros/imu.hpp
//
// The bus holds rotation as the PHYSICAL sensor reports it, already including
// under-report error, drift and noise. riku::ScaledIMU derives from this class
// and overrides get_rotation()/get_heading() to apply its correction, so the
// full calibration path is what runs under test.
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "error.h"
#include "rtos.hpp"
#include "../sim_runtime.hpp"

namespace pros {
inline namespace v5 {

struct imu_gyro_s_t { double x, y, z; };
struct imu_accel_s_t { double x, y, z; };
struct euler_s_t { double pitch, roll, yaw; };
struct quaternion_s_t { double x, y, z, w; };

class Imu {
public:
    explicit Imu(const std::uint8_t port) : mPort(port) {
        sim::bus().imus[port].present = true;
    }

    virtual ~Imu() = default;

    /// Continuous rotation, degrees, VEX convention (0 = forward, CW positive).
    virtual double get_rotation() const { return sim::bus().imus[mPort].rotationDeg; }

    /// Bounded heading [0, 360).
    virtual double get_heading() const {
        const double r = get_rotation();
        if (!std::isfinite(r)) return r;
        double h = std::fmod(r, 360.0);
        if (h < 0.0) h += 360.0;
        return h;
    }

    virtual std::int32_t reset(bool blocking = false) const {
        auto& s = sim::bus().imus[mPort];
        s.rotationDeg = 0.0;
        s.calibrating = true;
        if (blocking) {
            // A real V5 IMU takes roughly two seconds. Burning that in virtual
            // time matters: it is long enough for a caller that forgot to wait
            // to read a bogus heading, and we want the sim to reproduce that.
            ::pros::delay(2000);
        }
        s.calibrating = false;
        return 1;
    }

    virtual bool is_calibrating() const { return sim::bus().imus[mPort].calibrating; }

    virtual std::int32_t tare_rotation() const {
        sim::bus().imus[mPort].rotationDeg = 0.0;
        return 1;
    }
    virtual std::int32_t tare_heading() const { return tare_rotation(); }
    virtual std::int32_t tare() const { return tare_rotation(); }

    virtual std::int32_t set_rotation(double target) const {
        sim::bus().imus[mPort].rotationDeg = target;
        return 1;
    }
    virtual std::int32_t set_heading(double target) const { return set_rotation(target); }

    virtual double get_yaw() const {
        double y = std::fmod(get_rotation(), 360.0);
        if (y > 180.0) y -= 360.0;
        if (y < -180.0) y += 360.0;
        return y;
    }
    virtual double get_pitch() const { return 0.0; }
    virtual double get_roll() const { return 0.0; }

    virtual imu_gyro_s_t get_gyro_rate() const { return {0.0, 0.0, 0.0}; }
    virtual imu_accel_s_t get_accel() const { return {0.0, 0.0, 0.0}; }
    virtual euler_s_t get_euler() const { return {0.0, 0.0, get_yaw()}; }

    std::uint8_t get_port() const { return mPort; }
    static std::vector<Imu> get_all_devices() { return {}; }

private:
    std::uint8_t mPort{1};
};

using IMU = Imu;

}  // namespace v5
}  // namespace pros
