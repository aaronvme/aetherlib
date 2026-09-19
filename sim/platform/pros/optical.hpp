// chad-sim PROS shim — pros/optical.hpp
//
// Colour sorting is not modelled; values are settable from the browser so
// gamespecific logic that branches on hue can still be driven in the sim.
#pragma once

#include <cstdint>
#include <vector>

#include "error.h"

namespace pros {
inline namespace v5 {

struct optical_rgb_s_t { double red, green, blue, brightness; };
struct optical_raw_s_t { std::uint32_t clear, red, green, blue; };

class Optical {
public:
    explicit Optical(const std::uint8_t port) : mPort(port) {}
    virtual ~Optical() = default;

    virtual double get_hue() { return mHue; }
    virtual double get_saturation() { return mSaturation; }
    virtual double get_brightness() { return mBrightness; }
    virtual std::int32_t get_proximity() { return mProximity; }
    virtual optical_rgb_s_t get_rgb() { return {0, 0, 0, mBrightness}; }
    virtual optical_raw_s_t get_raw() { return {0, 0, 0, 0}; }

    virtual std::int32_t set_led_pwm(std::uint8_t value) { mLedPwm = value; return 1; }
    virtual std::int32_t get_led_pwm() { return mLedPwm; }
    virtual std::int32_t disable_gesture() { return 1; }
    virtual std::int32_t enable_gesture() { return 1; }

    std::uint8_t get_port() const { return mPort; }

    // Driven from the bridge.
    void sim_set(double hue, double sat, double bright, std::int32_t prox) {
        mHue = hue; mSaturation = sat; mBrightness = bright; mProximity = prox;
    }

private:
    std::uint8_t mPort{1};
    double mHue{0.0}, mSaturation{0.0}, mBrightness{0.0};
    std::int32_t mProximity{0}, mLedPwm{0};
};

}  // namespace v5
}  // namespace pros
