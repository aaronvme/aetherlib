// chad-sim PROS shim — pros/misc.hpp
//
// Controller inputs are fed from the browser (keyboard / on-screen sticks) via
// the bridge, so driver-control code and drive curves can be exercised too.
#pragma once

#include <cstdint>
#include <string>

#include "error.h"

namespace pros {

typedef enum {
    E_CONTROLLER_MASTER = 0,
    E_CONTROLLER_PARTNER
} controller_id_e_t;

typedef enum {
    E_CONTROLLER_ANALOG_LEFT_X = 0,
    E_CONTROLLER_ANALOG_LEFT_Y,
    E_CONTROLLER_ANALOG_RIGHT_X,
    E_CONTROLLER_ANALOG_RIGHT_Y
} controller_analog_e_t;

typedef enum {
    E_CONTROLLER_DIGITAL_L1 = 6,
    E_CONTROLLER_DIGITAL_L2,
    E_CONTROLLER_DIGITAL_R1,
    E_CONTROLLER_DIGITAL_R2,
    E_CONTROLLER_DIGITAL_UP,
    E_CONTROLLER_DIGITAL_DOWN,
    E_CONTROLLER_DIGITAL_LEFT,
    E_CONTROLLER_DIGITAL_RIGHT,
    E_CONTROLLER_DIGITAL_X,
    E_CONTROLLER_DIGITAL_B,
    E_CONTROLLER_DIGITAL_Y,
    E_CONTROLLER_DIGITAL_A
} controller_digital_e_t;

namespace sim_input {
/// Backing store the bridge writes into. Analog -127..127, digital 0/1.
struct ControllerInput {
    int analog[4]{0, 0, 0, 0};
    bool digital[32]{};
    bool digitalPrev[32]{};
};
ControllerInput& state(int id);
}  // namespace sim_input

inline namespace v5 {

class Controller {
public:
    explicit Controller(controller_id_e_t id) : mId(static_cast<int>(id)) {}

    std::int32_t get_analog(controller_analog_e_t channel) {
        return sim_input::state(mId).analog[static_cast<int>(channel)];
    }

    std::int32_t get_digital(controller_digital_e_t button) {
        return sim_input::state(mId).digital[static_cast<int>(button)] ? 1 : 0;
    }

    std::int32_t get_digital_new_press(controller_digital_e_t button) {
        auto& s = sim_input::state(mId);
        const int b = static_cast<int>(button);
        const bool pressed = s.digital[b] && !s.digitalPrev[b];
        s.digitalPrev[b] = s.digital[b];
        return pressed ? 1 : 0;
    }

    std::int32_t is_connected() { return 1; }
    std::int32_t get_battery_capacity() { return 100; }
    std::int32_t get_battery_level() { return 100; }
    std::int32_t rumble(const char*) { return 1; }
    std::int32_t clear() { return 1; }
    std::int32_t clear_line(std::uint8_t) { return 1; }
    std::int32_t set_text(std::uint8_t, std::uint8_t, const char*) { return 1; }
    std::int32_t set_text(std::uint8_t, std::uint8_t, const std::string&) { return 1; }
    std::int32_t print(std::uint8_t, std::uint8_t, const char*, ...) { return 1; }

private:
    int mId{0};
};

}  // namespace v5

namespace battery {
inline double get_capacity() { return 100.0; }
inline std::int32_t get_current() { return 0; }
inline double get_temperature() { return 30.0; }
inline std::int32_t get_voltage() { return 12000; }
}  // namespace battery

namespace usd {
inline std::int32_t is_installed() { return 1; }
}  // namespace usd

namespace competition {
inline std::uint8_t get_status() { return 0; }
inline bool is_autonomous() { return false; }
inline bool is_connected() { return false; }
inline bool is_disabled() { return false; }
}  // namespace competition

inline std::uint8_t competition_get_status() { return 0; }
inline bool competition_is_autonomous() { return false; }
inline bool competition_is_connected() { return false; }
inline bool competition_is_disabled() { return false; }

}  // namespace pros
