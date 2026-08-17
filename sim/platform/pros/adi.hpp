// riku-sim PROS shim — pros/adi.hpp
#pragma once

#include <cstdint>

#include "error.h"
#include "../sim_runtime.hpp"

namespace pros {

using adi_port_pair_t = std::pair<std::uint8_t, std::uint8_t>;

namespace adi {

namespace {
/// 'A'..'H' or 1..8 -> 0-based index.
inline int adiIndex(std::uint8_t port) {
    if (port >= 'A' && port <= 'H') return port - 'A';
    if (port >= 'a' && port <= 'h') return port - 'a';
    if (port >= 1 && port <= 8) return port - 1;
    return 0;
}
}  // namespace

// ============================================================================
// Encoder — quadrature dead-wheel encoder, 360 ticks/rev
// ============================================================================

class Encoder {
public:
    Encoder(std::uint8_t port_top, std::uint8_t port_bottom, bool reversed = false)
        : mIdx(adiIndex(port_top)), mReversed(reversed) {
        (void)port_bottom;
        sim::bus().adiEncoders[mIdx].present = true;
    }

    explicit Encoder(std::uint8_t port, bool reversed = false)
        : mIdx(adiIndex(port)), mReversed(reversed) {
        sim::bus().adiEncoders[mIdx].present = true;
    }

    std::int32_t get_value() const {
        const std::int32_t t = sim::bus().adiEncoders[mIdx].ticks;
        return mReversed ? -t : t;
    }

    std::int32_t reset() const {
        sim::bus().adiEncoders[mIdx].ticks = 0;
        return 1;
    }

private:
    int mIdx{0};
    bool mReversed{false};
};

// ============================================================================
// Pneumatics — no physical effect in the sim, but state is tracked so
// gamespecific code that toggles cylinders runs and can be inspected.
// ============================================================================

class Pneumatics {
public:
    Pneumatics(std::uint8_t adi_port, bool start_extended, bool input_active_low = false)
        : mIdx(adiIndex(adi_port)), mExtended(start_extended), mActiveLow(input_active_low) {}

    std::int32_t extend() { mExtended = true; return 1; }
    std::int32_t retract() { mExtended = false; return 1; }
    std::int32_t toggle() { mExtended = !mExtended; return 1; }
    bool is_extended() const { return mExtended; }

    std::int32_t set_value(std::int32_t value) { mExtended = (value != 0); return 1; }
    std::int32_t get_value() const { return mExtended ? 1 : 0; }

private:
    int mIdx{0};
    bool mExtended{false};
    bool mActiveLow{false};
};

// ============================================================================
// DigitalIn / DigitalOut / AnalogIn — stubs so config code links
// ============================================================================

class DigitalIn {
public:
    explicit DigitalIn(std::uint8_t port) : mIdx(adiIndex(port)) {}
    std::int32_t get_value() const { return 0; }
    std::int32_t get_new_press() const { return 0; }
private:
    int mIdx{0};
};

class DigitalOut {
public:
    DigitalOut(std::uint8_t port, bool init_state = false)
        : mIdx(adiIndex(port)), mState(init_state) {}
    std::int32_t set_value(std::int32_t value) { mState = (value != 0); return 1; }
    std::int32_t get_value() const { return mState ? 1 : 0; }
private:
    int mIdx{0};
    bool mState{false};
};

class AnalogIn {
public:
    explicit AnalogIn(std::uint8_t port) : mIdx(adiIndex(port)) {}
    std::int32_t get_value() const { return 0; }
    std::int32_t calibrate() const { return 0; }
    std::int32_t get_value_calibrated() const { return 0; }
private:
    int mIdx{0};
};

}  // namespace adi

using ADIEncoder = adi::Encoder;
using ADIDigitalIn = adi::DigitalIn;
using ADIDigitalOut = adi::DigitalOut;
using ADIAnalogIn = adi::AnalogIn;

}  // namespace pros
