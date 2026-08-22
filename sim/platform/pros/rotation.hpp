// chad-sim PROS shim — pros/rotation.hpp
#pragma once

#include <cstdint>
#include <vector>

#include "error.h"
#include "../sim_runtime.hpp"

namespace pros {
inline namespace v5 {

class Rotation {
public:
    explicit Rotation(const std::int8_t port)
        : mPort(port < 0 ? -port : port), mReversed(port < 0) {
        sim::bus().rotations[mPort].present = true;
    }

    virtual ~Rotation() = default;

    /// Centidegrees (36000 per revolution), continuous.
    virtual std::int32_t get_position() const {
        const std::int32_t p = sim::bus().rotations[mPort].positionCentideg;
        return mReversed ? -p : p;
    }

    virtual std::int32_t reset_position(void) const {
        sim::bus().rotations[mPort].positionCentideg = 0;
        return 1;
    }
    virtual std::int32_t reset(void) const { return reset_position(); }

    virtual std::int32_t set_position(std::uint32_t position) const {
        sim::bus().rotations[mPort].positionCentideg = static_cast<std::int32_t>(position);
        return 1;
    }

    virtual std::int32_t get_angle() const {
        std::int32_t a = get_position() % 36000;
        if (a < 0) a += 36000;
        return a;
    }

    virtual std::int32_t get_velocity() const { return 0; }

    virtual std::int32_t set_reversed(bool value) const {
        const_cast<Rotation*>(this)->mReversed = value;
        return 1;
    }
    virtual std::int32_t get_reversed() const { return mReversed ? 1 : 0; }

    std::uint8_t get_port() const { return mPort; }
    static std::vector<Rotation> get_all_devices() { return {}; }

private:
    std::uint8_t mPort{1};
    bool mReversed{false};
};

}  // namespace v5
}  // namespace pros
