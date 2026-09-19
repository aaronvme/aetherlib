// chad-sim PROS shim — pros/distance.hpp
//
// Readings come from the raycaster in sim/world/field.cpp, so field geometry
// genuinely occludes these sensors: a ray that hits a goal post reports the
// post, not the wall behind it.
#pragma once

#include <cstdint>
#include <vector>

#include "error.h"
#include "../sim_runtime.hpp"

namespace pros {
inline namespace v5 {

class Distance {
public:
    explicit Distance(const std::uint8_t port) : mPort(port) {
        sim::bus().distances[port].present = true;
    }

    virtual ~Distance() = default;

    /// Distance to the nearest object, millimetres. 9999 when nothing is in range.
    virtual std::int32_t get() { return sim::bus().distances[mPort].distanceMm; }
    virtual std::int32_t get_distance() { return get(); }

    /// 0-63. Falls off with range and with glancing incidence.
    virtual std::int32_t get_confidence() { return sim::bus().distances[mPort].confidence; }

    virtual std::int32_t get_object_size() { return sim::bus().distances[mPort].objectSize; }
    virtual double get_object_velocity() { return 0.0; }

    std::int32_t is_installed() const { return sim::bus().distances[mPort].present ? 1 : 0; }

    std::uint8_t get_port() const { return mPort; }
    static std::vector<Distance> get_all_devices() { return {}; }

private:
    std::uint8_t mPort{1};
};

}  // namespace v5
}  // namespace pros
