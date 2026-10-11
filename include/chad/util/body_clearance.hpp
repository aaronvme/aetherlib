#pragma once

#include <cmath>

namespace chad {
// True when a robot body of inscribed radius bodyRadiusIn centred at (x, y)
// overlaps any circle (.x, .y, .radius) in circles. The inscribed circle lies
// inside the footprint at every heading, so this never rejects a reachable
// pose and needs no trig: one squared distance per circle. A non-positive or
// NaN radius disables the test.
template <class Circles>
inline bool bodyOverlapsCircle(float x, float y, float bodyRadiusIn,
                               const Circles& circles) {
    if (!(bodyRadiusIn > 0.0f)) return false;
    for (const auto& c : circles) {
        const float reach = c.radius + bodyRadiusIn;
        const float dx = x - c.x;
        const float dy = y - c.y;
        if (dx * dx + dy * dy < reach * reach) return true;
    }
    return false;
}
}  // namespace chad
