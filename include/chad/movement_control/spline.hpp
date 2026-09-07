#pragma once

#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <vector>

#include "chad/util/util.hpp"

namespace chad {
inline constexpr std::size_t MAX_SPLINE_POINTS = 4000;

inline constexpr float SPLINE_SPACING_IN = 2.0f;

inline std::vector<Pose> splinePath(const std::vector<Pose>& controls,
                                    float spacingIn = SPLINE_SPACING_IN) {
    if (controls.size() < 2) return controls;

    const float spacing = (spacingIn > 0.05f) ? spacingIn : 0.05f;

    std::vector<Pose> p;
    p.reserve(controls.size() + 2);
    p.push_back(controls.front());
    p.insert(p.end(), controls.begin(), controls.end());
    p.push_back(controls.back());

    const auto at = [](float a, float b, float c, float d, float t) {
        const float t2 = t * t, t3 = t2 * t;
        return 0.5f * ((2.0f * b) + (-a + c) * t +
                       (2.0f * a - 5.0f * b + 4.0f * c - d) * t2 +
                       (-a + 3.0f * b - 3.0f * c + d) * t3);
    };

    std::vector<Pose> out;
    for (std::size_t i = 1; i + 2 < p.size(); i++) {
        const Pose& p0 = p[i - 1];
        const Pose& p1 = p[i];
        const Pose& p2 = p[i + 1];
        const Pose& p3 = p[i + 2];

        const float chord = std::hypot(p2.x - p1.x, p2.y - p1.y);
        int steps = static_cast<int>(std::ceil(chord / spacing));
        if (steps < 2) steps = 2;

        for (int s = 0; s < steps; s++) {
            if (out.size() >= MAX_SPLINE_POINTS - 1) break;
            const float t = static_cast<float>(s) / static_cast<float>(steps);
            out.push_back({at(p0.x, p1.x, p2.x, p3.x, t),
                           at(p0.y, p1.y, p2.y, p3.y, t),
                           0.0f});
        }
    }
    out.push_back({controls.back().x, controls.back().y, 0.0f});

    for (std::size_t i = 0; i < out.size(); i++) {
        const Pose& a = out[i > 0 ? i - 1 : 0];
        const Pose& b = out[i + 1 < out.size() ? i + 1 : out.size() - 1];
        const float dx = b.x - a.x, dy = b.y - a.y;

        if (std::fabs(dx) < 1e-6f && std::fabs(dy) < 1e-6f) {
            out[i].theta = (i > 0) ? out[i - 1].theta : 0.0f;
        } else {
            out[i].theta = std::atan2(dy, dx);
        }
    }
    return out;
}

inline std::vector<Pose> splinePath(std::initializer_list<Pose> controls,
                                    float spacingIn = SPLINE_SPACING_IN) {
    return splinePath(std::vector<Pose>(controls), spacingIn);
}
}
