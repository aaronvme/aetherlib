#include "chad/movement_control/arc.hpp"

#include <algorithm>
#include <cmath>

#include "chad/util/util.hpp"

namespace chad {
Pose computeArcCenter(const Pose& current, float radius) {
    float perpX, perpY;
    if (radius >= 0.0f) {
        perpX = -std::sin(current.theta);
        perpY =  std::cos(current.theta);
    } else {
        perpX =  std::sin(current.theta);
        perpY = -std::cos(current.theta);
    }
    float absRadius = std::fabs(radius);
    return {current.x + perpX * absRadius, current.y + perpY * absRadius, 0.0f};
}

float chordDistance(const Pose& current, float targetX, float targetY) {
    return distanceToPoint(current, targetX, targetY);
}

float arcAngle(const Pose& current, float targetX, float targetY,
               float radius) {
    float chord = chordDistance(current, targetX, targetY);
    float absRadius = std::fabs(radius);
    if (absRadius < 1e-6f) return 0.0f;
    float halfAngle = std::asin(std::min(chord / (2.0f * absRadius), 1.0f));
    return 2.0f * halfAngle;
}

float arcEndHeading(const Pose& current, float targetX, float targetY,
                    float radius) {
    float chordBearing = bearingToPoint(current, targetX, targetY);
    float angle = arcAngle(current, targetX, targetY, radius);
    float sign = (radius >= 0.0f) ? 1.0f : -1.0f;
    return wrapRad(chordBearing + sign * angle * 0.5f);
}

float arcStartHeading(const Pose& current, float targetX, float targetY,
                      float radius) {
    float chordBearing = bearingToPoint(current, targetX, targetY);
    float angle = arcAngle(current, targetX, targetY, radius);
    float sign = (radius >= 0.0f) ? 1.0f : -1.0f;
    return wrapRad(chordBearing - sign * angle * 0.5f);
}
}
