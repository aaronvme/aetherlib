#include "chad/movement_control/turntopoint.hpp"

#include <cmath>

#include "chad/util/util.hpp"

namespace chad {
float resolveTurnError(float rawErrorRad, TurnDirection direction) {
    switch (direction) {
    case TurnDirection::CCW:
        if (rawErrorRad < 0.0f) rawErrorRad += TWO_PI;
        break;
    case TurnDirection::CW:
        if (rawErrorRad > 0.0f) rawErrorRad -= TWO_PI;
        break;
    case TurnDirection::SHORTEST:
    default:
        break;
    }
    return rawErrorRad;
}

float headingToPoint(const Pose& current, float targetX, float targetY) {
    float desiredHeadingRad = bearingToPoint(current, targetX, targetY);
    return angleDiffRad(current.theta, desiredHeadingRad);
}

float headingToPointOffset(const Pose& current, float targetX, float targetY,
                            float offsetDeg) {
    float desiredHeadingRad = bearingToPoint(current, targetX, targetY) - degToRad(offsetDeg);
    return angleDiffRad(current.theta, desiredHeadingRad);
}

bool headingWithinTolerance(float headingErrorRad, float toleranceRad) {
    return std::fabs(headingErrorRad) < toleranceRad;
}
}
