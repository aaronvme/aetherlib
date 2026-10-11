#pragma once

#include "chad/util/util.hpp"

namespace chad {
enum class TurnDirection {
    CCW,
    CW,
    SHORTEST
};

enum class SwingSide {
    Left,
    Right
};

float resolveTurnError(float rawErrorRad, TurnDirection direction);

float headingToPoint(const Pose& current, float targetX, float targetY);

float headingToPointOffset(const Pose& current, float targetX, float targetY,
                            float offsetDeg);

bool headingWithinTolerance(float headingErrorRad, float toleranceRad);

struct TurntoHeadingParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{-1.0f};
    // degrees of heading error left at which the turn hands off; 0 = off
    float earlyExitRange{0.0f};
};

struct TurnToPointParams {
    // false turns the back of the robot toward the point
    bool forwards{true};
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{-1.0f};
    // degrees of heading error left at which the turn hands off; 0 = off
    float earlyExitRange{0.0f};
    float offset{0.0f};
};

struct SwingParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{-1.0f};
};
}
