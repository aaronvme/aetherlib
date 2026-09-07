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
};

struct TurnToPointParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{-1.0f};
    float offset{0.0f};
};

struct SwingParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{-1.0f};
};
}
