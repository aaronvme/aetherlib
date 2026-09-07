#pragma once

#include "chad/util/util.hpp"

namespace chad {
struct MoveDistanceParams {
    bool forwards{true};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};

    float targetTolerance{-1.0f};
    float earlyExitRange{0.0f};
};

struct MoveToPointParams {
    bool forwards{true};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};

    float targetTolerance{-1.0f};
    float earlyExitRange{0.0f};
    float lead{0.0f};

    float bearingHoldDist{-1.0f};
};
}
