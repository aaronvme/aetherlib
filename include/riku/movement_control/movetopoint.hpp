// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include "riku/util/util.hpp"

namespace riku {

// ============================================================================
// Parameter structs
// ============================================================================

struct MoveDistanceParams {
    bool forwards{true};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{1.0f};    // inches
    float earlyExitRange{0.0f};     // exit early if within this range (0 = disabled)
};

struct MoveToPointParams {
    bool forwards{true};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{1.0f};    // inches
    float earlyExitRange{0.0f};
    float lead{0.0f};               // carrot-point lead for curved approaches
};

}  // namespace riku
