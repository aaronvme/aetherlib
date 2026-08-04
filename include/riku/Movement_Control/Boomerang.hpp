// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include "riku/Util/Util.hpp"

namespace riku {

// ============================================================================
// Parameter structs
// ============================================================================

struct BoomerangParams {
    bool forwards{true};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{1.0f};
    float headingTolerance{2.0f};   // VEX degrees
    float lead{0.5f};
    float leadDecay{0.5f};
};

}  // namespace riku
