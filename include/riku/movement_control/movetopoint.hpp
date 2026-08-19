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

    // Inside this radius the bearing stops being recomputed and the last one is
    // held: atan2 over a shrinking offset is dominated by whatever noise the
    // pose carries, so under MCL a tuned angular loop chases it at 591 deg/s.
    // The sweep flattens at 6 in -- 29 deg/s against a 24 deg/s floor -- and
    // costs 0.05 in of endpoint accuracy. 0 disables the hold.
    float bearingHoldDist{6.0f};    // inches
};

}  // namespace riku
