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

    // Steering stops recomputing the bearing inside this radius and holds the
    // last one: atan2 over a shrinking offset is dominated by whatever noise the
    // pose carries, so under MCL a tuned angular loop chases it at 591 deg/s.
    // Only steering is frozen -- the along-track term stays live, or a stale
    // bearing would strand the robot on a line past the target.
    //
    // Bigger is not better. 1.5 in cuts peak rotation to 82 deg/s for nothing;
    // past it the held bearing goes stale while it still matters and arrivals
    // from a bad angle stop converging, 3 of 15 at 1.5 in against 12 of 15 at 6.
    float bearingHoldDist{1.5f};    // inches
};

}  // namespace riku
