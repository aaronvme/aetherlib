// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include "chad/util/util.hpp"

namespace chad {

// ============================================================================
// Parameter structs
// ============================================================================

struct BoomerangParams {
    /// Back into the target pose. The carrot moves to the far side of the
    /// target, which is the side a reversing robot approaches from.
    bool forwards{true};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    /// Inches. Position AND heading must both be inside tolerance to settle, so
    /// a tight headingTolerance is what makes a boomerang sit out its timeout.
    float targetTolerance{1.0f};
    float headingTolerance{2.0f};   // VEX degrees
    /// Carrot offset behind the target, as a fraction of the distance to it.
    float lead{0.5f};
    /// Winds the offset down over the last `lead * 12` inches.
    float leadDecay{0.5f};
};

}  // namespace chad
