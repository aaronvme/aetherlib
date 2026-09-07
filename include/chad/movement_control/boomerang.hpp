#pragma once

#include "chad/util/util.hpp"

namespace chad {
struct BoomerangParams {
    bool forwards{true};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{-1.0f};
    float headingTolerance{2.0f};
    float lead{0.5f};
    float leadDecay{0.5f};
};
}
