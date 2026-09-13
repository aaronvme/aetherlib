#pragma once

#include <vector>

#include "chad/movement_control/feedforward.hpp"
#include "chad/util/util.hpp"

namespace chad {
int closestPathIndex(const std::vector<Pose>& path, const Pose& pose);

int lookaheadIndex(const std::vector<Pose>& path, const Pose& pose,
                   float lookaheadDist);

struct RAMSETEParams {
    // false drives the path back-first: the reference heading flips and the
    // commanded velocity goes negative.
    bool forwards{true};

    float b{48.0f};
    float zeta{0.85f};

    float smoothSpacingIn{2.0f};
    float maxSpeed{127.0f};

    float minSpeed{12.0f};

    float targetTolerance{-1.0f};

    float headingTolerance{2.0f};

    float stallSpeed{1.5f};

    float lookaheadDist{10.0f};
    bool useVelocityProfile{true};
    float maxAccel{50.0f};

    float maxLateralAccel{60.0f};

    bool useVelocityController{false};

    FeedforwardGains ffGains{.kV = 6.2f, .ksStraight = 0.3f, .kP = 12.0f,
                             .integralClamp = 2.0f};

    uint32_t settleTimeMs{0};
};
}
