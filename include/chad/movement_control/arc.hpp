#pragma once

#include "chad/util/util.hpp"

namespace chad {
Pose computeArcCenter(const Pose& current, float radius);

float chordDistance(const Pose& current, float targetX, float targetY);

float arcAngle(const Pose& current, float targetX, float targetY, float radius);

float arcEndHeading(const Pose& current, float targetX, float targetY,
                    float radius);

float arcStartHeading(const Pose& current, float targetX, float targetY,
                      float radius);

struct ArcParams {
    bool forwards{true};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{-1.0f};
    float lead{0.0f};
};
}
