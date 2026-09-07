#pragma once

#include "chad/util/util.hpp"

namespace chad {
struct DrivetrainErrorPlant {
    static constexpr int N = 3;
    static constexpr int M = 2;

    static void discretise(float v, float dt, float Ad[9], float Bd[6]) {
        Ad[0] = 1.0f; Ad[1] = 0.0f; Ad[2] = 0.0f;
        Ad[3] = 0.0f; Ad[4] = 1.0f; Ad[5] = v * dt;
        Ad[6] = 0.0f; Ad[7] = 0.0f; Ad[8] = 1.0f;

        Bd[0] = dt;   Bd[1] = 0.0f;
        Bd[2] = 0.0f; Bd[3] = 0.5f * v * dt * dt;
        Bd[4] = 0.0f; Bd[5] = dt;
    }
};
}
