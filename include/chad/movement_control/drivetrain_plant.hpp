#pragma once

#include "chad/util/util.hpp"

namespace chad {

// Pose-error dynamics of a differential drive, robot frame.
//   e = [e_x, e_y, e_theta],  u = [dv, domega]
//   A(v) = [0 0 0; 0 0 v; 0 0 0],  B = [1 0; 0 0; 0 1]
// Cross-track reaches the inputs only through v, so gains must follow speed.
struct DrivetrainErrorPlant {
    static constexpr int N = 3;   // states
    static constexpr int M = 2;   // inputs

    /// ZOH over dt, exact: A^2 = 0, so expm(A dt) = I + A dt.
    static void discretise(float v, float dt, float Ad[9], float Bd[6]) {
        Ad[0] = 1.0f; Ad[1] = 0.0f; Ad[2] = 0.0f;
        Ad[3] = 0.0f; Ad[4] = 1.0f; Ad[5] = v * dt;
        Ad[6] = 0.0f; Ad[7] = 0.0f; Ad[8] = 1.0f;

        Bd[0] = dt;   Bd[1] = 0.0f;
        Bd[2] = 0.0f; Bd[3] = 0.5f * v * dt * dt;
        Bd[4] = 0.0f; Bd[5] = dt;
    }
};

}  // namespace chad
