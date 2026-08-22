#pragma once

#include <vector>

#include "chad/movement_control/drivetrain_plant.hpp"
#include "chad/util/util.hpp"

namespace chad {

/// Feedback gain K (2x3, row-major) for u = -K e.
struct LqrGain {
    float k[6]{};
};

// Diagonal costs. Each is SQUARED into the cost matrices, so these are error
// magnitudes you consider equally bad, not the matrix entries themselves.
// q=(1,1,2) r=(0.6,0.4) was an interior optimum on all five axes across three
// paths; qy=2 and romega=0.15 were both ~35% worse.
struct LqrWeights {
    float qx{1.0f};        // inches
    float qy{1.0f};        // inches
    float qtheta{2.0f};    // radians
    float rv{0.6f};        // in/s
    float romega{0.4f};    // rad/s
};

/// Discrete LQR at reference speed v, by backward Riccati recursion.
/// @param pOut  optional 3x3 row-major cost-to-go, for seeding a finite sweep.
LqrGain solveLqr(float v, float dt, const LqrWeights& w,
                 int maxIters = 600, float tol = 1e-5f, float* pOut = nullptr);

/// One backward Riccati sweep along a trajectory: the gain at point i is solved
/// from the cost-to-go at i+1, so it sees what is still ahead instead of only
/// the speed underneath it.
///
/// @param omega  per-point reference yaw rate, or null to drop the cross terms.
///               Null reproduces EXACTLY the plant LqrGainSchedule assumes, in
///               which A depends on speed alone -- so a null-omega sweep can
///               only anticipate future SPEED, never future curvature.
/// @param out    n gains, out[i] paired with point i
void solveTvlqr(const float* v, const float* omega, int n, float dt,
                const LqrWeights& w, LqrGain* out);

/// Gains over a speed grid, interpolated. The plant depends only on v, so every
/// solve can happen up front and the hot loop cost becomes fixed.
class LqrGainSchedule {
public:
    void build(const LqrWeights& w, float dt, float vMax = 80.0f, int points = 33);
    LqrGain at(float v) const;
    bool ready() const { return !mGains.empty(); }

private:
    std::vector<LqrGain> mGains;
    float mVMax{80.0f};
};

}  // namespace chad
