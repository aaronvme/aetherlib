#pragma once

#include <vector>

#include "chad/movement_control/drivetrain_plant.hpp"
#include "chad/util/util.hpp"

namespace chad {
struct LqrGain {
    float k[6]{};
};

struct LqrWeights {
    float qx{1.0f};
    float qy{1.0f};
    float qtheta{2.0f};
    float rv{0.6f};
    float romega{0.4f};
};

LqrGain solveLqr(float v, float dt, const LqrWeights& w,
                 int maxIters = 600, float tol = 1e-5f);

class LqrGainSchedule {
public:
    void build(const LqrWeights& w, float dt, float vMax = 80.0f, int points = 33);
    LqrGain at(float v) const;
    bool ready() const { return !mGains.empty(); }

private:
    std::vector<LqrGain> mGains;
    float mVMax{80.0f};
};
}
