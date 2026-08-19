#pragma once

#include "config.hpp"

namespace riku {

void autonExample();

// Characterisation, in dependency order. Run one per attempt and write the
// printed values into config.cpp before moving on: phase 3 drives with the PID
// phase 2 produces, and phase 4 measures the filter phase 3 configures.
void autonCharacterize1Drive();   // trackWidth, kV/kS/kA. No PID needed.
void autonCharacterize2Pid();     // PID, now that trackWidth is right.
void autonCharacterize3Odom();    // wheel scale, odometry drift.
void autonCharacterize4Filters(); // range noise, then localization noise.
void autonSkills();
void autonRedLeft();
void autonBlueLeft();

}  // namespace riku
