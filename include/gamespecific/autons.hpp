#pragma once

#include "config.hpp"

namespace chad {

void autonExample();

// Tuning, in the order they have to run: each one measures something the next
// depends on. Write the printed values into config.cpp before moving on --
// nothing survives a reboot.
void autonCalibrateImu();         // IMU scale. Everything below reads a heading.
void autonCharacterize1Drive();   // trackWidth, kV/kS/kA.
void autonProbe2Motions();
void autonCharacterize2Pid();
void autonVerifyPid();
void autonCalibrateMounts();      // sensor y offsets. Turns, so needs the PID.
void autonCharacterize3Odom();    // wheel scale, odometry drift.
void autonCharacterize4Filters(); // range noise, then localization noise.
void autonTuneRamsete();          // b and zeta. Needs phase 1's ffGains.

void autonSkills();
void autonRedLeft();
void autonBlueLeft();

}  // namespace chad
