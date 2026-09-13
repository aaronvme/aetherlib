#pragma once

#include "config.hpp"

namespace chad {
void autonExample();

//calibrating
void autonCalibrateImu();
void autonCharacterize1Drive();
void autonProbe2Motions();
void autonCharacterize2Pid();
void autonVerifyPid();
void autonCalibrateMounts();
void autonCharacterize3Odom();
void autonCharacterize4Filters();
void autonCalibrateLocalization();
void autonCalibrateOdomBias();
void autonTuneRamsete();
void autonCalibrateLift();
void autonTuneLiftPresets();



//tuffness
void autonSkills();
void autonRedLeft();
void autonBlueLeft();
}
