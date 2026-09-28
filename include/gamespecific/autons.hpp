#pragma once

#include "config.hpp"

namespace chad {
// Runs a routine written for one alliance on the other. The mirror is cleared
// again on the way out, including when the routine bails early.
void runMirrored(void (*routine)(), FieldMirror mirror);

// Red and blue are a half turn apart on this field.
inline void reflect(void (*routine)()) {
    runMirrored(routine, FieldMirror::Rot180);
}

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
void autonProbeMcl();
void autonProbeGeometry();



//tuffness
void autonSkills();

void autonRed3Pin();
void autonRed3PinFar();
void autonTemp();

void autonBlue3Pin();
void autonBlue3PinFar();
}
