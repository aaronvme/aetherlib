#pragma once

#include <cmath>

namespace chad::tuning {
inline float acrossBeamOffset(float mountX, float mountY, float mountThetaRad) {
    return -mountX * std::sin(mountThetaRad) + mountY * std::cos(mountThetaRad);
}

// Place a mount from its beam-frame components: alongIn down the beam,
// acrossIn to the left of it.
inline void setMountFromBeam(float mountThetaRad, float alongIn, float acrossIn,
                             float* mountX, float* mountY) {
    if (mountX == nullptr || mountY == nullptr) return;
    const float c = std::cos(mountThetaRad);
    const float s = std::sin(mountThetaRad);
    *mountX = alongIn * c - acrossIn * s;
    *mountY = alongIn * s + acrossIn * c;
}

// The sweep fit sees across but not along, so this keeps along as it was.
inline void applyMountOffset(float mountThetaRad, float offsetIn, float* mountX,
                             float* mountY) {
    if (mountX == nullptr || mountY == nullptr) return;
    const float c = std::cos(mountThetaRad);
    const float s = std::sin(mountThetaRad);
    const float along = *mountX * c + *mountY * s;
    setMountFromBeam(mountThetaRad, along, offsetIn, mountX, mountY);
}
}
