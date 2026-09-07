#pragma once

#include <cmath>

namespace chad::tuning {
inline float acrossBeamOffset(float mountX, float mountY, float mountThetaRad) {
    return -mountX * std::sin(mountThetaRad) + mountY * std::cos(mountThetaRad);
}

inline void applyMountOffset(float mountThetaRad, float offsetIn, float* mountX,
                             float* mountY) {
    if (mountX == nullptr || mountY == nullptr) return;
    const float c = std::cos(mountThetaRad);
    const float s = std::sin(mountThetaRad);
    const float along = *mountX * c + *mountY * s;
    *mountX = along * c - offsetIn * s;
    *mountY = along * s + offsetIn * c;
}
}
