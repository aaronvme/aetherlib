#pragma once

#include <cmath>

namespace chad::tuning {

/// The component of a sensor mount offset that a heading sweep can identify:
/// the distance from the robot origin to the beam line, positive to the left of
/// the beam. It is `mountOffset.y` only for a forward-pointing sensor.
inline float acrossBeamOffset(float mountX, float mountY, float mountThetaRad) {
    return -mountX * std::sin(mountThetaRad) + mountY * std::cos(mountThetaRad);
}

/// Writes a measured across-beam offset back into a mount pose, keeping the
/// along-beam arm the tape measure set.
///
/// The sweep cannot see the along-beam arm at all -- moving a sensor along its
/// own beam changes every reading by the same amount, which the fit absorbs into
/// its intercept. So a measurement can only replace the across-beam component,
/// and assigning it to `.y` corrupts every mount that does not point forward: at
/// theta = 90 degrees the sweep measures -x, so a mount at (0.50, 6.25) reports
/// -0.50 and writing that to `.y` moves the sensor 6.75 in.
inline void applyMountOffset(float mountThetaRad, float offsetIn, float* mountX,
                             float* mountY) {
    if (mountX == nullptr || mountY == nullptr) return;
    const float c = std::cos(mountThetaRad);
    const float s = std::sin(mountThetaRad);
    const float along = *mountX * c + *mountY * s;
    *mountX = along * c - offsetIn * s;
    *mountY = along * s + offsetIn * c;
}

}  // namespace chad::tuning
