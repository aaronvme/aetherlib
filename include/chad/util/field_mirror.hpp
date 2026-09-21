#pragma once

// util.hpp and geometry.hpp define the same names; this side of the
// library uses util.hpp.
#include "chad/util/util.hpp"

namespace chad {
// Field symmetries. Rot180 is a proper rotation and leaves handedness alone;
// the three reflections turn a left swing into a right one, so swing sides,
// CW/CCW and arc radii have to flip with them.
//
// Every one of the four is its own inverse, so the same call reads a pose
// back out of the mirrored frame that put it in.
enum class FieldMirror {
    None,
    Rot180,     // x -> -x, y -> -y
    FlipX,      // x -> -x, across the y axis
    FlipY,      // y -> -y, across the x axis
    Diagonal    // (x, y) -> (y, x), across y = x
};

inline bool mirrorFlipsHandedness(FieldMirror m) {
    return m == FieldMirror::FlipX || m == FieldMirror::FlipY ||
           m == FieldMirror::Diagonal;
}

inline void mirrorPointInPlace(FieldMirror m, float& x, float& y) {
    switch (m) {
        case FieldMirror::Rot180:   x = -x; y = -y; break;
        case FieldMirror::FlipX:    x = -x;         break;
        case FieldMirror::FlipY:            y = -y; break;
        case FieldMirror::Diagonal: { const float t = x; x = y; y = t; } break;
        case FieldMirror::None:                     break;
    }
}

inline float mirrorHeadingRad(FieldMirror m, float theta) {
    switch (m) {
        case FieldMirror::Rot180:   return wrapRad(theta + PI);
        case FieldMirror::FlipX:    return wrapRad(PI - theta);
        case FieldMirror::FlipY:    return wrapRad(-theta);
        case FieldMirror::Diagonal: return wrapRad(HALF_PI - theta);
        case FieldMirror::None:     break;
    }
    return theta;
}

inline Pose mirrorPose(FieldMirror m, const Pose& p) {
    if (m == FieldMirror::None) return p;
    Pose out = p;
    mirrorPointInPlace(m, out.x, out.y);
    out.theta = mirrorHeadingRad(m, p.theta);
    return out;
}
}
