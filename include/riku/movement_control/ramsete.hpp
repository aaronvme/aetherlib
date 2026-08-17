// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include <vector>

#include "riku/util/util.hpp"

namespace riku {

// ============================================================================
// RAMSETE utility functions
// ============================================================================

/// Find the closest point index on a path to a given pose
int closestPathIndex(const std::vector<Pose>& path, const Pose& pose);

/// Find the lookahead point index on a path (first point beyond lookaheadDist)
int lookaheadIndex(const std::vector<Pose>& path, const Pose& pose,
                   float lookaheadDist);

// ============================================================================
// Parameter structs
// ============================================================================

struct RAMSETEParams {
    // b and zeta use the conventional (meter-based) parameterization; the
    // controller scales b to inches internally. The textbook b≈2.0 is set for
    // FRC-scale paths and tracks a VEX field badly — its curves are far tighter
    // relative to the robot, and measured on this drivetrain b=2 leaves 6 in of
    // cross-track where b=48 leaves under 1 in at the same peak speed. Error
    // flattens out past ~48 and stays stable well beyond it.
    float b{48.0f};      // aggressiveness (higher = tighter tracking), b > 0
    float zeta{0.7f};    // damping ratio, 0 < zeta < 1
    float maxSpeed{127.0f};            // peak output, motor units (0–127)
    // Floor on the profiled reference speed, motor units. Must stay above 0 or
    // the motion cannot start; binds within ~minSpeed²/(2·maxAccel) of the start.
    float minSpeed{12.0f};
    float targetTolerance{1.0f};       // inches
    // Curved paths finish 5-12 deg off the final tangent and the law has no
    // authority left to fix it (every term scales with v_d, which the profile
    // has driven to 0), so a tight value here means the motion runs out its
    // timeout. Widen it, or follow the path with a turnToHeading.
    float headingTolerance{2.0f};      // VEX degrees
    // No longer read by the controller: RAMSETE's reference is the robot's
    // projection onto the path, and offsetting it forward left a standing e_x
    // that inflated every commanded speed by roughly half. Kept so existing
    // calls still compile.
    float lookaheadDist{10.0f};
    bool useVelocityProfile{true};     // profile v_d for accel/decel along path
    float maxAccel{50.0f};             // in/s², used when useVelocityProfile
};

}  // namespace riku
