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
    // controller scales b to inches internally. The textbook bâ‰ˆ2.0 is set for
    // FRC-scale paths and tracks a VEX field badly â€” its curves are far tighter
    // relative to the robot, and measured on this drivetrain b=2 leaves 6 in of
    // cross-track where b=48 leaves under 1 in at the same peak speed.
    //
    // Low zeta leaves a curved path off its final tangent with no authority to
    // converge, so the motion sits out its timeout instead of exiting; the exit
    // appears between 0.80 and 0.85. Raise b only on hardware -- the sim has no
    // mass, so it rewards aggression whose cost it cannot show.
    float b{48.0f};      // aggressiveness (higher = tighter tracking), b > 0
    float zeta{0.85f};   // damping ratio, 0 < zeta < 1
    float maxSpeed{127.0f};            // peak output, motor units (0â€“127)
    // Floor on the profiled reference speed, motor units. Must stay above 0 or
    // the motion cannot start; binds within ~minSpeedÂ²/(2Â·maxAccel) of the start.
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
    // Below this the robot has stopped, so the heading it has is the heading it
    // is going to get: exiting beats burning the timeout to no effect. 0 keeps
    // the old behaviour of waiting for headingTolerance or the timeout.
    float stallSpeed{1.5f};            // in/s
    bool useVelocityProfile{true};     // profile v_d for accel/decel along path
    float maxAccel{50.0f};             // in/sÂ², used when useVelocityProfile

    // Caps speed where the path bends, to v <= sqrt(maxLateralAccel/|curvature|).
    // Lets one set of gains cover paths of different tightness: slow into the
    // corner rather than raise b to fight it. Far below the traction limit --
    // it bounds tracking authority, not grip. 0 disables.
    float maxLateralAccel{60.0f};

    // How long the settle condition must hold, overriding
    // ChassisConfig::settleTimeMs for this motion. 0 exits on the first sample.
    //
    // A debounce is there to stop a motion ending while it still carries
    // velocity through the target. RAMSETE cannot do that: the profile has
    // already driven its reference speed to 0 by the last point. What it can do
    // is need position AND heading inside tolerance at the same instant, which
    // on a jittering pose estimate rarely holds for a fixed window -- a 110 ms
    // hold turned a 3.2 s path into a 9 s timeout. So it defaults to 0 here.
    uint32_t settleTimeMs{0};
};

}  // namespace riku
