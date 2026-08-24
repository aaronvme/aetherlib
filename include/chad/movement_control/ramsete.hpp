// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include <vector>

#include "chad/movement_control/feedforward.hpp"
#include "chad/util/util.hpp"

namespace chad {

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
    // b is rad^2/m^2 -- the conventional metre parameterization -- and
    // chassis.cpp multiplies it by INCH_TO_METER^2 before use. The textbook
    // b = 2 is set for FRC-scale paths and tracks a VEX field badly: swept over
    // three paths at the sim's 70 ms response lag, peak cross-track runs
    // b=2: 3.4-12.5 in, b=12: 1.6-5.0, b=48: 1.2-2.5, b=144: 1.1-1.5, and
    // settle time 4.2-5.1 s at b=2 against 2.0-2.9 s at b=48. Past b=48 the
    // curve is flat -- b=96 buys under 0.5 in -- and the sim has no mass, so it
    // cannot price the aggression it is rewarding. Hence 48, not higher.
    //
    // zeta 0.70 is the textbook value and is measurably worse here: at a 140 ms
    // response lag it overshoots the path end by 1.4-2.4 in against 0.6-0.8 in
    // at 0.85, and it leaves 2.7 in of final error where 0.85 leaves 0.9.
    //
    // Neither gain governs the overshoot itself. Every term of the law scales
    // with the reference speed and a profile ends at rest, so the last of a
    // motion is an open-loop coast: 0.00 in past the end at a 20 ms response
    // lag, 0.5 in at 70 ms, 2.1 in at 140 ms, 3.5 in at 200 ms, moving under
    // 0.35 in across b = 12..96 and zeta = 0.70..0.95. Shorten it with
    // maxAccel or useVelocityController, not with b.
    float b{48.0f};      // aggressiveness (higher = tighter tracking), b > 0
    float zeta{0.85f};   // damping ratio, 0 < zeta < 1
    float maxSpeed{127.0f};            // peak output, motor units (0–127)
    // Floor on the profiled reference speed, motor units. Must stay above 0 or
    // the motion cannot start; binds within ~minSpeed²/(2·maxAccel) of the start.
    float minSpeed{12.0f};
    // Must exceed the localization error or the motion never exits: under MCL,
    // 1 in against a 1.5 in estimate error timed out 3/3, and 3 in passed. So
    // this does NOT default to ChassisConfig::targetToleranceIn, which is set
    // for a straight drive and is 1 in on this robot -- the value measured to
    // fail. Set it to -1 to opt into the config value anyway.
    float targetTolerance{3.0f};       // inches, negative = ChassisConfig
    // Curved paths finish 5-12 deg off the final tangent and the law has no
    // authority left to fix it (every term scales with v_d, which the profile
    // has driven to 0), so a tight value here means the motion runs out its
    // timeout. Widen it, or follow the path with a turnToHeading.
    float headingTolerance{2.0f};      // VEX degrees
    // Below this the robot has stopped, so the heading it has is the heading it
    // is going to get: exiting beats burning the timeout to no effect. 0 keeps
    // the old behaviour of waiting for headingTolerance or the timeout.
    float stallSpeed{1.5f};            // in/s
    // No longer read by the controller: RAMSETE's reference is the robot's
    // projection onto the path, and offsetting it forward left a standing e_x
    // that inflated every commanded speed by roughly half. Kept so existing
    // calls still compile.
    float lookaheadDist{10.0f};
    bool useVelocityProfile{true};     // profile v_d for accel/decel along path
    float maxAccel{50.0f};             // in/s², used when useVelocityProfile

    // Caps speed where the path bends, to v <= sqrt(maxLateralAccel/|curvature|).
    // Lets one set of gains cover paths of different tightness: slow into the
    // corner rather than raise b to fight it. Far below the traction limit --
    // it bounds tracking authority, not grip. 0 disables.
    float maxLateralAccel{60.0f};

    // Open loop, RAMSETE's per-side speed becomes voltage by scaling against
    // the drivetrain's free speed -- true only unloaded, and wrong by more the
    // harder the path pushes. Closing the loop on measured wheel speed instead
    // is worth more on a curved path than any gain change.
    bool useVelocityController{false};
    // Fitted against the simulator, whose motor model is a first-order lag with
    // no load: refit on hardware before trusting them. kP is the sensitive one
    // -- it trades against motor response time, and 12 is the only value that
    // held over a 20-200 ms lag sweep on both test paths.
    // integralClamp is set even though kI is 0: "<= 0 disables" means a user who
    // sets only kI gets no gate and no clamp, and a drivetrain held short of its
    // target -- against a wall, against another robot -- winds up without bound.
    FeedforwardGains ffGains{.kV = 6.2f, .ksStraight = 0.3f, .kP = 12.0f,
                             .integralClamp = 2.0f};

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

}  // namespace chad
