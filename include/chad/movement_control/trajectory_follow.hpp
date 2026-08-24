#pragma once

#include <cstdint>

#include "chad/movement_control/feedforward.hpp"
#include "chad/movement_control/trajectory.hpp"

namespace chad {

/// Following a pre-parameterized trajectory, as opposed to `moveRAMSETE`, which
/// profiles a bare geometric path as it goes. The reference here already knows
/// its own speed, acceleration and curvature at every instant, so nothing has
/// to be inferred from finite differences and the constraints that shaped it
/// (grip, wheel budget, voltage) are already respected.
struct TrajectoryFollowParams {
    /// RAMSETE aggressiveness, in the conventional metre parameterization;
    /// scaled to inches internally. See RAMSETEParams for why the textbook
    /// b = 2 tracks a VEX-scale path badly.
    float b{48.0f};
    float zeta{0.85f};

    /// Output ceiling, motor units. This clips the FOLLOWER, not the reference:
    /// the trajectory was generated against the whole wheel budget, so a
    /// ceiling below it saturates every tick and the path is simply not tracked.
    /// On a 44 in diagonal, 127 ends 0.24 in out, 80 ends 1.7 in out, 40 ends
    /// 8.3 in and 20 ends 14.4 in -- and all of it perpendicular, which the
    /// hold cannot close. Use `timeScale` to drive the same geometry slower.
    float maxSpeed{127.0f};
    /// Inches from the final pose. Not 1.0: the follower already lands inside
    /// 0.79 in, so at 1.0 `holdAtEnd` never engages at all. Swept over ten
    /// approach geometries, hold time against the error it leaves:
    /// 1.0 -> 0 ms / 0.79 in, 0.5 -> 240 ms / 0.47, 0.25 -> 440 ms / 0.24,
    /// 0.1 -> 715 ms / 0.09, and 0.05 or tighter reports Stalled at 0.07 in on
    /// all ten because the drivetrain drops below `stallSpeed` first.
    float targetTolerance{0.25f};
    /// VEX degrees at the final pose. Never the binding term in that sweep --
    /// 0.5 through 8 deg gave byte-identical hold times and errors, because the
    /// follower lands within 0.1 deg. Left where it was rather than tuned.
    float headingTolerance{4.0f};

    /// Below this the robot has stopped, so the pose it has is the pose it is
    /// going to get. 0 waits for the tolerances or the timeout. A flat trade:
    /// 0 and 0.5 hold 150 ms longer on a perpendicular residual and finish
    /// 0.10 in better along the axis, 1.5 and 3.0 are indistinguishable.
    float stallSpeed{1.5f};          ///< in/s

    /// Close the loop on measured wheel speed instead of treating voltage as
    /// speed. On by default here: a trajectory's whole value is that its speeds
    /// mean something, which open-loop scaling against free speed throws away.
    bool useVelocityController{true};
    FeedforwardGains ffGains{.kV = 6.2f, .ksStraight = 0.3f, .kP = 12.0f,
                             .integralClamp = 2.0f};

    /// Stretch the reference in time. Above 1 drives the same geometry slower
    /// without regenerating, which is the cheap way to trade time for accuracy.
    float timeScale{1.0f};

    /// Keep correcting after the reference runs out, until the tolerances are
    /// met or the motion times out. False exits the instant the clock does.
    bool holdAtEnd{true};

    /// How long the tolerance must hold. Zero exits on the first sample; the
    /// hold keeps driving while the window runs, so 110 ms takes the final
    /// error from 0.24 to 0.16 in and 200 ms takes it to 0.12. That is a worse
    /// rate than tightening `targetTolerance`, which is why this stays at 0.
    uint32_t settleTimeMs{0};
};

}  // namespace chad
