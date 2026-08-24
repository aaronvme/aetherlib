#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "chad/movement_control/arc.hpp"
#include "chad/movement_control/boomerang.hpp"
#include "chad/movement_control/movetopoint.hpp"
#include "chad/movement_control/turntopoint.hpp"
#include "chad/util/settle.hpp"

// The motion loop's exit decision, with no PROS in it, so the tolerance/unit
// mapping and the outcome taxonomy are host-testable. chassis.cpp reads clocks
// and writes voltages; everything here is arithmetic.

namespace chad {

/// The order is load-bearing: telemetry logs the raw int (ChassisSample::type),
/// so inserting a case rewrites every CSV already recorded.
enum class MotionType {
    None, Distance, Point, Heading, TurnPoint, Arc, Boomerang, RAMSETE,
    Trajectory, Swing
};

/// How the last motion ended. A routine that cannot tell "arrived" from "gave
/// up" cannot decide anything about it.
enum class MotionOutcome {
    None,        ///< nothing has run yet, or a motion is still running
    Arrived,     ///< reached what it was asked to reach, within its own residual
    ChainedOut,  ///< inside earlyExitRange; the drive was left running
    TimedOut,
    Stalled,     ///< the drivetrain stopped short of the target
    Cancelled,
    Invalid,     ///< the request itself was unusable (empty path, no trajectory)
};

inline const char* outcomeName(MotionOutcome o) {
    switch (o) {
    case MotionOutcome::None:       return "none";
    case MotionOutcome::Arrived:    return "arrived";
    case MotionOutcome::ChainedOut: return "chained";
    case MotionOutcome::TimedOut:   return "timeout";
    case MotionOutcome::Stalled:    return "stalled";
    case MotionOutcome::Cancelled:  return "cancelled";
    case MotionOutcome::Invalid:    return "invalid";
    }
    return "?";
}

/// RAMSETE and Trajectory keep their parameters in structs that reach pros
/// through FeedforwardGains. These are the only fields the exit test reads, so
/// the request mirrors them here and this header stays host-testable. Written
/// only from the follower's own params, so the two cannot diverge.
struct FollowerSettle {
    float targetTolerance{1.0f};   ///< inches from the final pose
    float stallSpeed{0.0f};        ///< in/s, 0 = use the robot-wide value
    uint32_t settleTimeMs{0};      ///< overrides SettleConfig::settleTimeMs
    /// Trajectory only. False ends the motion when the reference clock does, so
    /// nothing else may gate the exit.
    bool holdAtEnd{true};
};

/// The ChassisConfig subset that decides when a motion is finished.
struct SettleConfig {
    uint32_t settleTimeMs{0};
    float settleSpeedIn{0.0f};
    float settleSpeedDeg{0.0f};
    float stallSpeedIn{0.5f};
    float stallSpeedDeg{2.0f};
    uint32_t stallTimeMs{500};
    uint32_t stallGraceMs{300};
};

/// A motion that rotates in place. Its error, its speed tolerance and its rest
/// threshold are all angular.
inline bool isTurn(MotionType type) {
    return type == MotionType::Heading || type == MotionType::TurnPoint ||
           type == MotionType::Swing;
}

/// The rest test's signal, in the motion's own units: wheel-surface speed for a
/// motion that translates, turn rate for one that rotates. Measured on the
/// wheels either way, never the pose -- a robot rotating in place makes no
/// headway toward a point yet is plainly not stalled.
inline float restSpeed(MotionType type, float leftInPerSec, float rightInPerSec,
                       float trackWidthIn) {
    if (isTurn(type)) {
        return (trackWidthIn > 1e-3f)
                   ? std::fabs((rightInPerSec - leftInPerSec) / trackWidthIn)
                   : 0.0f;
    }
    return std::max(std::fabs(leftInPerSec), std::fabs(rightInPerSec));
}

/// Everything the exit test needs from a queued motion. Chassis::MotionRequest
/// derives from this, so there is one copy of these fields rather than two.
struct MotionSettleParams {
    MotionType type{MotionType::None};
    float timeout{0.0f};               ///< milliseconds, 0 = none
    MoveDistanceParams moveDistParams{};
    MoveToPointParams movePointParams{};
    TurntoHeadingParams turnHeadingParams{};
    TurnToPointParams turnPointParams{};
    ArcParams arcParams{};
    BoomerangParams boomerangParams{};
    SwingParams swingParams{};
    FollowerSettle followerParams{};
};

/// Every motion's exit goes through one SettleMonitor built here, so the
/// tolerance, its units, the hold, the timeout and the chaining range come from
/// one place instead of a switch buried in the exit test.
inline SettleSpec settleSpecFor(const MotionSettleParams& req,
                                const SettleConfig& cfg) {
    SettleSpec spec;
    spec.holdMs = cfg.settleTimeMs;
    spec.timeoutMs = req.timeout > 0.0f ? static_cast<uint32_t>(req.timeout) : 0u;
    spec.speedTolerance = cfg.settleSpeedIn;
    spec.stallSpeed = cfg.stallSpeedIn;
    spec.stallHoldMs = cfg.stallTimeMs;
    spec.graceMs = cfg.stallGraceMs;

    // Turns settle on radians, so their speed tolerance has to be rad/s, and so
    // does their rest threshold: restSpeed() hands a turn the drivetrain's turn
    // rate, not its wheel-surface speed. Applied unchanged, 0.5 in/s of wheel
    // over an 11.9 in track is 4.8 deg/s -- a turn still converging, called
    // stopped, and ended Stalled 500 ms later.
    const float turnSpeedTol = degToRad(cfg.settleSpeedDeg);
    const float turnStall =
        (cfg.stallSpeedIn > 0.0f) ? degToRad(cfg.stallSpeedDeg) : 0.0f;

    switch (req.type) {
    case MotionType::Distance:
        spec.errorTolerance = req.moveDistParams.targetTolerance;
        spec.earlyExitRange = req.moveDistParams.earlyExitRange;
        spec.runFullWindow = req.moveDistParams.holdForTimeout;
        break;
    case MotionType::Point:
        spec.errorTolerance = req.movePointParams.targetTolerance;
        spec.earlyExitRange = req.movePointParams.earlyExitRange;
        break;
    case MotionType::Heading:
        spec.errorTolerance = degToRad(req.turnHeadingParams.targetTolerance);
        spec.speedTolerance = turnSpeedTol;
        spec.stallSpeed = turnStall;
        spec.runFullWindow = req.turnHeadingParams.holdForTimeout;
        break;
    case MotionType::TurnPoint:
        spec.errorTolerance = degToRad(req.turnPointParams.targetTolerance);
        spec.speedTolerance = turnSpeedTol;
        spec.stallSpeed = turnStall;
        break;
    case MotionType::Swing:
        spec.errorTolerance = degToRad(req.swingParams.targetTolerance);
        spec.speedTolerance = turnSpeedTol;
        spec.stallSpeed = turnStall;
        break;
    case MotionType::Arc:
        spec.errorTolerance = req.arcParams.targetTolerance;
        break;
    case MotionType::Boomerang:
        spec.errorTolerance = req.boomerangParams.targetTolerance;
        break;
    case MotionType::RAMSETE:
    case MotionType::Trajectory:
        spec.errorTolerance = req.followerParams.targetTolerance;
        spec.holdMs = req.followerParams.settleTimeMs;
        if (req.followerParams.stallSpeed > 0.0f) {
            spec.stallSpeed = req.followerParams.stallSpeed;
        }
        if (!req.followerParams.holdAtEnd) spec.speedTolerance = 0.0f;
        break;
    case MotionType::None:
        break;
    }

    // A full-window capture with no timeout would never end.
    if (spec.timeoutMs == 0) spec.runFullWindow = false;
    return spec;
}

/// Naming every state means a new one is a compile error here rather than a
/// silent TimedOut.
inline MotionOutcome outcomeFor(SettleState state) {
    switch (state) {
    case SettleState::Settled:   return MotionOutcome::Arrived;
    case SettleState::EarlyExit: return MotionOutcome::ChainedOut;
    case SettleState::Stalled:   return MotionOutcome::Stalled;
    case SettleState::TimedOut:  return MotionOutcome::TimedOut;
    case SettleState::Running:   break;
    }
    return MotionOutcome::None;
}

/// Below the floor the loop is asking for a crawl the drivetrain cannot turn a
/// wheel with. The floor overrides the PID and the slew but never the authority
/// ceiling, so it is applied inside it -- hence the clamp after, not before.
inline float applyOutputFloor(float out, float floor, float limit) {
    if (floor > 0.0f && std::fabs(out) > 1e-3f && std::fabs(out) < floor) {
        out = (out < 0.0f) ? -floor : floor;
    }
    return clamp(out, -limit, limit);
}

/// The floor goes off for good once the motion has arrived, or it kicks the
/// robot back out of tolerance and the pair limit cycles around the target. But
/// one jitter sample touching tolerance is not an arrival, and retiring on it
/// parked the robot outside tolerance with the one mechanism that would have
/// finished it disabled. Two ticks either side of the first, so a single sample
/// cannot do it and the retirement still lands well inside a 110 ms hold.
constexpr uint32_t kFloorRetireMs = 20;

/// Chaining is the one outcome that hands its voltage to the next motion.
/// Everything else -- arrival, timeout, stall, cancel, an unusable request --
/// must leave the drivetrain commanded to zero.
inline bool brakeOnEnd(MotionOutcome o) { return o != MotionOutcome::ChainedOut; }

/// Signed compare, so it survives the counter wrapping past 2^32.
inline bool sequenceSettled(uint32_t completedSeq, uint32_t enqueuedSeq) {
    return static_cast<int32_t>(completedSeq - enqueuedSeq) >= 0;
}

/// The completion counter may never go backwards. cancelMotion() publishes the
/// last id it enqueued, and the motion task then publishes the id of the motion
/// it was running -- which is lower, and would leave isSettled() false until the
/// next motion, locking out driver control and hanging waitUntilSettled().
inline uint32_t publishCompleted(uint32_t current, uint32_t seq) {
    return sequenceSettled(current, seq) ? current : seq;
}

/// How long the motion task may hold a chained-out motion's voltage waiting for
/// the follow-up. The blocking API cannot observe the completion for a poll
/// period and then still has to return through enqueueMotion, so zeroing the
/// instant the queue drains made earlyExitRange a hard stop instead.
constexpr uint32_t kChainGraceMs = 50;

enum class DrainAction { Wait, Stop };

/// A drained queue must never leave voltage on, so the wait is bounded and
/// every other outcome stops at once.
inline DrainAction drainAction(MotionOutcome last, uint32_t waitedMs) {
    return (last == MotionOutcome::ChainedOut && waitedMs < kChainGraceMs)
               ? DrainAction::Wait
               : DrainAction::Stop;
}

}  // namespace chad
