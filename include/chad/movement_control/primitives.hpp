#pragma once

#include "chad/movement_control/arc.hpp"
#include "chad/movement_control/boomerang.hpp"
#include "chad/movement_control/movetopoint.hpp"
#include "chad/movement_control/turntopoint.hpp"
#include "chad/util/util.hpp"

namespace chad {

/// What a primitive wants this tick, before gains, limits and mixing. Splitting
/// it out this way is what makes the geometry testable off-robot: the motion
/// loop reads clocks and writes voltages, and everything here is arithmetic.
struct MotionDemand {
    /// Signed along-track error, inches, forward positive. Signed so an
    /// overshoot backs up instead of turning around.
    float lateral{0.0f};
    /// Signed heading error, radians, CCW positive.
    float angular{0.0f};
    /// What decides arrival, in the motion's own units.
    float settleError{0.0f};
    /// Multiplies the lateral output.
    float lateralScale{1.0f};
    /// The primary criterion met on its own terms, where `settleError` folds in
    /// a secondary one: a boomerang inside its position tolerance but off its
    /// final heading is atTarget. Stopping here is arrival, not a stall.
    bool atTarget{false};
};

/// Per-side motor command, -127..127.
struct SidePair {
    float left{0.0f};
    float right{0.0f};
};

/// angularOut is CCW-positive and CCW needs the RIGHT side faster. Adding it to
/// the left instead makes the heading loop positive feedback, stable at
/// target ± 180 degrees.
SidePair tankMix(float lateral, float angular);

/// A swing pivots about one stationary side.
SidePair swingMix(float angular, SwingSide side);

/// Position and heading both inside their own tolerance, folded into one error
/// against `posTolerance` so a single scalar settle test covers both.
float dualSettleError(float posError, float posTolerance, float headingErrorRad,
                      float headingToleranceRad);

// ============================================================================
// moveDistance
// ============================================================================

/// Absolute field target for a relative drive, plus the axis it is measured on.
struct StraightTarget {
    float x{0.0f};
    float y{0.0f};
    float axisRad{0.0f};
};

/// Resolve once, at the start of execution. Recomputing per tick from the live
/// pose moves the target with the robot and the error never shrinks.
StraightTarget resolveStraightTarget(const Pose& start, float distance, bool forwards);

MotionDemand straightDemand(const Pose& pose, const StraightTarget& target);

// ============================================================================
// moveToPoint
// ============================================================================

/// Steering stops recomputing the bearing near the target and holds the last
/// one. See MoveToPointParams::bearingHoldDist.
struct BearingHold {
    bool held{false};
    float bearing{0.0f};
};

/// The hold releases only if the target is both far away again and back in
/// front of the direction of TRAVEL: a pose jump can latch a garbage bearing,
/// but an overshoot must keep the stale one, or the live bearing (now 180
/// degrees off) spins the robot.
constexpr float kBearingHoldReleaseFactor = 3.0f;

MotionDemand pointDemand(const Pose& pose, float targetX, float targetY, bool forwards,
                         float bearingHoldDist, BearingHold& hold);

// ============================================================================
// Turns
// ============================================================================

/// Latches once a forced-direction turn has travelled the direction it was
/// asked for.
struct TurnLatch {
    bool satisfied{false};
};

/// Signed heading error honouring CW/CCW, released to the shortest arc once the
/// target has been reached once.
float resolvedTurnError(float rawErrRad, TurnDirection direction, float toleranceRad,
                        TurnLatch& latch);

// ============================================================================
// moveArc
// ============================================================================

/// How far past the target an arc may read as a back-up, in radians of sweep.
/// A single-valued sweep must be cut somewhere on the circle; this puts the cut
/// just behind the robot, where it only decides how far an overshoot reverses
/// before going round again. wrapRad's +-pi cut sat mid-arc instead, and 0.24 in
/// of target movement reversed a 37 in travel command.
constexpr float kArcBackSweep = 0.5f;

/// Remaining travel is the signed sweep around the arc's own centre, not the
/// chord: the chord projected on the heading collapses to zero at a half-circle,
/// where the robot is pointing across the chord and still has half an arc left.
/// Continuous through the half-circle, so an arc may need more than a half turn.
/// @param radius  signed; positive puts the centre left of the direction of
///                travel, so it mirrors for a reverse arc
MotionDemand arcDemand(const Pose& pose, float targetX, float targetY, float radius,
                       bool forwards);

// ============================================================================
// moveBoomerang
// ============================================================================

MotionDemand boomerangDemand(const Pose& pose, float targetX, float targetY,
                             float targetThetaRad, const BoomerangParams& params);

}  // namespace chad
