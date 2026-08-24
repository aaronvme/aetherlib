// Motion primitives: the decision each one makes given a pose and a target.
// Named *settle* so `run.ps1 -Tests settle` builds it alongside settle_test.cpp.
#include "chad/movement_control/primitives.hpp"

#include "chad_test.hpp"

using namespace chad;

namespace {

/// The mix the drivetrain actually receives, for asserting sign conventions.
float turnRate(const SidePair& p) { return p.right - p.left; }

BoomerangParams boomParams() {
    BoomerangParams p;
    p.lead = 0.5f;
    p.leadDecay = 0.5f;
    p.maxSpeed = 127.0f;
    p.minSpeed = 0.0f;
    p.targetTolerance = 1.0f;
    p.headingTolerance = 2.0f;
    return p;
}

}  // namespace

// ============================================================================
// Output mixing: the sign convention a whole class of "tuning problems" hides in
// ============================================================================

CHAD_TEST(primitives, ccwNeedsTheRightSideFaster) {
    // angularOut is CCW-positive. Adding it to the left instead makes the
    // heading loop positive feedback, stable at target +- 180 degrees.
    const SidePair ccw = tankMix(0.0f, 40.0f);
    CHECK(ccw.right > ccw.left);
    CHECK(turnRate(ccw) > 0.0f);
    const SidePair cw = tankMix(0.0f, -40.0f);
    CHECK(turnRate(cw) < 0.0f);
}

CHAD_TEST(primitives, tankMixIsStraightWithNoAngular) {
    const SidePair p = tankMix(60.0f, 0.0f);
    CHECK_NEAR(p.left, 60.0f, 1e-4);
    CHECK_NEAR(p.right, 60.0f, 1e-4);
}

CHAD_TEST(primitives, tankMixClampsEachSide) {
    const SidePair p = tankMix(120.0f, 60.0f);
    CHECK_NEAR(p.left, 60.0f, 1e-4);
    CHECK_NEAR(p.right, 127.0f, 1e-4);
}

CHAD_TEST(primitives, swingTurnsTheSameWayAsATankMix) {
    // Left side held: CCW comes from driving the right side forward. Right side
    // held: CCW needs the left side backward. Either way the sign of the turn
    // must agree with tankMix or a swing turns the wrong way.
    for (float angular : {40.0f, -40.0f}) {
        const SidePair left = swingMix(angular, SwingSide::Left);
        const SidePair right = swingMix(angular, SwingSide::Right);
        CHECK_EQ(left.left, 0.0f);
        CHECK_EQ(right.right, 0.0f);
        CHECK(sign(turnRate(left)) == sign(turnRate(tankMix(0.0f, angular))));
        CHECK(sign(turnRate(right)) == sign(turnRate(tankMix(0.0f, angular))));
    }
}

CHAD_TEST(primitives, swingClampsToMotorRange) {
    CHECK_NEAR(swingMix(400.0f, SwingSide::Left).right, 127.0f, 1e-4);
    CHECK_NEAR(swingMix(400.0f, SwingSide::Right).left, -127.0f, 1e-4);
}

// ============================================================================
// dualSettleError: position AND heading, folded into one scalar
// ============================================================================

CHAD_TEST(primitives, dualSettleErrorIsExactlyTheConjunction) {
    const float posTol = 1.5f;
    const float headTol = degToRad(2.0f);
    // Both inside.
    CHECK(dualSettleError(1.0f, posTol, degToRad(1.0f), headTol) <= posTol);
    // Position inside, heading out.
    CHECK(dualSettleError(1.0f, posTol, degToRad(3.0f), headTol) > posTol);
    // Heading inside, position out.
    CHECK(dualSettleError(2.0f, posTol, degToRad(1.0f), headTol) > posTol);
    // Neither.
    CHECK(dualSettleError(9.0f, posTol, degToRad(30.0f), headTol) > posTol);
}

CHAD_TEST(primitives, dualSettleErrorIgnoresAnUnsetHeadingTolerance) {
    CHECK_NEAR(dualSettleError(2.0f, 1.5f, degToRad(90.0f), 0.0f), 2.0f, 1e-4);
}

// ============================================================================
// moveDistance
// ============================================================================

CHAD_TEST(primitives, straightTargetIsResolvedInTheStartHeading) {
    // A queued Distance after a turn must aim along the heading it inherits, not
    // along +X.
    const Pose afterTurn{10.0f, 5.0f, HALF_PI};
    const StraightTarget t = resolveStraightTarget(afterTurn, 24.0f, true);
    CHECK_NEAR(t.x, 10.0f, 1e-3);
    CHECK_NEAR(t.y, 29.0f, 1e-3);
    CHECK_NEAR(t.axisRad, HALF_PI, 1e-6);

    const StraightTarget back = resolveStraightTarget(afterTurn, 24.0f, false);
    CHECK_NEAR(back.y, -19.0f, 1e-3);
    CHECK_NEAR(back.axisRad, HALF_PI, 1e-6);
}

CHAD_TEST(primitives, straightErrorShrinksAndThenGoesNegative) {
    const Pose start{0.0f, 0.0f, 0.0f};
    const StraightTarget t = resolveStraightTarget(start, 24.0f, true);
    CHECK_NEAR(straightDemand(start, t).lateral, 24.0f, 1e-3);
    CHECK_NEAR(straightDemand({12.0f, 0.0f, 0.0f}, t).lateral, 12.0f, 1e-3);
    // Overshoot backs up rather than turning around.
    const MotionDemand past = straightDemand({30.0f, 0.0f, 0.0f}, t);
    CHECK_NEAR(past.lateral, -6.0f, 1e-3);
    CHECK_NEAR(past.angular, 0.0f, 1e-6);
}

CHAD_TEST(primitives, straightIgnoresCrossTrackOffset) {
    // Projection on the fixed axis: sliding sideways must not change the
    // along-track error, or the loop drives further to make up a lateral offset.
    const StraightTarget t = resolveStraightTarget({0.0f, 0.0f, 0.0f}, 24.0f, true);
    CHECK_NEAR(straightDemand({12.0f, 8.0f, 0.0f}, t).lateral, 12.0f, 1e-3);
}

CHAD_TEST(primitives, straightHoldsTheStartAxisWhenReversing) {
    // Reverse keeps the heading; the sign of the along-track error sends it back.
    const StraightTarget t = resolveStraightTarget({0.0f, 0.0f, 0.0f}, 24.0f, false);
    const MotionDemand d = straightDemand({0.0f, 0.0f, 0.0f}, t);
    CHECK_NEAR(d.lateral, -24.0f, 1e-3);
    CHECK_NEAR(d.angular, 0.0f, 1e-6);
}

// ============================================================================
// moveToPoint
// ============================================================================

CHAD_TEST(primitives, pointAlongTrackReversesOnOvershoot) {
    BearingHold hold;
    const MotionDemand ahead = pointDemand({0.0f, 0.0f, 0.0f}, 24.0f, 0.0f, true, 1.0f, hold);
    CHECK_NEAR(ahead.lateral, 24.0f, 1e-3);
    CHECK_NEAR(ahead.settleError, 24.0f, 1e-3);

    BearingHold hold2;
    const MotionDemand past = pointDemand({30.0f, 0.0f, 0.0f}, 24.0f, 0.0f, true, 1.0f, hold2);
    CHECK_NEAR(past.lateral, -6.0f, 1e-3);
    CHECK_NEAR(past.settleError, 6.0f, 1e-3);
}

CHAD_TEST(primitives, pointSteersTowardTheTarget) {
    BearingHold hold;
    // Target 90 degrees to the left: CCW-positive error.
    const MotionDemand d = pointDemand({0.0f, 0.0f, 0.0f}, 0.0f, 24.0f, true, 1.0f, hold);
    CHECK_NEAR(d.angular, HALF_PI, 1e-4);
    CHECK_NEAR(d.lateral, 0.0f, 1e-3);   // nothing to gain by driving yet
}

CHAD_TEST(primitives, pointReversesIntoTheTarget) {
    BearingHold hold;
    // Facing away from a target 24 in behind: already lined up for a reverse.
    const MotionDemand d =
        pointDemand({0.0f, 0.0f, PI}, 24.0f, 0.0f, false, 1.0f, hold);
    CHECK_NEAR(d.angular, 0.0f, 1e-4);
    CHECK_NEAR(d.lateral, -24.0f, 1e-3);
}

CHAD_TEST(primitives, bearingHoldLatchesInsideTheRadius) {
    BearingHold hold;
    pointDemand({23.5f, 0.0f, 0.0f}, 24.0f, 0.0f, true, 1.0f, hold);
    CHECK(hold.held);
    CHECK_NEAR(hold.bearing, 0.0f, 1e-4);

    // A pose that jitters sideways at 0.4 in would swing the live bearing by
    // ~39 degrees; the held one does not move.
    const MotionDemand d =
        pointDemand({23.5f, 0.4f, 0.0f}, 24.0f, 0.0f, true, 1.0f, hold);
    CHECK_NEAR(d.angular, 0.0f, 1e-4);
}

CHAD_TEST(primitives, bearingHoldSurvivesAnOvershoot) {
    BearingHold hold;
    pointDemand({23.5f, 0.0f, 0.0f}, 24.0f, 0.0f, true, 1.0f, hold);
    CHECK(hold.held);
    // 6 in past the target: the live bearing is now 180 degrees off, and
    // steering to it would spin the robot instead of backing it up.
    const MotionDemand d =
        pointDemand({30.0f, 0.0f, 0.0f}, 24.0f, 0.0f, true, 1.0f, hold);
    CHECK(hold.held);
    CHECK_NEAR(d.angular, 0.0f, 1e-4);
    CHECK(d.lateral < 0.0f);
}

CHAD_TEST(primitives, bearingHoldReleasesAfterAReversePoseJumpToo) {
    BearingHold hold;
    // Backing into the origin from 0.9 in away, so the bearing latches pointing
    // at the target while the nose (facing +X) is pi away from it.
    pointDemand({0.9f, 0.0f, 0.0f}, 0.0f, 0.0f, false, 1.0f, hold);
    CHECK(hold.held);
    // MCL corrects to 30 in away and off to one side. "Back in front" has to
    // mean in front of the direction of TRAVEL: measured on the forward bearing
    // a reversing robot never satisfies it, and keeps a garbage bearing for the
    // rest of the motion while both axes go quiet 30 in short.
    const MotionDemand d =
        pointDemand({2.0f, 30.0f, 0.0f}, 0.0f, 0.0f, false, 1.0f, hold);
    CHECK(!hold.held);
    CHECK(d.angular > 1.0f);        // swinging the tail back toward the target

    // And a reverse overshoot still keeps the stale bearing: 2 in past the
    // target is inside 3 * bearingHoldDist.
    BearingHold over;
    pointDemand({0.9f, 0.0f, 0.0f}, 0.0f, 0.0f, false, 1.0f, over);
    pointDemand({-2.0f, 0.0f, 0.0f}, 0.0f, 0.0f, false, 1.0f, over);
    CHECK(over.held);
}

CHAD_TEST(primitives, bearingHoldReleasesAfterAPoseJump) {
    BearingHold hold;
    // Latched by a momentary estimate that put the robot on the target.
    pointDemand({23.5f, 0.0f, 0.0f}, 24.0f, 0.0f, true, 1.0f, hold);
    CHECK(hold.held);
    // The estimate corrects to 12 in short and still in front: the stale bearing
    // would strand the robot on the wrong line for the rest of the motion.
    const MotionDemand d =
        pointDemand({12.0f, -6.0f, 0.0f}, 24.0f, 0.0f, true, 1.0f, hold);
    CHECK(!hold.held);
    CHECK(d.angular > 0.1f);   // steering back toward the target
}

// ============================================================================
// Turns
// ============================================================================

CHAD_TEST(primitives, shortestIsAPassThroughAndTakesTheShorterArc) {
    // SHORTEST must add nothing: forcing a direction here would be silent. The
    // old closure check, angleDiffRad(from + err, target) == 0, holds for any
    // err congruent to raw mod 2pi -- including a full lap the wrong way -- so
    // it caught no mutation, and its real subject was angleDiffRad, which
    // geometry_test.cpp already owns.
    const float targets[4] = {0.0f, HALF_PI, PI, -HALF_PI};
    for (float target : targets) {
        for (int i = 0; i < 24; ++i) {
            const float from = -PI + (TWO_PI * static_cast<float>(i)) / 24.0f;
            TurnLatch latch;
            const float raw = angleDiffRad(from, target);
            const float err = resolvedTurnError(raw, TurnDirection::SHORTEST,
                                                degToRad(2.0f), latch);
            CHECK_EQ(err, raw);
            // Shorter than either lap round it, hence at most half a turn.
            CHECK(std::fabs(err) <= std::fabs(err + TWO_PI));
            CHECK(std::fabs(err) <= std::fabs(err - TWO_PI));
            CHECK(std::fabs(err) <= PI + 1e-4f);
        }
    }
}

CHAD_TEST(primitives, wrapAcrossPiIsStillTheShortWay) {
    // 175 degrees to -175 degrees is 10 degrees CCW, not 350 CW.
    TurnLatch latch;
    const float raw = angleDiffRad(degToRad(175.0f), degToRad(-175.0f));
    const float err = resolvedTurnError(raw, TurnDirection::SHORTEST, degToRad(2.0f), latch);
    CHECK_NEAR(radToDeg(err), 10.0f, 0.01);
}

CHAD_TEST(primitives, forcedDirectionKeepsItsSign) {
    for (int i = 1; i < 24; ++i) {
        const float raw = -PI + (TWO_PI * static_cast<float>(i)) / 24.0f;
        // The forced answer is the unique representative of `raw` on the right
        // side of zero: raw itself when it already points that way, one lap
        // added or removed when it does not. Asserting only
        // wrapRad(e) == wrapRad(raw) accepted raw, raw + 2pi and raw - 2pi
        // alike, so it passed for the OPPOSITE direction.
        TurnLatch ccw;
        const float e1 = resolvedTurnError(raw, TurnDirection::CCW, degToRad(2.0f), ccw);
        CHECK(e1 >= 0.0f);
        CHECK(e1 < TWO_PI);
        CHECK_NEAR(e1, raw >= 0.0f ? raw : raw + TWO_PI, 1e-4);

        TurnLatch cw;
        const float e2 = resolvedTurnError(raw, TurnDirection::CW, degToRad(2.0f), cw);
        CHECK(e2 <= 0.0f);
        CHECK(e2 > -TWO_PI);
        CHECK_NEAR(e2, raw <= 0.0f ? raw : raw - TWO_PI, 1e-4);
    }
}

CHAD_TEST(primitives, forcedTurnDoesNotTakeAnotherLapAfterOvershoot) {
    // The hazard: CCW forced, the robot overshoots past tolerance, and the
    // resolved error flips to +355 degrees. Once the direction has been
    // travelled the forcing is done.
    TurnLatch latch;
    const float tol = degToRad(2.0f);
    CHECK_NEAR(radToDeg(resolvedTurnError(degToRad(-30.0f), TurnDirection::CCW, tol, latch)),
               330.0f, 0.01);         // still forcing: the long way round
    resolvedTurnError(degToRad(1.0f), TurnDirection::CCW, tol, latch);   // arrives
    CHECK(latch.satisfied);
    const float after =
        resolvedTurnError(degToRad(-5.0f), TurnDirection::CCW, tol, latch);
    CHECK_NEAR(radToDeg(after), -5.0f, 0.01);
}

CHAD_TEST(primitives, latchOnlyTripsInsideTolerance) {
    TurnLatch latch;
    resolvedTurnError(degToRad(-3.0f), TurnDirection::CCW, degToRad(2.0f), latch);
    CHECK(!latch.satisfied);
}

// ============================================================================
// moveArc
// ============================================================================

CHAD_TEST(primitives, arcTravelIsTheSweepNotTheChord) {
    // Quarter arc: robot at the origin facing +X, centre (0, 12), target (12, 12).
    const Pose pose{0.0f, 0.0f, 0.0f};
    const MotionDemand d = arcDemand(pose, 12.0f, 12.0f, 12.0f, true);
    CHECK_NEAR(d.lateral, HALF_PI * 12.0f, 1e-3);
    CHECK_NEAR(d.settleError, std::sqrt(2.0f) * 12.0f, 1e-3);
    CHECK_NEAR(d.angular, 0.0f, 1e-4);   // exactly on that arc already
}

CHAD_TEST(primitives, arcSurvivesTheHalfCircleTheChordCannotDescribe) {
    // Target diametrically opposite: the robot points across the chord, so a
    // chord projected on the heading reads zero travel with half an arc to go.
    const Pose pose{0.0f, 0.0f, 0.0f};
    const float chordBearing = bearingToPoint(pose, 0.0f, 24.0f);
    CHECK_NEAR(24.0f * std::cos(angleDiffRad(pose.theta, chordBearing)), 0.0f, 1e-4);

    const MotionDemand d = arcDemand(pose, 0.0f, 24.0f, 12.0f, true);
    CHECK_NEAR(d.lateral, PI * 12.0f, 1e-3);
    CHECK_NEAR(d.angular, 0.0f, 1e-4);
}

CHAD_TEST(primitives, arcBacksUpPastTheTargetWithoutSpinning) {
    // 0.1 rad past a target on the same circle.
    const float r = 12.0f;
    const float phi = -HALF_PI - 0.1f;
    const float tx = 0.0f + r * std::cos(phi);
    const float ty = 12.0f + r * std::sin(phi);
    const MotionDemand d = arcDemand({0.0f, 0.0f, 0.0f}, tx, ty, r, true);
    CHECK_NEAR(d.lateral, -0.1f * r, 1e-3);
    // Without the reverse-tangent hold this is ~174 degrees and the robot spins.
    CHECK(std::fabs(radToDeg(d.angular)) < 15.0f);
}

CHAD_TEST(primitives, negativeRadiusCurvesTheOtherWay) {
    // Mirror of the quarter-arc case: centre (0, -12), target (12, -12).
    const MotionDemand d = arcDemand({0.0f, 0.0f, 0.0f}, 12.0f, -12.0f, -12.0f, true);
    CHECK_NEAR(d.lateral, HALF_PI * 12.0f, 1e-3);
    CHECK_NEAR(d.angular, 0.0f, 1e-4);
}

CHAD_TEST(primitives, arcSteersBackOntoTheArcWhenOffIt) {
    // Same target, but the robot is turned 20 degrees away from the tangent.
    const MotionDemand tooFarLeft =
        arcDemand({0.0f, 0.0f, degToRad(20.0f)}, 12.0f, 12.0f, 12.0f, true);
    CHECK(tooFarLeft.angular < -0.05f);   // needs to come back CW
    const MotionDemand tooFarRight =
        arcDemand({0.0f, 0.0f, degToRad(-20.0f)}, 12.0f, 12.0f, 12.0f, true);
    CHECK(tooFarRight.angular > 0.05f);
}

CHAD_TEST(primitives, reverseArcBacksAlongTheArcWithoutTurningRound) {
    // Positive radius puts the centre left of TRAVEL, so reversing mirrors it to
    // (0, -12); the quarter arc that backs up from the origin ends at (-12, -12).
    const MotionDemand rev = arcDemand({0.0f, 0.0f, 0.0f}, -12.0f, -12.0f, 12.0f, false);
    CHECK_NEAR(rev.lateral, -HALF_PI * 12.0f, 1e-3);
    CHECK_NEAR(rev.angular, 0.0f, 1e-4);
    CHECK_NEAR(rev.settleError, std::sqrt(2.0f) * 12.0f, 1e-3);
}

CHAD_TEST(primitives, reverseArcCorrectsForwardWhenItOvershoots) {
    // 0.1 rad past the target on the reverse arc: it edges forward again rather
    // than carrying on round.
    const float r = 12.0f;
    const float phi = HALF_PI - 0.1f;
    const MotionDemand d = arcDemand({0.0f, 0.0f, 0.0f}, r * std::cos(phi),
                                     -12.0f + r * std::sin(phi), r, false);
    CHECK_NEAR(d.lateral, 0.1f * r, 1e-3);
    CHECK(std::fabs(radToDeg(d.angular)) < 15.0f);
}

CHAD_TEST(primitives, zeroRadiusIsAStraightDrive) {
    const MotionDemand d = arcDemand({0.0f, 0.0f, 0.0f}, 24.0f, 0.0f, 0.0f, true);
    CHECK_NEAR(d.lateral, 24.0f, 1e-3);
    CHECK_NEAR(d.angular, 0.0f, 1e-4);
    // And it still reverses past the target instead of driving away.
    const MotionDemand past = arcDemand({30.0f, 0.0f, 0.0f}, 24.0f, 0.0f, 0.0f, true);
    CHECK_NEAR(past.lateral, -6.0f, 1e-3);
    CHECK(std::fabs(radToDeg(past.angular)) < 1.0f);
}

CHAD_TEST(primitives, arcOnItsOwnCentreDrivesStraightAtIt) {
    // No arc of radius 12 passes through the centre of curvature, so the
    // fallback is a straight drive: steer AT the point, and commit no travel
    // while broadside to it. CHECK_FINITE alone blessed any answer, including a
    // 12 in travel command 90 degrees off the direction of travel, or a steer
    // to the arc tangent (60 degrees away) instead of the bearing.
    const MotionDemand d = arcDemand({0.0f, 0.0f, 0.0f}, 0.0f, 12.0f, 12.0f, true);
    CHECK_NEAR(d.angular, HALF_PI, 1e-4);
    CHECK_NEAR(d.lateral, 0.0f, 1e-3);
    CHECK_NEAR(d.settleError, 12.0f, 1e-3);
}

CHAD_TEST(primitives, arcSweepIsContinuousThroughTheHalfCircle) {
    // A single-valued sweep has to be cut somewhere on the circle. Cutting it at
    // +-pi put the break mid-arc: two targets 0.24 in apart either side of the
    // half turn commanded +37.6 in and -37.6 in of travel, the reference flipped
    // by pi with it, and the lateral PID's kD saw the whole 75 in step.
    const float r = 12.0f;
    auto at = [r](float sweep) {
        const float phi = -HALF_PI + sweep;     // the robot sits at -pi/2
        return arcDemand({0.0f, 0.0f, 0.0f}, r * std::cos(phi),
                         12.0f + r * std::sin(phi), r, true);
    };
    float prev = at(0.02f).lateral;
    for (int i = 2; i <= 235; ++i) {            // sweep 0.04 .. 4.70 rad
        const MotionDemand d = at(0.02f * static_cast<float>(i));
        CHECK(d.lateral > 0.0f);                        // still driving forward
        CHECK(std::fabs(d.lateral - prev) < 1.0f);      // 0.02 rad = 0.24 in
        CHECK(std::fabs(d.angular) < 0.01f);            // exactly on that circle
        prev = d.lateral;
    }
    // So an arc of more than a half turn is expressible at all.
    CHECK_NEAR(at(1.5f * PI).lateral, 1.5f * PI * r, 1e-2);
    // A target just behind still reads as a short back-up rather than a lap.
    CHECK_NEAR(at(-0.1f).lateral, -0.1f * r, 1e-3);
}

// ============================================================================
// moveBoomerang
// ============================================================================

CHAD_TEST(primitives, boomerangCarrotSitsOnTheApproachSide) {
    // Target at (24, 0) facing +X, robot 24 in short of it. The carrot has to be
    // between the robot and the target, or the approach does not curve in.
    const BoomerangParams p = boomParams();
    const MotionDemand d = boomerangDemand({0.0f, 0.0f, 0.0f}, 24.0f, 0.0f, 0.0f, p);
    CHECK(d.lateral > 0.0f);
    CHECK(d.lateral < 24.0f);
}

CHAD_TEST(primitives, boomerangCarrotFlipsWhenReversing) {
    // Backing into a pose at (24, 0) facing +X, the robot comes from beyond it,
    // so the carrot has to sit on that side too. The robot starts at (48, 0)
    // facing +X, which is exactly lined up to reverse into the target.
    BoomerangParams reverse = boomParams();
    reverse.forwards = false;
    const MotionDemand rev =
        boomerangDemand({48.0f, 0.0f, 0.0f}, 24.0f, 0.0f, 0.0f, reverse);
    CHECK(rev.lateral < 0.0f);                    // back up
    CHECK(std::fabs(rev.lateral) < 24.0f);        // carrot between robot and target
    CHECK_NEAR(rev.angular, 0.0f, 1e-3);          // already pointing the right way

    // The forward carrot is on the other side, so from here it is past the
    // target: the same pose is 30 in of travel away, not 18.
    const MotionDemand fwd =
        boomerangDemand({48.0f, 0.0f, 0.0f}, 24.0f, 0.0f, 0.0f, boomParams());
    CHECK(std::fabs(fwd.lateral) > 24.0f);
}

CHAD_TEST(primitives, boomerangBacksUpPastTheTarget) {
    const BoomerangParams p = boomParams();
    const MotionDemand d = boomerangDemand({30.0f, 0.0f, 0.0f}, 24.0f, 0.0f, 0.0f, p);
    CHECK(d.lateral < 0.0f);
}

CHAD_TEST(primitives, boomerangSteersToTheCarrotFarAndTheFinalHeadingNear) {
    const BoomerangParams p = boomParams();
    // Far out and off to the side: steering is dominated by the carrot bearing.
    const MotionDemand far =
        boomerangDemand({0.0f, 0.0f, 0.0f}, 48.0f, 48.0f, 0.0f, p);
    CHECK(far.angular > degToRad(20.0f));
    // Sitting on the target 30 degrees off the final heading: pure alignment.
    const MotionDemand near =
        boomerangDemand({24.0f, 0.0f, degToRad(30.0f)}, 24.0f, 0.0f, 0.0f, p);
    CHECK_NEAR(radToDeg(near.angular), -30.0f, 0.5);
}

CHAD_TEST(primitives, boomerangNeedsPositionAndHeadingTogether) {
    const BoomerangParams p = boomParams();
    // On the target, heading 30 degrees off: not settled.
    CHECK(boomerangDemand({24.0f, 0.0f, degToRad(30.0f)}, 24.0f, 0.0f, 0.0f, p)
              .settleError > p.targetTolerance);
    // Heading right, 6 in short: not settled.
    CHECK(boomerangDemand({18.0f, 0.0f, 0.0f}, 24.0f, 0.0f, 0.0f, p).settleError >
          p.targetTolerance);
    // Both: settled.
    CHECK(boomerangDemand({23.5f, 0.0f, degToRad(1.0f)}, 24.0f, 0.0f, 0.0f, p)
              .settleError <= p.targetTolerance);
}

CHAD_TEST(primitives, boomerangTapersSpeedIntoTheTargetAndFloorsIt) {
    BoomerangParams p = boomParams();
    p.minSpeed = 20.0f;
    p.maxSpeed = 100.0f;
    const MotionDemand far = boomerangDemand({0.0f, 0.0f, 0.0f}, 48.0f, 0.0f, 0.0f, p);
    CHECK_NEAR(far.lateralScale, 1.0f, 1e-4);
    const MotionDemand close =
        boomerangDemand({23.5f, 0.0f, 0.0f}, 24.0f, 0.0f, 0.0f, p);
    CHECK_NEAR(close.lateralScale, 0.2f, 1e-4);   // floored at minSpeed/maxSpeed
}

CHAD_TEST(primitives, boomerangIsFiniteOnTheTargetWithNoLead) {
    BoomerangParams p = boomParams();
    p.lead = 0.0f;
    const MotionDemand d = boomerangDemand({24.0f, 0.0f, 0.0f}, 24.0f, 0.0f, 0.0f, p);
    CHECK_FINITE(d.lateral);
    CHECK_FINITE(d.angular);
    CHECK_FINITE(d.settleError);
    CHECK_FINITE(d.lateralScale);
}

CHAD_TEST(primitives, boomerangSurvivesAZeroMaxSpeed) {
    BoomerangParams p = boomParams();
    p.maxSpeed = 0.0f;
    p.minSpeed = 10.0f;
    const MotionDemand d = boomerangDemand({0.0f, 0.0f, 0.0f}, 24.0f, 0.0f, 0.0f, p);
    CHECK_FINITE(d.lateralScale);
}

CHAD_TEST(primitives, boomerangLeadIsFullAtRangeAndWindsDownOnTheApproach) {
    // boomerang.hpp: the offset winds down over the last lead*12 inches. With
    // the robot on the target's final-heading line the carrot is at
    // (target - offset, 0) and d.lateral is exactly the distance to it, so the
    // offset can be read straight off the demand.
    auto offsetFrac = [](float dist) {
        const BoomerangParams p = boomParams();     // lead 0.5, leadDecay 0.5
        const MotionDemand m = boomerangDemand({0.0f, 0.0f, 0.0f}, dist, 0.0f, 0.0f, p);
        return (dist - m.lateral) / dist;
    };
    // Outside the 6 in window the configured lead is delivered in full. Measured
    // on the raw distance instead this read 0.25: lead silently halved from
    // range, and the carrot's angular offset then GREW on the approach.
    CHECK_NEAR(offsetFrac(48.0f), 0.5f, 1e-3);
    CHECK_NEAR(offsetFrac(6.0f), 0.5f, 1e-3);
    // Inside it, monotonically down to the floor leadDecay sets.
    CHECK(offsetFrac(3.0f) < offsetFrac(6.0f) - 1e-3f);
    CHECK(offsetFrac(1.0f) < offsetFrac(3.0f) - 1e-3f);
    CHECK_NEAR(offsetFrac(0.006f), 0.5f * (1.0f - 0.5f), 1e-3);
}

CHAD_TEST(primitives, boomerangIsAtTargetOnPositionAloneNotHeading) {
    const BoomerangParams p = boomParams();         // targetTolerance 1 in
    // On the target, 30 degrees off the final heading. settleError says no, but
    // a robot that has come to rest has no authority left to fix the heading, so
    // position alone is what separates arrival from a stall.
    const MotionDemand off =
        boomerangDemand({24.0f, 0.0f, degToRad(30.0f)}, 24.0f, 0.0f, 0.0f, p);
    CHECK(off.settleError > p.targetTolerance);
    CHECK(off.atTarget);
    // 6 in short with the heading right: not at target either way.
    const MotionDemand shortOf =
        boomerangDemand({18.0f, 0.0f, 0.0f}, 24.0f, 0.0f, 0.0f, p);
    CHECK(!shortOf.atTarget);
}
