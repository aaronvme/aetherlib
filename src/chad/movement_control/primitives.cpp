#include "chad/movement_control/primitives.hpp"

#include <algorithm>
#include <cmath>

namespace chad {

SidePair tankMix(float lateral, float angular) {
    return {clamp(lateral - angular, -127.0f, 127.0f),
            clamp(lateral + angular, -127.0f, 127.0f)};
}

SidePair swingMix(float angular, SwingSide side) {
    // About the left wheel, driving the right side forward rotates CCW; about
    // the right wheel, CCW needs the left side backward.
    if (side == SwingSide::Left) return {0.0f, clamp(angular, -127.0f, 127.0f)};
    return {clamp(-angular, -127.0f, 127.0f), 0.0f};
}

float dualSettleError(float posError, float posTolerance, float headingErrorRad,
                      float headingToleranceRad) {
    const float pos = std::fabs(posError);
    if (!(headingToleranceRad > 0.0f) || !(posTolerance > 0.0f)) return pos;
    return std::max(pos, std::fabs(headingErrorRad) * (posTolerance / headingToleranceRad));
}

StraightTarget resolveStraightTarget(const Pose& start, float distance, bool forwards) {
    const float dir = forwards ? 1.0f : -1.0f;
    return {start.x + distance * dir * std::cos(start.theta),
            start.y + distance * dir * std::sin(start.theta), start.theta};
}

MotionDemand straightDemand(const Pose& pose, const StraightTarget& target) {
    MotionDemand d;
    d.lateral = (target.x - pose.x) * std::cos(target.axisRad) +
                (target.y - pose.y) * std::sin(target.axisRad);
    // A straight drive holds one heading; a live bearing to the target flips
    // 180 degrees the moment the robot passes it.
    d.angular = angleDiffRad(pose.theta, target.axisRad);
    d.settleError = d.lateral;
    return d;
}

MotionDemand pointDemand(const Pose& pose, float targetX, float targetY, bool forwards,
                         float bearingHoldDist, BearingHold& hold) {
    MotionDemand d;
    const float distErr = distanceToPoint(pose, targetX, targetY);
    const float liveBearing = bearingToPoint(pose, targetX, targetY);
    const float toLive = angleDiffRad(pose.theta, liveBearing);

    d.lateral = distErr * std::cos(toLive);
    d.settleError = distErr;

    // "In front" has to mean in front of the direction of TRAVEL. Backing into a
    // target puts the nose away from it, so measured on the forward bearing the
    // release is dead code for every reverse motion.
    const float toTravel = forwards ? toLive : wrapRad(toLive + PI);

    if (hold.held) {
        if (distErr > kBearingHoldReleaseFactor * bearingHoldDist &&
            std::fabs(toTravel) < HALF_PI) {
            hold.held = false;
        }
    } else if (distErr < bearingHoldDist) {
        hold.held = true;
        hold.bearing = liveBearing;
    }

    float steer = hold.held ? hold.bearing : liveBearing;
    // Backward: point away from the target. The along-track sign already sends
    // the robot the right way, so the lateral term is not negated.
    if (!forwards) steer = wrapRad(steer + PI);
    d.angular = angleDiffRad(pose.theta, steer);
    return d;
}

float resolvedTurnError(float rawErrRad, TurnDirection direction, float toleranceRad,
                        TurnLatch& latch) {
    if (std::fabs(rawErrRad) < toleranceRad) latch.satisfied = true;
    if (latch.satisfied) return rawErrRad;
    return resolveTurnError(rawErrRad, direction);
}

MotionDemand arcDemand(const Pose& pose, float targetX, float targetY, float radius,
                       bool forwards) {
    // Reverse is the same geometry driven backwards: solve it for a robot facing
    // the other way, then negate the travel. The heading error is unchanged,
    // because both the reference and the robot are offset by pi.
    const Pose frame{pose.x, pose.y, forwards ? pose.theta : wrapRad(pose.theta + PI)};
    const float travelSign = forwards ? 1.0f : -1.0f;

    MotionDemand d;
    const float chord = distanceToPoint(frame, targetX, targetY);
    d.settleError = chord;

    float reference;   // heading the frame should hold
    float sweep;       // signed travel left, inches, frame-forward positive

    const Pose centre = computeArcCenter(frame, radius);
    if (std::fabs(radius) < 1e-6f ||
        distance(centre.x, centre.y, targetX, targetY) < 1e-6f) {
        // No arc to follow, or the target is the centre of curvature and no arc
        // of this radius reaches it. Drive straight at it.
        reference = bearingToPoint(frame, targetX, targetY);
        sweep = chord * std::cos(angleDiffRad(frame.theta, reference));
    } else {
        const float s = (radius >= 0.0f) ? 1.0f : -1.0f;
        const float phiRobot = std::atan2(frame.y - centre.y, frame.x - centre.x);
        const float phiTarget = std::atan2(targetY - centre.y, targetX - centre.x);
        // Both atan2 results are in (-pi, pi], so the raw difference is in
        // (-2pi, 2pi) and one shift lands it in [-kArcBackSweep, 2pi - ...).
        float travel = s * (phiTarget - phiRobot);
        if (travel < -kArcBackSweep) travel += TWO_PI;
        else if (travel >= TWO_PI - kArcBackSweep) travel -= TWO_PI;
        sweep = travel * std::fabs(radius);
        // arcStartHeading is the tangent the robot would hold if it were already
        // on an arc of this radius through the target, so the difference is an
        // off-the-arc error. The tangent at its own position would be identically
        // the current heading and carry no information at all.
        //
        // arcAngle is an asin, so it only ever describes the MINOR arc; past a
        // half turn the tangent belongs to the major arc, which is the minor
        // arc's end tangent reversed. The two agree at exactly pi.
        reference = (travel <= PI)
                        ? arcStartHeading(frame, targetX, targetY, radius)
                        : wrapRad(arcEndHeading(frame, targetX, targetY, radius) + PI);
    }

    // Past the target both the tangent and the bearing point back at it, and
    // steering to either turns the robot around. Hold the reverse of it and let
    // the negative travel back up along the same arc. Deadbanded because a
    // broadside target projects to cos(pi/2), which in float32 is -4.4e-8: a
    // sweep of -5e-7 in used to flip the steer command by pi.
    if (sweep < -1e-4f) reference = wrapRad(reference + PI);

    d.lateral = travelSign * sweep;
    d.angular = angleDiffRad(frame.theta, reference);
    return d;
}

MotionDemand boomerangDemand(const Pose& pose, float targetX, float targetY,
                             float targetThetaRad, const BoomerangParams& params) {
    MotionDemand d;
    const float distToTarget = distanceToPoint(pose, targetX, targetY);

    float offset = std::min(params.lead * distToTarget, 24.0f);
    // How far INTO the last lead*12 inches the robot is, so the offset winds
    // down as it closes. Measured on the raw distance instead, the decay ran
    // backwards: `lead` was silently halved from range and the carrot's angular
    // offset grew on the approach. lead == 0 would also be a 0/0 on the target.
    const float closing = (params.lead > 1e-6f)
                              ? 1.0f - std::min(distToTarget / (params.lead * 12.0f), 1.0f)
                              : 0.0f;
    offset *= 1.0f - closing * params.leadDecay;

    // The carrot sits off the target along the target's FINAL heading, not the
    // robot-to-target bearing: that offset direction is what curves the approach
    // in so the robot arrives facing the right way. Behind when driving
    // forwards, ahead when reversing -- either way, on the side it comes from.
    const float side = params.forwards ? -1.0f : 1.0f;
    const float carrotX = targetX + side * offset * std::cos(targetThetaRad);
    const float carrotY = targetY + side * offset * std::sin(targetThetaRad);

    const float distToCarrot = distanceToPoint(pose, carrotX, carrotY);
    const float carrotBearing = bearingToPoint(pose, carrotX, carrotY);
    const float toFinal = angleDiffRad(pose.theta, targetThetaRad);

    d.lateral = distToCarrot * std::cos(angleDiffRad(pose.theta, carrotBearing));

    const float steer = params.forwards ? carrotBearing : wrapRad(carrotBearing + PI);
    const float toCarrot = angleDiffRad(pose.theta, steer);
    // Weight shifts from facing the carrot (far) to facing the final heading
    // (near), which is the phase change from guidance to alignment.
    const float weight = std::min(distToTarget / 12.0f, 1.0f);
    d.angular = toCarrot * weight + toFinal * (1.0f - weight);

    d.lateralScale = std::min(distToTarget / 6.0f, 1.0f);
    if (params.maxSpeed > 1e-6f) {
        d.lateralScale = std::max(d.lateralScale, params.minSpeed / params.maxSpeed);
    }

    d.settleError = dualSettleError(distToTarget, params.targetTolerance, toFinal,
                                    degToRad(params.headingTolerance));
    // Position alone. settleError also demands the final heading, which a
    // stopped robot has no authority left to fix.
    d.atTarget = distToTarget <= params.targetTolerance;
    return d;
}

}  // namespace chad
