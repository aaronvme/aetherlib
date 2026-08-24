// Critic reproductions for the reworked motion loop / settle logic. Each case
// is a defect that was found, written so it fails until the defect is fixed.
#include "chad/movement_control/primitives.hpp"
#include "chad/util/settle.hpp"

#include "chad_test.hpp"

using namespace chad;

namespace {

/// settleSpecFor(RAMSETE) with the shipped ChassisConfig, chassis.cpp:790-838:
///   holdMs      = ramseteParams.settleTimeMs   (0)
///   stallSpeed  = ramseteParams.stallSpeed     (1.5 in/s, overrides config)
///   stallHoldMs = mConfig.stallTimeMs          (500)
///   graceMs     = mConfig.stallGraceMs         (300)
///   speedTol    = mConfig.settleSpeedIn        (0, unset in config.cpp)
SettleSpec ramseteSpec(float targetTolerance, uint32_t timeoutMs) {
    SettleSpec s;
    s.errorTolerance = targetTolerance;
    s.speedTolerance = 0.0f;
    s.holdMs = 0;
    s.timeoutMs = timeoutMs;
    s.earlyExitRange = 0.0f;
    s.stallSpeed = 1.5f;
    s.stallHoldMs = 500;
    s.graceMs = 300;
    return s;
}

}  // namespace

// ============================================================================
// D1 — a RAMSETE path that ends exactly as its own documentation says it will
//      is reported as Stalled, 500 ms late.
// ============================================================================

CHAD_TEST(criticChassis, ramseteNormalEndIsReportedAsAStall) {
    // ramsete.hpp:47 — "Curved paths finish 5-12 deg off the final tangent and
    // the law has no authority left to fix it". ramsete.hpp:53 — below
    // stallSpeed "the heading it has is the heading it is going to get:
    // exiting beats burning the timeout". So this is the DESIGNED end state:
    // inside the position tolerance, 6 deg off the tangent, wheels at rest
    // because the profile drove v_d to zero.
    SettleMonitor m{ramseteSpec(3.0f, 5000)};
    m.reset(0);

    for (uint32_t t = 0; t < 3200; t += 10) {
        CHECK(m.update(t, SettleInput{20.0f, -20.0f, 40.0f, true}) == SettleState::Running);
    }

    // chassis.cpp:611 folds heading into the one scalar the monitor tests.
    const float err = dualSettleError(1.0f, 3.0f, degToRad(6.0f), degToRad(2.0f));
    CHECK(err > 3.0f);          // so `arrived` can never be true from here

    SettleState st = SettleState::Running;
    uint32_t exitAt = 0;
    for (uint32_t t = 3200; t <= 5000; t += 10) {
        st = m.update(t, SettleInput{err, 0.0f, 0.05f, true});
        if (st != SettleState::Running) { exitAt = t; break; }
    }

    // chassis.cpp:769-773 maps this to MotionOutcome::Stalled, so
    // lastMotionArrived() is false for a path that arrived.
    CHECK(st == SettleState::Settled);
    // The pre-rework code exited on the first stalled sample (old chassis.cpp
    // `settled = nearEnd && (headingOk || stalled)`); stallHoldMs now costs
    // 500 ms per path segment.
    CHECK(exitAt <= 3210u);
}

// ============================================================================
// D2 — the stall arming condition disarms itself in exactly the case stall
//      detection exists for: a path that ends short of tolerance.
// ============================================================================

CHAD_TEST(criticChassis, ramseteShortOfToleranceCannotStall) {
    // RAMSETEParams::targetTolerance defaults to -1, so orConfig
    // (chassis.cpp:940) resolves it to ChassisConfig::targetToleranceIn = 1.0,
    // the value ramsete.hpp:43 records as timing out 3/3 under MCL.
    // At the end of the profile v_d == 0 and distToEnd == 4 in, so
    // chassis.cpp:669 computes
    //     stallArmed = (nearEnd || v_d > stallSpeed) = (false || false) = false
    // and chassis.cpp:602 then feeds driveSpeed = 0 with the test disarmed.
    // k = 2*zeta*sqrt(w_d^2 + b*v_d^2) is 0 with v_d == 0, so the drive is
    // commanded to zero: nothing will change for the rest of the timeout.
    SettleMonitor m{ramseteSpec(1.0f, 5000)};
    m.reset(0);
    for (uint32_t t = 0; t < 3200; t += 10) {
        CHECK(m.update(t, SettleInput{20.0f, -20.0f, 40.0f, true}) == SettleState::Running);
    }

    SettleState st = SettleState::Running;
    uint32_t exitAt = 0;
    for (uint32_t t = 3200; t <= 6000; t += 10) {
        st = m.update(t, SettleInput{4.0f, 0.0f, 0.0f, /*stallArmed=*/false});
        if (st != SettleState::Running) { exitAt = t; break; }
    }
    CHECK(st == SettleState::Stalled);
    CHECK(exitAt <= 3700u);       // grace is long spent; 500 ms should end it
}

// ============================================================================
// D3 — BoomerangParams::leadDecay acts in the opposite sense to its doc.
// ============================================================================

CHAD_TEST(criticChassis, boomerangLeadDecayRunsBackwards) {
    // boomerang.hpp:24 — "Winds the offset down over the last `lead * 12`
    // inches." With the robot on the target's final-heading line the carrot is
    // at (target - offset, 0) and d.lateral is exactly distToCarrot, so the
    // offset can be read straight off the demand.
    auto offsetFrac = [](float d) {
        BoomerangParams p;
        p.lead = 0.5f;
        p.leadDecay = 0.5f;
        const MotionDemand m = boomerangDemand({0.0f, 0.0f, 0.0f}, d, 0.0f, 0.0f, p);
        return (d - m.lateral) / d;
    };

    // Outside the lead*12 = 6 in window nothing should be decayed yet: the
    // configured lead of 0.5 must be delivered in full.
    CHECK_NEAR(offsetFrac(24.0f), 0.5f, 1e-3);
    // And inside the window the fraction must FALL toward the target.
    CHECK(offsetFrac(1.0f) <= offsetFrac(6.0f) + 1e-4f);
}

// ============================================================================
// D4 — the bearing hold can never release on a reverse moveToPoint, because
//      the release test is written against the forward bearing.
// ============================================================================

CHAD_TEST(criticChassis, reverseBearingHoldCanNeverRelease) {
    BearingHold hold;
    // Backing into the origin from 0.9 in away, inside bearingHoldDist = 1.0
    // (config.cpp), so the bearing latches. `forwards == false` means the nose
    // points away from the target, i.e. |toLive| is near pi for the whole
    // motion.
    pointDemand({0.9f, 0.0f, 0.0f}, 0.0f, 0.0f, /*forwards=*/false, 1.0f, hold);
    CHECK(hold.held);
    CHECK_NEAR(hold.bearing, PI, 1e-4);

    // MCL relocalises: the robot is really 30 in away and off to one side, so
    // the held bearing is garbage. primitives.cpp:56 requires
    // distErr > 3*bearingHoldDist AND |toLive| < pi/2 to release; a reversing
    // robot never satisfies the second, so the hold is permanent.
    const MotionDemand d =
        pointDemand({2.0f, 30.0f, 0.0f}, 0.0f, 0.0f, /*forwards=*/false, 1.0f, hold);
    CHECK(!hold.held);
    // With the stale bearing both axes go quiet 30 in from the target: the
    // steer error is 0 because the robot happens to face the stale bearing, and
    // the along-track projection onto the live bearing is nearly perpendicular.
    CHECK(std::fabs(d.angular) > 0.1f);
}

// ============================================================================
// D5 — arcDemand's travel term reverses sign across the half-circle.
// ============================================================================

CHAD_TEST(criticChassis, arcSweepFlipsSignJustPastTheHalfCircle) {
    // Robot at the origin facing +X with radius 12, so the centre is (0, 12).
    // Two targets on that circle, 0.24 in apart, either side of the half turn.
    const float r = 12.0f;
    auto atPhi = [r](float phi) {
        return arcDemand({0.0f, 0.0f, 0.0f}, r * std::cos(phi),
                         12.0f + r * std::sin(phi), r, true);
    };
    const MotionDemand before = atPhi(HALF_PI - 0.01f);
    const MotionDemand after = atPhi(HALF_PI + 0.01f);

    CHECK(before.lateral > 0.0f);
    // 0.24 in of target movement must not reverse a 37 in travel command into
    // a 37 in reverse command with `forwards == true`. The live pose crosses
    // this boundary too, and the lateral PID's kD sees the full 75 in step.
    CHECK(std::fabs(after.lateral - before.lateral) < 2.0f);
}

// ============================================================================
// D6 — the Trajectory settle error is a synthetic step, and RateEstimator
//      differentiates it into a speed the robot does not have.
// ============================================================================

CHAD_TEST(criticChassis, trajectorySyntheticErrorFabricatesASpeed) {
    // chassis.cpp:652-657: with holdAtEnd == false, settleError is
    // targetTolerance + 1 until the reference clock expires and 0 after. The
    // same RateEstimator that feeds SettleSpec::speedTolerance sees that step.
    RateEstimator rate;                  // alpha 0.1, as chassis.cpp constructs
    for (int i = 0; i < 200; ++i) rate.update(2.0f, 0.01f);
    CHECK_NEAR(rate.rate(), 0.0f, 1e-3);
    const float spurious = rate.update(0.0f, 0.01f);
    CHECK_NEAR(spurious, 0.0f, 1.0f);    // the robot is standing still
}
