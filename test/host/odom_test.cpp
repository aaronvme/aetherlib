#include "chad/tracking/odom_core.hpp"

#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

OdomCore fresh(const OdomCore::Config& cfg = {}) {
    OdomCore core;
    core.setConfig(cfg);
    core.rebaseline(0.0f, 0.0f, 0.0f);
    return core;
}

}  // namespace

// ============================================================================
// Wheel deltas to a twist
// ============================================================================

CHAD_TEST(odomCore, straightDriveIntegratesEveryInch) {
    OdomCore core = fresh();
    float d = 0.0f;
    for (int i = 0; i < 100; i++) {
        d += 0.37f;
        core.step(d, 0.0f, 0.0f, true);
    }
    CHECK_NEAR(core.pose().x, d, 1e-3);
    CHECK_NEAR(core.pose().y, 0.0f, 1e-6);
    CHECK_NEAR(core.pose().theta, 0.0f, 1e-9);
}

CHAD_TEST(odomCore, arcMatchesTheClosedFormCircle) {
    // Driving an arc of length L while turning dTheta traces a circle of radius
    // L/dTheta, ending at (R sin dTheta, R (1 - cos dTheta)).
    const float L = 20.0f;
    const float dTheta = 1.2f;
    const float R = L / dTheta;

    OdomCore core = fresh();
    core.step(L, 0.0f, dTheta, true);
    CHECK_NEAR(core.pose().x, R * std::sin(dTheta), 2e-3);
    CHECK_NEAR(core.pose().y, R * (1.0f - std::cos(dTheta)), 2e-3);
    CHECK_NEAR(core.pose().theta, dTheta, 1e-5);
}

CHAD_TEST(odomCore, splittingATickDoesNotChangeWhereItEnds) {
    // The pose exponential is exact for a constant twist, so the same arc taken
    // in one bite and in two hundred must land in the same place. Straight-line
    // integration fails this, always short on the inside of the turn.
    const float L = 20.0f;
    const float dTheta = 1.1f;

    OdomCore one = fresh();
    one.step(L, 0.0f, dTheta, true);

    OdomCore many = fresh();
    const int n = 200;
    for (int i = 0; i < n; i++) {
        const float f = static_cast<float>(i + 1) / static_cast<float>(n);
        many.step(L * f, 0.0f, dTheta * f, true);
    }

    CHECK_NEAR(many.pose().x, one.pose().x, 3e-3);
    CHECK_NEAR(many.pose().y, one.pose().y, 3e-3);
    CHECK_NEAR(many.pose().theta, one.pose().theta, 1e-4);
}

CHAD_TEST(odomCore, lateralTravelGoesToTheRobotsLeft) {
    OdomCore core = fresh();
    core.step(0.0f, 5.0f, 0.0f, true);
    CHECK_NEAR(core.pose().x, 0.0f, 1e-6);
    CHECK_NEAR(core.pose().y, 5.0f, 1e-5);

    OdomCore turned = fresh();
    turned.setPose({0.0f, 0.0f, HALF_PI});
    turned.rebaseline(0.0f, 0.0f, HALF_PI);
    turned.step(0.0f, 5.0f, HALF_PI, true);
    CHECK_NEAR(turned.pose().x, -5.0f, 1e-4);
    CHECK_NEAR(turned.pose().y, 0.0f, 1e-4);
}

// ============================================================================
// setPose must not eat the pending tick
// ============================================================================

CHAD_TEST(odomCore, setPoseDoesNotConsumeTheSensorBaseline) {
    OdomCore core = fresh();
    core.step(4.0f, 0.0f, 0.0f, true);
    CHECK_NEAR(core.pose().x, 4.0f, 1e-6);

    core.setPose({0.0f, 0.0f, 0.0f});
    // The wheels turned another inch while the pose was being written; it is
    // real motion and belongs to the next delta.
    const OdomDelta d = core.step(5.0f, 0.0f, 0.0f, true);
    CHECK_NEAR(d.dVert, 1.0f, 1e-6);
    CHECK_NEAR(core.pose().x, 1.0f, 1e-6);
}

CHAD_TEST(odomCore, reBaseliningOnEverySetPoseLosesHalfTheDistance) {
    // The shape of a filter writing its fused pose back every cycle while the
    // odometry task ticks in between. Policy A keeps the pose write and the
    // sensor baseline separate; policy B re-takes the baseline on every write,
    // which is what OdomTracking::setPose() used to do.
    OdomCore keepBaseline = fresh();
    OdomCore reBaseline = fresh();

    float d = 0.0f;
    for (int tick = 0; tick < 20; tick++) {
        d += 0.5f;   // half a tick of driving, then the filter writes the pose
        keepBaseline.setPose(keepBaseline.pose());
        reBaseline.setPose(reBaseline.pose());
        reBaseline.rebaseline(d, 0.0f, 0.0f);

        d += 0.5f;   // the rest of the tick, then odometry integrates
        keepBaseline.step(d, 0.0f, 0.0f, true);
        reBaseline.step(d, 0.0f, 0.0f, true);
    }

    CHECK_NEAR(d, 20.0f, 1e-4);
    CHECK_NEAR(keepBaseline.pose().x, 20.0f, 1e-3);
    CHECK_NEAR(reBaseline.pose().x, 10.0f, 1e-3);
}

CHAD_TEST(odomCore, consumedDeltasAccountForEveryInchDriven) {
    OdomCore core = fresh();
    float d = 0.0f;
    float consumed = 0.0f;
    for (int i = 0; i < 50; i++) {
        d += 0.4f;
        core.setPose(core.pose());   // as a filter would, every cycle
        core.step(d, 0.0f, 0.0f, true);
        if (i % 3 == 0) consumed += core.consumeDelta(OdomCore::CHANNEL_EKF).dVert;
    }
    consumed += core.consumeDelta(OdomCore::CHANNEL_EKF).dVert;
    CHECK_NEAR(consumed, d, 1e-3);
}

// ============================================================================
// Dropouts
// ============================================================================

CHAD_TEST(odomCore, aReplugedEncoderIsDroppedNotIntegrated) {
    OdomCore core = fresh();
    float d = 0.0f;
    for (int i = 0; i < 50; i++) {
        d += 1.0f;
        core.step(d, 0.0f, 0.0f, true);
    }
    CHECK_NEAR(core.pose().x, 50.0f, 1e-3);

    // Unplugged and back: the sensor counts from zero again.
    const OdomDelta bad = core.step(0.0f, 0.0f, 0.0f, true);
    CHECK_NEAR(bad.dVert, 0.0f, 1e-9);
    CHECK_NEAR(core.pose().x, 50.0f, 1e-3);
    CHECK_EQ(core.jumpRejects(), 1u);

    // And it resumes on the next tick rather than rejecting for ever.
    const OdomDelta good = core.step(1.0f, 0.0f, 0.0f, true);
    CHECK_NEAR(good.dVert, 1.0f, 1e-6);
    CHECK_NEAR(core.pose().x, 51.0f, 1e-3);
    CHECK_EQ(core.jumpRejects(), 1u);
}

CHAD_TEST(odomCore, jumpGuardCanBeDisabled) {
    OdomCore::Config cfg;
    cfg.maxDeltaIn = 0.0f;
    OdomCore core = fresh(cfg);
    core.step(500.0f, 0.0f, 0.0f, true);
    CHECK_NEAR(core.pose().x, 500.0f, 1e-2);
    CHECK_EQ(core.jumpRejects(), 0u);
}

CHAD_TEST(odomCore, aHeldHeadingContributesNoRotation) {
    OdomCore core = fresh();
    core.step(0.0f, 0.0f, 0.8f, true);
    CHECK_NEAR(core.pose().theta, 0.8f, 1e-6);

    // A dead IMU repeats its last good value; that looks exactly like a robot
    // that is not turning, so it must not be fused as one.
    const OdomDelta d = core.step(1.0f, 0.0f, 2.5f, false);
    CHECK_NEAR(d.dTheta, 0.0f, 1e-9);
    CHECK_NEAR(core.pose().theta, 0.8f, 1e-6);
    CHECK_EQ(core.headingHolds(), 1u);
}

CHAD_TEST(odomCore, headingJumpsAreDroppedAndResynced) {
    OdomCore core = fresh();
    core.step(0.0f, 0.0f, 0.2f, true);
    const OdomDelta bad = core.step(0.0f, 0.0f, -2.9f, true);
    CHECK_NEAR(bad.dTheta, 0.0f, 1e-9);
    CHECK_EQ(core.headingJumpRejects(), 1u);
    // Baseline moved with it, so the next tick measures from the new reading
    // instead of repeating the rejection for ever.
    const OdomDelta good = core.step(0.0f, 0.0f, -2.8f, true);
    CHECK_NEAR(good.dTheta, 0.1f, 1e-5);
}

CHAD_TEST(odomCore, nonFiniteReadingsHoldRatherThanTeleport) {
    OdomCore core = fresh();
    core.step(3.0f, 1.0f, 0.1f, true);
    const Pose before = core.pose();

    const OdomDelta a = core.step(kNan, kInf, kNan, true);
    CHECK_NEAR(a.dVert, 0.0f, 1e-9);
    CHECK_NEAR(a.dHoriz, 0.0f, 1e-9);
    CHECK_NEAR(a.dTheta, 0.0f, 1e-9);
    CHECK_NEAR(core.pose().x, before.x, 1e-6);
    CHECK_NEAR(core.pose().y, before.y, 1e-6);
    CHECK_NEAR(core.pose().theta, before.theta, 1e-6);

    const OdomDelta b = core.step(4.0f, 1.0f, 0.1f, true);
    CHECK_NEAR(b.dVert, 1.0f, 1e-6);
}

// ============================================================================
// Heading wrap
// ============================================================================

CHAD_TEST(odomCore, headingCrossingPiGivesTheShortDelta) {
    OdomCore core = fresh();
    core.rebaseline(0.0f, 0.0f, 3.13f);
    core.setPose({0.0f, 0.0f, 3.13f});

    const OdomDelta d = core.step(0.0f, 0.0f, -3.13f, true);
    // Unwrapped this is -6.26 rad, which would spin the pose most of a turn and
    // hand every filter downstream the same lie.
    CHECK_NEAR(d.dTheta, TWO_PI - 6.26f, 1e-5);
    CHECK(std::fabs(d.dTheta) < 0.05f);
    CHECK_NEAR(core.pose().theta, -3.13f, 1e-5);
}

CHAD_TEST(odomCore, manyLapsAccumulateWithoutUnwrapping) {
    OdomCore core = fresh();
    float turned = 0.0f;
    for (int i = 0; i < 400; i++) {
        turned += 0.1f;
        core.step(0.0f, 0.0f, wrapRad(turned), true);
    }
    // Ten and a bit turns, and the reported deltas still sum to all of it.
    core.clearDeltas();
    for (int i = 0; i < 10; i++) core.step(0.0f, 0.0f, wrapRad(turned + 0.1f * (i + 1)), true);
    const OdomDelta acc = core.consumeDelta(OdomCore::CHANNEL_MCL);
    CHECK_NEAR(acc.dTheta, 1.0f, 1e-4);
    CHECK(core.pose().theta >= -PI && core.pose().theta <= PI);
}

// ============================================================================
// Heading source
// ============================================================================

CHAD_TEST(odomCore, slavedHeadingOverwritesAWrittenPoseHeading) {
    // Current shipping behaviour, pinned here deliberately: with the pose
    // heading slaved to the IMU, setHeading() and a filter's heading correction
    // survive exactly one tick.
    OdomCore core = fresh();
    CHECK(core.config().headingFromImu);
    core.setPose({0.0f, 0.0f, 1.0f});
    core.step(0.0f, 0.0f, 0.2f, true);
    CHECK_NEAR(core.pose().theta, 0.2f, 1e-6);

    // What that costs the EKF, stated rather than left to be discovered.
    // EkfTracking::update blends its fused heading into the odometry pose with
    // EkfCore::Config::thetaSyncProp; this overwrites the blend on the next
    // 10 ms tick, so thetaSyncProp is INERT in the shipped configuration.
    // Chassis::getPose() returns this pose, not the filter's, and nothing in the
    // library calls OdomTracking::setCoreConfig to change headingFromImu.
    const float imu = 0.2f;
    const float fused = 0.6f;
    const float kt = 0.20f;   // the shipped thetaSyncProp
    const float blended =
        wrapRad(core.pose().theta + kt * angleDiffRad(core.pose().theta, fused));
    core.setPose({0.0f, 0.0f, blended});
    CHECK_NEAR(core.pose().theta, 0.28f, 1e-5);   // the blend did land
    core.step(0.0f, 0.0f, imu, true);
    CHECK_NEAR(core.pose().theta, imu, 1e-6);     // and is gone one tick later

    // A relocation is a hard write rather than a blend, and goes the same way.
    core.setPose({0.0f, 0.0f, 1.5f});
    core.step(0.0f, 0.0f, imu, true);
    CHECK_NEAR(core.pose().theta, imu, 1e-6);
}

CHAD_TEST(odomCore, integratedHeadingKeepsAWrittenPoseHeading) {
    OdomCore::Config cfg;
    cfg.headingFromImu = false;
    OdomCore core = fresh(cfg);
    core.setPose({0.0f, 0.0f, 1.0f});
    core.step(0.0f, 0.0f, 0.2f, true);
    CHECK_NEAR(core.pose().theta, 1.2f, 1e-5);
    core.step(0.0f, 0.0f, 0.3f, true);
    CHECK_NEAR(core.pose().theta, 1.3f, 1e-5);
}

CHAD_TEST(odomCore, integratedHeadingBendsByTheImuFromItsOwnPoseHeading) {
    // The old version of this ran both configurations from the same pose
    // heading, where they are identical by construction, so it could not fail.
    // Offset the integrated core's pose heading first: its arc must be the
    // slaved core's arc rotated by that offset, and both must bend by the same
    // dTheta, which comes from the IMU either way.
    OdomCore::Config cfg;
    cfg.headingFromImu = false;
    OdomCore slaved = fresh();
    OdomCore integrated = fresh(cfg);

    const float offset = 0.5f;
    integrated.setPose({0.0f, 0.0f, offset});
    for (int i = 1; i <= 20; i++) {
        const float th = 0.05f * static_cast<float>(i);
        slaved.step(static_cast<float>(i), 0.0f, th, true);
        integrated.step(static_cast<float>(i), 0.0f, th, true);
        // Same measured rotation, whatever the heading source.
        CHECK_NEAR(integrated.lastDelta().dTheta, slaved.lastDelta().dTheta, 1e-7);
    }

    const float c = std::cos(offset), s = std::sin(offset);
    CHECK_NEAR(integrated.pose().x, c * slaved.pose().x - s * slaved.pose().y, 3e-3);
    CHECK_NEAR(integrated.pose().y, s * slaved.pose().x + c * slaved.pose().y, 3e-3);
    CHECK_NEAR(integrated.pose().theta, slaved.pose().theta + offset, 1e-4);
    // The rotation is real: without it the two paths would coincide.
    CHECK(std::fabs(integrated.pose().y - slaved.pose().y) > 1.0f);
}

// ============================================================================
// Delta channels
// ============================================================================

CHAD_TEST(odomCore, channelsAccumulateIndependently) {
    OdomCore core = fresh();
    core.step(1.0f, 0.0f, 0.0f, true);
    core.step(2.0f, 0.0f, 0.0f, true);

    const OdomDelta mcl = core.consumeDelta(OdomCore::CHANNEL_MCL);
    CHECK_NEAR(mcl.dVert, 2.0f, 1e-6);
    CHECK_NEAR(core.consumeDelta(OdomCore::CHANNEL_MCL).dVert, 0.0f, 1e-9);

    core.step(3.0f, 0.0f, 0.0f, true);
    // The EKF never read, so it still owns all three inches.
    CHECK_NEAR(core.consumeDelta(OdomCore::CHANNEL_EKF).dVert, 3.0f, 1e-6);
    CHECK_NEAR(core.consumeDelta(OdomCore::CHANNEL_MCL).dVert, 1.0f, 1e-6);
}

CHAD_TEST(odomCore, outOfRangeChannelIsZeroNotOutOfBounds) {
    OdomCore core = fresh();
    core.step(5.0f, 0.0f, 0.0f, true);
    CHECK_NEAR(core.consumeDelta(-1).dVert, 0.0f, 1e-9);
    CHECK_NEAR(core.consumeDelta(OdomCore::DELTA_CHANNELS).dVert, 0.0f, 1e-9);
    CHECK_NEAR(core.consumeDelta(OdomCore::CHANNEL_MCL).dVert, 5.0f, 1e-6);
}

CHAD_TEST(odomCore, clearDeltasDropsMotionThatPredatesAReset) {
    OdomCore core = fresh();
    core.step(9.0f, 0.0f, 0.3f, true);
    core.clearDeltas();
    CHECK_NEAR(core.lastDelta().dVert, 0.0f, 1e-9);
    CHECK_NEAR(core.consumeDelta(OdomCore::CHANNEL_MCL).dVert, 0.0f, 1e-9);
    CHECK_NEAR(core.consumeDelta(OdomCore::CHANNEL_EKF).dTheta, 0.0f, 1e-9);
}

CHAD_TEST(odomCore, lastDeltaIsTheLastTickAndNotAnAccumulator) {
    // The old version compared lastDelta() to the return of the same call, and
    // both are `return mLastDelta;` -- no mutation of the class could fail it.
    // What matters is that lastDelta() is ONE tick while the channels accumulate,
    // and that draining a channel does not disturb it.
    OdomCore core = fresh();
    core.step(1.5f, -0.5f, 0.07f, true);
    core.step(4.0f, -0.5f, 0.10f, true);

    CHECK_NEAR(core.lastDelta().dVert, 2.5f, 1e-6);
    CHECK_NEAR(core.lastDelta().dHoriz, 0.0f, 1e-6);
    CHECK_NEAR(core.lastDelta().dTheta, 0.03f, 1e-5);

    // Both ticks are still owed to the channel, so lastDelta is not the
    // accumulator wearing a different name.
    const OdomDelta acc = core.consumeDelta(OdomCore::CHANNEL_EKF);
    CHECK_NEAR(acc.dVert, 4.0f, 1e-6);
    CHECK_NEAR(acc.dTheta, 0.10f, 1e-5);
    CHECK_NEAR(core.lastDelta().dVert, 2.5f, 1e-6);
    CHECK_NEAR(core.consumeDelta(OdomCore::CHANNEL_EKF).dVert, 0.0f, 1e-9);
    CHECK_NEAR(core.lastDelta().dVert, 2.5f, 1e-6);

    // A tick the jump guard drops is a zero delta, not the previous one repeated.
    core.step(400.0f, -0.5f, 0.10f, true);
    CHECK_NEAR(core.lastDelta().dVert, 0.0f, 1e-9);
    CHECK_EQ(core.jumpRejects(), 1u);
}

// ============================================================================
// Redundant distance sources
// ============================================================================

CHAD_TEST(odomDistance, aFaultedSensorIsDroppedNotAveragedIn) {
    // TrackingWheel holds its last good distance when the port reports PROS_ERR.
    // Averaging that held value in with the live ones scaled every delta by
    // live/total -- half the distance with one dead wheel of two -- and because
    // each delta was merely SMALLER, no jump guard could fire and no counter
    // moved. This is the arithmetic OdomTracking::averageDistance now uses.
    const float values[2] = {20.0f, 8.0f};   // wheel 1 frozen 12 inches back
    const bool none[2] = {false, false};
    const bool second[2] = {false, true};

    const DistanceAverage all = averageLiveDistance(values, none, 2);
    CHECK_EQ(all.live, 2);
    CHECK_EQ(all.faulted, 0);
    CHECK_NEAR(all.value, 14.0f, 1e-6);      // the halving, in one number

    const DistanceAverage live = averageLiveDistance(values, second, 2);
    CHECK_EQ(live.live, 1);
    CHECK_EQ(live.faulted, 1);
    CHECK_NEAR(live.value, 20.0f, 1e-6);     // the honest sensor, unscaled

    // Everything faulted is an outage the caller must hold through, not a zero
    // that teleports the pose to the origin.
    const bool both[2] = {true, true};
    const DistanceAverage out = averageLiveDistance(values, both, 2);
    CHECK(!out.valid());
    CHECK_EQ(out.faulted, 2);
    CHECK_NEAR(out.value, 0.0f, 1e-9);

    // A non-finite reading is a fault whatever the port said about it.
    const float withNan[3] = {5.0f, kNan, kInf};
    const DistanceAverage finite = averageLiveDistance(withNan, nullptr, 3);
    CHECK_EQ(finite.live, 1);
    CHECK_EQ(finite.faulted, 2);
    CHECK_NEAR(finite.value, 5.0f, 1e-6);

    // Degenerate inputs, since this runs in a control tick.
    CHECK(!averageLiveDistance(nullptr, nullptr, 2).valid());
    CHECK(!averageLiveDistance(values, none, 0).valid());
    CHECK(!averageLiveDistance(values, none, -1).valid());
}

CHAD_TEST(odomDistance, droppingTheFaultedSensorKeepsTheScale) {
    // End to end through OdomCore: the halved average loses half the distance
    // driven and trips nothing, the live-only average keeps all of it.
    OdomCore averaged = fresh();
    OdomCore live = fresh();
    float healthy = 0.0f;
    const float frozen = 0.0f;
    for (int i = 0; i < 100; i++) {
        healthy += 0.5f;                     // the robot really drives 50 inches
        const float values[2] = {healthy, frozen};
        const bool bad[2] = {false, true};
        averaged.step(averageLiveDistance(values, nullptr, 2).value, 0.0f, 0.0f, true);
        live.step(averageLiveDistance(values, bad, 2).value, 0.0f, 0.0f, true);
    }
    CHECK_NEAR(averaged.pose().x, 25.0f, 1e-2);
    CHECK_NEAR(live.pose().x, 50.0f, 1e-2);
    // Neither is a jump, which is the point: maxDeltaIn cannot see this.
    CHECK_EQ(averaged.jumpRejects(), 0u);
    CHECK_EQ(live.jumpRejects(), 0u);
}
