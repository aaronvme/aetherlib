#include "chad/tracking/pose_estimator.hpp"

#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kExact[3] = {0.0f, 0.0f, 0.0f};

LatencyPoseEstimator makeEstimator(uint32_t historyMs = 1000) {
    LatencyPoseEstimator::Config c;
    c.historyMs = historyMs;
    c.stateStdDev[0] = 1.0f;
    c.stateStdDev[1] = 1.0f;
    c.stateStdDev[2] = 0.1f;
    return LatencyPoseEstimator(c);
}

/// Drive straight forward one inch per 10 ms tick.
void driveForward(LatencyPoseEstimator& est, uint32_t& nowMs, int ticks) {
    for (int i = 0; i < ticks; i++) {
        nowMs += 10;
        est.update(nowMs, {1.0f, 0.0f, 0.0f});
    }
}

}  // namespace

CHAD_TEST(poseBuffer, samplesAndInterpolates) {
    PoseBuffer<8> buf;
    CHECK(buf.empty());
    Pose out{};
    CHECK(!buf.sample(100, out));

    buf.add(100, {0.0f, 0.0f, 0.0f});
    buf.add(200, {10.0f, 0.0f, 0.0f});
    CHECK_EQ(buf.size(), 2);

    CHECK(buf.sample(150, out));
    CHECK_NEAR(out.x, 5.0f, 1e-4);
    CHECK(buf.sample(100, out));
    CHECK_NEAR(out.x, 0.0f, 1e-6);
    CHECK(buf.sample(200, out));
    CHECK_NEAR(out.x, 10.0f, 1e-6);
}

CHAD_TEST(poseBuffer, clampsOutsideTheHistory) {
    PoseBuffer<4> buf;
    buf.add(100, {1.0f, 0.0f, 0.0f});
    buf.add(200, {2.0f, 0.0f, 0.0f});
    Pose out{};
    CHECK(buf.sample(50, out));
    CHECK_NEAR(out.x, 1.0f, 1e-6);
    CHECK(buf.sample(9999, out));
    CHECK_NEAR(out.x, 2.0f, 1e-6);
}

CHAD_TEST(poseBuffer, overflowDropsTheOldest) {
    PoseBuffer<4> buf;
    for (uint32_t i = 0; i < 10; i++) {
        buf.add(100 + i * 10, {static_cast<float>(i), 0.0f, 0.0f});
    }
    CHECK_EQ(buf.size(), 4);
    CHECK_EQ(buf.oldestMs(), uint32_t{160});
    CHECK_EQ(buf.newestMs(), uint32_t{190});
    Pose out{};
    CHECK(buf.sample(160, out));
    CHECK_NEAR(out.x, 6.0f, 1e-6);
}

CHAD_TEST(poseBuffer, pruneKeepsTheWindow) {
    PoseBuffer<32> buf;
    for (uint32_t t = 0; t <= 1000; t += 100) {
        buf.add(t, {static_cast<float>(t) * 0.01f, 0.0f, 0.0f});
    }
    buf.prune(300);
    CHECK(buf.oldestMs() >= 700);
    CHECK_EQ(buf.newestMs(), uint32_t{1000});
    CHECK(!buf.covers(500));
    CHECK(buf.covers(800));
}

CHAD_TEST(poseBuffer, backwardsSampleTruncates) {
    PoseBuffer<8> buf;
    buf.add(100, {1.0f, 0.0f, 0.0f});
    buf.add(200, {2.0f, 0.0f, 0.0f});
    buf.add(300, {3.0f, 0.0f, 0.0f});
    buf.add(150, {9.0f, 0.0f, 0.0f});
    CHECK_EQ(buf.newestMs(), uint32_t{150});
    Pose out{};
    CHECK(buf.sample(150, out));
    CHECK_NEAR(out.x, 9.0f, 1e-6);
}

CHAD_TEST(poseBuffer, survivesMillisWraparound) {
    PoseBuffer<8> buf;
    const uint32_t base = 0xFFFFFF00u;
    buf.add(base, {0.0f, 0.0f, 0.0f});
    buf.add(base + 200, {10.0f, 0.0f, 0.0f});
    CHECK(buf.covers(base + 100));
    Pose out{};
    CHECK(buf.sample(base + 100, out));
    CHECK_NEAR(out.x, 5.0f, 1e-3);
}

CHAD_TEST(poseBuffer, interpolatesAlongTheArcNotTheChord) {
    PoseBuffer<4> buf;
    const Pose start{};
    const Pose end = start.exp({20.0f, 0.0f, 1.0f});
    buf.add(0, start);
    buf.add(100, end);

    Pose mid{};
    CHECK(buf.sample(50, mid));
    const Pose expected = start.exp({10.0f, 0.0f, 0.5f});
    CHECK_NEAR(mid.x, expected.x, 2e-3);
    CHECK_NEAR(mid.y, expected.y, 2e-3);
}

CHAD_TEST(poseEstimator, withoutMeasurementsItIsJustOdometry) {
    LatencyPoseEstimator est = makeEstimator();
    uint32_t now = 0;
    est.reset({0.0f, 0.0f, 0.0f}, now);
    driveForward(est, now, 10);

    CHECK_NEAR(est.pose().x, 10.0f, 1e-3);
    CHECK_NEAR(est.pose().y, 0.0f, 1e-4);
    CHECK_NEAR(est.odometryPose().x, est.pose().x, 1e-6);
}

CHAD_TEST(poseEstimator, exactMeasurementAtTheLatestSampleWins) {
    LatencyPoseEstimator est = makeEstimator();
    uint32_t now = 0;
    est.reset({0.0f, 0.0f, 0.0f}, now);
    driveForward(est, now, 10);

    CHECK(est.addMeasurement(now, {4.0f, -2.0f, 0.3f}, kExact));
    CHECK_NEAR(est.pose().x, 4.0f, 2e-3);
    CHECK_NEAR(est.pose().y, -2.0f, 2e-3);
    CHECK_NEAR(est.pose().theta, 0.3f, 1e-4);
}

CHAD_TEST(poseEstimator, lateMeasurementIsAppliedInThePastThenReplayed) {
    LatencyPoseEstimator est = makeEstimator();
    uint32_t now = 0;
    est.reset({0.0f, 0.0f, 0.0f}, now);
    driveForward(est, now, 10);
    const uint32_t stamp = now - 50;   // five inches ago

    // The robot was three inches left of where dead reckoning thought.
    CHECK(est.addMeasurement(stamp, {5.0f, 3.0f, 0.0f}, kExact));

    CHECK_NEAR(est.pose().x, 10.0f, 3e-3);
    CHECK_NEAR(est.pose().y, 3.0f, 3e-3);
    CHECK_NEAR(est.pose().theta, 0.0f, 1e-4);
    // Odometry itself is never rewritten.
    CHECK_NEAR(est.odometryPose().y, 0.0f, 1e-4);
}

CHAD_TEST(poseEstimator, lateMeasurementRespectsHeadingChangeSince) {
    LatencyPoseEstimator est = makeEstimator();
    uint32_t now = 0;
    est.reset({0.0f, 0.0f, 0.0f}, now);
    driveForward(est, now, 5);
    const uint32_t stamp = now;

    for (int i = 0; i < 10; i++) {
        now += 10;
        est.update(now, {1.0f, 0.0f, 0.15f});
    }

    const Pose beforeFix = est.pose();
    CHECK(est.addMeasurement(stamp, {5.0f, 2.0f, 0.0f}, kExact));
    // Correcting a pure translation in the past must translate the present by
    // the same amount, not rotate it.
    CHECK_NEAR(est.pose().theta, beforeFix.theta, 1e-4);
    CHECK_NEAR(est.pose().x - beforeFix.x, 0.0f, 3e-3);
    CHECK_NEAR(est.pose().y - beforeFix.y, 2.0f, 3e-3);
}

CHAD_TEST(poseEstimator, partialTrustMovesPartWay) {
    LatencyPoseEstimator est = makeEstimator();
    uint32_t now = 0;
    est.reset({0.0f, 0.0f, 0.0f}, now);
    driveForward(est, now, 5);

    // state stddev 1 against measurement stddev 1 gives a gain of one half.
    const float stdDev[3] = {1.0f, 1.0f, 1.0f};
    CHECK(est.addMeasurement(now, {5.0f, 4.0f, 0.0f}, stdDev));
    CHECK_NEAR(est.pose().y, 2.0f, 5e-2);
}

CHAD_TEST(poseEstimator, gainFollowsTheStdDevRatio) {
    LatencyPoseEstimator::Config c;
    c.stateStdDev[0] = 1.0f;
    c.stateStdDev[1] = 1.0f;
    c.stateStdDev[2] = 1.0f;
    LatencyPoseEstimator loose(c);
    uint32_t now = 0;
    loose.reset({}, now);
    now += 10;
    loose.update(now, {0.0f, 0.0f, 0.0f});

    const float noisy[3] = {3.0f, 3.0f, 3.0f};
    CHECK(loose.addMeasurement(now, {10.0f, 0.0f, 0.0f}, noisy));
    // k = q / (q + sqrt(q r)) = 1 / (1 + 3) = 0.25
    CHECK_NEAR(loose.pose().x, 2.5f, 5e-2);
}

CHAD_TEST(poseEstimator, zeroStateStdDevPinsAnAxis) {
    LatencyPoseEstimator::Config c;
    c.stateStdDev[0] = 1.0f;
    c.stateStdDev[1] = 0.0f;
    c.stateStdDev[2] = 0.1f;
    LatencyPoseEstimator est(c);
    uint32_t now = 0;
    est.reset({}, now);
    now += 10;
    est.update(now, {1.0f, 0.0f, 0.0f});

    CHECK(est.addMeasurement(now, {5.0f, 5.0f, 0.0f}, kExact));
    CHECK_NEAR(est.pose().x, 5.0f, 1e-3);
    CHECK_NEAR(est.pose().y, 0.0f, 1e-3);
}

CHAD_TEST(poseEstimator, staleMeasurementIsRejected) {
    LatencyPoseEstimator est = makeEstimator(100);
    uint32_t now = 0;
    est.reset({}, now);
    driveForward(est, now, 30);

    const Pose before = est.pose();
    CHECK(!est.addMeasurement(5, {0.0f, 50.0f, 0.0f}, kExact));
    CHECK_NEAR(est.pose().y, before.y, 1e-6);
}

CHAD_TEST(poseEstimator, nonFiniteMeasurementIsRejected) {
    LatencyPoseEstimator est = makeEstimator();
    uint32_t now = 0;
    est.reset({}, now);
    driveForward(est, now, 3);
    const Pose before = est.pose();

    CHECK(!est.addMeasurement(now, {kNan, 0.0f, 0.0f}, kExact));
    CHECK(!est.addMeasurement(now, {0.0f, kNan, 0.0f}, kExact));
    CHECK_NEAR(est.pose().x, before.x, 1e-6);
}

CHAD_TEST(poseEstimator, futureStampClampsToTheLatestSample) {
    LatencyPoseEstimator est = makeEstimator();
    uint32_t now = 0;
    est.reset({}, now);
    driveForward(est, now, 5);

    CHECK(est.addMeasurement(now + 5000, {5.0f, 1.0f, 0.0f}, kExact));
    CHECK_NEAR(est.pose().y, 1.0f, 3e-3);
}

CHAD_TEST(poseEstimator, repeatedMeasurementsConverge) {
    LatencyPoseEstimator est = makeEstimator();
    uint32_t now = 0;
    est.reset({}, now);

    const float stdDev[3] = {1.0f, 1.0f, 1.0f};
    for (int i = 0; i < 30; i++) {
        now += 10;
        est.update(now, {0.0f, 0.0f, 0.0f});
        est.addMeasurement(now, {0.0f, 6.0f, 0.0f}, stdDev);
    }
    CHECK_NEAR(est.pose().y, 6.0f, 1e-2);
}

CHAD_TEST(poseEstimator, resetDiscardsHistoryAndCorrection) {
    LatencyPoseEstimator est = makeEstimator();
    uint32_t now = 0;
    est.reset({}, now);
    driveForward(est, now, 5);
    est.addMeasurement(now, {5.0f, 9.0f, 0.0f}, kExact);

    now += 10;
    est.reset({1.0f, 2.0f, 0.5f}, now);
    CHECK_NEAR(est.pose().x, 1.0f, 1e-6);
    CHECK_NEAR(est.pose().y, 2.0f, 1e-6);
    CHECK_NEAR(est.odometryPose().y, 2.0f, 1e-6);
    CHECK_EQ(est.historySize(), 1);
    CHECK(!est.addMeasurement(now - 500, {0.0f, 0.0f, 0.0f}, kExact));
}

CHAD_TEST(poseEstimator, historyIsBounded) {
    LatencyPoseEstimator est = makeEstimator(200);
    uint32_t now = 0;
    est.reset({}, now);
    driveForward(est, now, 500);
    CHECK(est.historySize() <= LatencyPoseEstimator::HISTORY_CAPACITY);
    CHECK(est.historySize() <= 25);
}
