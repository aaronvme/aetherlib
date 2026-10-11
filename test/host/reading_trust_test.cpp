#include "chad/util/reading_trust.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {
using V = ReadingTrust::Verdict;
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kRange = 20.0f;   // tolerance there: 2 + 0.05 * 20 = 3 in

ReadingTrust::Config cfg() { return ReadingTrust::Config{}; }

/// Feed one sample of `innov` at `t`, judged the way RclTracking judges it.
float feed(ReadingTrust& r, float innov, std::uint32_t t, float range = kRange) {
    const auto c = cfg();
    return r.observe(ReadingTrust::judge(innov, range, c), innov, range, t, c);
}

/// A sensor that has been reading the wall cleanly up to `until`.
ReadingTrust settled(std::uint32_t until) {
    ReadingTrust r;
    for (std::uint32_t t = 0; t <= until; t += 30) feed(r, 0.3f, t);
    return r;
}
}  // namespace

CHAD_TEST(readingTrust, judgeBandsAreAsymmetric) {
    const auto c = cfg();
    CHECK_NEAR(ReadingTrust::tolerance(kRange, c), 3.0f, 1e-5);
    CHECK(ReadingTrust::judge(0.0f, kRange, c) == V::Consistent);
    CHECK(ReadingTrust::judge(-2.9f, kRange, c) == V::Consistent);
    CHECK(ReadingTrust::judge(2.9f, kRange, c) == V::Consistent);
    CHECK(ReadingTrust::judge(-3.5f, kRange, c) == V::Short);
    // Long inside the innovation gate is a plausible pose correction.
    CHECK(ReadingTrust::judge(5.0f, kRange, c) == V::None);
    CHECK(ReadingTrust::judge(9.0f, kRange, c) == V::Long);
    // The band widens with range, like the sensor's own error.
    CHECK(ReadingTrust::judge(-4.0f, 60.0f, c) == V::Consistent);
}

CHAD_TEST(readingTrust, consistentReadingsKeepFullTrust) {
    ReadingTrust r;
    for (std::uint32_t t = 0; t <= 3000; t += 30) {
        const float innov = ((t / 30) % 2) ? 1.2f : -1.4f;
        CHECK_NEAR(feed(r, innov, t), 1.0f, 1e-6);
    }
    CHECK(r.usable(cfg()));
}

CHAD_TEST(readingTrust, persistentShortIsExcludedWithinFourSamples) {
    ReadingTrust r = settled(600);
    // A cup 5 in in front of the wall, at the device rate.
    CHECK_NEAR(feed(r, -5.0f, 630), 1.0f, 1e-6);   // one sample is a spike
    feed(r, -5.0f, 660);
    feed(r, -5.0f, 690);
    CHECK(!r.usable(cfg()));
    CHECK(r.trust() < 0.5f);
    feed(r, -5.0f, 720);
    CHECK(r.trust() < 0.1f);
}

CHAD_TEST(readingTrust, oneSpikeBarelyMovesTrust) {
    ReadingTrust r = settled(600);
    feed(r, -6.0f, 630);
    CHECK(r.trust() > 0.95f);
    for (std::uint32_t t = 660; t <= 1500; t += 30) feed(r, 0.2f, t);
    CHECK_NEAR(r.trust(), 1.0f, 1e-6);
    // ... and a second, well after the first was paid back, is a spike too.
    feed(r, -6.0f, 1530);
    CHECK(r.trust() > 0.95f);
}

CHAD_TEST(readingTrust, grossShortDropsTheSensorOnItsFirstSample) {
    ReadingTrust r = settled(600);
    // A robot pulled up 40 in in front of a wall 45 in away.
    feed(r, -40.0f, 630, 45.0f);
    CHECK(!r.usable(cfg()));
}

CHAD_TEST(readingTrust, recoveryNeedsARunOfConsistentReadings) {
    ReadingTrust r = settled(600);
    std::uint32_t t = 630;
    for (; t <= 900; t += 30) feed(r, -6.0f, t);
    CHECK_NEAR(r.trust(), 0.0f, 1e-6);

    // Inside the hold nothing is paid back.
    const std::uint32_t clear = t;
    for (; t < clear + 150; t += 30) feed(r, 0.1f, t);
    CHECK_NEAR(r.trust(), 0.0f, 1e-6);

    // One short sample during the rebuild restarts the hold.
    feed(r, -6.0f, t);
    t += 30;
    for (const std::uint32_t again = t; t < again + 150; t += 30) feed(r, 0.1f, t);
    CHECK(r.trust() < 0.1f);

    // Past the hold it rebuilds over recoverMs, not instantly.
    const std::uint32_t hold = t;
    bool usableEarly = false;
    for (; t < hold + 60; t += 30) usableEarly |= feed(r, 0.1f, t) >= 0.5f;
    CHECK(!usableEarly);
    for (; t < hold + 330; t += 30) feed(r, 0.1f, t);
    CHECK(r.usable(cfg()));
    for (; t < hold + 600; t += 30) feed(r, 0.1f, t);
    CHECK_NEAR(r.trust(), 1.0f, 1e-6);
}

CHAD_TEST(readingTrust, longIsSlowerThanShort) {
    ReadingTrust s = settled(600), l = settled(600);
    for (std::uint32_t t = 630; t <= 750; t += 30) {
        feed(s, -12.0f, t);
        feed(l, 12.0f, t);
    }
    CHECK(!s.usable(cfg()));
    CHECK_NEAR(l.trust(), 1.0f, 1e-6);          // 150 ms of beam miss: still in grace
    for (std::uint32_t t = 780; t <= 1300; t += 30) feed(l, 12.0f, t);
    CHECK(!l.usable(cfg()));                    // half a second of it is not
}

CHAD_TEST(readingTrust, longInsideTheGateNeverCostsTrust) {
    ReadingTrust r = settled(600);
    for (std::uint32_t t = 630; t <= 3000; t += 30) feed(r, 6.0f, t);
    CHECK_NEAR(r.trust(), 1.0f, 1e-6);
}

CHAD_TEST(readingTrust, chargedByElapsedTimeNotSampleCount) {
    ReadingTrust a = settled(600), b = settled(600);
    for (std::uint32_t t = 610; t <= 690; t += 10) feed(a, -5.0f, t);   // 9 samples
    for (std::uint32_t t = 630; t <= 690; t += 30) feed(b, -5.0f, t);   // 3 samples
    CHECK_NEAR(a.trust(), b.trust(), 1e-4);
    CHECK(a.trust() < 0.5f);

    // A gap is charged as at most two device periods, not its full length.
    ReadingTrust g = settled(600);
    feed(g, -5.0f, 5000);
    CHECK(g.trust() > 0.0f);
    CHECK(g.usable(cfg()));
}

CHAD_TEST(readingTrust, survivesAMillisWrap) {
    ReadingTrust r;
    const std::uint32_t t0 = 0xFFFFFFFFu - 50u;
    feed(r, 0.0f, t0);
    feed(r, -5.0f, t0 + 30u);
    feed(r, -5.0f, t0 + 60u);       // wrapped
    feed(r, -5.0f, t0 + 90u);
    CHECK(!r.usable(cfg()));
}

CHAD_TEST(readingTrust, nonFiniteInputIsNoInformation) {
    ReadingTrust r = settled(600);
    const auto c = cfg();
    CHECK(ReadingTrust::judge(kNaN, kRange, c) == V::None);
    CHECK(ReadingTrust::judge(-5.0f, kNaN, c) == V::None);
    for (std::uint32_t t = 630; t <= 2000; t += 30) feed(r, kNaN, t);
    CHECK_NEAR(r.trust(), 1.0f, 1e-6);
    // None does not advance the sample clock either: the next real sample is
    // charged as a gap, not as 1.4 s of disagreement.
    feed(r, -5.0f, 2030);
    CHECK(r.trust() > 0.5f);
}

CHAD_TEST(readingTrust, floorAtOrBelowZeroNeverExcludes) {
    ReadingTrust r = settled(600);
    for (std::uint32_t t = 630; t <= 900; t += 30) feed(r, -20.0f, t);
    auto c = cfg();
    c.floor = 0.0f;
    CHECK_NEAR(r.trust(), 0.0f, 1e-6);
    CHECK(r.usable(c));
    r.reset();
    CHECK_NEAR(r.trust(), 1.0f, 1e-6);
}

CHAD_TEST(readingTrust, accuracyMatchesTheV5Rating) {
    CHECK_NEAR(distanceAccuracyIn(100.0f / 25.4f), 15.0f / 25.4f, 1e-5);
    CHECK_NEAR(distanceAccuracyIn(1000.0f / 25.4f), 50.0f / 25.4f, 1e-4);
    CHECK_NEAR(distanceAccuracyIn(-3.0f), 15.0f / 25.4f, 1e-5);
}

CHAD_TEST(readingTrust, weightedEstimateIsDominatedByTrustedSensors) {
    const float v[2] = {-48.0f, -43.0f};
    const float w[2] = {readingWeight(1.0f, 20.0f, false), readingWeight(0.05f, 20.0f, false)};
    const float m = weightedMean(v, w, 2);
    CHECK(std::fabs(m - -48.0f) < 0.3f);

    // Same trust: the near reading, rated 0.6 in, outweighs one at 60 in, 3 in.
    const float wr[2] = {readingWeight(1.0f, 8.0f, true), readingWeight(1.0f, 60.0f, true)};
    const float mr = weightedMean(v, wr, 2);
    CHECK(std::fabs(mr - -48.0f) < 0.3f);
    // Without range weighting it is the plain mean.
    const float wu[2] = {readingWeight(1.0f, 8.0f, false), readingWeight(1.0f, 60.0f, false)};
    CHECK_NEAR(weightedMean(v, wu, 2), -45.5f, 1e-5);
}

CHAD_TEST(readingTrust, degenerateWeightsFallBackToThePlainMean) {
    const float v[3] = {1.0f, 2.0f, 6.0f};
    const float w[3] = {0.0f, kNaN, -1.0f};
    CHECK_NEAR(weightedMean(v, w, 3), 3.0f, 1e-6);
    CHECK_NEAR(weightedMean(v, w, 0), 0.0f, 1e-6);
    CHECK_NEAR(readingWeight(kNaN, 10.0f, true), 0.0f, 1e-9);
    CHECK_FINITE(readingWeight(1.0f, kNaN, true));
}
