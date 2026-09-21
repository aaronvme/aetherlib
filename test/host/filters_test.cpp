#include "chad/util/filters.hpp"

#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
}  // namespace

CHAD_TEST(slew, respectsSeparateRiseAndFallRates) {
    SlewRateLimiter limiter{10.0f, 100.0f};
    CHECK_NEAR(limiter.calculate(50.0f, 0.1f), 1.0f, 1e-5);
    CHECK_NEAR(limiter.calculate(50.0f, 0.1f), 2.0f, 1e-5);
    CHECK_NEAR(limiter.calculate(0.0f, 0.1f), 0.0f, 1e-5);
}

CHAD_TEST(slew, reachesTargetAndHolds) {
    SlewRateLimiter limiter{100.0f, 100.0f};
    for (int i = 0; i < 50; i++) limiter.calculate(5.0f, 0.01f);
    CHECK_NEAR(limiter.value(), 5.0f, 1e-4);
    CHECK_NEAR(limiter.calculate(5.0f, 0.01f), 5.0f, 1e-5);
}

CHAD_TEST(slew, degenerateInputsAreInert) {
    SlewRateLimiter limiter{10.0f, 10.0f, 3.0f};
    CHECK_EQ(limiter.calculate(99.0f, 0.0f), 3.0f);
    CHECK_EQ(limiter.calculate(99.0f, -1.0f), 3.0f);
    CHECK_EQ(limiter.calculate(kNan, 0.1f), 3.0f);
    CHECK_EQ(limiter.calculate(kInf, 0.1f), 3.0f);

    SlewRateLimiter unlimited{0.0f, 0.0f};
    CHECK_EQ(unlimited.calculate(42.0f, 0.01f), 42.0f);
}

CHAD_TEST(slew, resetJumpsWithoutRamping) {
    SlewRateLimiter limiter{1.0f, 1.0f};
    limiter.reset(20.0f);
    CHECK_EQ(limiter.value(), 20.0f);
}

CHAD_TEST(movingAverage, averagesPartialThenFullWindow) {
    MovingAverage<4> avg;
    CHECK_NEAR(avg.calculate(4.0f), 4.0f, 1e-6);
    CHECK_NEAR(avg.calculate(8.0f), 6.0f, 1e-6);
    CHECK(!avg.full());
    avg.calculate(0.0f);
    CHECK_NEAR(avg.calculate(0.0f), 3.0f, 1e-6);
    CHECK(avg.full());
}

CHAD_TEST(movingAverage, staysCorrectAfterManyWraps) {
    MovingAverage<3> avg;
    for (int i = 0; i < 100; i++) avg.calculate(static_cast<float>(i));
    CHECK_NEAR(avg.value(), 98.0f, 1e-4);
    avg.reset();
    CHECK_EQ(avg.value(), 0.0f);
    CHECK_EQ(avg.count(), std::size_t{0});
}

CHAD_TEST(movingAverage, ignoresNonFinite) {
    MovingAverage<3> avg;
    avg.calculate(2.0f);
    CHECK_NEAR(avg.calculate(kNan), 2.0f, 1e-6);
    CHECK_NEAR(avg.value(), 2.0f, 1e-6);
}

CHAD_TEST(median, rejectsSingleSampleSpike) {
    MedianFilter<5> med;
    for (float v : {10.0f, 10.2f, 9.8f, 10.1f}) med.calculate(v);
    const float spiked = med.calculate(9000.0f);
    CHECK(spiked > 9.5f && spiked < 10.5f);
}

CHAD_TEST(median, evenWindowAveragesMiddlePair) {
    MedianFilter<4> med;
    med.calculate(1.0f);
    med.calculate(2.0f);
    med.calculate(3.0f);
    CHECK_NEAR(med.calculate(4.0f), 2.5f, 1e-6);
}

CHAD_TEST(median, partialWindowUsesOnlyRealSamples) {
    MedianFilter<7> med;
    CHECK_NEAR(med.calculate(5.0f), 5.0f, 1e-6);
    CHECK_NEAR(med.calculate(9.0f), 7.0f, 1e-6);
    CHECK(!med.full());
}

CHAD_TEST(lowPass, stepReachesOneTimeConstant) {
    LowPassFilter lp{0.5f};
    lp.calculate(0.0f, 0.01f);
    float t = 0.0f;
    while (t < 0.5f) {
        lp.calculate(1.0f, 0.001f);
        t += 0.001f;
    }
    CHECK_NEAR(lp.value(), 1.0f - std::exp(-1.0f), 2e-2);
}

CHAD_TEST(lowPass, firstSampleDoesNotRampFromZero) {
    LowPassFilter lp{1.0f};
    CHECK_NEAR(lp.calculate(37.0f, 0.01f), 37.0f, 1e-6);
}

CHAD_TEST(lowPass, degenerateInputsPassThrough) {
    // No time constant is no filtering, so this one really does pass through.
    LowPassFilter lp{0.0f};
    CHECK_EQ(lp.calculate(5.0f, 0.01f), 5.0f);

    // A zero dt is not new information and must not wipe the state -- two
    // samples inside one millisecond tick would otherwise hand back the raw
    // input and undo everything the filter had accumulated.
    LowPassFilter jitter{0.2f};
    jitter.calculate(0.0f, 0.01f);
    CHECK_EQ(jitter.calculate(9.0f, 0.0f), 0.0f);
    CHECK_EQ(jitter.value(), 0.0f);
    CHECK_EQ(jitter.calculate(kNan, 0.01f), 0.0f);
}

CHAD_TEST(lowPass, seededResetSurvivesTheNextSample) {
    LowPassFilter lp{1.0f};
    lp.reset(10.0f);
    // Filtered against the seed, not replaced by the sample.
    const float out = lp.calculate(20.0f, 0.01f);
    CHECK(out > 10.0f);
    CHECK(out < 11.0f);
}

CHAD_TEST(slew, reversalDeceleratesAtTheFallRate) {
    // Rise is slow, fall is fast. Coming down from +10 to -10 must use the
    // fall rate on the way to zero, not the rise rate.
    SlewRateLimiter limiter{1.0f, 100.0f, 10.0f};
    CHECK_NEAR(limiter.calculate(-10.0f, 0.1f), 0.0f, 1e-5);
    // Below zero it is building magnitude again, so the rise rate applies.
    CHECK_NEAR(limiter.calculate(-10.0f, 0.1f), -0.1f, 1e-5);
}

CHAD_TEST(lowPass, dtIsHonouredNotAssumed) {
    LowPassFilter coarse{0.2f};
    LowPassFilter fine{0.2f};
    coarse.calculate(0.0f, 0.01f);
    fine.calculate(0.0f, 0.01f);
    coarse.calculate(1.0f, 0.04f);
    for (int i = 0; i < 4; i++) fine.calculate(1.0f, 0.01f);
    CHECK_NEAR(coarse.value(), fine.value(), 2e-2);
}

CHAD_TEST(highPass, rejectsSteadyLevelAndPassesChange) {
    HighPassFilter hp{0.1f};
    hp.calculate(5.0f, 0.01f);
    for (int i = 0; i < 200; i++) hp.calculate(5.0f, 0.01f);
    CHECK_NEAR(hp.calculate(5.0f, 0.01f), 0.0f, 1e-3);
    CHECK(std::fabs(hp.calculate(9.0f, 0.01f)) > 3.0f);
}

CHAD_TEST(derivative, measuresConstantSlope) {
    Derivative d;
    float x = 0.0f;
    CHECK_EQ(d.calculate(x, 0.01f), 0.0f);
    for (int i = 0; i < 20; i++) {
        x += 0.25f;
        d.calculate(x, 0.01f);
    }
    CHECK_NEAR(d.value(), 25.0f, 1e-3);
}

CHAD_TEST(derivative, degenerateInputsHold) {
    Derivative d;
    d.calculate(0.0f, 0.01f);
    d.calculate(1.0f, 0.01f);
    const float held = d.value();
    CHECK_EQ(d.calculate(2.0f, 0.0f), held);
    CHECK_EQ(d.calculate(kInf, 0.01f), held);
    d.reset();
    CHECK_EQ(d.value(), 0.0f);
}

CHAD_TEST(derivative, smoothingLagsButConverges) {
    Derivative smoothed{0.05f};
    float x = 0.0f;
    smoothed.calculate(x, 0.01f);
    x += 0.1f;
    const float firstStep = smoothed.calculate(x, 0.01f);
    CHECK(firstStep <= 10.0f + 1e-3f);
    for (int i = 0; i < 200; i++) {
        x += 0.1f;
        smoothed.calculate(x, 0.01f);
    }
    CHECK_NEAR(smoothed.value(), 10.0f, 0.2);
}

CHAD_TEST(debouncer, risingEdgeMustHold) {
    Debouncer d{0.05f, Debouncer::Edge::Rising};
    CHECK(!d.calculate(true, 0.01f));
    CHECK(!d.calculate(true, 0.01f));
    CHECK(!d.calculate(true, 0.02f));
    CHECK(d.calculate(true, 0.02f));
    CHECK(!d.calculate(false, 0.01f));
}

CHAD_TEST(debouncer, interruptedHoldRestarts) {
    Debouncer d{0.05f, Debouncer::Edge::Rising};
    d.calculate(true, 0.04f);
    d.calculate(false, 0.01f);
    CHECK(!d.calculate(true, 0.04f));
    CHECK(d.calculate(true, 0.02f));
}

CHAD_TEST(debouncer, bothEdgesAreWatched) {
    Debouncer d{0.02f, Debouncer::Edge::Both, false};
    CHECK(d.calculate(true, 0.03f));
    CHECK(d.calculate(false, 0.01f));
    CHECK(!d.calculate(false, 0.03f));
}

CHAD_TEST(debouncer, resetForcesState) {
    Debouncer d{1.0f, Debouncer::Edge::Rising};
    d.reset(true);
    CHECK(d.value());
}
