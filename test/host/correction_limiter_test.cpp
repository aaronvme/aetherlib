#include "chad/util/correction_limiter.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

CorrectionLimiter::Config config() {
    CorrectionLimiter::Config c;
    c.rateInPerSec = 30.0f;
    c.burstIn = 2.0f;
    return c;
}

float len(float dx, float dy) { return std::sqrt(dx * dx + dy * dy); }
}  // namespace

CHAD_TEST(correctionLimiter, smallStepsPassUnchanged) {
    CorrectionLimiter l(config());
    // 0.12 in every 10 ms is 12 in/s, well under the rate.
    for (std::uint32_t t = 0; t <= 2000; t += 10) {
        float dx = 0.096f, dy = -0.072f;
        const float applied = l.limit(t, dx, dy);
        CHECK_NEAR(dx, 0.096f, 1e-6);
        CHECK_NEAR(dy, -0.072f, 1e-6);
        CHECK_NEAR(applied, 0.12f, 1e-5);
    }
}

CHAD_TEST(correctionLimiter, oneHugeStepIsClampedToTheBurst) {
    CorrectionLimiter l(config());
    float dx = 18.0f, dy = 0.0f;
    const float applied = l.limit(1000, dx, dy);
    CHECK_NEAR(applied, 2.0f, 1e-5);
    CHECK_NEAR(dx, 2.0f, 1e-5);
    CHECK_NEAR(dy, 0.0f, 1e-6);
}

CHAD_TEST(correctionLimiter, sustainedLargeRequestsConvergeAtExactlyTheRate) {
    CorrectionLimiter l(config());
    float total = 0.0f;
    for (std::uint32_t t = 0; t <= 1000; t += 10) {
        float dx = 5.0f, dy = 0.0f;
        const float applied = l.limit(t, dx, dy);
        if (t > 0) CHECK_NEAR(applied, 0.3f, 1e-4);   // 30 in/s * 10 ms
        total += applied;
    }
    // Burst up front, then the rate for the 1 s that followed.
    CHECK_NEAR(total, 2.0f + 30.0f * 1.0f, 1e-3);
}

CHAD_TEST(correctionLimiter, anyWindowIsBoundedByBurstPlusRateTimesT) {
    CorrectionLimiter l(config());
    // Irregular periods and mixed request sizes.
    const std::uint32_t steps[] = {3, 10, 27, 1, 40, 10, 10, 5, 90, 10, 2, 33};
    const float sizes[] = {0.1f, 6.0f, 0.4f, 3.0f, 9.0f, 0.05f, 1.0f, 4.0f, 7.0f, 0.2f, 5.0f, 2.5f};
    constexpr int kN = 400;
    std::uint32_t times[kN];
    float applied[kN];
    std::uint32_t t = 500;
    for (int i = 0; i < kN; i++) {
        t += steps[i % 12];
        float dx = sizes[i % 12] * 0.6f, dy = -sizes[i % 12] * 0.8f;
        times[i] = t;
        applied[i] = l.limit(t, dx, dy);
    }
    float worstExcess = -1e9f;
    for (int a = 0; a < kN; a++) {
        float sum = 0.0f;
        for (int b = a; b < kN; b++) {
            sum += applied[b];
            const float T = static_cast<float>(times[b] - times[a]) * 0.001f;
            const float excess = sum - (2.0f + 30.0f * T);
            if (excess > worstExcess) worstExcess = excess;
        }
    }
    CHECK(worstExcess <= 1e-3f);
}

CHAD_TEST(correctionLimiter, budgetRefillsButNeverAboveTheBurst) {
    CorrectionLimiter l(config());
    float dx = 10.0f, dy = 0.0f;
    CHECK_NEAR(l.limit(0, dx, dy), 2.0f, 1e-5);   // drained

    // 40 ms later: 1.2 in back.
    dx = 10.0f; dy = 0.0f;
    CHECK_NEAR(l.limit(40, dx, dy), 1.2f, 1e-4);

    // A long quiet spell refills to the burst and no further.
    dx = 0.0f; dy = 0.0f;
    l.limit(100, dx, dy);
    dx = 10.0f; dy = 0.0f;
    CHECK_NEAR(l.limit(60000, dx, dy), 2.0f, 1e-5);
    CHECK_NEAR(l.available(60000), 0.0f, 1e-5);
    CHECK_NEAR(l.available(60010), 0.3f, 1e-4);
    CHECK_NEAR(l.available(70000), 2.0f, 1e-5);
}

CHAD_TEST(correctionLimiter, scalingPreservesDirection) {
    CorrectionLimiter l(config());
    float dx = -9.0f, dy = 12.0f;                  // 15 in at (-0.6, 0.8)
    const float applied = l.limit(0, dx, dy);
    CHECK_NEAR(applied, 2.0f, 1e-5);
    CHECK_NEAR(len(dx, dy), 2.0f, 1e-5);
    CHECK_NEAR(dx, -1.2f, 1e-5);
    CHECK_NEAR(dy, 1.6f, 1e-5);
}

CHAD_TEST(correctionLimiter, nonFiniteRequestsApplyNothingAndDoNotPoisonState) {
    CorrectionLimiter l(config());
    const float bad[][2] = {{kNaN, 1.0f}, {1.0f, kNaN}, {kInf, 0.0f}, {0.0f, -kInf}};
    std::uint32_t t = 0;
    for (const auto& b : bad) {
        float dx = b[0], dy = b[1];
        CHECK_NEAR(l.limit(t, dx, dy), 0.0f, 0.0);
        CHECK_EQ(dx, 0.0f);
        CHECK_EQ(dy, 0.0f);
        t += 10;
    }
    // The bucket is still whole and still finite.
    CHECK_NEAR(l.available(t), 2.0f, 1e-5);
    float dx = 1.0f, dy = 0.0f;
    CHECK_NEAR(l.limit(t, dx, dy), 1.0f, 1e-6);
    CHECK_NEAR(dx, 1.0f, 1e-6);
}

CHAD_TEST(correctionLimiter, resetRestoresAFullBudget) {
    CorrectionLimiter l(config());
    float dx = 50.0f, dy = 0.0f;
    l.limit(100, dx, dy);
    CHECK_NEAR(l.available(100), 0.0f, 1e-6);
    l.reset();
    dx = 50.0f; dy = 0.0f;
    CHECK_NEAR(l.limit(101, dx, dy), 2.0f, 1e-5);
}

CHAD_TEST(correctionLimiter, millisWrapIsAnOrdinaryPeriod) {
    CorrectionLimiter l(config());
    const std::uint32_t before = 0xFFFFFFFFu - 4u;   // 5 ms short of the wrap
    float dx = 50.0f, dy = 0.0f;
    CHECK_NEAR(l.limit(before, dx, dy), 2.0f, 1e-5);
    dx = 50.0f; dy = 0.0f;
    // 10 ms later, across the wrap.
    CHECK_NEAR(l.limit(before + 10u, dx, dy), 0.3f, 1e-4);
}

CHAD_TEST(correctionLimiter, aSameMillisecondCallGetsNoRefill) {
    CorrectionLimiter l(config());
    float dx = 1.5f, dy = 0.0f;
    CHECK_NEAR(l.limit(7, dx, dy), 1.5f, 1e-6);
    dx = 1.5f; dy = 0.0f;
    CHECK_NEAR(l.limit(7, dx, dy), 0.5f, 1e-5);
}

CHAD_TEST(correctionLimiter, aNonPositiveRateOrBurstDisablesTheGuard) {
    CorrectionLimiter::Config c = config();
    c.rateInPerSec = 0.0f;
    CorrectionLimiter off(c);
    float dx = 30.0f, dy = 40.0f;
    CHECK_NEAR(off.limit(0, dx, dy), 50.0f, 1e-4);
    CHECK_NEAR(dx, 30.0f, 1e-6);

    c = config();
    c.burstIn = 0.0f;
    CorrectionLimiter off2(c);
    dx = 30.0f; dy = 40.0f;
    CHECK_NEAR(off2.limit(0, dx, dy), 50.0f, 1e-4);
}

CHAD_TEST(correctionLimiter, shrinkingTheBurstClampsTheHeldBudget) {
    CorrectionLimiter l(config());
    float dx = 0.0f, dy = 0.0f;
    l.limit(0, dx, dy);
    CorrectionLimiter::Config c = config();
    c.burstIn = 0.5f;
    l.setConfig(c);
    dx = 10.0f; dy = 0.0f;
    CHECK_NEAR(l.limit(0, dx, dy), 0.5f, 1e-6);
}
