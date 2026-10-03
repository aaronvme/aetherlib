#include "chad/util/pid.hpp"

#include <cmath>
#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {
constexpr float kDt = 0.01f;
}  // namespace

CHAD_TEST(pid, windupRangeClampsTheIntegral) {
    float integral = 0.0f;
    for (int i = 0; i < 10000; i++) {
        integral = PID::accumulateIntegral(integral, 50.0f, kDt, 0.5f, 3.0f);
    }
    CHECK_NEAR(integral, 3.0f, 1e-6);
}

CHAD_TEST(pid, integralTermIsBoundedWithoutAWindupRange) {
    // Held against a wall for a whole match: 60 in of error for 105 s would
    // bank a 252-unit kI term.
    const float kI = 0.04f;
    float integral = 0.0f;
    for (int i = 0; i < 10500; i++) {
        integral = PID::accumulateIntegral(integral, 60.0f, kDt, kI, 0.0f);
    }
    CHECK_NEAR(kI * integral, PID::kIntegralOutputLimit, 1e-3);

    integral = 0.0f;
    for (int i = 0; i < 10500; i++) {
        integral = PID::accumulateIntegral(integral, -60.0f, kDt, -kI, 0.0f);
    }
    CHECK_NEAR(std::fabs(kI * integral), PID::kIntegralOutputLimit, 1e-3);
}

CHAD_TEST(pid, integralBelowTheBoundIsUntouched) {
    float integral = 0.0f;
    for (int i = 0; i < 100; i++) {
        integral = PID::accumulateIntegral(integral, 2.0f, kDt, 0.1f, 0.0f);
    }
    CHECK_NEAR(integral, 2.0f, 1e-4);
}

CHAD_TEST(pid, zeroKiStoresNothing) {
    // Nothing banked for a later setGains() with kI > 0 to release at once.
    float integral = 0.0f;
    for (int i = 0; i < 1000; i++) {
        integral = PID::accumulateIntegral(integral, 100.0f, kDt, 0.0f, 0.0f);
    }
    CHECK_EQ(integral, 0.0f);
}

CHAD_TEST(pid, nonFiniteErrorOutputsZeroAndLeavesStateAlone) {
    const float nan = std::numeric_limits<float>::quiet_NaN();

    PID d(0.0f, 0.0f, 1.0f);
    d.setDerivativeFilter(1.0f);
    CHECK_EQ(d.step(1.0f, kDt), 0.0f);
    CHECK_EQ(d.step(nan, kDt), 0.0f);
    CHECK_EQ(d.step(std::numeric_limits<float>::infinity(), kDt), 0.0f);
    CHECK_NEAR(d.step(2.0f, kDt), (2.0f - 1.0f) / kDt, 1e-3);

    PID i(0.0f, 1.0f, 0.0f);
    CHECK_NEAR(i.step(1.0f, kDt), 0.01f, 1e-7);
    CHECK_EQ(i.step(nan, kDt), 0.0f);
    CHECK_NEAR(i.step(1.0f, kDt), 0.02f, 1e-7);
}
