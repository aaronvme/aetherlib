#include "chad/util/pid.hpp"

#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

PidCore proportionalOnly(float kP) { return PidCore(PIDGains{kP, 0.0f, 0.0f}); }
}  // namespace

CHAD_TEST(pid, proportionalIsExact) {
    PidCore pid = proportionalOnly(2.5f);
    CHECK_NEAR(pid.update(4.0f, 0.01f), 10.0f, 1e-6);
    CHECK_NEAR(pid.update(-4.0f, 0.01f), -10.0f, 1e-6);
    CHECK_NEAR(pid.update(0.0f, 0.01f), 0.0f, 1e-6);
}

CHAD_TEST(pid, firstTickHasNoDerivativeSpike) {
    PidCore pid(PIDGains{0.0f, 0.0f, 1.0f});
    pid.setDerivativeFilter(1.0f);
    // Differencing a 24 inch error against a zeroed history would report
    // 2400 in/s on tick one.
    CHECK_NEAR(pid.update(24.0f, 0.01f), 0.0f, 1e-6);
    CHECK_NEAR(pid.update(24.0f, 0.01f), 0.0f, 1e-6);
    CHECK_NEAR(pid.update(23.0f, 0.01f), -100.0f, 1e-3);
}

CHAD_TEST(pid, integralAccumulatesWithTime) {
    PidCore pid(PIDGains{0.0f, 2.0f, 0.0f});
    pid.update(1.0f, 0.5f);
    CHECK_NEAR(pid.integral(), 0.5f, 1e-6);
    CHECK_NEAR(pid.output(), 1.0f, 1e-6);
    pid.update(1.0f, 0.5f);
    CHECK_NEAR(pid.output(), 2.0f, 1e-6);
}

CHAD_TEST(pid, integralClampBounds) {
    PidCore pid(PIDGains{0.0f, 1.0f, 0.0f});
    pid.setIntegralClamp(0.75f);
    for (int i = 0; i < 100; i++) pid.update(5.0f, 0.01f);
    CHECK_NEAR(pid.integral(), 0.75f, 1e-6);
    for (int i = 0; i < 400; i++) pid.update(-5.0f, 0.01f);
    CHECK_NEAR(pid.integral(), -0.75f, 1e-6);
}

CHAD_TEST(pid, integralZoneKeepsTheApproachClean) {
    PidCore pid(PIDGains{0.0f, 1.0f, 0.0f});
    pid.setIntegralZone(2.0f);
    for (int i = 0; i < 50; i++) pid.update(20.0f, 0.01f);
    CHECK_NEAR(pid.integral(), 0.0f, 1e-9);
    for (int i = 0; i < 50; i++) pid.update(1.0f, 0.01f);
    CHECK_NEAR(pid.integral(), 0.5f, 1e-5);
}

CHAD_TEST(pid, signFlipCanDumpTheIntegral) {
    PidCore pid(PIDGains{0.0f, 1.0f, 0.0f});
    for (int i = 0; i < 20; i++) pid.update(3.0f, 0.01f);
    CHECK(pid.integral() > 0.5f);

    pid.setResetIntegralOnSignFlip(true);
    pid.update(-1.0f, 0.01f);
    CHECK_NEAR(pid.integral(), -0.01f, 1e-5);
}

CHAD_TEST(pid, outputLimitSaturates) {
    PidCore pid = proportionalOnly(100.0f);
    pid.setOutputLimit(12.0f);
    CHECK_NEAR(pid.update(5.0f, 0.01f), 12.0f, 1e-6);
    CHECK_NEAR(pid.update(-5.0f, 0.01f), -12.0f, 1e-6);
    CHECK_NEAR(pid.update(0.05f, 0.01f), 5.0f, 1e-6);
}

CHAD_TEST(pid, deadSensorDoesNotPoisonTheLoop) {
    PidCore pid(PIDGains{1.0f, 1.0f, 1.0f});
    pid.update(2.0f, 0.01f);
    const float good = pid.output();
    const float integral = pid.integral();

    CHECK_EQ(pid.update(kNan, 0.01f), good);
    CHECK_EQ(pid.update(kInf, 0.01f), good);
    CHECK_EQ(pid.integral(), integral);
    CHECK_FINITE(pid.update(2.0f, 0.01f));
}

CHAD_TEST(pid, zeroAndNegativeDtAreInert) {
    PidCore pid(PIDGains{1.0f, 1.0f, 1.0f});
    pid.update(1.0f, 0.01f);
    const float integral = pid.integral();
    pid.update(1.0f, 0.0f);
    CHECK_EQ(pid.integral(), integral);
    pid.update(1.0f, -0.5f);
    CHECK_EQ(pid.integral(), integral);
    CHECK_FINITE(pid.output());
}

CHAD_TEST(pid, derivativeFilterIsClampedToUsefulRange) {
    PidCore pid;
    pid.setDerivativeFilter(-5.0f);
    CHECK_NEAR(pid.derivativeFilter(), 0.01f, 1e-9);
    pid.setDerivativeFilter(50.0f);
    CHECK_NEAR(pid.derivativeFilter(), 1.0f, 1e-9);
}

CHAD_TEST(pid, filteredDerivativeLagsTheRawOne) {
    PidCore raw(PIDGains{0.0f, 0.0f, 1.0f});
    raw.setDerivativeFilter(1.0f);
    PidCore damped(PIDGains{0.0f, 0.0f, 1.0f});
    damped.setDerivativeFilter(0.1f);

    float e = 10.0f;
    for (int i = 0; i < 3; i++) {
        e -= 0.1f;
        raw.update(e, 0.01f);
        damped.update(e, 0.01f);
    }
    CHECK(std::fabs(damped.derivative()) < std::fabs(raw.derivative()));

    for (int i = 0; i < 200; i++) {
        e -= 0.1f;
        damped.update(e, 0.01f);
    }
    CHECK_NEAR(damped.derivative(), -10.0f, 0.2);
}

CHAD_TEST(pid, resetClearsOrFlipsTheIntegral) {
    PidCore zeroing(PIDGains{0.0f, 1.0f, 0.0f});
    for (int i = 0; i < 10; i++) zeroing.update(1.0f, 0.01f);
    zeroing.reset();
    CHECK_NEAR(zeroing.integral(), 0.0f, 1e-9);

    PidCore flipping(PIDGains{0.0f, 1.0f, 0.0f}, 0.0f, true);
    for (int i = 0; i < 10; i++) flipping.update(1.0f, 0.01f);
    const float before = flipping.integral();
    flipping.reset();
    CHECK_NEAR(flipping.integral(), -before, 1e-9);
}

CHAD_TEST(pid, resetRearmsTheDerivativeGuard) {
    PidCore pid(PIDGains{0.0f, 0.0f, 1.0f});
    pid.setDerivativeFilter(1.0f);
    pid.update(5.0f, 0.01f);
    pid.update(4.0f, 0.01f);
    CHECK(std::fabs(pid.derivative()) > 50.0f);

    pid.reset();
    CHECK_NEAR(pid.update(20.0f, 0.01f), 0.0f, 1e-6);
}

CHAD_TEST(pid, convergesOnAFirstOrderPlant) {
    PidCore pid(PIDGains{4.0f, 2.0f, 0.2f});
    pid.setOutputLimit(12.0f);
    pid.setIntegralZone(5.0f);

    float position = 0.0f;
    const float target = 10.0f;
    const float dt = 0.01f;
    for (int i = 0; i < 2000; i++) {
        const float command = pid.update(target - position, dt);
        position += command * 0.5f * dt;   // first-order, velocity ~ command
    }
    CHECK_NEAR(position, target, 0.05);
}
