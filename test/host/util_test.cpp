#include "chad/util/util.hpp"

#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
}  // namespace

CHAD_TEST(util, wrapRadRange) {
    for (int i = -40; i <= 40; i++) {
        const float raw = static_cast<float>(i) * 0.7f;
        const float w = wrapRad(raw);
        CHECK(w > -PI - 1e-5f && w <= PI + 1e-5f);
        CHECK_NEAR(std::sin(w), std::sin(raw), 1e-4);
        CHECK_NEAR(std::cos(w), std::cos(raw), 1e-4);
    }
}

CHAD_TEST(util, wrapReducesInOneStepNotByRepeatedSubtraction) {
    // 1e9 and 3.4e30 are past 2^27, where subtracting 2*PI rounds back to the
    // same float and a subtractive loop never terminates; 1e7 and 5e7 only take
    // ~1e6 turns. Reaching the assertions at all is most of what this checks.
    for (float big : {1.0e7f, 5.0e7f, 1.0e9f, 3.4e30f, -1.0e9f, -3.4e30f}) {
        const float w = wrapRad(big);
        CHECK(w >= -PI - 1e-5f && w <= PI + 1e-5f);
        const float p = wrapRadPositive(big);
        CHECK(p >= 0.0f && p < TWO_PI);
        const float d = wrapDeg(big);
        CHECK(d >= 0.0f && d < 360.0f);
        CHECK(std::isfinite(botHeadingToTrig(big)));
    }

    // Exactness is only meaningful where the input still resolves the angle.
    for (int i = -400; i <= 400; i++) {
        const float raw = static_cast<float>(i) * 1.9f;
        const float w = wrapRad(raw);
        CHECK(w >= -PI - 1e-5f && w <= PI + 1e-5f);
        CHECK_NEAR(std::sin(w), std::sin(raw), 2e-4);
        CHECK_NEAR(std::cos(w), std::cos(raw), 2e-4);

        const float p = wrapRadPositive(raw);
        CHECK(p >= 0.0f && p < TWO_PI);
        CHECK_NEAR(std::sin(p), std::sin(raw), 2e-4);
    }

    // A dead sensor reads PROS_ERR_F, which is INFINITY.
    CHECK_EQ(botHeadingToTrig(kInf), 0.0f);
    CHECK_EQ(botHeadingToTrig(-kInf), 0.0f);
    CHECK_EQ(botHeadingToTrig(kNan), 0.0f);
}

CHAD_TEST(util, wrapRadRejectsNonFinite) {
    CHECK_EQ(wrapRad(kInf), 0.0f);
    CHECK_EQ(wrapRad(-kInf), 0.0f);
    CHECK_EQ(wrapRad(kNan), 0.0f);
    CHECK_EQ(wrapRadPositive(kInf), 0.0f);
    CHECK_EQ(wrapDeg(kInf), 0.0f);
    CHECK_EQ(angleDiffRad(0.0f, kInf), 0.0f);
}

CHAD_TEST(util, angleDiffIsShortestSigned) {
    CHECK_NEAR(angleDiffRad(0.1f, -0.1f), -0.2f, 1e-6);
    CHECK_NEAR(angleDiffRad(-3.0f, 3.0f), -(TWO_PI - 6.0f), 1e-5);
    CHECK_NEAR(angleDistRad(-3.0f, 3.0f), TWO_PI - 6.0f, 1e-5);
    CHECK_NEAR(std::fabs(angleDiffRad(0.0f, PI)), PI, 1e-5);
}

CHAD_TEST(util, vexConversionRoundTrips) {
    for (int deg = 0; deg < 360; deg += 7) {
        const float d = static_cast<float>(deg);
        CHECK_NEAR(stdRadToVexDeg(vexToStdRad(d)), d, 2e-3);
    }
    CHECK_NEAR(vexToStdRad(0.0f), HALF_PI, 1e-6);
    CHECK_NEAR(vexToStdRad(90.0f), 0.0f, 1e-6);
    CHECK_NEAR(stdRadToVexDeg(0.0f), 90.0f, 1e-4);
}

CHAD_TEST(util, unitConversionsInvert) {
    CHECK_NEAR(meterToInch(inchToMeter(37.5f)), 37.5f, 1e-3);
    CHECK_NEAR(mmToInch(inchToMm(12.25f)), 12.25f, 1e-3);
    CHECK_NEAR(inchToMeter(1.0f), 0.0254f, 1e-9);
}

CHAD_TEST(util, geometryHelpers) {
    const Pose p{1.0f, 2.0f, 0.0f};
    CHECK_NEAR(distanceToPoint(p, 4.0f, 6.0f), 5.0f, 1e-6);
    CHECK_NEAR(bearingToPoint(p, 1.0f, 5.0f), HALF_PI, 1e-6);
    CHECK_NEAR(bearingToPoint(p, 1.0f, 2.0f), 0.0f, 1e-6);
}

CHAD_TEST(util, clampAndSign) {
    CHECK_EQ(clamp(5.0f, 0.0f, 1.0f), 1.0f);
    CHECK_EQ(clamp(-5.0f, 0.0f, 1.0f), 0.0f);
    CHECK_EQ(sign(0.0f), 0.0f);
    CHECK_EQ(sign(-3.0f), -1.0f);
}
