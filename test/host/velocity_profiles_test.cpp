#include "chad/movement_control/velocity_profiles.hpp"

#include <cmath>
#include <vector>

#include "chad_test.hpp"

using namespace chad;

namespace {

float peakVelocity(const std::vector<ProfilePoint>& p) {
    float peak = 0.0f;
    for (const ProfilePoint& pt : p) peak = std::fmax(peak, std::fabs(pt.velocity));
    return peak;
}

float peakAccel(const std::vector<ProfilePoint>& p) {
    float peak = 0.0f;
    for (size_t i = 1; i < p.size(); i++) {
        const float dt = p[i].time - p[i - 1].time;
        if (dt <= 1e-9f) continue;
        peak = std::fmax(peak, std::fabs((p[i].velocity - p[i - 1].velocity) / dt));
    }
    return peak;
}

float peakJerk(const std::vector<ProfilePoint>& p) {
    float peak = 0.0f;
    float prevAccel = 0.0f;
    bool primed = false;
    for (size_t i = 1; i < p.size(); i++) {
        const float dt = p[i].time - p[i - 1].time;
        if (dt <= 1e-9f) continue;
        const float accel = (p[i].velocity - p[i - 1].velocity) / dt;
        if (primed) peak = std::fmax(peak, std::fabs((accel - prevAccel) / dt));
        prevAccel = accel;
        primed = true;
    }
    return peak;
}

void checkMonotonic(const std::vector<ProfilePoint>& p) {
    for (size_t i = 1; i < p.size(); i++) {
        CHECK(p[i].time > p[i - 1].time - 1e-9f);
        CHECK(p[i].position >= p[i - 1].position - 1e-3f);
        CHECK_FINITE(p[i].position);
        CHECK_FINITE(p[i].velocity);
    }
}

}  // namespace

CHAD_TEST(trapezoid, reachesTheDistanceAtRest) {
    const std::vector<ProfilePoint> p = generateTrapezoidal(60.0f, 40.0f, 80.0f);
    CHECK(p.size() > 10);
    checkMonotonic(p);
    CHECK_NEAR(p.back().position, 60.0f, 0.5);
    CHECK_NEAR(p.back().velocity, 0.0f, 1e-4);
    CHECK_NEAR(p.front().velocity, 0.0f, 1e-6);
}

CHAD_TEST(trapezoid, respectsVelocityAndAccelerationLimits) {
    // Swept over distances, because a short move takes the triangular branch and
    // a long one does not, and the two write their position axis differently.
    for (float d : {2.0f, 18.0f, 100.0f, 400.0f}) {
        const std::vector<ProfilePoint> p = generateTrapezoidal(d, 30.0f, 50.0f);
        checkMonotonic(p);
        // 1.0000 and 1.0000 of the limits measured. The old slack was 5% on the
        // acceleration, which is 2.5 in/s^2 of room the profile never uses.
        CHECK(peakVelocity(p) <= 30.0f * 1.002f);
        CHECK(peakAccel(p) <= 50.0f * 1.002f);
        CHECK_NEAR(p.back().position, d, 1e-3);
    }
}

CHAD_TEST(trapezoid, analyticTotalTimeForACruisingMove) {
    const float d = 100.0f, vMax = 25.0f, a = 50.0f;
    const std::vector<ProfilePoint> p = generateTrapezoidal(d, vMax, a);
    // Two ramps plus a cruise: t = d/vMax + vMax/a.
    const float expected = d / vMax + vMax / a;
    CHECK_NEAR(p.back().time, expected, 0.05);
}

CHAD_TEST(trapezoid, triangularMoveNeverReachesMaxVelocity) {
    // 2 in at 50 in/s^2 peaks at sqrt(a*d) = 10 in/s, far below the 40 asked for.
    const std::vector<ProfilePoint> p = generateTrapezoidal(2.0f, 40.0f, 50.0f);
    checkMonotonic(p);
    CHECK_NEAR(peakVelocity(p), 10.0f, 0.6);
    CHECK_NEAR(p.back().position, 2.0f, 0.05);
}

CHAD_TEST(trapezoid, positionIsContinuousThroughTheTriangularPeak) {
    const std::vector<ProfilePoint> p = generateTrapezoidal(3.0f, 40.0f, 50.0f);
    for (size_t i = 1; i < p.size(); i++) {
        const float dt = p[i].time - p[i - 1].time;
        const float mean = 0.5f * (p[i].velocity + p[i - 1].velocity);
        const float step = p[i].position - p[i - 1].position;
        // Trapezoidal integration of the velocity must agree with the position
        // axis; a mismatch means the two are describing different profiles.
        CHECK_NEAR(step, mean * dt, 0.02);
    }
}

CHAD_TEST(trapezoid, degenerateInputsGiveOneRestingPoint) {
    for (const std::vector<ProfilePoint>& p :
         {generateTrapezoidal(0.0f, 40.0f, 50.0f),
          generateTrapezoidal(-5.0f, 40.0f, 50.0f),
          generateTrapezoidal(10.0f, 0.0f, 50.0f),
          generateTrapezoidal(10.0f, 40.0f, 0.0f),
          generateTrapezoidal(10.0f, 40.0f, 50.0f, 0.0f)}) {
        CHECK(!p.empty());
        CHECK_EQ(p.front().velocity, 0.0f);
        CHECK_FINITE(p.back().position);
    }
}

CHAD_TEST(scurve, reachesTheDistanceAtRest) {
    const std::vector<ProfilePoint> p = generateSCurve(60.0f, 40.0f, 80.0f, 400.0f);
    CHECK(p.size() > 10);
    checkMonotonic(p);
    CHECK_NEAR(p.back().position, 60.0f, 1e-3);
    CHECK_NEAR(p.back().velocity, 0.0f, 1e-3);
}

CHAD_TEST(scurve, respectsEveryLimit) {
    const float vMax = 30.0f, aMax = 60.0f, jMax = 300.0f;
    // Measured 1.0000 / 1.0000 / 1.0004 of the three limits over these
    // distances. The old slacks were 2% / 10% / 60%, and 60% of a jerk limit is
    // not a jerk limit -- the trapezoid this is meant to be smoother than
    // clears it by a factor of 26.
    for (float d : {10.0f, 40.0f, 120.0f, 240.0f}) {
        const std::vector<ProfilePoint> p = generateSCurve(d, vMax, aMax, jMax);
        checkMonotonic(p);
        CHECK(peakVelocity(p) <= vMax * 1.002f);
        CHECK(peakAccel(p) <= aMax * 1.002f);
        CHECK(peakJerk(p) <= jMax * 1.005f);
        CHECK_NEAR(p.back().position, d, 1e-3);
    }
}

CHAD_TEST(scurve, isSmootherThanTheTrapezoid) {
    const std::vector<ProfilePoint> smooth = generateSCurve(60.0f, 40.0f, 80.0f, 300.0f);
    const std::vector<ProfilePoint> sharp = generateTrapezoidal(60.0f, 40.0f, 80.0f);
    CHECK(peakJerk(smooth) < peakJerk(sharp));
}

CHAD_TEST(scurve, shortMovesShrinkThePeakSpeed) {
    const std::vector<ProfilePoint> p = generateSCurve(1.0f, 40.0f, 80.0f, 300.0f);
    checkMonotonic(p);
    CHECK(peakVelocity(p) < 40.0f);
    CHECK_NEAR(p.back().position, 1.0f, 1e-3);
    CHECK_NEAR(p.back().velocity, 0.0f, 1e-3);
}

CHAD_TEST(scurve, zeroJerkFallsBackToTheTrapezoid) {
    const std::vector<ProfilePoint> fallback = generateSCurve(50.0f, 30.0f, 60.0f, 0.0f);
    const std::vector<ProfilePoint> trapezoid = generateTrapezoidal(50.0f, 30.0f, 60.0f);
    CHECK_EQ(fallback.size(), trapezoid.size());
    CHECK_NEAR(fallback.back().time, trapezoid.back().time, 1e-4);
}

CHAD_TEST(scurve, degenerateInputsGiveOneRestingPoint) {
    for (const std::vector<ProfilePoint>& p :
         {generateSCurve(0.0f, 40.0f, 80.0f, 300.0f),
          generateSCurve(-5.0f, 40.0f, 80.0f, 300.0f),
          generateSCurve(10.0f, 0.0f, 80.0f, 300.0f),
          generateSCurve(10.0f, 40.0f, 0.0f, 300.0f)}) {
        CHECK(!p.empty());
        CHECK_FINITE(p.back().position);
        CHECK_FINITE(p.back().velocity);
    }
}

CHAD_TEST(scurve, positionAgreesWithIntegratedVelocity) {
    const std::vector<ProfilePoint> p = generateSCurve(40.0f, 25.0f, 50.0f, 250.0f);
    float integrated = 0.0f;
    for (size_t i = 1; i < p.size(); i++) {
        const float dt = p[i].time - p[i - 1].time;
        integrated += 0.5f * (p[i].velocity + p[i - 1].velocity) * dt;
    }
    CHECK_NEAR(integrated, p.back().position, 0.05);
}
