#include "chad/tracking/odom_inputs.hpp"

#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {
constexpr double kErrF = std::numeric_limits<double>::infinity();   // PROS_ERR_F
constexpr double kErrI = 2147483647.0;                               // PROS_ERR via int

// 450 rpm wheels on a 600 rpm cartridge, 2.75 in wheels.
constexpr double kInPerDeg = 0.75 / 360.0 * 3.14159265358979 * 2.75;
constexpr float kDt = 0.01f;
const double kMaxStep = EncoderDeltaMean::maxStepDeg(kDt);

struct Side {
    double pos[3];
};
}  // namespace

CHAD_TEST(encoderDelta, meanOfDeltasWhenAllReadable) {
    EncoderDeltaMean m;
    double a[3] = {100.0, 200.0, 300.0};
    m.rebase(a, 3);
    double b[3] = {110.0, 212.0, 308.0};
    CHECK_NEAR(m.update(b, 3, kMaxStep), 10.0, 1e-9);
    CHECK_EQ(m.contributing(), 3);
}

CHAD_TEST(encoderDelta, droppedEncoderDoesNotStepTheMean) {
    // Positions far apart, as they are after turning. Differencing the mean
    // of absolute positions would step by (5000 - -3000) / 3 here.
    EncoderDeltaMean m;
    double a[3] = {5000.0, -3000.0, 1000.0};
    m.rebase(a, 3);
    double b[3] = {kErrF, -2990.0, 1010.0};
    CHECK_NEAR(m.update(b, 3, kMaxStep), 10.0, 1e-9);
    CHECK_EQ(m.contributing(), 2);
    double c[3] = {kErrI, -2980.0, 1020.0};
    CHECK_NEAR(m.update(c, 3, kMaxStep), 10.0, 1e-9);
}

CHAD_TEST(encoderDelta, returningEncoderRebasesInsteadOfContributing) {
    EncoderDeltaMean m;
    double a[3] = {5000.0, 5000.0, 5000.0};
    m.rebase(a, 3);
    double b[3] = {kErrF, 5010.0, 5010.0};
    m.update(b, 3, kMaxStep);
    // Back, re-zeroed by a brown-out.
    double c[3] = {3.0, 5020.0, 5020.0};
    CHECK_NEAR(m.update(c, 3, kMaxStep), 10.0, 1e-9);
    CHECK_EQ(m.contributing(), 2);
    double d[3] = {13.0, 5030.0, 5030.0};
    CHECK_NEAR(m.update(d, 3, kMaxStep), 10.0, 1e-9);
    CHECK_EQ(m.contributing(), 3);
}

CHAD_TEST(encoderDelta, implausibleStepIsDroppedForThatTick) {
    // A re-zero between two samples, with no unreadable read in between.
    EncoderDeltaMean m;
    double a[3] = {5000.0, 5000.0, 5000.0};
    m.rebase(a, 3);
    double b[3] = {0.0, 5010.0, 5010.0};
    CHECK_NEAR(m.update(b, 3, kMaxStep), 10.0, 1e-9);
    CHECK_EQ(m.rejectedSteps(), 1u);
    double c[3] = {10.0, 5020.0, 5020.0};
    CHECK_NEAR(m.update(c, 3, kMaxStep), 10.0, 1e-9);
    CHECK_EQ(m.contributing(), 3);
}

CHAD_TEST(encoderDelta, fullSpeedIsNeverRejected) {
    // 600 rpm output shaft = 3600 deg/s.
    EncoderDeltaMean m;
    double p[1] = {0.0};
    m.rebase(p, 1);
    for (int i = 0; i < 100; i++) {
        p[0] += 36.0;
        CHECK_NEAR(m.update(p, 1, kMaxStep), 36.0, 1e-9);
    }
    CHECK_EQ(m.rejectedSteps(), 0u);
}

CHAD_TEST(encoderDelta, nothingReadableReportsNoTravel) {
    EncoderDeltaMean m;
    double a[2] = {1.0, 2.0};
    m.rebase(a, 2);
    double b[2] = {kErrF, kErrF};
    CHECK_EQ(m.update(b, 2, kMaxStep), 0.0);
    CHECK_EQ(m.contributing(), 0);
}

CHAD_TEST(encoderDelta, moreEncodersThanCapacityAreIgnoredNotOverrun) {
    EncoderDeltaMean m;
    double a[EncoderDeltaMean::kMaxEncoders + 4] = {};
    m.rebase(a, EncoderDeltaMean::kMaxEncoders + 4);
    for (double& v : a) v = 5.0;
    CHECK_NEAR(m.update(a, EncoderDeltaMean::kMaxEncoders + 4, kMaxStep), 5.0, 1e-9);
    CHECK_EQ(m.contributing(), EncoderDeltaMean::kMaxEncoders);
}

CHAD_TEST(centreTravel, bothSidesAverage) {
    CHECK_NEAR(differentialCentreTravel(2.0, true, 4.0, true, 0.5), 3.0, 1e-9);
}

CHAD_TEST(centreTravel, missingSideIsRebuiltFromHeadingChange) {
    // CCW turn: right wheel covers centre + arc, left centre - arc.
    const double centre = 1.0, arc = 0.25;
    CHECK_NEAR(differentialCentreTravel(0.0, false, centre + arc, true, arc), centre, 1e-9);
    CHECK_NEAR(differentialCentreTravel(centre - arc, true, 0.0, false, arc), centre, 1e-9);
    CHECK_EQ(differentialCentreTravel(5.0, false, 5.0, false, arc), 0.0);
}

// A 3+3 drive turns first so its sides diverge by thousands of degrees, then
// drives straight while one left motor drops out and comes back re-zeroed.
CHAD_TEST(encoderDelta, motorDropoutMidDriveKeepsTravelWithinHalfAnInch) {
    Side left{{0.0, 0.0, 0.0}}, right{{0.0, 0.0, 0.0}};
    for (int i = 0; i < 3; i++) {
        left.pos[i] = -4000.0;
        right.pos[i] = 4000.0;
    }
    EncoderDeltaMean l, r;
    l.rebase(left.pos, 3);
    r.rebase(right.pos, 3);

    const double stepDeg = 30.0;   // 3000 deg/s
    double travelled = 0.0;
    double truth = 0.0;
    // What differencing the mean of absolute positions made of the same reads.
    double absMeanTravelled = 0.0;
    double prevAbsMean = 0.0;
    double worstAbsMeanErr = 0.0;
    double worstErr = 0.0;
    for (int tick = 0; tick < 200; tick++) {
        for (int i = 0; i < 3; i++) {
            left.pos[i] += stepDeg;
            right.pos[i] += stepDeg;
        }
        truth += stepDeg * kInPerDeg;

        double readL[3] = {left.pos[0], left.pos[1], left.pos[2]};
        if (tick >= 50 && tick < 80) readL[1] = kErrF;
        if (tick == 80) left.pos[1] = 7.0;   // power returned at zero
        if (tick >= 80) readL[1] = left.pos[1];

        double absSum = 0.0;
        int absCount = 0;
        for (int i = 0; i < 3; i++) {
            if (EncoderDeltaMean::readable(readL[i])) { absSum += readL[i]; absCount++; }
            absSum += right.pos[i];
            absCount++;
        }
        const double absMean = absSum / absCount;
        if (tick > 0) absMeanTravelled += (absMean - prevAbsMean) * kInPerDeg;
        prevAbsMean = absMean;

        const double dl = l.update(readL, 3, kMaxStep);
        const double dr = r.update(right.pos, 3, kMaxStep);
        travelled += differentialCentreTravel(dl * kInPerDeg, l.contributing() > 0,
                                              dr * kInPerDeg, r.contributing() > 0, 0.0);
        worstErr = std::fmax(worstErr, std::fabs(travelled - truth));
        worstAbsMeanErr = std::fmax(worstAbsMeanErr, std::fabs(absMeanTravelled - truth));
    }
    CHECK(worstErr < 0.5);
    CHECK(worstAbsMeanErr > 10.0);
}

CHAD_TEST(imuGuard, followsNormalTurning) {
    ImuHeadingGuard g;
    g.seed(0.0f);
    float raw = 0.0f;
    // 900 deg/s peak, well past what an 11.5 in / 450 rpm drive can spin.
    const float step = degToRad(900.0f) * kDt;
    for (int i = 0; i < 100; i++) {
        raw = wrapRad(raw + step);
        CHECK(g.update(raw, kDt));
    }
    CHECK_NEAR(g.headingRad, wrapRad(step * 100.0f), 1e-4);
}

CHAD_TEST(imuGuard, jumpIsReanchoredNotTakenAsRotation) {
    ImuHeadingGuard g;
    g.seed(0.0f);
    g.update(degToRad(1.0f), kDt);
    g.update(degToRad(2.0f), kDt);
    // A restart reads near zero from wherever it was.
    CHECK(!g.update(degToRad(-90.0f), kDt));
    CHECK_NEAR(g.headingRad, degToRad(2.0f), 1e-6);
    CHECK(g.update(degToRad(-89.0f), kDt));
    CHECK_NEAR(g.headingRad, degToRad(3.0f), 1e-5);
}

CHAD_TEST(imuGuard, readingAfterAGapReanchors) {
    ImuHeadingGuard g;
    g.seed(degToRad(45.0f));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    CHECK(!g.update(nan, kDt));
    CHECK_NEAR(g.headingRad, degToRad(45.0f), 1e-6);
    // Back re-zeroed: small raw value, but it is not a 45 deg turn.
    CHECK(!g.update(degToRad(0.5f), kDt));
    CHECK_NEAR(g.headingRad, degToRad(45.0f), 1e-6);
    CHECK(g.update(degToRad(1.5f), kDt));
    CHECK_NEAR(g.headingRad, degToRad(46.0f), 1e-5);
}

CHAD_TEST(imuGuard, anchorSetsFieldHeading) {
    ImuHeadingGuard g;
    g.seed(degToRad(10.0f));
    g.anchor(degToRad(10.0f), degToRad(90.0f));
    CHECK_NEAR(g.headingRad, degToRad(90.0f), 1e-6);
    CHECK(g.update(degToRad(11.0f), kDt));
    CHECK_NEAR(g.headingRad, degToRad(91.0f), 1e-5);
}

CHAD_TEST(imuGuard, anchorWhileUnreadableTakesEffectOnReturn) {
    ImuHeadingGuard g;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    g.seed(nan);
    g.anchor(nan, degToRad(-30.0f));
    CHECK_NEAR(g.headingRad, degToRad(-30.0f), 1e-6);
    CHECK(!g.update(degToRad(170.0f), kDt));
    CHECK_NEAR(g.headingRad, degToRad(-30.0f), 1e-6);
}

CHAD_TEST(imuGuard, oneMissedReadMidTurnKeepsTheTurn) {
    ImuHeadingGuard g;
    g.seed(0.0f);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float step = degToRad(5.0f);   // 500 deg/s
    float raw = 0.0f;
    for (int i = 0; i < 40; i++) {
        raw = wrapRad(raw + step);
        g.update(i == 20 ? nan : raw, kDt);
    }
    CHECK_NEAR(g.headingRad, wrapRad(40.0f * step), 1e-4);
}

CHAD_TEST(imuGuard, longGapReanchorsEvenOnASmallStep) {
    // Unplugged long enough to have restarted and recalibrated: whatever it
    // reads now is in a new frame, however close it lands.
    ImuHeadingGuard g;
    g.seed(degToRad(45.0f));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (int i = 0; i < 150; i++) g.update(nan, kDt);
    CHECK(!g.update(degToRad(45.5f), kDt));
    CHECK_NEAR(g.headingRad, degToRad(45.0f), 1e-6);
    CHECK(g.update(degToRad(46.5f), kDt));
    CHECK_NEAR(g.headingRad, degToRad(46.0f), 1e-5);
}

CHAD_TEST(imuGuard, nonFiniteAnchorIsIgnored) {
    ImuHeadingGuard g;
    g.seed(degToRad(10.0f));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    g.anchor(degToRad(10.0f), nan);
    CHECK_NEAR(g.headingRad, degToRad(10.0f), 1e-6);
    CHECK(std::isfinite(g.offsetRad));
    CHECK(g.update(degToRad(11.0f), kDt));
    CHECK_NEAR(g.headingRad, degToRad(11.0f), 1e-5);
}
