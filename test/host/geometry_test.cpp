#include "chad/util/geometry.hpp"

#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();

Pose integrateUnicycle(const Pose& start, float v, float omega, float seconds,
                       int steps) {
    double x = start.x, y = start.y, th = start.theta;
    const double h = static_cast<double>(seconds) / steps;
    for (int i = 0; i < steps; i++) {
        const double tm = th + 0.5 * h * omega;
        x += h * v * std::cos(tm);
        y += h * v * std::sin(tm);
        th += h * omega;
    }
    return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(th)};
}

}  // namespace

CHAD_TEST(rotation, composesAsAngleAddition) {
    const Rotation2d a = Rotation2d::fromDegrees(30.0f);
    const Rotation2d b = Rotation2d::fromDegrees(45.0f);
    CHECK_NEAR((a + b).degrees(), 75.0f, 1e-3);
    CHECK_NEAR((a - b).degrees(), -15.0f, 1e-3);
    CHECK_NEAR((-a).degrees(), -30.0f, 1e-3);
    CHECK_NEAR((a + (-a)).degrees(), 0.0f, 1e-4);
}

CHAD_TEST(rotation, compositionNeverNeedsWrapping) {
    Rotation2d r;
    for (int i = 0; i < 1000; i++) r += Rotation2d::fromDegrees(179.0f);
    CHECK_NEAR(r.cos * r.cos + r.sin * r.sin, 1.0f, 1e-3);
    CHECK(r.radians() >= -PI && r.radians() <= PI);
}

CHAD_TEST(rotation, fromVectorHandlesDegenerateInput) {
    CHECK_NEAR(Rotation2d::fromVector(0.0f, 0.0f).radians(), 0.0f, 1e-9);
    CHECK_NEAR(Rotation2d::fromVector(0.0f, 3.0f).radians(), HALF_PI, 1e-6);
    CHECK_NEAR(Rotation2d::fromVector(-2.0f, 0.0f).radians(), PI, 1e-6);
    const Rotation2d unit = Rotation2d::fromVector(3.0f, 4.0f);
    CHECK_NEAR(unit.cos, 0.6f, 1e-6);
    CHECK_NEAR(unit.sin, 0.8f, 1e-6);
}

CHAD_TEST(rotation, rejectsNonFinite) {
    CHECK_EQ(Rotation2d::fromRadians(kInf).cos, 1.0f);
    CHECK_EQ(Rotation2d::fromRadians(kInf).sin, 0.0f);
    CHECK_FINITE(Rotation2d::fromVector(kInf, 1.0f).radians());
}

CHAD_TEST(rotation, distanceIsShortestSigned) {
    const Rotation2d a = Rotation2d::fromDegrees(170.0f);
    const Rotation2d b = Rotation2d::fromDegrees(-170.0f);
    CHECK_NEAR(radToDeg(a.distanceTo(b)), 20.0f, 1e-3);
    CHECK_NEAR(radToDeg(b.distanceTo(a)), -20.0f, 1e-3);
    const Rotation2d mid = a.interpolate(b, 0.5f);
    CHECK_NEAR(std::fabs(radToDeg(mid.radians())), 180.0f, 1e-3);
}

CHAD_TEST(translation, rotationPreservesNorm) {
    const Translation2d t{3.0f, -4.0f};
    for (int deg = 0; deg < 360; deg += 13) {
        const Translation2d r = t.rotateBy(Rotation2d::fromDegrees(static_cast<float>(deg)));
        CHECK_NEAR(r.norm(), 5.0f, 1e-4);
    }
}

CHAD_TEST(translation, dotCrossAndPolar) {
    const Translation2d a{1.0f, 0.0f};
    const Translation2d b{0.0f, 2.0f};
    CHECK_NEAR(a.dot(b), 0.0f, 1e-9);
    CHECK_NEAR(a.cross(b), 2.0f, 1e-9);
    CHECK_NEAR(b.cross(a), -2.0f, 1e-9);
    const Translation2d p = Translation2d::fromPolar(5.0f, Rotation2d::fromDegrees(90.0f));
    CHECK_NEAR(p.x, 0.0f, 1e-5);
    CHECK_NEAR(p.y, 5.0f, 1e-5);
}

CHAD_TEST(pose, expStraightLineIsTranslation) {
    const Pose start{1.0f, 2.0f, HALF_PI};
    const Pose end = start.exp({10.0f, 0.0f, 0.0f});
    CHECK_NEAR(end.x, 1.0f, 1e-4);
    CHECK_NEAR(end.y, 12.0f, 1e-4);
    CHECK_NEAR(end.theta, HALF_PI, 1e-6);
}

CHAD_TEST(pose, expQuarterCircleMatchesGeometry) {
    const Pose start{};
    const float r = 10.0f;
    const Pose end = start.exp({r * HALF_PI, 0.0f, HALF_PI});
    CHECK_NEAR(end.x, r, 2e-3);
    CHECK_NEAR(end.y, r, 2e-3);
    CHECK_NEAR(end.theta, HALF_PI, 1e-5);
}

CHAD_TEST(pose, expMatchesNumericalIntegration) {
    const Pose start{-4.0f, 7.0f, 0.9f};
    const float v = 24.0f, omega = 1.7f, dt = 0.85f;
    const Pose ref = integrateUnicycle(start, v, omega, dt, 400000);
    const Pose got = start.exp({v * dt, 0.0f, omega * dt});
    CHECK_NEAR(got.x, ref.x, 2e-3);
    CHECK_NEAR(got.y, ref.y, 2e-3);
    CHECK_NEAR(angleDiffRad(got.theta, ref.theta), 0.0f, 1e-5);
}

CHAD_TEST(pose, expIsClosedUnderSubdivision) {
    const Pose start{2.0f, -3.0f, 0.4f};
    for (float dtheta : {2.5f, 0.3f, 1e-3f, 1e-6f, -1.9f}) {
        const Twist2d full{12.0f, 1.5f, dtheta};
        const Pose once = start.exp(full);
        Pose stepped = start;
        const int n = 16;
        for (int i = 0; i < n; i++) stepped = stepped.exp(full * (1.0f / n));
        CHECK_NEAR(stepped.x, once.x, 2e-3);
        CHECK_NEAR(stepped.y, once.y, 2e-3);
        CHECK_NEAR(angleDiffRad(stepped.theta, once.theta), 0.0f, 1e-5);
    }
}

CHAD_TEST(pose, logInvertsExp) {
    const Pose start{5.0f, -2.0f, -1.2f};
    for (float dtheta : {0.0f, 1e-7f, 1e-3f, 0.7f, 3.0f, -2.2f}) {
        const Twist2d t{7.0f, -1.0f, dtheta};
        const Twist2d back = start.log(start.exp(t));
        CHECK_NEAR(back.dx, t.dx, 2e-3);
        CHECK_NEAR(back.dy, t.dy, 2e-3);
        CHECK_NEAR(back.dtheta, t.dtheta, 1e-5);
    }
}

CHAD_TEST(pose, logIsStableAcrossTheSeriesThreshold) {
    const Pose start{};
    float prevDx = 0.0f;
    for (float dtheta : {2e-3f, 1.5e-3f, 1.1e-3f, 9e-4f, 5e-4f, 1e-4f}) {
        const Twist2d back = start.log(start.exp({10.0f, 0.0f, dtheta}));
        CHECK_NEAR(back.dx, 10.0f, 5e-3);
        if (prevDx != 0.0f) CHECK_NEAR(back.dx, prevDx, 5e-3);
        prevDx = back.dx;
    }
}

CHAD_TEST(pose, relativeToAndTransformByRoundTrip) {
    const Pose origin{10.0f, -5.0f, 1.1f};
    const Pose target{3.0f, 8.0f, -0.4f};
    const Pose rel = target.relativeTo(origin);
    const Pose back = origin.transformBy({{rel.x, rel.y}, Rotation2d::fromRadians(rel.theta)});
    CHECK_NEAR(back.x, target.x, 1e-3);
    CHECK_NEAR(back.y, target.y, 1e-3);
    CHECK_NEAR(angleDiffRad(back.theta, target.theta), 0.0f, 1e-5);
}

CHAD_TEST(pose, relativeToSelfIsIdentity) {
    const Pose p{7.0f, 7.0f, 2.0f};
    const Pose rel = p.relativeTo(p);
    CHECK_NEAR(rel.x, 0.0f, 1e-5);
    CHECK_NEAR(rel.y, 0.0f, 1e-5);
    CHECK_NEAR(rel.theta, 0.0f, 1e-6);
}

CHAD_TEST(pose, relativeToPutsTargetAheadInRobotFrame) {
    const Pose robot{0.0f, 0.0f, HALF_PI};
    const Pose ahead{0.0f, 10.0f, HALF_PI};
    const Pose rel = ahead.relativeTo(robot);
    CHECK_NEAR(rel.x, 10.0f, 1e-4);
    CHECK_NEAR(rel.y, 0.0f, 1e-4);
}

CHAD_TEST(pose, interpolateStaysOnTheArc) {
    const Pose a{};
    const Pose b = a.exp({20.0f, 0.0f, 1.0f});
    const Pose mid = a.interpolate(b, 0.5f);
    const Pose expected = a.exp({10.0f, 0.0f, 0.5f});
    CHECK_NEAR(mid.x, expected.x, 2e-3);
    CHECK_NEAR(mid.y, expected.y, 2e-3);
    CHECK_NEAR(mid.theta, expected.theta, 1e-5);
    CHECK_NEAR(a.interpolate(b, 0.0f).x, a.x, 1e-6);
    CHECK_NEAR(a.interpolate(b, 1.0f).x, b.x, 1e-6);
    CHECK_NEAR(a.interpolate(b, 2.0f).y, b.y, 1e-6);
}

CHAD_TEST(pose, equalsWrapsHeading) {
    const Pose a{1.0f, 1.0f, PI - 1e-5f};
    const Pose b{1.0f, 1.0f, -PI + 1e-5f};
    CHECK(a.equals(b, 1e-3f, 1e-3f));
    CHECK(!a.equals({1.0f, 1.0f, 0.0f}));
}

CHAD_TEST(pose, vexConventionBridge) {
    const Pose p = Pose::fromVex(0.0f, 0.0f, 0.0f);
    CHECK_NEAR(p.theta, HALF_PI, 1e-6);
    CHECK_NEAR(p.thetaVexDeg(), 0.0f, 1e-3);
    CHECK_NEAR(Pose::fromVex(0.0f, 0.0f, 90.0f).theta, 0.0f, 1e-6);
    const Pose facingPlusX{};
    CHECK_NEAR(facingPlusX.rotation().vexDegrees(), 90.0f, 1e-3);
}

CHAD_TEST(transform, inverseUndoesComposition) {
    const Transform2d t{{3.0f, 4.0f}, Rotation2d::fromDegrees(37.0f)};
    const Transform2d identity = t + t.inverse();
    CHECK_NEAR(identity.translation.x, 0.0f, 1e-4);
    CHECK_NEAR(identity.translation.y, 0.0f, 1e-4);
    CHECK_NEAR(identity.rotation.radians(), 0.0f, 1e-5);
}

CHAD_TEST(transform, poseTransformIsAssociative) {
    const Pose base{1.0f, 2.0f, 0.6f};
    const Transform2d a{{5.0f, 0.0f}, Rotation2d::fromDegrees(20.0f)};
    const Transform2d b{{0.0f, 3.0f}, Rotation2d::fromDegrees(-50.0f)};
    const Pose viaSteps = base.transformBy(a).transformBy(b);
    const Pose viaCombined = base.transformBy(a + b);
    CHECK_NEAR(viaSteps.x, viaCombined.x, 1e-3);
    CHECK_NEAR(viaSteps.y, viaCombined.y, 1e-3);
    CHECK_NEAR(angleDiffRad(viaSteps.theta, viaCombined.theta), 0.0f, 1e-5);
}

CHAD_TEST(series, sincAndCotAreContinuousAtThreshold) {
    const float eps = 1e-3f;
    CHECK_NEAR(sinc(eps * 0.999f), sinc(eps * 1.001f), 1e-6);
    CHECK_NEAR(hCotH(eps * 0.999f), hCotH(eps * 1.001f), 1e-6);
    CHECK_NEAR(sinc(0.0f), 1.0f, 1e-9);
    CHECK_NEAR(hCotH(0.0f), 1.0f, 1e-9);
    CHECK_NEAR(sinc(1.0f), std::sin(1.0f), 1e-6);
    CHECK_NEAR(hCotH(1.0f), 1.0f / std::tan(1.0f), 1e-6);
}
