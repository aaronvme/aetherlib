#include "chad/util/kinematics.hpp"

#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {
constexpr float kInf = std::numeric_limits<float>::infinity();
const DifferentialKinematics kKin{12.0f};
}  // namespace

CHAD_TEST(kinematics, wheelSpeedsRoundTrip) {
    const ChassisSpeeds in{30.0f, 0.0f, 1.5f};
    const ChassisSpeeds back = kKin.toChassisSpeeds(kKin.toWheelSpeeds(in));
    CHECK_NEAR(back.vx, in.vx, 1e-4);
    CHECK_NEAR(back.omega, in.omega, 1e-5);
    CHECK_EQ(back.vy, 0.0f);
}

CHAD_TEST(kinematics, turnInPlaceIsAntisymmetric) {
    const WheelSpeeds w = kKin.toWheelSpeeds({0.0f, 0.0f, 1.0f});
    CHECK_NEAR(w.left, -6.0f, 1e-5);
    CHECK_NEAR(w.right, 6.0f, 1e-5);
}

CHAD_TEST(kinematics, twistFromWheelDeltas) {
    const Twist2d straight = kKin.toTwist(5.0f, 5.0f);
    CHECK_NEAR(straight.dx, 5.0f, 1e-6);
    CHECK_NEAR(straight.dtheta, 0.0f, 1e-9);

    const Twist2d spin = kKin.toTwist(-3.0f, 3.0f);
    CHECK_NEAR(spin.dx, 0.0f, 1e-6);
    CHECK_NEAR(spin.dtheta, 0.5f, 1e-6);
    CHECK_EQ(spin.dy, 0.0f);
}

CHAD_TEST(kinematics, zeroTrackWidthDegradesToStraightLine) {
    const DifferentialKinematics degenerate{0.0f};
    const Twist2d t = degenerate.toTwist(2.0f, 4.0f);
    CHECK_FINITE(t.dtheta);
    CHECK_NEAR(t.dx, 3.0f, 1e-6);
    CHECK_EQ(t.dtheta, 0.0f);
    CHECK_FINITE(degenerate.toChassisSpeeds({1.0f, 2.0f}).omega);
}

CHAD_TEST(kinematics, desaturatePreservesCurvature) {
    WheelSpeeds w{100.0f, 50.0f};
    const float ratio = w.left / w.right;
    const float scale = w.desaturate(60.0f);
    CHECK_NEAR(scale, 0.6f, 1e-6);
    CHECK_NEAR(w.left, 60.0f, 1e-4);
    CHECK_NEAR(w.left / w.right, ratio, 1e-5);
}

CHAD_TEST(kinematics, desaturateNoOpsAndDegenerates) {
    WheelSpeeds under{10.0f, -20.0f};
    CHECK_EQ(under.desaturate(60.0f), 1.0f);
    CHECK_EQ(under.left, 10.0f);

    WheelSpeeds any{10.0f, 10.0f};
    CHECK_EQ(any.desaturate(0.0f), 1.0f);
    CHECK_EQ(any.desaturate(-5.0f), 1.0f);

    WheelSpeeds bad{kInf, 1.0f};
    CHECK_EQ(bad.desaturate(60.0f), 1.0f);
}

CHAD_TEST(kinematics, desaturateHandlesNegativePeak) {
    WheelSpeeds w{-100.0f, -20.0f};
    CHECK_NEAR(w.desaturate(50.0f), 0.5f, 1e-6);
    CHECK_NEAR(w.left, -50.0f, 1e-4);
    CHECK_NEAR(w.right, -10.0f, 1e-4);
}

CHAD_TEST(kinematics, discretizeLandsOnTheArcEndpoint) {
    const float dt = 0.02f;
    const ChassisSpeeds want{40.0f, 0.0f, 2.5f};
    const ChassisSpeeds corrected = want.discretize(dt);
    const Pose landed = Pose{}.exp({corrected.vx * dt, corrected.vy * dt,
                                   corrected.omega * dt});
    CHECK_NEAR(landed.x, want.vx * dt, 1e-4);
    CHECK_NEAR(landed.y, want.vy * dt, 1e-4);
    CHECK_NEAR(landed.theta, want.omega * dt, 1e-6);
}

CHAD_TEST(kinematics, discretizeIsIdentityWithoutRotation) {
    const ChassisSpeeds straight{40.0f, 0.0f, 0.0f};
    const ChassisSpeeds out = straight.discretize(0.01f);
    CHECK_NEAR(out.vx, 40.0f, 1e-3);
    CHECK_NEAR(out.omega, 0.0f, 1e-9);
    CHECK_EQ(straight.discretize(0.0f).vx, 40.0f);
    CHECK_EQ(straight.discretize(-1.0f).vx, 40.0f);
}

CHAD_TEST(kinematics, discretizeIntroducesLateralTermWhenTurning) {
    const ChassisSpeeds out = ChassisSpeeds{40.0f, 0.0f, 3.0f}.discretize(0.05f);
    CHECK(std::fabs(out.vy) > 1e-3f);
    CHECK_NEAR(out.omega, 3.0f, 1e-6);
}

CHAD_TEST(kinematics, fieldRelativeRoundTrip) {
    const Rotation2d heading = Rotation2d::fromDegrees(37.0f);
    const ChassisSpeeds robot = ChassisSpeeds::fromFieldRelative(10.0f, -4.0f, 0.5f, heading);
    const Translation2d back = robot.toFieldRelative(heading);
    CHECK_NEAR(back.x, 10.0f, 1e-4);
    CHECK_NEAR(back.y, -4.0f, 1e-4);
    CHECK_NEAR(robot.norm(), std::sqrt(116.0f), 1e-4);
}

CHAD_TEST(driveModel, gripLimitOrdersByWheelType) {
    CHECK(defaultGripLimit(WheelType::Traction) > defaultGripLimit(WheelType::Mixed));
    CHECK(defaultGripLimit(WheelType::Mixed) > defaultGripLimit(WheelType::Omni));

    DriveModel m;
    m.wheels = WheelType::Omni;
    CHECK_EQ(m.gripLimit(), defaultGripLimit(WheelType::Omni));
    m.gripLimitOverride = 123.0f;
    CHECK_EQ(m.gripLimit(), 123.0f);
}

CHAD_TEST(driveModel, curvatureSpeedObeysGripLaw) {
    DriveModel m;
    m.wheels = WheelType::Omni;
    m.maxWheelSpeed = 1000.0f;
    m.trackWidth = 0.0f;

    const float k = 1.0f / 30.0f;
    const float v = m.maxSpeedForCurvature(k);
    CHECK_NEAR(v * v * k, m.gripLimit(), 1e-2);
    CHECK_NEAR(m.maxCurvatureAtSpeed(v), k, 1e-4);
}

CHAD_TEST(driveModel, straightPathIsWheelSpeedLimited) {
    DriveModel m;
    CHECK_EQ(m.maxSpeedForCurvature(0.0f), m.maxWheelSpeed);
    CHECK_EQ(m.maxSpeedForCurvature(kInf), m.maxWheelSpeed);
    CHECK(m.maxSpeedForCurvature(1.0f) < m.maxWheelSpeed);
}

CHAD_TEST(driveModel, tractionCornersFasterThanOmni) {
    DriveModel omni;
    omni.wheels = WheelType::Omni;
    DriveModel traction = omni;
    traction.wheels = WheelType::Traction;
    const float k = 1.0f / 24.0f;
    CHECK(traction.maxSpeedForCurvature(k) > omni.maxSpeedForCurvature(k));
}

CHAD_TEST(driveModel, tightCurvatureIsBudgetLimitedNotGripLimited) {
    DriveModel m;
    m.wheels = WheelType::Traction;
    m.trackWidth = 12.0f;
    m.maxWheelSpeed = 60.0f;
    const float k = 1.0f;
    const float budget = m.maxWheelSpeed / (1.0f + 0.5f * k * m.trackWidth);
    CHECK_NEAR(m.maxSpeedForCurvature(k), budget, 1e-4);
    const ChassisSpeeds atLimit{m.maxSpeedForCurvature(k), 0.0f,
                                m.maxSpeedForCurvature(k) * k};
    CHECK(m.isFeasible(atLimit));
}

CHAD_TEST(driveModel, feasibilityAndOmegaBudget) {
    DriveModel m;
    m.trackWidth = 12.0f;
    m.maxWheelSpeed = 60.0f;
    CHECK(m.isFeasible({60.0f, 0.0f, 0.0f}));
    CHECK(!m.isFeasible({60.0f, 0.0f, 0.1f}));
    CHECK(m.isFeasible({0.0f, 0.0f, 10.0f}));
    CHECK_NEAR(m.omegaBudgetAt(0.0f), 10.0f, 1e-5);
    CHECK_NEAR(m.omegaBudgetAt(60.0f), 0.0f, 1e-9);
    CHECK_EQ(m.omegaBudgetAt(100.0f), 0.0f);
    const ChassisSpeeds edge{30.0f, 0.0f, m.omegaBudgetAt(30.0f)};
    CHECK(m.isFeasible(edge));
}
