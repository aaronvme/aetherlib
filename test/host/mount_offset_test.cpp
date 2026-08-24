#include "chad/util/mount_offset.hpp"

#include <cmath>

#include "chad_test.hpp"

using chad::tuning::acrossBeamOffset;
using chad::tuning::applyMountOffset;

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Mount {
    const char* name;
    float x, y, thetaDeg;
};

// The four mounts config.cpp ships.
constexpr Mount kShipped[] = {
    {"front", 1.00f, -3.75f, 0.0f},
    {"left", 0.50f, 6.25f, 90.0f},
    {"back", -0.50f, 5.00f, 180.0f},
    {"right", -0.75f, -6.75f, 270.0f},
};

float rad(float deg) {
    return static_cast<float>(deg * kPi / 180.0);
}

// Second, independent derivation: rotate the mount into beam frame with an
// explicit rotation matrix and read the perpendicular component off it. Shares
// no code with acrossBeamOffset.
float acrossByRotation(float x, float y, float thetaRad) {
    const float c = std::cos(thetaRad);
    const float s = std::sin(thetaRad);
    const float beamFrameY = -s * x + c * y;   // rotate by -theta, take y
    return beamFrameY;
}

}  // namespace

CHAD_TEST(mountOffset, twoDerivationsOfTheAcrossComponentAgree) {
    for (const Mount& m : kShipped) {
        const float t = rad(m.thetaDeg);
        CHECK_NEAR(acrossBeamOffset(m.x, m.y, t), acrossByRotation(m.x, m.y, t),
                   1e-5);
    }
}

// The bug this exists to prevent: the sweep measures the across-beam component,
// which is NOT mountOffset.y unless the sensor points forward.
CHAD_TEST(mountOffset, theSweepDoesNotMeasureMountY) {
    // Forward-facing is the coincidence that hid this.
    CHECK_NEAR(acrossBeamOffset(1.00f, -3.75f, rad(0.0f)), -3.75, 1e-5);
    // Every other orientation disagrees, and by a lot.
    CHECK_NEAR(acrossBeamOffset(0.50f, 6.25f, rad(90.0f)), -0.50, 1e-5);
    CHECK_NEAR(acrossBeamOffset(-0.50f, 5.00f, rad(180.0f)), -5.00, 1e-5);
    CHECK_NEAR(acrossBeamOffset(-0.75f, -6.75f, rad(270.0f)), -0.75, 1e-5);

    // Writing the measurement to .y would move three of the four mounts.
    int moved = 0;
    for (const Mount& m : kShipped) {
        const float across = acrossBeamOffset(m.x, m.y, rad(m.thetaDeg));
        if (std::fabs(across - m.y) > 0.01f) moved++;
    }
    CHECK(moved == 3);
}

CHAD_TEST(mountOffset, writingBackAMeasurementLeavesAGoodMountAlone) {
    for (const Mount& m : kShipped) {
        const float t = rad(m.thetaDeg);
        float x = m.x, y = m.y;
        applyMountOffset(t, acrossBeamOffset(m.x, m.y, t), &x, &y);
        CHECK_NEAR(x, m.x, 1e-5);
        CHECK_NEAR(y, m.y, 1e-5);
    }
}

CHAD_TEST(mountOffset, aCorrectedMeasurementMovesOnlyAcrossTheBeam) {
    // Sensor pointing left; the sweep says it sits 1.25 in further left than
    // the pose claims. The along-beam arm must not move.
    const float t = rad(90.0f);
    float x = 0.50f, y = 6.25f;
    const float before = acrossBeamOffset(x, y, t);
    const float alongBefore = x * std::cos(t) + y * std::sin(t);

    applyMountOffset(t, before + 1.25f, &x, &y);

    CHECK_NEAR(acrossBeamOffset(x, y, t), before + 1.25, 1e-5);
    CHECK_NEAR(x * std::cos(t) + y * std::sin(t), alongBefore, 1e-5);
    // theta = 90 means across is -x, so the correction lands on x, not y.
    CHECK_NEAR(y, 6.25, 1e-5);
    CHECK_NEAR(x, 0.50 - 1.25, 1e-5);
}

CHAD_TEST(mountOffset, anAngledMountSplitsTheCorrectionOverBothAxes) {
    const float t = rad(45.0f);
    float x = 3.0f, y = 2.0f;
    const float alongBefore = x * std::cos(t) + y * std::sin(t);
    applyMountOffset(t, 0.0f, &x, &y);
    // Across driven to zero puts the mount on the beam axis, arm preserved.
    CHECK_NEAR(acrossBeamOffset(x, y, t), 0.0, 1e-5);
    CHECK_NEAR(x * std::cos(t) + y * std::sin(t), alongBefore, 1e-5);
    CHECK_NEAR(x, y, 1e-5);
}

CHAD_TEST(mountOffset, nullPointersAreIgnored) {
    float v = 1.0f;
    applyMountOffset(0.5f, 2.0f, nullptr, &v);
    CHECK(v == 1.0f);
    applyMountOffset(0.5f, 2.0f, &v, nullptr);
    CHECK(v == 1.0f);
}
