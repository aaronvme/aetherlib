#include "chad/util/field_mirror.hpp"

#include <cmath>
#include <vector>

#include "chad_test.hpp"

using namespace chad;

namespace {

const FieldMirror kAll[5] = {FieldMirror::None,  FieldMirror::Rot180,
                             FieldMirror::FlipX, FieldMirror::FlipY,
                             FieldMirror::Diagonal};

const char* nameOf(FieldMirror m) {
    switch (m) {
        case FieldMirror::None: return "None";
        case FieldMirror::Rot180: return "Rot180";
        case FieldMirror::FlipX: return "FlipX";
        case FieldMirror::FlipY: return "FlipY";
        case FieldMirror::Diagonal: return "Diagonal";
    }
    return "?";
}

// nine posts plus the four wall corners: the map the mirror has to preserve
std::vector<std::pair<float, float>> fieldLandmarks() {
    std::vector<std::pair<float, float>> v{{0.0f, 0.0f}};
    for (float x : {23.5f, -23.5f})
        for (float y : {47.0f, -47.0f}) v.push_back({x, y});
    for (float x : {47.0f, -47.0f})
        for (float y : {23.5f, -23.5f}) v.push_back({x, y});
    for (float x : {70.25f, -70.25f})
        for (float y : {70.25f, -70.25f}) v.push_back({x, y});
    return v;
}

bool hasLandmark(const std::vector<std::pair<float, float>>& v, float x, float y) {
    for (const auto& p : v) {
        if (std::fabs(p.first - x) < 1e-3f && std::fabs(p.second - y) < 1e-3f) {
            return true;
        }
    }
    return false;
}

float angDiff(float a, float b) {
    return std::fabs(wrapRad(a - b));
}

// drive the pose forward by d along its own heading
Pose advance(const Pose& p, float d) {
    return {p.x + d * std::cos(p.theta), p.y + d * std::sin(p.theta), p.theta};
}

std::vector<Pose> samplePoses() {
    std::vector<Pose> v;
    const float xs[] = {0.0f, 12.3f, -61.5f, 70.25f, -70.25f, -46.1f};
    const float ys[] = {0.0f, -23.5f, 47.0f, -70.25f, 8.75f, -43.0f};
    // includes the wrap boundaries exactly
    const float ths[] = {0.0f,   HALF_PI, PI,      -PI,
                         -HALF_PI, 0.7854f, -2.6f, 3.1415f};
    for (float x : xs)
        for (float y : ys)
            for (float th : ths) v.push_back({x, y, th});
    return v;
}

}  // namespace

CHAD_TEST(fieldMirror, everyTransformIsItsOwnInverse) {
    for (FieldMirror m : kAll) {
        for (const Pose& p : samplePoses()) {
            const Pose back = mirrorPose(m, mirrorPose(m, p));
            CHECK_NEAR(back.x, p.x, 1e-3);
            CHECK_NEAR(back.y, p.y, 1e-3);
            if (angDiff(back.theta, p.theta) > 1e-3f) {
                CHAD_FAIL(std::string(nameOf(m)) + " heading is not an involution");
            }
        }
    }
}

// The one that catches a heading transform that does not match the position
// transform: mirroring a pose then driving forward must land where driving
// forward then mirroring lands.
CHAD_TEST(fieldMirror, headingAgreesWithPosition) {
    for (FieldMirror m : kAll) {
        for (const Pose& p : samplePoses()) {
            const Pose a = advance(mirrorPose(m, p), 10.0f);
            const Pose b = mirrorPose(m, advance(p, 10.0f));
            if (!chadtest::closeTo(a.x, b.x, 2e-3) ||
                !chadtest::closeTo(a.y, b.y, 2e-3)) {
                CHAD_FAIL(std::string(nameOf(m)) +
                          " heading disagrees with its position transform");
            }
        }
    }
}

CHAD_TEST(fieldMirror, mapsThePostsAndCornersOntoThemselves) {
    const auto marks = fieldLandmarks();
    for (FieldMirror m : kAll) {
        for (const auto& q : marks) {
            float x = q.first, y = q.second;
            mirrorPointInPlace(m, x, y);
            if (!hasLandmark(marks, x, y)) {
                CHAD_FAIL(std::string(nameOf(m)) + " moved a landmark off the map");
            }
        }
    }
}

CHAD_TEST(fieldMirror, reflectionsReverseTurnDirectionAndRotationsDoNot) {
    const float eps = 0.05f;
    for (FieldMirror m : kAll) {
        const Pose p{10.0f, -20.0f, 0.6f};
        const Pose turned{p.x, p.y, wrapRad(p.theta + eps)};
        const float before = mirrorPose(m, p).theta;
        const float after = mirrorPose(m, turned).theta;
        const float delta = wrapRad(after - before);
        const float want = mirrorFlipsHandedness(m) ? -eps : eps;
        CHECK_NEAR(delta, want, 1e-4);
    }
}

CHAD_TEST(fieldMirror, redToBlueMatchesTheHandMirroredStartPose) {
    // red_3_pin starts at (-61.5, 0) facing vex 270; blue was authored at
    // (61.5, 0) facing vex 90 before reflect() replaced it.
    const Pose red{-61.5f, 0.0f, vexToStdRad(270.0f)};
    const Pose blue = mirrorPose(FieldMirror::Rot180, red);
    CHECK_NEAR(blue.x, 61.5f, 1e-3);
    CHECK_NEAR(blue.y, 0.0f, 1e-3);
    CHECK_NEAR(stdRadToVexDeg(blue.theta), 90.0f, 1e-2);
}

CHAD_TEST(fieldMirror, headingsStayWrappedAtTheBoundary) {
    for (FieldMirror m : kAll) {
        for (float th : {PI, -PI, 0.0f, HALF_PI, -HALF_PI, 3.14159f, -3.14159f}) {
            const float out = mirrorHeadingRad(m, th);
            CHECK_FINITE(out);
            if (out < -PI - 1e-4f || out > PI + 1e-4f) {
                CHAD_FAIL(std::string(nameOf(m)) + " left a heading unwrapped");
            }
        }
    }
}

CHAD_TEST(fieldMirror, noneIsExactlyIdentity) {
    for (const Pose& p : samplePoses()) {
        const Pose out = mirrorPose(FieldMirror::None, p);
        CHECK_EQ(out.x, p.x);
        CHECK_EQ(out.y, p.y);
        CHECK_EQ(out.theta, p.theta);
    }
}

// A beam fix arrives in true field coordinates. setPoseFieldFrame must not
// transform it, and getPose must hand the routine back its own frame -- which
// only works because the transforms are involutions.
CHAD_TEST(fieldMirror, routineFrameRoundTripsThroughTheTrueFrame) {
    for (FieldMirror m : kAll) {
        for (const Pose& authored : samplePoses()) {
            const Pose trueFrame = mirrorPose(m, authored);   // setPose
            const Pose readBack = mirrorPose(m, trueFrame);   // getPose
            CHECK_NEAR(readBack.x, authored.x, 1e-3);
            CHECK_NEAR(readBack.y, authored.y, 1e-3);
            if (angDiff(readBack.theta, authored.theta) > 1e-3f) {
                CHAD_FAIL(std::string(nameOf(m)) + " round trip lost the heading");
            }
        }
    }
}
