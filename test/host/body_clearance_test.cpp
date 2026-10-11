#include "chad/util/body_clearance.hpp"

#include <limits>
#include <vector>

#include "chad_test.hpp"

using namespace chad;

namespace {
struct Circle {
    float x, y, radius;
};

// The nine Override goals, as gamespecific/config.cpp registers them.
const std::vector<Circle> kPosts = {
    {-47.0f, -23.5f, 2.85f}, {-23.5f, -47.0f, 2.85f}, {23.5f, 47.0f, 2.85f},
    {47.0f, 23.5f, 2.85f},   {0.0f, 0.0f, 2.85f},     {-47.0f, 23.5f, 2.85f},
    {23.5f, -47.0f, 2.85f},  {-23.5f, 47.0f, 2.85f},  {47.0f, -23.5f, 2.85f},
};
}  // namespace

CHAD_TEST(bodyClearance, poseInsideAPostOverlaps) {
    // The disturbance case: the estimate ended 4.5 in from a post centre.
    CHECK(bodyOverlapsCircle(4.5f, 0.0f, 6.0f, kPosts));
    CHECK(bodyOverlapsCircle(-47.0f, -23.5f, 6.0f, kPosts));
}

CHAD_TEST(bodyClearance, boundaryIsRadiusPlusBody) {
    // 2.85 + 6.0 = 8.85 in from the centre.
    CHECK(bodyOverlapsCircle(8.8f, 0.0f, 6.0f, kPosts));
    CHECK(!bodyOverlapsCircle(8.9f, 0.0f, 6.0f, kPosts));
    CHECK(bodyOverlapsCircle(-6.2f, 6.2f, 6.0f, kPosts));   // 8.77 in, diagonal
    CHECK(!bodyOverlapsCircle(-6.3f, 6.3f, 6.0f, kPosts));  // 8.91 in
}

CHAD_TEST(bodyClearance, openFieldIsClear) {
    CHECK(!bodyOverlapsCircle(-36.0f, -10.0f, 6.0f, kPosts));
    CHECK(!bodyOverlapsCircle(-60.0f, 0.0f, 6.0f, kPosts));
    CHECK(!bodyOverlapsCircle(-12.0f, -36.0f, 6.0f, kPosts));
}

CHAD_TEST(bodyClearance, zeroOrInvalidBodyDisablesTheTest) {
    CHECK(!bodyOverlapsCircle(0.0f, 0.0f, 0.0f, kPosts));
    CHECK(!bodyOverlapsCircle(0.0f, 0.0f, -1.0f, kPosts));
    CHECK(!bodyOverlapsCircle(0.0f, 0.0f, std::numeric_limits<float>::quiet_NaN(), kPosts));
}

CHAD_TEST(bodyClearance, noCirclesNeverOverlaps) {
    const std::vector<Circle> none;
    CHECK(!bodyOverlapsCircle(0.0f, 0.0f, 6.0f, none));
}
