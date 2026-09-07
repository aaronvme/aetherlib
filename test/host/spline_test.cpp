#include "chad/movement_control/spline.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "chad_test.hpp"

using chad::Pose;
using chad::splinePath;

namespace {

/// Total polyline length of a sampled path.
float pathLength(const std::vector<Pose>& p) {
    float total = 0.0f;
    for (std::size_t i = 1; i < p.size(); i++) {
        total += std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
    }
    return total;
}

/// Largest gap between consecutive samples.
float maxGap(const std::vector<Pose>& p) {
    float worst = 0.0f;
    for (std::size_t i = 1; i < p.size(); i++) {
        worst = std::max(worst, std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y));
    }
    return worst;
}

/// Closest approach of any sample to a point -- how near the curve passes.
float closestTo(const std::vector<Pose>& p, float x, float y) {
    float best = 1e9f;
    for (const auto& q : p) best = std::min(best, std::hypot(q.x - x, q.y - y));
    return best;
}

/// Distance from a point to the polyline through `p`, not to its vertices.
///
/// Comparing to the nearest vertex is what a resampling test must NOT do: a new
/// sample halfway between two vertices 1 in apart is 0.5 in from both of them
/// while sitting exactly on the curve.
float distanceToCurve(const std::vector<Pose>& p, float x, float y) {
    float best = 1e9f;
    for (std::size_t i = 1; i < p.size(); i++) {
        const float ex = p[i].x - p[i - 1].x, ey = p[i].y - p[i - 1].y;
        const float len2 = ex * ex + ey * ey;
        float t = (len2 > 1e-9f)
                      ? ((x - p[i - 1].x) * ex + (y - p[i - 1].y) * ey) / len2
                      : 0.0f;
        t = std::clamp(t, 0.0f, 1.0f);
        best = std::min(best, std::hypot(x - (p[i - 1].x + t * ex),
                                        y - (p[i - 1].y + t * ey)));
    }
    return best;
}

const std::vector<Pose> kElbow{{0.0f, 0.0f, 0.0f},
                               {24.0f, 0.0f, 0.0f},
                               {24.0f, 24.0f, 0.0f}};

}  // namespace

CHAD_TEST(spline, passes_through_its_endpoints) {
    const auto p = splinePath(kElbow);
    CHECK(p.size() > 6);
    CHECK_NEAR(p.front().x, 0.0, 1e-4);
    CHECK_NEAR(p.front().y, 0.0, 1e-4);
    CHECK_NEAR(p.back().x, 24.0, 1e-4);
    CHECK_NEAR(p.back().y, 24.0, 1e-4);
}

CHAD_TEST(spline, passes_near_the_interior_control_point) {
    // Catmull-Rom interpolates its control points, so the middle one is ON the
    // curve -- not merely approached, the way a B-spline would.
    const auto p = splinePath(kElbow);
    CHECK_NEAR(closestTo(p, 24.0f, 0.0f), 0.0, 1e-4);
}

CHAD_TEST(spline, the_default_spacing_is_the_one_the_preview_uses) {
    // 2 in, matching buildPath() in sim/web/route_dsl.js on the sim branch. If
    // these drift the sim draws one curve and drives another.
    CHECK_NEAR(chad::SPLINE_SPACING_IN, 2.0, 1e-9);
    CHECK(splinePath(kElbow).size() == splinePath(kElbow, 2.0f).size());
}

CHAD_TEST(spline, samples_near_the_requested_spacing) {
    for (float spacing : {1.0f, 2.0f, 4.0f, 8.0f}) {
        const auto p = splinePath(kElbow, spacing);
        // Steps come from the chord and the curve is longer, so a gap may
        // exceed the request -- but not by much, and never by half again.
        CHECK(maxGap(p) <= spacing * 1.5f);
        CHECK(p.size() >= 3);
    }
}

CHAD_TEST(spline, never_fewer_than_two_samples_per_segment) {
    // A short segment between two long ones must still be sampled, or it
    // collapses to its chord and the curve kinks there.
    const auto p = splinePath({{0.0f, 0.0f, 0.0f},
                               {30.0f, 0.0f, 0.0f},
                               {30.5f, 0.5f, 0.0f},     // 0.7 in segment
                               {60.0f, 0.0f, 0.0f}}, 8.0f);
    // Three segments, at least two samples each, plus the final point.
    CHECK(p.size() >= 3 * 2 + 1);
}

CHAD_TEST(spline, a_finer_spacing_gives_more_points_and_the_same_shape) {
    const auto coarse = splinePath(kElbow, 8.0f);
    const auto fine = splinePath(kElbow, 1.0f);
    CHECK(fine.size() > coarse.size() * 3);
    // Same curve, so the arc length agrees; the coarse one cuts corners
    // slightly and reads a little short.
    CHECK_NEAR(pathLength(fine), pathLength(coarse), pathLength(fine) * 0.05);
}

CHAD_TEST(spline, headings_are_tangent_to_travel) {
    const auto p = splinePath(kElbow);
    for (std::size_t i = 1; i + 1 < p.size(); i++) {
        const float dx = p[i + 1].x - p[i - 1].x;
        const float dy = p[i + 1].y - p[i - 1].y;
        if (std::hypot(dx, dy) < 1e-5f) continue;
        const float want = std::atan2(dy, dx);
        const float diff = std::atan2(std::sin(p[i].theta - want),
                                      std::cos(p[i].theta - want));
        CHECK_NEAR(diff, 0.0, 1e-3);
    }
}

CHAD_TEST(spline, headings_actually_turn_through_a_corner) {
    // The profile limits speed by differentiating these, so a curve whose
    // headings never change would silently have no cornering limit at all.
    const auto p = splinePath(kElbow);
    float lo = 1e9f, hi = -1e9f;
    for (const auto& q : p) { lo = std::min(lo, q.theta); hi = std::max(hi, q.theta); }
    // The elbow turns 90 degrees, so the tangent has to sweep about that much.
    CHECK((hi - lo) > 1.3);
}

CHAD_TEST(spline, control_headings_are_ignored) {
    // A control point says where the curve goes, not which way the robot faces.
    // Anything else would let a stale theta fight the tangent the profiler
    // needs.
    std::vector<Pose> nonsense = kElbow;
    for (auto& c : nonsense) c.theta = 3.0f;
    const auto a = splinePath(kElbow);
    const auto b = splinePath(nonsense);
    CHECK(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); i++) {
        CHECK_NEAR(a[i].x, b[i].x, 1e-5);
        CHECK_NEAR(a[i].theta, b[i].theta, 1e-5);
    }
}

CHAD_TEST(spline, two_control_points_densify_a_straight_line) {
    const auto p = splinePath({{0.0f, 0.0f, 0.0f}, {40.0f, 0.0f, 0.0f}}, 4.0f);
    CHECK(p.size() >= 10);
    CHECK_NEAR(pathLength(p), 40.0, 1e-3);
    for (const auto& q : p) CHECK_NEAR(q.y, 0.0, 1e-4);
}

CHAD_TEST(spline, resampling_a_dense_path_is_nearly_a_no_op) {
    // moveRAMSETE resamples by default, so passing it a path that is already
    // dense must not deform it -- otherwise the default would quietly change
    // the trajectory of every existing caller.
    const auto dense = splinePath(kElbow, 1.0f);
    const auto again = splinePath(dense, 1.0f);
    CHECK_NEAR(pathLength(again), pathLength(dense), pathLength(dense) * 0.02);
    CHECK_NEAR(again.front().x, dense.front().x, 1e-4);
    CHECK_NEAR(again.back().x, dense.back().x, 1e-4);
    // Every sample of the re-run lands on the original CURVE, measured to the
    // polyline rather than to its vertices.
    for (const auto& q : again) CHECK(distanceToCurve(dense, q.x, q.y) < 0.05f);
}

CHAD_TEST(spline, degenerate_input_is_returned_rather_than_crashing) {
    CHECK(splinePath(std::vector<Pose>{}).empty());

    const std::vector<Pose> single{{5.0f, 6.0f, 1.0f}};
    const auto one = splinePath(single);
    CHECK(one.size() == 1);
    CHECK_NEAR(one[0].x, 5.0, 1e-6);
    // Returned unchanged, theta included: there is no tangent to measure.
    CHECK_NEAR(one[0].theta, 1.0, 1e-6);
}

CHAD_TEST(spline, coincident_control_points_do_not_produce_a_nan_heading) {
    const auto p = splinePath({{0.0f, 0.0f, 0.0f},
                               {10.0f, 0.0f, 0.0f},
                               {10.0f, 0.0f, 0.0f},
                               {20.0f, 0.0f, 0.0f}});
    CHECK(p.size() >= 4);
    for (const auto& q : p) {
        CHECK(std::isfinite(q.x));
        CHECK(std::isfinite(q.y));
        CHECK(std::isfinite(q.theta));
    }
}

CHAD_TEST(spline, a_tiny_spacing_is_capped_rather_than_exhausting_memory) {
    const auto p = splinePath(kElbow, 0.0f);
    CHECK(p.size() <= chad::MAX_SPLINE_POINTS);
    CHECK(p.size() > 3);

    const auto neg = splinePath(kElbow, -5.0f);
    CHECK(neg.size() <= chad::MAX_SPLINE_POINTS);
    CHECK(neg.size() > 3);
}

CHAD_TEST(spline, a_closed_loop_returns_to_its_start) {
    // The shape a route actually wants: out, around, and back.
    const auto p = splinePath({{0.0f, 0.0f, 0.0f},
                               {24.0f, 12.0f, 0.0f},
                               {0.0f, 24.0f, 0.0f},
                               {-24.0f, 12.0f, 0.0f},
                               {0.0f, 0.0f, 0.0f}});
    CHECK_NEAR(p.back().x, 0.0, 1e-4);
    CHECK_NEAR(p.back().y, 0.0, 1e-4);
    // Longer than the straight-line circuit through the same corners, because
    // it is a curve; a spline that had collapsed to the polyline would match it.
    const float polyline = 4.0f * std::hypot(24.0f, 12.0f);
    CHECK(pathLength(p) > polyline * 1.01f);
}
