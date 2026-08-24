#include "chad/movement_control/spline.hpp"

#include <cmath>
#include <vector>

#include "chad_test.hpp"

using namespace chad;

namespace {

SplineControlVector cv(float px, float py, float dx, float dy, float ddx = 0.0f,
                       float ddy = 0.0f) {
    return {{px, py}, {dx, dy}, {ddx, ddy}};
}

void checkVec(const Translation2d& got, const Translation2d& want, float tol) {
    CHECK_NEAR(got.x, want.x, tol);
    CHECK_NEAR(got.y, want.y, tol);
}

/// Curvature from finite differences of heading against arc length. Shares no
/// code with the analytic formula under test.
double numericCurvature(const Spline& spline, double t, double h = 5e-3) {
    PoseWithCurvature before{};
    PoseWithCurvature after{};
    if (!spline.pointAt(static_cast<float>(t - h), before)) return NAN;
    if (!spline.pointAt(static_cast<float>(t + h), after)) return NAN;
    const double dtheta = angleDiffRad(before.pose.theta, after.pose.theta);
    const double ds = before.pose.translation().distanceTo(after.pose.translation());
    return ds > 0.0 ? dtheta / ds : NAN;
}

std::vector<PathWaypoint> curvyPath() {
    return {{Pose{0.0f, 0.0f, 0.0f}, 0.0f},
            {Pose{24.0f, 24.0f, HALF_PI}, 0.0f},
            {Pose{0.0f, 48.0f, PI}, 0.0f},
            {Pose{-30.0f, 20.0f, -HALF_PI}, 0.0f}};
}

}  // namespace

CHAD_TEST(spline, cubic_hits_its_control_vectors) {
    const CubicHermiteSpline s(cv(1.0f, 2.0f, 10.0f, -3.0f), cv(20.0f, 8.0f, -4.0f, 6.0f));
    checkVec(s.positionAt(0.0f), {1.0f, 2.0f}, 1e-4f);
    checkVec(s.positionAt(1.0f), {20.0f, 8.0f}, 1e-3f);
    checkVec(s.derivativeAt(0.0f), {10.0f, -3.0f}, 1e-4f);
    checkVec(s.derivativeAt(1.0f), {-4.0f, 6.0f}, 1e-3f);
}

CHAD_TEST(spline, quintic_hits_its_control_vectors) {
    const QuinticHermiteSpline s(cv(1.0f, 2.0f, 10.0f, -3.0f, 5.0f, 7.0f),
                                 cv(20.0f, 8.0f, -4.0f, 6.0f, -9.0f, 2.0f));
    checkVec(s.positionAt(0.0f), {1.0f, 2.0f}, 1e-4f);
    checkVec(s.positionAt(1.0f), {20.0f, 8.0f}, 2e-3f);
    checkVec(s.derivativeAt(0.0f), {10.0f, -3.0f}, 1e-4f);
    checkVec(s.derivativeAt(1.0f), {-4.0f, 6.0f}, 5e-3f);
    checkVec(s.secondDerivativeAt(0.0f), {5.0f, 7.0f}, 1e-4f);
    checkVec(s.secondDerivativeAt(1.0f), {-9.0f, 2.0f}, 2e-2f);
}

CHAD_TEST(spline, straight_line_has_zero_curvature) {
    const CubicHermiteSpline s(cv(0.0f, 0.0f, 120.0f, 0.0f), cv(100.0f, 0.0f, 120.0f, 0.0f));
    for (int i = 0; i <= 20; i++) {
        PoseWithCurvature p{};
        CHECK(s.pointAt(static_cast<float>(i) / 20.0f, p));
        CHECK_NEAR(p.curvature, 0.0, 1e-9);
        CHECK_NEAR(p.pose.y, 0.0, 1e-4);
        CHECK_NEAR(p.pose.theta, 0.0, 1e-6);
    }
}

CHAD_TEST(spline, curvature_of_a_known_circle) {
    constexpr float R = 20.0f;
    const float sweep = PI / 3.0f;
    const float c0 = std::cos(0.0f);
    const float s0 = std::sin(0.0f);
    const float c1 = std::cos(sweep);
    const float s1 = std::sin(sweep);

    // Exact circle derivatives at both ends: k must come out at 1/R there.
    const QuinticHermiteSpline s(
        cv(R * c0, R * s0, -R * sweep * s0, R * sweep * c0, -R * sweep * sweep * c0,
           -R * sweep * sweep * s0),
        cv(R * c1, R * s1, -R * sweep * s1, R * sweep * c1, -R * sweep * sweep * c1,
           -R * sweep * sweep * s1));

    PoseWithCurvature at0{};
    PoseWithCurvature at1{};
    CHECK(s.pointAt(0.0f, at0));
    CHECK(s.pointAt(1.0f, at1));
    CHECK_NEAR(at0.curvature, 1.0 / R, 1e-6);
    CHECK_NEAR(at1.curvature, 1.0 / R, 1e-4);

    for (int i = 1; i < 20; i++) {
        const float t = static_cast<float>(i) / 20.0f;
        PoseWithCurvature p{};
        CHECK(s.pointAt(t, p));
        CHECK_NEAR(p.curvature, 1.0 / R, 0.02 / R);
        CHECK_NEAR(p.pose.translation().norm(), R, 0.02);
    }
}

CHAD_TEST(spline, curvature_matches_finite_differences) {
    const QuinticHermiteSpline s(cv(0.0f, 0.0f, 30.0f, 10.0f, -20.0f, 40.0f),
                                 cv(40.0f, 30.0f, 5.0f, 35.0f, 15.0f, -10.0f));
    for (int i = 1; i < 20; i++) {
        const double t = static_cast<double>(i) / 20.0;
        PoseWithCurvature p{};
        CHECK(s.pointAt(static_cast<float>(t), p));
        const double reference = numericCurvature(s, t);
        CHECK_FINITE(reference);
        CHECK_NEAR(p.curvature, reference, 2e-3 * std::fmax(1.0, std::fabs(reference)) + 2e-4);
    }
}

CHAD_TEST(spline, cubic_chain_is_c2_at_every_join) {
    const Translation2d interior[3] = {{20.0f, 15.0f}, {45.0f, -10.0f}, {70.0f, 25.0f}};
    std::vector<CubicHermiteSpline> splines;
    CHECK(cubicSplinesFromWaypoints({0.0f, 0.0f, 0.4f}, interior, 3, {90.0f, 0.0f, -0.3f},
                                    splines) == SplineStatus::Ok);
    CHECK_EQ(splines.size(), static_cast<size_t>(4));

    for (size_t i = 0; i + 1 < splines.size(); i++) {
        checkVec(splines[i].positionAt(1.0f), splines[i + 1].positionAt(0.0f), 2e-3f);
        checkVec(splines[i].derivativeAt(1.0f), splines[i + 1].derivativeAt(0.0f), 5e-2f);
        checkVec(splines[i].secondDerivativeAt(1.0f), splines[i + 1].secondDerivativeAt(0.0f),
                 0.2f);

        PoseWithCurvature a{};
        PoseWithCurvature b{};
        CHECK(splines[i].pointAt(1.0f, a));
        CHECK(splines[i + 1].pointAt(0.0f, b));
        CHECK_NEAR(a.curvature, b.curvature, 1e-4);
    }
}

CHAD_TEST(spline, cubic_chain_tridiagonal_residual_is_zero) {
    const Translation2d interior[4] = {
        {15.0f, 10.0f}, {30.0f, -5.0f}, {50.0f, 20.0f}, {65.0f, 5.0f}};
    std::vector<CubicHermiteSpline> splines;
    CHECK(cubicSplinesFromWaypoints({0.0f, 0.0f, 0.2f}, interior, 4, {80.0f, 0.0f, 0.0f},
                                    splines) == SplineStatus::Ok);

    // m[i-1] + 4 m[i] + m[i+1] == 3 (w[i+1] - w[i-1]) is exactly the C2 system.
    for (size_t i = 1; i < splines.size(); i++) {
        const Translation2d prev = splines[i - 1].startControl().d;
        const Translation2d here = splines[i].startControl().d;
        const Translation2d next = splines[i].endControl().d;
        const Translation2d wPrev = splines[i - 1].startControl().p;
        const Translation2d wNext = splines[i].endControl().p;

        const Translation2d lhs = prev + here * 4.0f + next;
        const Translation2d rhs = (wNext - wPrev) * 3.0f;
        CHECK_NEAR(lhs.x, rhs.x, 5e-3);
        CHECK_NEAR(lhs.y, rhs.y, 5e-3);
    }
}

CHAD_TEST(spline, cubic_chain_single_interior_uses_the_same_system) {
    const Translation2d interior[1] = {{50.0f, 30.0f}};
    std::vector<CubicHermiteSpline> splines;
    CHECK(cubicSplinesFromWaypoints({0.0f, 0.0f, 0.0f}, interior, 1, {100.0f, 0.0f, 0.0f},
                                    splines) == SplineStatus::Ok);
    CHECK_EQ(splines.size(), static_cast<size_t>(2));

    // The one unknown row carries both boundary corrections:
    // 4 m1 = 3 (w2 - w0) - m0 - m2.
    const Translation2d m0 = splines[0].startControl().d;
    const Translation2d m1 = splines[0].endControl().d;
    const Translation2d m2 = splines[1].endControl().d;
    const Translation2d w0 = splines[0].startControl().p;
    const Translation2d w2 = splines[1].endControl().p;
    const Translation2d expected = ((w2 - w0) * 3.0f - m0 - m2) * 0.25f;
    checkVec(m1, expected, 1e-3f);

    checkVec(splines[0].secondDerivativeAt(1.0f), splines[1].secondDerivativeAt(0.0f), 5e-2f);
}

CHAD_TEST(spline, quintic_chain_is_c1_before_optimizing) {
    const std::vector<PathWaypoint> path = curvyPath();
    std::vector<QuinticHermiteSpline> splines;
    CHECK(quinticSplinesFromWaypoints(path.data(), path.size(), splines) == SplineStatus::Ok);
    CHECK_EQ(splines.size(), static_cast<size_t>(3));

    for (size_t i = 0; i + 1 < splines.size(); i++) {
        checkVec(splines[i].positionAt(1.0f), splines[i + 1].positionAt(0.0f), 2e-3f);
        // A per-waypoint tangent magnitude is what makes this hold; a
        // per-segment one would leave the two sides different lengths.
        checkVec(splines[i].derivativeAt(1.0f), splines[i + 1].derivativeAt(0.0f), 5e-3f);
    }
}

CHAD_TEST(spline, quintic_chain_is_c2_after_optimizing) {
    const std::vector<PathWaypoint> path = curvyPath();
    std::vector<QuinticHermiteSpline> splines;
    CHECK(quinticSplinesFromWaypoints(path.data(), path.size(), splines) == SplineStatus::Ok);
    optimizeQuinticCurvature(splines);

    for (size_t i = 0; i + 1 < splines.size(); i++) {
        checkVec(splines[i].positionAt(1.0f), splines[i + 1].positionAt(0.0f), 2e-3f);
        checkVec(splines[i].derivativeAt(1.0f), splines[i + 1].derivativeAt(0.0f), 5e-3f);
        // Every knot, not just the last: this is where WPILib's pairwise
        // rebuild drops the second derivative it assigned one knot earlier.
        checkVec(splines[i].secondDerivativeAt(1.0f), splines[i + 1].secondDerivativeAt(0.0f),
                 0.05f);

        PoseWithCurvature a{};
        PoseWithCurvature b{};
        CHECK(splines[i].pointAt(1.0f, a));
        CHECK(splines[i + 1].pointAt(0.0f, b));
        CHECK_NEAR(a.curvature, b.curvature, 1e-4);
    }
}

CHAD_TEST(spline, explicit_tangent_magnitude_is_honoured) {
    std::vector<PathWaypoint> path = {{Pose{0.0f, 0.0f, 0.0f}, 60.0f},
                                      {Pose{40.0f, 20.0f, 0.0f}, 5.0f}};
    std::vector<QuinticHermiteSpline> splines;
    CHECK(quinticSplinesFromWaypoints(path.data(), path.size(), splines) == SplineStatus::Ok);
    CHECK_NEAR(splines[0].derivativeAt(0.0f).norm(), 60.0, 1e-3);
    CHECK_NEAR(splines[0].derivativeAt(1.0f).norm(), 5.0, 1e-3);
}

CHAD_TEST(spline, parameterize_respects_the_tolerances) {
    const std::vector<PathWaypoint> path = curvyPath();
    std::vector<QuinticHermiteSpline> splines;
    CHECK(quinticSplinesFromWaypoints(path.data(), path.size(), splines) == SplineStatus::Ok);
    optimizeQuinticCurvature(splines);

    const ParameterizeTolerance tol{1.0f, 0.02f, 0.05f, 20};
    std::vector<PoseWithCurvature> points;
    CHECK(parameterizeChain(splines, points, tol) == SplineStatus::Ok);
    CHECK(points.size() > 20);

    for (size_t i = 0; i + 1 < points.size(); i++) {
        const Twist2d twist = points[i].pose.log(points[i + 1].pose);
        CHECK(std::fabs(twist.dx) <= tol.maxDx * 1.001f);
        CHECK(std::fabs(twist.dy) <= tol.maxDy * 1.001f + 1e-5f);
        CHECK(std::fabs(twist.dtheta) <= tol.maxDtheta * 1.001f);
    }
}

CHAD_TEST(spline, parameterize_keeps_both_endpoints_and_no_duplicate_joins) {
    const std::vector<PathWaypoint> path = curvyPath();
    std::vector<QuinticHermiteSpline> splines;
    CHECK(quinticSplinesFromWaypoints(path.data(), path.size(), splines) == SplineStatus::Ok);
    optimizeQuinticCurvature(splines);

    std::vector<PoseWithCurvature> points;
    CHECK(parameterizeChain(splines, points) == SplineStatus::Ok);

    PoseWithCurvature first{};
    PoseWithCurvature last{};
    CHECK(splines.front().pointAt(0.0f, first));
    CHECK(splines.back().pointAt(1.0f, last));
    checkVec(points.front().pose.translation(), first.pose.translation(), 1e-3f);
    checkVec(points.back().pose.translation(), last.pose.translation(), 1e-3f);

    for (size_t i = 0; i + 1 < points.size(); i++) {
        CHECK(points[i].pose.distanceTo(points[i + 1].pose) > 1e-6f);
    }
}

CHAD_TEST(spline, parameterize_with_a_loose_tolerance_keeps_only_the_ends) {
    const CubicHermiteSpline s(cv(0.0f, 0.0f, 12.0f, 0.0f), cv(10.0f, 0.0f, 12.0f, 0.0f));
    std::vector<PoseWithCurvature> points;
    CHECK(parameterize(s, points, {1000.0f, 1000.0f, 10.0f, 20}) == SplineStatus::Ok);
    CHECK_EQ(points.size(), static_cast<size_t>(2));
}

CHAD_TEST(spline, parameterize_fails_on_an_unreachable_tolerance) {
    const std::vector<PathWaypoint> path = curvyPath();
    std::vector<QuinticHermiteSpline> splines;
    CHECK(quinticSplinesFromWaypoints(path.data(), path.size(), splines) == SplineStatus::Ok);

    std::vector<PoseWithCurvature> points;
    CHECK(parameterizeChain(splines, points, {5.0f, 1e-12f, 0.0872f, 20}) ==
          SplineStatus::DepthExceeded);
}

CHAD_TEST(spline, parameterize_honours_a_shallow_depth_cap) {
    const std::vector<PathWaypoint> path = curvyPath();
    std::vector<QuinticHermiteSpline> splines;
    CHECK(quinticSplinesFromWaypoints(path.data(), path.size(), splines) == SplineStatus::Ok);

    std::vector<PoseWithCurvature> points;
    CHECK(parameterizeChain(splines, points, {5.0f, 0.05f, 0.0872f, 1}) ==
          SplineStatus::DepthExceeded);
}

CHAD_TEST(spline, parameterize_rejects_a_vanishing_tangent) {
    const CubicHermiteSpline zero;
    std::vector<PoseWithCurvature> points;
    CHECK(parameterize(zero, points) == SplineStatus::TangentVanished);
    CHECK(points.empty());
}

CHAD_TEST(spline, parameterize_rejects_an_inline_180_degree_cusp) {
    // Opposed headings on one line make the tangent pass through zero. Whether
    // a sampled t lands on the zero or merely straddles it decides which of the
    // two malformed statuses comes back; both are failures, neither hangs.
    const std::vector<PathWaypoint> path = {{Pose{0.0f, 0.0f, 0.0f}, 0.0f},
                                            {Pose{24.0f, 0.0f, PI}, 0.0f}};
    std::vector<QuinticHermiteSpline> splines;
    CHECK(quinticSplinesFromWaypoints(path.data(), path.size(), splines) == SplineStatus::Ok);

    // The tangent really does reverse: it is +x at both ends of a segment that
    // ends 24 inches ahead of where it started.
    CHECK(splines[0].derivativeAt(0.0f).x > 0.0f);
    CHECK(splines[0].derivativeAt(1.0f).x < 0.0f);

    std::vector<PoseWithCurvature> points;
    const SplineStatus status = parameterizeChain(splines, points);
    CHECK(status == SplineStatus::TangentVanished || status == SplineStatus::DepthExceeded);
}

CHAD_TEST(spline, parameterize_terminates_on_near_coincident_opposed_waypoints) {
    const std::vector<PathWaypoint> path = {{Pose{0.0f, 0.0f, 0.0f}, 0.0f},
                                            {Pose{0.02f, 0.0f, PI}, 0.0f}};
    std::vector<QuinticHermiteSpline> splines;
    CHECK(quinticSplinesFromWaypoints(path.data(), path.size(), splines) == SplineStatus::Ok);

    std::vector<PoseWithCurvature> points;
    const SplineStatus status = parameterizeChain(splines, points);
    CHECK(status != SplineStatus::Ok);
}

CHAD_TEST(spline, helper_rejects_bad_waypoint_counts) {
    std::vector<QuinticHermiteSpline> quintics;
    CHECK(quinticSplinesFromWaypoints(nullptr, 0, quintics) ==
          SplineStatus::NotEnoughWaypoints);

    const PathWaypoint one{Pose{1.0f, 1.0f, 0.0f}, 0.0f};
    CHECK(quinticSplinesFromWaypoints(&one, 1, quintics) == SplineStatus::NotEnoughWaypoints);

    std::vector<PathWaypoint> many;
    for (size_t i = 0; i <= MAX_PATH_WAYPOINTS; i++) {
        many.push_back({Pose{static_cast<float>(i) * 5.0f, 0.0f, 0.0f}, 0.0f});
    }
    CHECK(quinticSplinesFromWaypoints(many.data(), many.size(), quintics) ==
          SplineStatus::TooManyWaypoints);

    std::vector<CubicHermiteSpline> cubics;
    std::vector<Translation2d> interior(MAX_PATH_WAYPOINTS, Translation2d{1.0f, 1.0f});
    CHECK(cubicSplinesFromWaypoints({0.0f, 0.0f, 0.0f}, interior.data(), interior.size(),
                                    {50.0f, 0.0f, 0.0f},
                                    cubics) == SplineStatus::TooManyWaypoints);
}

CHAD_TEST(spline, helper_rejects_coincident_waypoints) {
    const std::vector<PathWaypoint> path = {{Pose{10.0f, 10.0f, 0.0f}, 0.0f},
                                            {Pose{10.0f, 10.0f, 1.0f}, 0.0f}};
    std::vector<QuinticHermiteSpline> quintics;
    CHECK(quinticSplinesFromWaypoints(path.data(), path.size(), quintics) ==
          SplineStatus::DegenerateWaypoints);

    const Translation2d interior[1] = {{10.0f, 10.0f}};
    std::vector<CubicHermiteSpline> cubics;
    CHECK(cubicSplinesFromWaypoints({10.0f, 10.0f, 0.0f}, interior, 1, {50.0f, 0.0f, 0.0f},
                                    cubics) == SplineStatus::DegenerateWaypoints);
}

CHAD_TEST(spline, straight_line_arc_length_is_exact) {
    const std::vector<PathWaypoint> path = {{Pose{0.0f, 0.0f, 0.0f}, 0.0f},
                                            {Pose{100.0f, 0.0f, 0.0f}, 0.0f}};
    std::vector<QuinticHermiteSpline> splines;
    CHECK(quinticSplinesFromWaypoints(path.data(), path.size(), splines) == SplineStatus::Ok);

    std::vector<PoseWithCurvature> points;
    CHECK(parameterizeChain(splines, points, {0.5f, 0.05f, 0.0872f, 20}) == SplineStatus::Ok);

    double length = 0.0;
    for (size_t i = 0; i + 1 < points.size(); i++) {
        length += points[i].pose.distanceTo(points[i + 1].pose);
        CHECK(points[i + 1].pose.x > points[i].pose.x);
    }
    CHECK_NEAR(length, 100.0, 1e-2);
}

CHAD_TEST(spline, optimize_curvature_is_a_noop_below_two_splines) {
    std::vector<QuinticHermiteSpline> splines;
    optimizeQuinticCurvature(splines);
    CHECK(splines.empty());

    splines.emplace_back(cv(0.0f, 0.0f, 10.0f, 0.0f, 3.0f, 4.0f),
                         cv(10.0f, 0.0f, 10.0f, 0.0f, 1.0f, 2.0f));
    optimizeQuinticCurvature(splines);
    checkVec(splines[0].secondDerivativeAt(0.0f), {3.0f, 4.0f}, 1e-4f);
}
