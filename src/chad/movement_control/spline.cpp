#include "chad/movement_control/spline.hpp"

#include <cmath>

namespace chad {
namespace {

Translation2d horner(const Translation2d* c, int n, float t) {
    Translation2d acc{};
    for (int i = 0; i < n; i++) acc = acc * t + c[i];
    return acc;
}

/// Second derivative at t of the cubic Hermite through the same endpoints and
/// tangents. Used only to pick quintic knot second derivatives.
Translation2d cubicSecondDerivative(const Translation2d& p0, const Translation2d& d0,
                                    const Translation2d& p1, const Translation2d& d1,
                                    float t) {
    const Translation2d a3 = p0 * 2.0f + d0 - p1 * 2.0f + d1;
    const Translation2d a2 = p0 * -3.0f - d0 * 2.0f + p1 * 3.0f - d1;
    return a3 * (6.0f * t) + a2 * 2.0f;
}

}  // namespace

void Spline::setCoefficients(const Translation2d* c, int degree) {
    mDegree = degree;
    for (int i = 0; i <= degree; i++) mC[i] = c[i];
    for (int i = 0; i < degree; i++) mDC[i] = mC[i] * static_cast<float>(degree - i);
    for (int i = 0; i < degree - 1; i++) mDDC[i] = mDC[i] * static_cast<float>(degree - 1 - i);
}

Translation2d Spline::positionAt(float t) const { return horner(mC, mDegree + 1, t); }

Translation2d Spline::derivativeAt(float t) const { return horner(mDC, mDegree, t); }

Translation2d Spline::secondDerivativeAt(float t) const {
    return horner(mDDC, mDegree - 1, t);
}

bool Spline::pointAt(float t, PoseWithCurvature& out) const {
    const Translation2d p = positionAt(t);
    const Translation2d d = derivativeAt(t);
    const Translation2d dd = secondDerivativeAt(t);

    const float speedSq = d.x * d.x + d.y * d.y;
    if (!std::isfinite(speedSq) || speedSq < 1e-12f) return false;
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) return false;

    const float k = d.cross(dd) / (speedSq * std::sqrt(speedSq));
    if (!std::isfinite(k)) return false;

    out.pose = {p.x, p.y, std::atan2(d.y, d.x)};
    out.curvature = k;
    return true;
}

void CubicHermiteSpline::rebuild() {
    const Translation2d& p0 = mStart.p;
    const Translation2d& d0 = mStart.d;
    const Translation2d& p1 = mEnd.p;
    const Translation2d& d1 = mEnd.d;
    const Translation2d c[4] = {
        p0 * 2.0f + d0 - p1 * 2.0f + d1,
        p0 * -3.0f - d0 * 2.0f + p1 * 3.0f - d1,
        d0,
        p0,
    };
    setCoefficients(c, 3);
}

void QuinticHermiteSpline::rebuild() {
    const Translation2d& p0 = mStart.p;
    const Translation2d& d0 = mStart.d;
    const Translation2d& q0 = mStart.dd;
    const Translation2d& p1 = mEnd.p;
    const Translation2d& d1 = mEnd.d;
    const Translation2d& q1 = mEnd.dd;
    const Translation2d c[6] = {
        p0 * -6.0f + d0 * -3.0f + q0 * -0.5f + p1 * 6.0f + d1 * -3.0f + q1 * 0.5f,
        p0 * 15.0f + d0 * 8.0f + q0 * 1.5f + p1 * -15.0f + d1 * 7.0f + q1 * -1.0f,
        p0 * -10.0f + d0 * -6.0f + q0 * -1.5f + p1 * 10.0f + d1 * -4.0f + q1 * 0.5f,
        q0 * 0.5f,
        d0,
        p0,
    };
    setCoefficients(c, 5);
}

SplineStatus parameterize(const Spline& spline, std::vector<PoseWithCurvature>& out,
                          const ParameterizeTolerance& tol, float t0, float t1,
                          bool includeStart) {
    int maxDepth = tol.maxDepth;
    if (maxDepth < 1) maxDepth = 1;
    if (maxDepth > PARAMETERIZE_MAX_DEPTH) maxDepth = PARAMETERIZE_MAX_DEPTH;

    PoseWithCurvature start{};
    PoseWithCurvature end{};
    if (!spline.pointAt(t0, start)) return SplineStatus::TangentVanished;
    if (out.capacity() < 64) out.reserve(64);
    if (includeStart) out.push_back(start);

    struct Interval {
        float t0;
        float t1;
        int depth;
    };
    constexpr int STACK_CAP = PARAMETERIZE_MAX_DEPTH + 2;
    Interval stack[STACK_CAP];
    int top = 0;
    stack[top++] = {t0, t1, 0};

    while (top > 0) {
        const Interval cur = stack[--top];
        if (!spline.pointAt(cur.t0, start)) return SplineStatus::TangentVanished;
        if (!spline.pointAt(cur.t1, end)) return SplineStatus::TangentVanished;

        const Twist2d twist = start.pose.log(end.pose);
        // Written as a negated conjunction so a non-finite twist subdivides
        // (and eventually fails) instead of being silently accepted.
        const bool withinTolerance = std::fabs(twist.dx) <= tol.maxDx &&
                                     std::fabs(twist.dy) <= tol.maxDy &&
                                     std::fabs(twist.dtheta) <= tol.maxDtheta;
        if (!withinTolerance) {
            if (cur.depth >= maxDepth || top + 2 > STACK_CAP) {
                return SplineStatus::DepthExceeded;
            }
            const float mid = 0.5f * (cur.t0 + cur.t1);
            stack[top++] = {mid, cur.t1, cur.depth + 1};
            stack[top++] = {cur.t0, mid, cur.depth + 1};
        } else {
            if (out.size() >= PARAMETERIZE_MAX_POINTS) return SplineStatus::TooManyPoints;
            out.push_back(end);
        }
    }
    return SplineStatus::Ok;
}

SplineStatus cubicSplinesFromWaypoints(const Pose& start, const Translation2d* interior,
                                       size_t interiorCount, const Pose& end,
                                       std::vector<CubicHermiteSpline>& out,
                                       float startTangent, float endTangent) {
    if (interior == nullptr && interiorCount > 0) return SplineStatus::NotEnoughWaypoints;
    const size_t count = interiorCount + 2;
    if (count > MAX_PATH_WAYPOINTS) return SplineStatus::TooManyWaypoints;

    Translation2d w[MAX_PATH_WAYPOINTS];
    w[0] = start.translation();
    for (size_t i = 0; i < interiorCount; i++) w[i + 1] = interior[i];
    w[count - 1] = end.translation();

    for (size_t i = 1; i < count; i++) {
        if (w[i].distanceTo(w[i - 1]) < WAYPOINT_COINCIDENT_EPS) {
            return SplineStatus::DegenerateWaypoints;
        }
    }

    Translation2d m[MAX_PATH_WAYPOINTS];
    const float autoStart = 1.2f * w[0].distanceTo(w[1]);
    const float autoEnd = 1.2f * w[count - 1].distanceTo(w[count - 2]);
    m[0] = Translation2d::fromPolar(startTangent > 0.0f ? startTangent : autoStart,
                                    start.rotation());
    m[count - 1] =
        Translation2d::fromPolar(endTangent > 0.0f ? endTangent : autoEnd, end.rotation());

    // Interior tangents from C2 continuity: m[i-1] + 4 m[i] + m[i+1] =
    // 3 (w[i+1] - w[i-1]), with the known end tangents moved to the rhs. The
    // n == 1 row takes both corrections, which is why they are subtracted
    // rather than assigned.
    const size_t n = count - 2;
    if (n > 0) {
        Translation2d rhs[MAX_PATH_WAYPOINTS];
        for (size_t j = 0; j < n; j++) rhs[j] = (w[j + 2] - w[j]) * 3.0f;
        rhs[0] = rhs[0] - m[0];
        rhs[n - 1] = rhs[n - 1] - m[count - 1];

        // Thomas sweep. Unit off-diagonals and a diagonal of 4 make the pivot
        // 4 - cStar converge to 4 - (2 - sqrt(3)), so it never approaches zero
        // and no pivoting or zero check is needed.
        float cStar[MAX_PATH_WAYPOINTS];
        Translation2d dStar[MAX_PATH_WAYPOINTS];
        cStar[0] = 0.25f;
        dStar[0] = rhs[0] * 0.25f;
        for (size_t j = 1; j < n; j++) {
            const float inv = 1.0f / (4.0f - cStar[j - 1]);
            cStar[j] = inv;
            dStar[j] = (rhs[j] - dStar[j - 1]) * inv;
        }
        m[n] = dStar[n - 1];
        for (size_t j = n - 1; j-- > 0;) m[j + 1] = dStar[j] - m[j + 2] * cStar[j];
    }

    out.clear();
    out.reserve(count - 1);
    for (size_t i = 0; i + 1 < count; i++) {
        out.emplace_back(SplineControlVector{w[i], m[i], {}},
                         SplineControlVector{w[i + 1], m[i + 1], {}});
    }
    return SplineStatus::Ok;
}

SplineStatus quinticSplinesFromWaypoints(const PathWaypoint* waypoints, size_t count,
                                         std::vector<QuinticHermiteSpline>& out) {
    if (waypoints == nullptr || count < 2) return SplineStatus::NotEnoughWaypoints;
    if (count > MAX_PATH_WAYPOINTS) return SplineStatus::TooManyWaypoints;

    float seg[MAX_PATH_WAYPOINTS];
    for (size_t i = 0; i + 1 < count; i++) {
        seg[i] =
            waypoints[i].pose.translation().distanceTo(waypoints[i + 1].pose.translation());
        if (seg[i] < WAYPOINT_COINCIDENT_EPS) return SplineStatus::DegenerateWaypoints;
    }

    // One tangent magnitude per waypoint, not per segment: a knot shared by two
    // segments of different length would otherwise get two different tangent
    // lengths, which breaks C1 there and makes a shared second derivative
    // produce two different curvatures.
    float mag[MAX_PATH_WAYPOINTS];
    for (size_t i = 0; i < count; i++) {
        float sum = 0.0f;
        int adjacent = 0;
        if (i > 0) {
            sum += seg[i - 1];
            adjacent++;
        }
        if (i + 1 < count) {
            sum += seg[i];
            adjacent++;
        }
        mag[i] = waypoints[i].tangentMagnitude > 0.0f
                     ? waypoints[i].tangentMagnitude
                     : 1.2f * sum / static_cast<float>(adjacent);
    }

    out.clear();
    out.reserve(count - 1);
    for (size_t i = 0; i + 1 < count; i++) {
        out.emplace_back(SplineControlVector::fromPose(waypoints[i].pose, mag[i]),
                         SplineControlVector::fromPose(waypoints[i + 1].pose, mag[i + 1]));
    }
    return SplineStatus::Ok;
}

void optimizeQuinticCurvature(std::vector<QuinticHermiteSpline>& splines) {
    const size_t n = splines.size();
    if (n < 2) return;

    // Every knot value is computed from the unmodified chain before anything is
    // written back. WPILib rebuilds each interior segment from its original
    // control vectors on the next iteration, which drops the second derivative
    // it assigned at the previous knot, so its chain is only C2 at the last one.
    if (n > MAX_PATH_WAYPOINTS) return;
    Translation2d knot[MAX_PATH_WAYPOINTS]{};
    for (size_t i = 0; i + 1 < n; i++) {
        const SplineControlVector& a0 = splines[i].startControl();
        const SplineControlVector& a1 = splines[i].endControl();
        const SplineControlVector& b0 = splines[i + 1].startControl();
        const SplineControlVector& b1 = splines[i + 1].endControl();

        const Translation2d ddA = cubicSecondDerivative(a0.p, a0.d, a1.p, a1.d, 1.0f);
        const Translation2d ddB = cubicSecondDerivative(b0.p, b0.d, b1.p, b1.d, 0.0f);

        const float dAB = a0.p.distanceTo(a1.p);
        const float dBC = b0.p.distanceTo(b1.p);
        const float total = dAB + dBC;
        if (!(total > 0.0f)) {
            knot[i] = {};
            continue;
        }
        knot[i] = ddA * (dBC / total) + ddB * (dAB / total);
    }

    for (size_t i = 0; i + 1 < n; i++) {
        splines[i].setEndSecondDerivative(knot[i]);
        splines[i + 1].setStartSecondDerivative(knot[i]);
    }
}

}  // namespace chad
