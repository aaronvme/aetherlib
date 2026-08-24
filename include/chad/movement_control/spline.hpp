// Design derived from WPILib's frc::Spline / SplineHelper / SplineParameterizer
// (BSD-3-Clause). See ATTRIBUTION.md.
#pragma once

#include <cstddef>
#include <vector>

#include "chad/util/geometry.hpp"

namespace chad {

/// A point on a path: where the robot is, and how hard the path bends there.
/// Position in inches, heading in standard math radians, curvature in 1/inches
/// and signed counter-clockwise-positive.
struct PoseWithCurvature {
    Pose pose{};
    float curvature{0.0f};
};

/// One end of a Hermite segment: position and derivatives with respect to the
/// segment parameter t, which runs 0..1 over that segment.
struct SplineControlVector {
    Translation2d p{};
    Translation2d d{};
    Translation2d dd{};  ///< ignored by the cubic

    static SplineControlVector fromPose(const Pose& pose, float tangentMagnitude) {
        const Rotation2d r = pose.rotation();
        return {pose.translation(), {tangentMagnitude * r.cos, tangentMagnitude * r.sin}, {}};
    }
};

/// A polynomial segment in x and y.
class Spline {
public:
    static constexpr int MAX_DEGREE = 5;

    int degree() const { return mDegree; }

    Translation2d positionAt(float t) const;
    Translation2d derivativeAt(float t) const;
    Translation2d secondDerivativeAt(float t) const;

    /// Pose (heading = tangent direction) and curvature at t. Returns false
    /// where the tangent vanishes: heading and curvature are genuinely
    /// undefined there, and that is what a malformed control vector produces.
    bool pointAt(float t, PoseWithCurvature& out) const;

protected:
    /// Non-virtual, so the destructor is protected: this hierarchy exists to
    /// share evaluation, not to be owned polymorphically.
    Spline() = default;
    ~Spline() = default;

    /// `c` holds degree+1 coefficients, index 0 multiplying t^degree.
    void setCoefficients(const Translation2d* c, int degree);

    Translation2d mC[MAX_DEGREE + 1]{};
    Translation2d mDC[MAX_DEGREE]{};
    Translation2d mDDC[MAX_DEGREE - 1]{};
    int mDegree{1};
};

class CubicHermiteSpline : public Spline {
public:
    CubicHermiteSpline() { rebuild(); }
    CubicHermiteSpline(const SplineControlVector& start, const SplineControlVector& end)
        : mStart(start), mEnd(end) {
        rebuild();
    }

    const SplineControlVector& startControl() const { return mStart; }
    const SplineControlVector& endControl() const { return mEnd; }

private:
    void rebuild();

    SplineControlVector mStart{};
    SplineControlVector mEnd{};
};

class QuinticHermiteSpline : public Spline {
public:
    QuinticHermiteSpline() { rebuild(); }
    QuinticHermiteSpline(const SplineControlVector& start, const SplineControlVector& end)
        : mStart(start), mEnd(end) {
        rebuild();
    }

    const SplineControlVector& startControl() const { return mStart; }
    const SplineControlVector& endControl() const { return mEnd; }

    void setStartSecondDerivative(const Translation2d& dd) {
        mStart.dd = dd;
        rebuild();
    }
    void setEndSecondDerivative(const Translation2d& dd) {
        mEnd.dd = dd;
        rebuild();
    }

private:
    void rebuild();

    SplineControlVector mStart{};
    SplineControlVector mEnd{};
};

enum class SplineStatus {
    Ok,
    /// The tangent vanished somewhere on the segment, so no heading exists.
    TangentVanished,
    /// Subdivision hit the depth cap. Two waypoints very close together with
    /// opposing headings do this: the arc never comes inside tolerance.
    DepthExceeded,
    /// Subdivision produced more points than PARAMETERIZE_MAX_POINTS. A legal
    /// but wildly overscaled tangent does this, and the depth cap alone does
    /// not bound it: 2^depth points would exhaust the heap long before.
    TooManyPoints,
    NoSplines,
    NotEnoughWaypoints,
    TooManyWaypoints,
    /// Consecutive waypoints share a position but not a heading. An arc-length
    /// parameterization cannot express a turn in place.
    DegenerateWaypoints,
};

/// Tolerances on one subdivided arc, measured in the frame of its start pose.
struct ParameterizeTolerance {
    float maxDx{5.0f};          ///< inches along the arc
    float maxDy{0.05f};         ///< inches across it
    float maxDtheta{0.0872f};   ///< radians
    /// Halving [0, 1] much past 20 times is at the edge of float resolution in
    /// t, so a deeper request means the spline is malformed, not that the path
    /// needs finer sampling.
    int maxDepth{20};
};

/// Hard ceiling on `ParameterizeTolerance::maxDepth`; sizes the explicit stack.
constexpr int PARAMETERIZE_MAX_DEPTH = 24;

/// Hard ceiling on points emitted per parameterize call. At ~24 bytes a point
/// this bounds one path's samples to roughly 50 KB of heap.
constexpr size_t PARAMETERIZE_MAX_POINTS = 2048;

/// A path waypoint. `tangentMagnitude` (inches) scales the Hermite tangent at
/// this pose. <= 0 means "pick 1.2x the distance to the neighbouring waypoint",
/// which is only a shape heuristic, not a physical quantity.
struct PathWaypoint {
    Pose pose{};
    float tangentMagnitude{0.0f};
};

constexpr size_t MAX_PATH_WAYPOINTS = 16;

/// Two waypoints closer than this are the same waypoint as far as a path is
/// concerned; the tangent heuristics divide by their separation. Inches.
constexpr float WAYPOINT_COINCIDENT_EPS = 1e-3f;

/// Recursive subdivision of one segment into arcs within `tol`. Appends the
/// point at t0 (unless `includeStart` is false) and then one point per
/// accepted arc.
SplineStatus parameterize(const Spline& spline, std::vector<PoseWithCurvature>& out,
                          const ParameterizeTolerance& tol = {}, float t0 = 0.0f,
                          float t1 = 1.0f, bool includeStart = true);

/// Same, over a chain. Every segment after the first skips its start point,
/// which duplicates the previous segment's last point.
template <typename SplineT>
SplineStatus parameterizeChain(const std::vector<SplineT>& splines,
                               std::vector<PoseWithCurvature>& out,
                               const ParameterizeTolerance& tol = {}) {
    if (splines.empty()) return SplineStatus::NoSplines;
    for (size_t i = 0; i < splines.size(); i++) {
        const SplineStatus s = parameterize(splines[i], out, tol, 0.0f, 1.0f, i == 0);
        if (s != SplineStatus::Ok) return s;
    }
    return SplineStatus::Ok;
}

/// Clamped cubic chain: the end tangent directions come from `start`/`end`, and
/// the interior tangents from a tridiagonal solve, which makes the chain C2 in
/// the spline parameter rather than only C1.
SplineStatus cubicSplinesFromWaypoints(const Pose& start, const Translation2d* interior,
                                       size_t interiorCount, const Pose& end,
                                       std::vector<CubicHermiteSpline>& out,
                                       float startTangent = 0.0f, float endTangent = 0.0f);

/// Quintic chain through every pose. Second derivatives start at zero, so the
/// raw chain is only C1 at the knots; `optimizeQuinticCurvature` fixes that.
SplineStatus quinticSplinesFromWaypoints(const PathWaypoint* waypoints, size_t count,
                                         std::vector<QuinticHermiteSpline>& out);

/// Give both segments at each knot the same second derivative — a
/// segment-length-weighted average of what a cubic through the same positions
/// and tangents would have had (Sprunk 2008, 4.1.2). Makes the chain C2.
void optimizeQuinticCurvature(std::vector<QuinticHermiteSpline>& splines);

}  // namespace chad
