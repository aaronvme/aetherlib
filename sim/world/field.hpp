// chad-sim — field geometry and the ground-truth raycaster.
//
// This is the part of the sim that has to be right for MCL/RCL work: obstacles
// are real geometry, and a distance sensor ray stops at whatever it hits first.
// A sensor aimed at a goal post reports the post, not the wall behind it, which
// is exactly the failure mode that makes particle filters diverge on a field.
//
// Hits carry a surface normal so the sensor model can weigh incidence angle --
// a real V5 distance sensor loses confidence, and eventually drops out
// entirely, on a glancing hit.
//
// Geometry is data-driven and replaceable at runtime from the browser, so a new
// season is a JSON edit rather than a rebuild.
#pragma once

#include <string>
#include <vector>

namespace sim {

struct Segment {
    float x1, y1, x2, y2;
    /// Tag carried through to the UI so the editor can colour and group
    /// elements (0 = perimeter, 1 = structure, 2 = user-added).
    int kind{0};
};

struct Circle {
    float x, y, r;
    int kind{0};
};

struct RayHit {
    bool  hit{false};
    float dist{0.0f};
    float nx{0.0f}, ny{0.0f};   // unit surface normal, facing the ray origin
    /// True when the hit was a circle -- a goal post rather than a wall. The
    /// filters treat posts as deadzones, so the UI has to be able to say which
    /// returns came from one.
    bool  isCircle{false};
};

class Field {
public:
    void clear();

    void addSegment(float x1, float y1, float x2, float y2, int kind = 2);
    void addCircle(float x, float y, float r, int kind = 2);

    /// Geometry that is replaced wholesale every tick -- the fake robots. Kept
    /// apart from the loaded field so clear(), loadDefault() and the geometry
    /// editor cannot see or destroy it, and so a robot that moves does not have
    /// to be found and erased from the middle of the static list. Rays stop on
    /// it and, when obstacles are enabled, so does the body.
    void setDynamic(std::vector<Segment> segs);
    void clearDynamic();
    const std::vector<Segment>& dynamic() const { return mDynamic; }

    /// The same, for round things that move -- the game pieces. A separate
    /// list from the segments because two publishers share this layer and a
    /// single setter would have each one erasing the other's work every tick.
    void setDynamicCircles(std::vector<Circle> circles);
    const std::vector<Circle>& dynamicCircles() const { return mDynamicCircles; }

    /// Axis-aligned box as four segments -- convenience for goals and loaders.
    void addBox(float cx, float cy, float w, float h, int kind = 1);

    /// The VRC perimeter plus the goal posts registered by
    /// gamespecific/config.cpp. See loadDefault()'s comment for what is
    /// exact and what is approximate.
    void loadDefault();

    /// Which layers a ray is allowed to stop on.
    ///
    /// The localization beams do NOT see loose pins and cups. A pin is 1.6 in
    /// across and a cup is half transparent, and neither fills enough of a V5
    /// distance sensor's cone at range to return anything -- so a sim where
    /// they do is a sim where the filter is fighting returns the robot never
    /// gets. What the beams DO see is the walls, the goals, anything the
    /// geometry editor drew, and the other robots.
    ///
    /// The claw's own sensor is the exception: it is inches from what it is
    /// looking at, and seeing a pin is its entire job.
    enum CastLayer : unsigned {
        CAST_STATIC = 1u,   // perimeter, goals, edited geometry
        CAST_ROBOTS = 2u,   // the other three robots
        CAST_PIECES = 4u,   // pins and cups
        CAST_BEAMS  = CAST_STATIC | CAST_ROBOTS,
        CAST_ALL    = CAST_STATIC | CAST_ROBOTS | CAST_PIECES,
    };

    /// Nearest obstacle along the unit direction (dc, ds) from (ox, oy).
    RayHit cast(float ox, float oy, float dc, float ds, float maxRange,
                unsigned layers = CAST_ALL) const;

    /// Distance only -- kept for callers that do not care about the normal.
    float raycast(float ox, float oy, float dc, float ds, float maxRange,
                  unsigned layers = CAST_ALL) const;

    /// True when the robot's BODY -- an oriented rectangle, half-extents
    /// (halfLen along its x, halfWid along its y) centred at (x, y) and rotated
    /// by (cosT, sinT) -- overlaps any loaded geometry.
    ///
    /// Perimeter segments (kind 0) always stop the robot. Everything else --
    /// goal walls, posts, whatever the editor drew -- is an obstacle and is
    /// switchable, because driving through them is sometimes what you want
    /// while testing a route.
    ///
    /// Exact, and against the geometry that is actually loaded. Two things this
    /// is deliberately not:
    ///
    ///   * a circle standing in for the rectangle. No single radius works at
    ///     both angles -- the circumscribed one (half a body diagonal) holds
    ///     the robot a corner short of a wall it is square to, and the
    ///     inscribed one lets a corner through by up to the difference, 3.1 in
    ///     on a 15 in square measured against a post at 45 degrees.
    ///   * a hardcoded +-70.2 box for the walls. That is where the VRC
    ///     perimeter is, but asserting it here meant an edited field was half
    ///     applied: clearing the walls left the raycaster seeing an empty field
    ///     while the robot was still boxed in, and re-adding one anywhere put
    ///     the box back wherever the segment happened to be.
    bool collidesBody(float x, float y, float cosT, float sinT,
                      float halfLen, float halfWid,
                      bool includeObstacles = true) const;

    struct Nearest {
        float dist{0.0f};       // to the point, not to any footprint
        float x{0.0f}, y{0.0f}; // the closest point on the closest obstacle
    };

    /// Nearest wall or post to a point, with the contact point itself so a
    /// caller can allow for a footprint in that direction.
    ///
    /// Measures to a point, not to the body, so a caller has to allow for its
    /// own footprint in the returned direction -- which is what the route
    /// report's clearance column does. Counts everything loaded, so it and
    /// collidesBody() now agree about what is there; they used to disagree on
    /// purpose, because the physics only stopped on a hardcoded perimeter and
    /// the posts and drove through goal walls this reported.
    Nearest nearest(float x, float y) const;

    const std::vector<Segment>& segments() const { return mSegments; }
    const std::vector<Circle>&  circles()  const { return mCircles; }

private:
    static RayHit raySegment(float ox, float oy, float dc, float ds,
                             const Segment& s, float maxRange);
    static RayHit rayCircle(float ox, float oy, float dc, float ds,
                            const Circle& c, float maxRange);

    std::vector<Segment> mSegments;
    std::vector<Circle>  mCircles;
    std::vector<Segment> mDynamic;
    std::vector<Circle>  mDynamicCircles;
};

Field& field();

}  // namespace sim
