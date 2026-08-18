// riku-sim — field geometry and the ground-truth raycaster.
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

    /// Axis-aligned box as four segments -- convenience for goals and loaders.
    void addBox(float cx, float cy, float w, float h, int kind = 1);

    /// The VRC perimeter plus the goal posts registered by
    /// gamespecific/config.cpp. See loadDefault()'s comment for what is
    /// exact and what is approximate.
    void loadDefault();

    /// Nearest obstacle along the unit direction (dc, ds) from (ox, oy).
    RayHit cast(float ox, float oy, float dc, float ds, float maxRange) const;

    /// Distance only -- kept for callers that do not care about the normal.
    float raycast(float ox, float oy, float dc, float ds, float maxRange) const;

    /// True when the point is outside the perimeter, or inside any circle when
    /// `includeCircles`. Driving through posts is sometimes what you want while
    /// testing a route, so it is switchable.
    bool collides(float x, float y, float radius, bool includeCircles = true) const;

    const std::vector<Segment>& segments() const { return mSegments; }
    const std::vector<Circle>&  circles()  const { return mCircles; }

private:
    static RayHit raySegment(float ox, float oy, float dc, float ds,
                             const Segment& s, float maxRange);
    static RayHit rayCircle(float ox, float oy, float dc, float ds,
                            const Circle& c, float maxRange);

    std::vector<Segment> mSegments;
    std::vector<Circle>  mCircles;
};

Field& field();

}  // namespace sim
