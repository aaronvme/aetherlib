#include "field.hpp"

#include <cmath>
#include <utility>

namespace sim {
namespace {

// Interior wall-to-wall, matching chad::Field::FIELD_HALF_WALL.
constexpr float kHalf = 70.2f;

}  // namespace

Field& field() {
    static Field f;
    return f;
}

void Field::clear() {
    mSegments.clear();
    mCircles.clear();
}

void Field::addSegment(float x1, float y1, float x2, float y2, int kind) {
    mSegments.push_back({x1, y1, x2, y2, kind});
}

void Field::addCircle(float x, float y, float r, int kind) {
    mCircles.push_back({x, y, r, kind});
}

void Field::setDynamic(std::vector<Segment> segs) {
    mDynamic = std::move(segs);
}

void Field::clearDynamic() {
    mDynamic.clear();
    mDynamicCircles.clear();
}

void Field::setDynamicCircles(std::vector<Circle> circles) {
    mDynamicCircles = std::move(circles);
}

void Field::addBox(float cx, float cy, float w, float h, int kind) {
    const float hw = w * 0.5f, hh = h * 0.5f;
    addSegment(cx - hw, cy - hh, cx + hw, cy - hh, kind);
    addSegment(cx + hw, cy - hh, cx + hw, cy + hh, kind);
    addSegment(cx + hw, cy + hh, cx - hw, cy + hh, kind);
    addSegment(cx - hw, cy + hh, cx - hw, cy - hh, kind);
}

void Field::loadDefault() {
    clear();

    // --- exact ------------------------------------------------------------
    // The perimeter is fixed by the VRC field spec and matches kFieldWalls in
    // MclTracking, so a correct filter agrees with ground truth here.
    addSegment(-kHalf, -kHalf,  kHalf, -kHalf, 0);
    addSegment( kHalf, -kHalf,  kHalf,  kHalf, 0);
    addSegment( kHalf,  kHalf, -kHalf,  kHalf, 0);
    addSegment(-kHalf,  kHalf, -kHalf, -kHalf, 0);

    // The nine Goals, in the order and at the positions kFieldPosts in
    // gamespecific/config.cpp registers them -- your numbers, not guesses.
    // The radius is 2.85, half the 5.7 in across-flats measured off Figure
    // FO-2 in the Override manual with the Alliance Goals as scale.
    //
    // Kind says which Goal it is, 10 + GoalKind, so the browser can colour
    // them without a second table: red, blue, neutral short, neutral tall.
    // Nothing in the physics reads it -- every circle stops a beam and a body.
    constexpr float kGoalR = 2.85f;
    addCircle(-47.0f, -23.5f, kGoalR, 10);
    addCircle(-23.5f, -47.0f, kGoalR, 10);
    addCircle( 23.5f,  47.0f, kGoalR, 11);
    addCircle( 47.0f,  23.5f, kGoalR, 11);
    addCircle(  0.0f,   0.0f, kGoalR, 13);
    addCircle(-47.0f,  23.5f, kGoalR, 12);
    addCircle( 23.5f, -47.0f, kGoalR, 12);
    addCircle(-23.5f,  47.0f, kGoalR, 12);
    addCircle( 47.0f, -23.5f, kGoalR, 12);
}

// ============================================================================
// Raycasting
// ============================================================================

RayHit Field::raySegment(float ox, float oy, float dc, float ds,
                         const Segment& s, float maxRange) {
    RayHit out;

    const float ex = s.x2 - s.x1;
    const float ey = s.y2 - s.y1;

    const float den = dc * ey - ds * ex;
    if (std::fabs(den) < 1e-9f) return out;     // parallel

    const float ox1 = s.x1 - ox;
    const float oy1 = s.y1 - oy;

    const float t = (ox1 * ey - oy1 * ex) / den;   // along the ray
    const float u = (ox1 * ds - oy1 * dc) / den;   // along the segment

    if (t < 0.0f || t > maxRange) return out;
    if (u < 0.0f || u > 1.0f) return out;

    const float len = std::sqrt(ex * ex + ey * ey);
    if (len < 1e-9f) return out;

    // Normal is perpendicular to the edge, flipped to face the incoming ray.
    float nx = -ey / len;
    float ny =  ex / len;
    if (nx * dc + ny * ds > 0.0f) { nx = -nx; ny = -ny; }

    out.hit = true;
    out.dist = t;
    out.nx = nx;
    out.ny = ny;
    return out;
}

RayHit Field::rayCircle(float ox, float oy, float dc, float ds,
                        const Circle& c, float maxRange) {
    RayHit out;

    const float fx = ox - c.x;
    const float fy = oy - c.y;

    // |origin + t*dir - center|^2 = r^2, with dir a unit vector so a == 1.
    const float b = 2.0f * (fx * dc + fy * ds);
    const float cc = fx * fx + fy * fy - c.r * c.r;

    float disc = b * b - 4.0f * cc;
    if (disc < 0.0f) return out;
    disc = std::sqrt(disc);

    const float t1 = (-b - disc) * 0.5f;
    const float t2 = (-b + disc) * 0.5f;

    float t = -1.0f;
    if (t1 >= 0.0f && t1 <= maxRange) t = t1;
    else if (t2 >= 0.0f && t2 <= maxRange) t = t2;
    if (t < 0.0f) return out;

    const float hx = ox + dc * t;
    const float hy = oy + ds * t;
    float nx = hx - c.x;
    float ny = hy - c.y;
    const float len = std::sqrt(nx * nx + ny * ny);
    if (len < 1e-9f) return out;
    nx /= len; ny /= len;
    if (nx * dc + ny * ds > 0.0f) { nx = -nx; ny = -ny; }

    out.hit = true;
    out.dist = t;
    out.nx = nx;
    out.ny = ny;
    return out;
}

RayHit Field::cast(float ox, float oy, float dc, float ds, float maxRange,
                   unsigned layers) const {
    RayHit best;
    best.dist = maxRange;

    if (layers & CAST_STATIC) {
        for (const auto& s : mSegments) {
            const RayHit h = raySegment(ox, oy, dc, ds, s, best.dist);
            if (h.hit && h.dist < best.dist) best = h;
        }
        for (const auto& c : mCircles) {
            RayHit h = rayCircle(ox, oy, dc, ds, c, best.dist);
            if (h.hit && h.dist < best.dist) { h.isCircle = true; best = h; }
        }
    }
    // A robot parked in front of a sensor is the whole reason this exists: the
    // beam has to stop on it and report its range, exactly as it would stop on
    // a wall. Nothing downstream is told the difference, which is the point.
    if (layers & CAST_ROBOTS) {
        for (const auto& s : mDynamic) {
            const RayHit h = raySegment(ox, oy, dc, ds, s, best.dist);
            if (h.hit && h.dist < best.dist) best = h;
        }
    }
    if (layers & CAST_PIECES) {
        for (const auto& c : mDynamicCircles) {
            RayHit h = rayCircle(ox, oy, dc, ds, c, best.dist);
            if (h.hit && h.dist < best.dist) { h.isCircle = true; best = h; }
        }
    }
    return best;
}

float Field::raycast(float ox, float oy, float dc, float ds, float maxRange,
                     unsigned layers) const {
    const RayHit h = cast(ox, oy, dc, ds, maxRange, layers);
    return h.hit ? h.dist : maxRange;
}

namespace {

/// Does segment (x0,y0)-(x1,y1) overlap the box [-hx,hx] x [-hy,hy]?
///
/// Liang-Barsky: clip the segment's parameter range against each of the four
/// slabs and see whether anything survives. The box is treated as OPEN, so a
/// segment lying exactly along a face is NOT an overlap -- which is what lets
/// the robot come to rest in contact with a wall instead of being stopped a
/// float's width short of it every tick.
bool segmentHitsBox(float x0, float y0, float x1, float y1, float hx, float hy) {
    const float dx = x1 - x0, dy = y1 - y0;
    const float p[4] = {-dx, dx, -dy, dy};
    const float q[4] = {x0 + hx, hx - x0, y0 + hy, hy - y0};

    float t0 = 0.0f, t1 = 1.0f;
    for (int i = 0; i < 4; i++) {
        if (std::fabs(p[i]) < 1e-9f) {
            // Parallel to this slab: it is in or out for the whole segment.
            if (q[i] <= 0.0f) return false;
            continue;
        }
        const float r = q[i] / p[i];
        if (p[i] < 0.0f) {
            if (r > t1) return false;
            if (r > t0) t0 = r;
        } else {
            if (r < t0) return false;
            if (r < t1) t1 = r;
        }
    }
    return t0 < t1;
}

}  // namespace

bool Field::collidesBody(float x, float y, float cosT, float sinT,
                         float halfLen, float halfWid, bool includeObstacles) const {
    // Everything is tested in the robot's frame, where the body is the
    // axis-aligned box [-halfLen, halfLen] x [-halfWid, halfWid].
    auto toBody = [&](float wx, float wy) {
        const float dx = wx - x, dy = wy - y;
        return std::pair<float, float>{ dx * cosT + dy * sinT,
                                       -dx * sinT + dy * cosT };
    };

    for (const auto& s : mSegments) {
        // The perimeter always stops the robot; goal walls and anything drawn
        // in the editor are obstacles, switchable with the posts.
        if (s.kind != 0 && !includeObstacles) continue;
        const auto [ax, ay] = toBody(s.x1, s.y1);
        const auto [bx, by] = toBody(s.x2, s.y2);
        if (segmentHitsBox(ax, ay, bx, by, halfLen, halfWid)) return true;
    }

    if (!includeObstacles) return false;

    for (const auto& s : mDynamic) {
        const auto [ax, ay] = toBody(s.x1, s.y1);
        const auto [bx, by] = toBody(s.x2, s.y2);
        if (segmentHitsBox(ax, ay, bx, by, halfLen, halfWid)) return true;
    }

    // mDynamicCircles -- the game pieces -- deliberately do NOT stop the body.
    // A pin weighs nothing next to a robot; driving over one is something the
    // routines do on purpose, and a filter that has to survive a beam stopping
    // on a pin is a different question from the robot being stopped by it.
    for (const auto& c : mCircles) {
        // Post centre in the robot frame, then the nearest point on the body to
        // it. Clamping inside the rectangle gives a zero separation, so a post
        // swallowed by the body counts too.
        const auto [lx, ly] = toBody(c.x, c.y);
        const float qx = (lx < -halfLen) ? -halfLen : (lx > halfLen ? halfLen : lx);
        const float qy = (ly < -halfWid) ? -halfWid : (ly > halfWid ? halfWid : ly);
        const float px = lx - qx, py = ly - qy;
        if (px * px + py * py < c.r * c.r) return true;
    }
    return false;
}

Field::Nearest Field::nearest(float x, float y) const {
    Nearest best;
    best.dist = 1e9f;

    auto trySeg = [&](const Segment& s) {
        const float ex = s.x2 - s.x1, ey = s.y2 - s.y1;
        const float len2 = ex * ex + ey * ey;
        float t = (len2 > 1e-9f) ? ((x - s.x1) * ex + (y - s.y1) * ey) / len2 : 0.0f;
        t = (t < 0.0f) ? 0.0f : (t > 1.0f ? 1.0f : t);
        const float cx = s.x1 + t * ex, cy = s.y1 + t * ey;
        const float d = std::sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
        if (d < best.dist) best = {d, cx, cy};
    };
    for (const auto& s : mDynamic) trySeg(s);

    for (const auto& s : mSegments) {
        const float ex = s.x2 - s.x1, ey = s.y2 - s.y1;
        const float len2 = ex * ex + ey * ey;
        // Clamp the projection so an endpoint answers for anything past it,
        // otherwise a point beyond a short goal wall measures to the infinite
        // line and reads closer than it is.
        float t = (len2 > 1e-9f) ? ((x - s.x1) * ex + (y - s.y1) * ey) / len2 : 0.0f;
        t = (t < 0.0f) ? 0.0f : (t > 1.0f ? 1.0f : t);
        const float cx = s.x1 + t * ex, cy = s.y1 + t * ey;
        const float d = std::sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
        if (d < best.dist) best = {d, cx, cy};
    }

    for (const auto& c : mCircles) {
        const float dx = x - c.x, dy = y - c.y;
        const float len = std::sqrt(dx * dx + dy * dy);
        const float d = len - c.r;
        if (d < best.dist) {
            // Project onto the post's surface, so the direction is the same one
            // a footprint has to make room in.
            const float ux = (len > 1e-6f) ? dx / len : 1.0f;
            const float uy = (len > 1e-6f) ? dy / len : 0.0f;
            best = {d, c.x + ux * c.r, c.y + uy * c.r};
        }
    }
    return best;
}

}  // namespace sim
