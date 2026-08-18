#include "field.hpp"

#include <cmath>

namespace sim {
namespace {

// Interior wall-to-wall, matching riku::Field::FIELD_HALF_WALL.
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

    // The posts are taken verbatim from kFieldPosts in
    // gamespecific/config.cpp -- your numbers, not guesses.
    addCircle(-47.0f, -23.5f, 2.0f, 1);
    addCircle(-23.5f, -47.0f, 2.0f, 1);
    addCircle( 23.5f,  47.0f, 2.0f, 1);
    addCircle( 47.0f,  23.5f, 2.0f, 1);
    addCircle(  0.0f,   0.0f, 2.0f, 1);
    addCircle(-47.0f,  23.5f, 2.0f, 1);
    addCircle( 23.5f, -47.0f, 2.0f, 1);
    addCircle(-23.5f,  47.0f, 2.0f, 1);
    addCircle( 47.0f, -23.5f, 2.0f, 1);
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

RayHit Field::cast(float ox, float oy, float dc, float ds, float maxRange) const {
    RayHit best;
    best.dist = maxRange;

    for (const auto& s : mSegments) {
        const RayHit h = raySegment(ox, oy, dc, ds, s, best.dist);
        if (h.hit && h.dist < best.dist) best = h;
    }
    for (const auto& c : mCircles) {
        RayHit h = rayCircle(ox, oy, dc, ds, c, best.dist);
        if (h.hit && h.dist < best.dist) { h.isCircle = true; best = h; }
    }
    return best;
}

float Field::raycast(float ox, float oy, float dc, float ds, float maxRange) const {
    const RayHit h = cast(ox, oy, dc, ds, maxRange);
    return h.hit ? h.dist : maxRange;
}

bool Field::collides(float x, float y, float radius, bool includeCircles) const {
    if (std::fabs(x) > kHalf - radius || std::fabs(y) > kHalf - radius) return true;
    if (!includeCircles) return false;
    for (const auto& c : mCircles) {
        const float dx = x - c.x;
        const float dy = y - c.y;
        const float rr = c.r + radius;
        if (dx * dx + dy * dy < rr * rr) return true;
    }
    return false;
}

}  // namespace sim
