#include "toggles.hpp"

#include <cmath>

namespace sim {
namespace {

/// Outward normal of each wall, in the order Toggle::wall numbers them.
constexpr float kNormal[4][2] = {{0.0f, 1.0f}, {1.0f, 0.0f},
                                 {0.0f, -1.0f}, {-1.0f, 0.0f}};

/// Which way round the colours go, per wall. North and east are blue's
/// quadrants and south and west are red's; the two halves are the same roller
/// mounted a half turn apart, so a ram walks them in opposite directions.
constexpr int kCycleDir[4] = {+1, +1, -1, -1};

}  // namespace

Toggles& toggles() {
    static Toggles t;
    return t;
}

void Toggles::clear() {
    mToggles.clear();
}

void Toggles::loadDefault() {
    mToggles.clear();
    for (int w = 0; w < 4; w++) {
        Toggle t;
        t.id = w;
        t.wall = w;
        t.x = kNormal[w][0] * kToggleWallIn;
        t.y = kNormal[w][1] * kToggleWallIn;
        mToggles.push_back(t);
    }
}

void Toggles::resetStates() {
    for (auto& t : mToggles) {
        t.state = ToggleState::Neutral;
        t.contact = false;
    }
}

Toggle* Toggles::get(int id) {
    for (auto& t : mToggles) if (t.id == id) return &t;
    return nullptr;
}

void Toggles::setState(int id, ToggleState s) {
    if (Toggle* t = get(id)) t->state = s;
}

void Toggles::cycle(int id) {
    Toggle* t = get(id);
    if (t == nullptr) return;
    const int dir = kCycleDir[t->wall & 3];
    t->state = static_cast<ToggleState>(
        ((static_cast<int>(t->state) + dir) % 3 + 3) % 3);
}

int Toggles::step(const chad::Pose& robot, float halfLenIn, float halfWidIn,
                  float driveIn) {
    (void)halfWidIn;
    if (mToggles.empty()) return -1;

    const float ct = std::cos(robot.theta), st = std::sin(robot.theta);
    // Middle of the front face. The corners reach the wall first on an angled
    // approach, but a flipper is built on the centreline, so that is what has
    // to arrive.
    const float fx = robot.x + halfLenIn * ct;
    const float fy = robot.y + halfLenIn * st;

    int flipped = -1;
    for (auto& t : mToggles) {
        const float nx = kNormal[t.wall][0], ny = kNormal[t.wall][1];

        // How far the front face still is from the wall, and how far along the
        // wall it sits. Along-the-wall is the normal turned ninety degrees.
        const float gap = kToggleWallIn - (fx * nx + fy * ny);
        const float along = fx * -ny + fy * nx;
        const bool onIt = std::fabs(along) <= 0.5f * t.lengthIn;

        // Facing the wall, not backing onto it: the claw is on the rear and
        // ramming a toggle with it is not what this is for.
        const float facing = ct * nx + st * ny;

        if (!t.contact) {
            if (onIt && gap <= reachIn && facing > 0.5f && driveIn > minPushIn) {
                t.contact = true;
                cycle(t.id);
                flipped = t.id;
            }
        } else if (!onIt || gap > releaseIn) {
            // Off it again, so the next ram counts as a new one.
            t.contact = false;
        }
    }
    return flipped;
}

}  // namespace sim
