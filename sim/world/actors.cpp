#include "actors.hpp"
#include "field.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

Actors& actors() {
    static Actors a;
    return a;
}

int Actors::add(float x, float y, float thetaRad, int team) {
    Actor a;
    a.id = mNextId++;
    a.x = x;
    a.y = y;
    a.theta = thetaRad;
    a.team = team;
    mActors.push_back(a);
    return a.id;
}

bool Actors::remove(int id) {
    const auto it = std::find_if(mActors.begin(), mActors.end(),
                                 [id](const Actor& a) { return a.id == id; });
    if (it == mActors.end()) return false;
    mActors.erase(it);
    return true;
}

void Actors::clear() {
    mActors.clear();
}

Actor* Actors::get(int id) {
    for (auto& a : mActors) if (a.id == id) return &a;
    return nullptr;
}

const Actor* Actors::get(int id) const {
    for (const auto& a : mActors) if (a.id == id) return &a;
    return nullptr;
}

void Actors::step(double dtMs) {
    const float dt = static_cast<float>(dtMs / 1000.0);
    if (dt <= 0.0f) return;

    for (auto& a : mActors) {
        if (!a.enabled || !a.patrol) continue;
        const int n = static_cast<int>(a.path.size() / 2);
        if (n < 2) continue;

        // A closed circuit numbers its legs 0..n-1 and always runs forwards.
        // An open one numbers them 0..n-2 and flips `dir` at each end, so the
        // pair (leg, dir) says which point it is coming from and going to.
        const int lastLeg = a.loop ? n - 1 : n - 2;
        if (a.loop) a.dir = 1;
        if (a.dir >= 0) a.dir = 1; else a.dir = -1;
        if (a.leg < 0) a.leg = 0;
        if (a.leg > lastLeg) a.leg = lastLeg;

        auto endpoints = [&](int& from, int& to) {
            if (a.loop) { from = a.leg; to = (a.leg + 1) % n; }
            else if (a.dir > 0) { from = a.leg; to = a.leg + 1; }
            else { from = a.leg + 1; to = a.leg; }
        };
        auto advance = [&]() {
            a.legT = 0.0f;
            if (a.loop) { a.leg = (a.leg + 1) % n; return; }
            if (a.dir > 0) {
                if (a.leg < lastLeg) a.leg++; else a.dir = -1;
            } else {
                if (a.leg > 0) a.leg--; else a.dir = 1;
            }
        };

        float budget = a.speedIn * dt;
        // A loop of short legs can be crossed several times in one tick at a
        // high sim speed, so this consumes the budget rather than stepping once.
        int guard = 0;
        while (budget > 1e-6f && guard++ < 64) {
            int from = 0, to = 0;
            endpoints(from, to);
            const float ax = a.path[2 * from], ay = a.path[2 * from + 1];
            const float bx = a.path[2 * to],   by = a.path[2 * to + 1];
            const float len = std::hypot(bx - ax, by - ay);
            if (len < 1e-4f) { advance(); continue; }

            const float remaining = (1.0f - a.legT) * len;
            if (budget < remaining) {
                a.legT += budget / len;
                budget = 0.0f;
            } else {
                budget -= remaining;
                advance();
            }
        }

        int from = 0, to = 0;
        endpoints(from, to);
        const float fx = a.path[2 * from], fy = a.path[2 * from + 1];
        const float tx = a.path[2 * to],   ty = a.path[2 * to + 1];
        a.x = fx + (tx - fx) * a.legT;
        a.y = fy + (ty - fy) * a.legT;
        if (a.faceTravel && std::hypot(tx - fx, ty - fy) > 1e-4f) {
            a.theta = std::atan2(ty - fy, tx - fx);
        }
    }
}

void Actors::publish() const {
    std::vector<Segment> segs;
    segs.reserve(mActors.size() * 4);

    for (const auto& a : mActors) {
        if (!a.enabled) continue;
        const float c = std::cos(a.theta), s = std::sin(a.theta);
        const float hl = 0.5f * a.lengthIn, hw = 0.5f * a.widthIn;

        // Corners in the actor's frame, rotated out to the field.
        const float lx[4] = { hl,  hl, -hl, -hl };
        const float ly[4] = { hw, -hw, -hw,  hw };
        float px[4], py[4];
        for (int i = 0; i < 4; i++) {
            px[i] = a.x + lx[i] * c - ly[i] * s;
            py[i] = a.y + lx[i] * s + ly[i] * c;
        }
        // kind 4 so the UI can colour them apart from everything else.
        for (int i = 0; i < 4; i++) {
            const int j = (i + 1) & 3;
            segs.push_back({px[i], py[i], px[j], py[j], 4});
        }
    }

    field().setDynamic(std::move(segs));
}

void Actors::rewind() {
    for (auto& a : mActors) {
        a.leg = 0;
        a.legT = 0.0f;
        a.dir = 1;
        if (a.path.size() >= 2) {
            a.x = a.path[0];
            a.y = a.path[1];
            if (a.faceTravel && a.path.size() >= 4) {
                a.theta = std::atan2(a.path[3] - a.path[1], a.path[2] - a.path[0]);
            }
        }
    }
    publish();
}

}  // namespace sim
