#include "pieces.hpp"
#include "actors.hpp"
#include "field.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "gamespecific/config.hpp"

namespace sim {
namespace {

/// How far in front of the jaws a carried piece rides.
constexpr float kCarryOffsetIn = 2.0f;

/// The tile-corner grid the Goals sit on, kept identical to kFieldPosts so a
/// piece placed "on the diagonal" lands on the same lattice the Goals do.
constexpr float kInner = 23.5f;
constexpr float kOuter = 47.0f;

/// A Cup touching a wall. The sim's perimeter is at 70.2.
constexpr float kWallEdge = 70.2f;
constexpr float kWallCup = kWallEdge - kCupRadiusIn;
/// Centre-to-centre inside a wall group of three, measured off Figure FO-2.
constexpr float kWallSpan = 8.0f;

/// How far out from its Cup a lying Pin's centre sits, in one of the four
/// assemblies on the Autonomous Line.
constexpr float kArmReach = 4.0f;

/// Slack on top of a Cup's own radius for a Pin to count as nested in it.
constexpr float kNestSlack = 1.5f;

constexpr GoalKind kGoalKinds[] = {
    GoalKind::RedAlliance,  GoalKind::RedAlliance,
    GoalKind::BlueAlliance, GoalKind::BlueAlliance,
    GoalKind::NeutralTall,
    GoalKind::NeutralShort, GoalKind::NeutralShort,
    GoalKind::NeutralShort, GoalKind::NeutralShort,
};

float clampIn(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

}  // namespace

GoalKind goalKind(int index) {
    const int n = static_cast<int>(sizeof(kGoalKinds) / sizeof(kGoalKinds[0]));
    if (index < 0 || index >= n) return GoalKind::NeutralShort;
    return kGoalKinds[index];
}

Pieces& pieces() {
    static Pieces p;
    return p;
}

void Pieces::clear() {
    mPieces.clear();
    mHeld = -1;
    mDirty = true;
    // Numbering restarts with the field. These ids are shown in the HUD and on
    // the piece list, and a reload that walked them to #70 made the labels
    // meaningless -- they are names, not a running total.
    mNextId = 1;
}

int Pieces::add(PieceKind kind, float x, float y, float thetaRad) {
    Piece p;
    p.id = mNextId++;
    p.kind = kind;
    p.x = x;
    p.y = y;
    p.theta = thetaRad;
    p.radiusIn = (kind == PieceKind::Cup) ? kCupRadiusIn : kPinRadiusIn;
    mPieces.push_back(p);
    mDirty = true;
    return p.id;
}

bool Pieces::remove(int id) {
    const auto it = std::find_if(mPieces.begin(), mPieces.end(),
                                 [id](const Piece& p) { return p.id == id; });
    if (it == mPieces.end()) return false;
    if (mHeld == id) mHeld = -1;
    mPieces.erase(it);
    // Anything riding it is on the floor now, not riding a hole.
    for (auto& p : mPieces) if (p.inCup == id) p.inCup = -1;
    mDirty = true;
    return true;
}

Piece* Pieces::get(int id) {
    for (auto& p : mPieces) if (p.id == id) return &p;
    return nullptr;
}

const Piece* Pieces::get(int id) const {
    for (const auto& p : mPieces) if (p.id == id) return &p;
    return nullptr;
}

// ============================================================================
// Figure FO-2
// ============================================================================

void Pieces::loadDefault() {
    clear();

    auto addPin = [&](float x, float y, PinColor c, bool upright, float th) {
        const int id = add(PieceKind::Pin, x, y, th);
        Piece* p = get(id);
        p->color = c;
        p->upright = upright;
        return id;
    };
    auto addCup = [&](float x, float y, bool clearUp) {
        const int id = add(PieceKind::Cup, x, y, 0.0f);
        get(id)->clearUp = clearUp;
        return id;
    };
    auto nest = [&](int pinId, int cupId) { get(pinId)->inCup = cupId; };

    // 1. A yellow Pin standing in each of the five neutral Goals. The four
    //    Alliance Goals start empty.
    const auto& goals = chad::fieldCircleObstacles();
    for (std::size_t i = 0; i < goals.size(); i++) {
        const GoalKind k = goalKind(static_cast<int>(i));
        if (k != GoalKind::NeutralShort && k != GoalKind::NeutralTall) continue;
        const int id = addPin(goals[i].x, goals[i].y,
                              PinColor::YellowYellow, true, 0.0f);
        Piece* p = get(id);
        p->goal = static_cast<int>(i);
        p->level = 0;
    }

    // 2. The main diagonal: a Cup clear-side-up with a yellow Pin in it.
    const float mainDiag[4][2] = {{-kOuter, -kOuter}, {-kInner, -kInner},
                                  { kInner,  kInner}, { kOuter,  kOuter}};
    for (const auto& d : mainDiag) {
        const int cup = addCup(d[0], d[1], true);
        nest(addPin(d[0], d[1], PinColor::YellowYellow, true, 0.0f), cup);
    }

    // 3. The Autonomous Line: a Cup with four Pins lying around it, pointing
    //    out. These twenty objects are the ones <SG7b> says belong to neither
    //    Alliance during the Autonomous Period.
    const float autonDiag[4][2] = {{-kOuter,  kOuter}, {-kInner,  kInner},
                                   { kInner, -kInner}, { kOuter, -kOuter}};
    const PinColor kArmColor[4] = {PinColor::BlueYellow, PinColor::BlueYellow,
                                   PinColor::RedYellow,  PinColor::RedYellow};
    const float kArmDir[4] = {chad::PI / 2.0f, 0.0f,
                              -chad::PI / 2.0f, chad::PI};
    for (const auto& d : autonDiag) {
        addCup(d[0], d[1], true);
        // North and east are blue/yellow, south and west red/yellow: eight of
        // each over the four assemblies, which is what the manual counts.
        for (int i = 0; i < 4; i++) {
            addPin(d[0] + std::cos(kArmDir[i]) * kArmReach,
                   d[1] + std::sin(kArmDir[i]) * kArmReach,
                   kArmColor[i], false, kArmDir[i]);
        }
    }

    // 4. The four corners of the Midfield diamond: a Cup with the red/blue Pin.
    const float mid[4][2] = {{0.0f, kInner}, {kInner, 0.0f},
                             {0.0f, -kInner}, {-kInner, 0.0f}};
    for (const auto& m : mid) {
        const int cup = addCup(m[0], m[1], true);
        nest(addPin(m[0], m[1], PinColor::RedBlue, true, 0.0f), cup);
    }

    // 5. Eight groups of three Cups against the walls, two per wall, the middle
    //    one holding a yellow Pin.
    const float normal[4][2] = {{0.0f, 1.0f}, {0.0f, -1.0f},
                                {1.0f, 0.0f}, {-1.0f, 0.0f}};
    const float centres[2] = {-kInner, kInner};
    for (const auto& n : normal) {
        // Along the wall is the normal turned ninety degrees.
        const float ax = -n[1], ay = n[0];
        for (const float centre : centres) {
            for (int k = -1; k <= 1; k++) {
                const float t = centre + static_cast<float>(k) * kWallSpan;
                const float x = n[0] * kWallCup + ax * t;
                const float y = n[1] * kWallCup + ay * t;
                // All twenty-four wall Cups start opaque side up; the twelve
                // clear ones are the diagonals and the Midfield diamond.
                const int cup = addCup(x, y, false);
                if (k == 0) {
                    nest(addPin(x, y, PinColor::YellowYellow, true, 0.0f), cup);
                }
            }
        }
    }

    publish();
}

void Pieces::hold(int id) {
    if (mHeld >= 0 && mHeld != id) {
        if (Piece* old = get(mHeld)) old->held = false;
    }
    mHeld = -1;
    Piece* p = get(id);
    if (p == nullptr) return;
    p->held = true;
    p->goal = -1;
    p->inCup = -1;
    p->upright = true;
    mHeld = id;
    mDirty = true;
}

void Pieces::release() {
    if (mHeld < 0) return;
    if (Piece* p = get(mHeld)) {
        p->held = false;
        p->goal = -1;
    }
    mHeld = -1;
    mDirty = true;
}

void Pieces::clawPoint(const chad::Pose& robot, float& cx, float& cy,
                       float& cdir) const {
    const float ct = std::cos(robot.theta), st = std::sin(robot.theta);
    cx = robot.x + mClawMount.x * ct - mClawMount.y * st;
    cy = robot.y + mClawMount.x * st + mClawMount.y * ct;
    cdir = robot.theta + mClawMount.theta;
}

int Pieces::nearestGoal(float x, float y, float& distIn) {
    distIn = 1e9f;
    int best = -1;
    const auto& goals = chad::fieldCircleObstacles();
    for (std::size_t i = 0; i < goals.size(); i++) {
        const float d = std::hypot(x - goals[i].x, y - goals[i].y);
        if (d < distIn) { distIn = d; best = static_cast<int>(i); }
    }
    return best;
}

// ============================================================================
// Plowing
// ============================================================================

void Pieces::shove(const chad::Pose& robot, float halfLenIn, float halfWidIn) {
    if (halfLenIn <= 0.0f || halfWidIn <= 0.0f) return;

    const float ct = std::cos(robot.theta), st = std::sin(robot.theta);

    for (auto& p : mPieces) {
        if (p.held) continue;
        if (p.inCup >= 0) continue;          // rides its Cup, moved with it

        // The body is the rectangle [-halfLen, halfLen] x [-halfWid, halfWid]
        // in its own frame; the piece is a circle, so grow the box by its
        // radius and the overlap test is a point-in-box.
        const float dx = p.x - robot.x, dy = p.y - robot.y;
        const float lx =  dx * ct + dy * st;
        const float ly = -dx * st + dy * ct;
        const float ex = halfLenIn + p.radiusIn;
        const float ey = halfWidIn + p.radiusIn;
        if (std::fabs(lx) >= ex || std::fabs(ly) >= ey) continue;

        // Out the shallowest way, which for anything but a dead-centre hit is
        // the direction the body was travelling relative to the piece.
        float nlx = lx, nly = ly;
        if (ex - std::fabs(lx) < ey - std::fabs(ly)) {
            nlx = (lx < 0.0f) ? -ex : ex;
        } else {
            nly = (ly < 0.0f) ? -ey : ey;
        }
        p.x = robot.x + nlx * ct - nly * st;
        p.y = robot.y + nlx * st + nly * ct;

        // Knocked off whatever it was nested on.
        p.goal = -1;
        p.level = 0;

        // <SG4>: Scoring Objects stay on the Field.
        const float lim = kWallEdge - p.radiusIn;
        p.x = clampIn(p.x, -lim, lim);
        p.y = clampIn(p.y, -lim, lim);
    }
}

// A piece being held by something else -- a Cup, or a Goal -- goes where that
// thing is rather than anywhere physics puts it. Neither is given a body of its
// own, because two colliders at the same point shove each other apart on the
// first step, and a Pin nested in a Goal would be ejected by the Goal's own
// collider the moment it was placed.
void Pieces::carryNested() {
    const auto& goals = chad::fieldCircleObstacles();
    for (auto& p : mPieces) {
        if (p.inCup >= 0) {
            const Piece* c = get(p.inCup);
            if (!c) { p.inCup = -1; continue; }
            p.x = c->x;
            p.y = c->y;
        } else if (!p.held && p.goal >= 0 &&
                   p.goal < static_cast<int>(goals.size())) {
            p.x = goals[p.goal].x;
            p.y = goals[p.goal].y;
        }
    }
}

void Pieces::seat(Piece& p, int liftPin) {
    float goalDist = 0.0f;
    const int g = nearestGoal(p.x, p.y, goalDist);
    if (g >= 0 && goalDist <= seatRangeIn) {
        const auto& goals = chad::fieldCircleObstacles();
        p.goal = g;
        p.level = liftPin;
        p.x = goals[g].x;
        p.y = goals[g].y;
        p.upright = true;
        mDirty = true;
        return;
    }

    // Not on a Goal. A Pin dropped over a Cup nests in it instead -- <SC2>
    // scores that the same way.
    if (p.kind == PieceKind::Pin) {
        for (const auto& c : mPieces) {
            if (c.kind != PieceKind::Cup || c.held) continue;
            if (std::hypot(p.x - c.x, p.y - c.y) > c.radiusIn + kNestSlack) continue;
            p.inCup = c.id;
            p.goal = -1;
            p.level = 0;
            p.upright = true;
            mDirty = true;
            return;
        }
    }

    p.goal = -1;
    p.level = 0;
}

void Pieces::step(const chad::Pose& robot, bool clawClosed, int liftPin,
                  float halfLenIn, float halfWidIn, double dtMs) {
    const bool solver = externalPhysics();
    // Without Rapier the robot has to shove pieces out of its own way by hand.
    // With it, the body is a kinematic collider and the solver does it properly
    // -- pieces roll, spin, and gather in front of a plough.
    if (!solver) shove(robot, halfLenIn, halfWidIn);

    float cx = 0.0f, cy = 0.0f, cdir = 0.0f;
    clawPoint(robot, cx, cy, cdir);
    const float dirX = std::cos(cdir), dirY = std::sin(cdir);

    if (!clawClosed) {
        // Opening drops whatever was in the jaws. Against a Goal it nests
        // there, which is what a routine's final claw.toggle() is for; over a
        // Cup it goes into the Cup; anywhere else it is left on the floor.
        if (mHeld >= 0) {
            if (Piece* p = get(mHeld)) {
                p->held = false;
                seat(*p, liftPin);
            }
            mHeld = -1;
            mDirty = true;
        }
    } else if (mHeld < 0) {
        // Closed and empty: bite on whatever is in front of the jaws -- nearest
        // first, so a claw closing between two pieces takes the one it is
        // actually against. Which piece, and whether it may be taken, stays
        // here rather than in the solver: it is a rule, not a mechanic.
        const float coneCos = std::cos(grabConeDeg * chad::DEG_TO_RAD);
        int best = -1;
        float bestGap = 1e9f;

        for (const auto& p : mPieces) {
            const float dx = p.x - cx, dy = p.y - cy;
            const float dist = std::hypot(dx, dy);
            // Measured to the surface, not the centre, so the range that lets
            // it bite is the range the claw's own sensor reports.
            const float gap = dist - p.radiusIn;
            if (gap > grabRangeIn) continue;
            if (dist > 1e-4f) {
                const float along = (dx * dirX + dy * dirY) / dist;
                if (along < coneCos) continue;      // beside or behind the jaws
            }
            if (gap < bestGap) { bestGap = gap; best = p.id; }
        }
        if (best >= 0) {
            mHeld = best;
            if (Piece* p = get(best)) {
                p->held = true;
                p->goal = -1;
                p->inCup = -1;      // lifted out of whatever Cup it sat in
                p->upright = true;  // a claw picks a Pin up standing
                mDirty = true;
            }
        }
    }

    // Where the jaws are holding it. The solver flies the held piece there as a
    // kinematic body, so it still pushes everything it is dragged through.
    float heldX = cx, heldY = cy, heldTheta = cdir;
    if (mHeld >= 0) {
        if (Piece* p = get(mHeld)) {
            const float reach = kCarryOffsetIn + p->radiusIn;
            heldX = cx + dirX * reach;
            heldY = cy + dirY * reach;
            if (!solver) { p->x = heldX; p->y = heldY; p->theta = heldTheta; }
        }
    }

    if (solver) {
        pumpSolver(robot, dtMs, halfLenIn, halfWidIn, heldX, heldY, heldTheta);
    }
    carryNested();
    publish();
}

// ============================================================================
// Rapier
// ============================================================================
//
// The solver lives in JavaScript (sim/web/rapier_pieces.js) and is optional:
// nothing installs it in a build that only wants the kinematic model, and the
// hooks below report it missing.
//
// Everything crosses in two flat buffers rather than a call per piece, because
// this runs on the 10 ms physics tick.

namespace {

constexpr int kMaxSolverPieces = 256;
constexpr int kMaxSolverActors = 8;
constexpr int kSolverHeader = 10;

float gSolverIn[kSolverHeader + 5 * kMaxSolverActors];
float gSolverOut[4 * kMaxSolverPieces];

}  // namespace

#ifdef __EMSCRIPTEN__
#include <emscripten.h>

EM_JS(int, chadRapierReady, (), {
    return (Module.rapierStep && Module.rapierSync) ? 1 : 0;
});
EM_JS(void, chadRapierSync, (const char* json), {
    Module.rapierSync(UTF8ToString(json));
});
// The parameter names become JavaScript identifiers, so none of them may be a
// reserved word -- `in` compiled fine and then failed in the JS optimiser.
EM_JS(int, chadRapierStep, (double dtMs, float* inPtr, int inCount,
                            float* outPtr, int cap), {
    return Module.rapierStep(dtMs, inPtr >> 2, inCount, outPtr >> 2, cap);
});
#else
static int  chadRapierReady() { return 0; }
static void chadRapierSync(const char*) {}
static int  chadRapierStep(double, float*, int, float*, int) { return 0; }
#endif

bool Pieces::externalPhysics() const {
    return chadRapierReady() != 0;
}

void Pieces::pushToSolver() {
    // A Pin nested in a Cup is not a second body: it sits inside the Cup and
    // goes where the Cup goes. Handing both to the solver at the same point
    // would have them shove each other apart on the first step.
    std::string json = "{\"pieces\":[";
    bool first = true;
    for (const auto& p : mPieces) {
        if (p.inCup >= 0 || p.goal >= 0) continue;
        char b[160];
        std::snprintf(b, sizeof(b),
            "%s{\"id\":%d,\"kind\":%d,\"x\":%.4f,\"y\":%.4f,\"theta\":%.4f,"
            "\"upright\":%s}",
            first ? "" : ",", p.id, static_cast<int>(p.kind), p.x, p.y, p.theta,
            p.upright ? "true" : "false");
        json += b;
        first = false;
    }
    // The goals go with them. They are fixed colliders over there, and the
    // field is editable at runtime, so sending them together is what keeps the
    // solver's idea of the field from going stale.
    json += "],\"goals\":[";
    first = true;
    for (const auto& c : field().circles()) {
        char b[96];
        std::snprintf(b, sizeof(b), "%s{\"x\":%.4f,\"y\":%.4f,\"r\":%.4f}",
                      first ? "" : ",", c.x, c.y, c.r);
        json += b;
        first = false;
    }
    json += "]}";
    chadRapierSync(json.c_str());
    mDirty = false;
}

void Pieces::pumpSolver(const chad::Pose& robot, double dtMs,
                        float halfLenIn, float halfWidIn,
                        float heldX, float heldY, float heldTheta) {
    if (mDirty) pushToSolver();

    gSolverIn[0] = robot.x;
    gSolverIn[1] = robot.y;
    gSolverIn[2] = robot.theta;
    gSolverIn[3] = halfLenIn;
    gSolverIn[4] = halfWidIn;
    gSolverIn[5] = static_cast<float>(mHeld);
    gSolverIn[6] = heldX;
    gSolverIn[7] = heldY;
    gSolverIn[8] = heldTheta;

    int n = 0;
    for (const auto& a : actors().all()) {
        if (!a.enabled || n >= kMaxSolverActors) continue;
        float* slot = gSolverIn + kSolverHeader + n * 5;
        slot[0] = a.x;
        slot[1] = a.y;
        slot[2] = a.theta;
        slot[3] = 0.5f * a.lengthIn;
        slot[4] = 0.5f * a.widthIn;
        n++;
    }
    gSolverIn[9] = static_cast<float>(n);

    const int count = chadRapierStep(dtMs, gSolverIn, kSolverHeader + n * 5,
                                     gSolverOut, 4 * kMaxSolverPieces);
    for (int i = 0; i < count; i++) {
        const int id = static_cast<int>(gSolverOut[i * 4 + 0]);
        Piece* p = get(id);
        if (p == nullptr) continue;
        p->x = gSolverOut[i * 4 + 1];
        p->y = gSolverOut[i * 4 + 2];
        p->theta = gSolverOut[i * 4 + 3];
    }
}

void Pieces::publish() const {
    std::vector<Circle> circles;
    circles.reserve(mPieces.size() + 32);
    for (const auto& p : mPieces) {
        // kind 5 so the browser can colour them apart from the Goals, which
        // are kind 10-13 and are not going anywhere.
        if (p.kind == PieceKind::Pin && !p.upright) {
            // A Pin on its side is 6.5 in of obstacle, not 1.6. Three circles
            // down its axis put the beam's stop where the pin really is.
            const float h = 0.5f * kPinLengthIn - p.radiusIn;
            const float ux = std::cos(p.theta) * h, uy = std::sin(p.theta) * h;
            circles.push_back({p.x - ux, p.y - uy, p.radiusIn, 5});
            circles.push_back({p.x,      p.y,      p.radiusIn, 5});
            circles.push_back({p.x + ux, p.y + uy, p.radiusIn, 5});
        } else {
            circles.push_back({p.x, p.y, p.radiusIn, 5});
        }
    }
    field().setDynamicCircles(std::move(circles));
}

}  // namespace sim
