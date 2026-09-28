// chad-sim -- the Override Scoring Objects, and the claw that moves them.
//
// Two kinds, from the V5RC Override manual: a PIN is the 1.6 in hexagonal peg
// that gets nested into a Goal or a Cup, a CUP is the 3.16 in hourglass. Both
// are round to the raycaster, because a distance sensor does not care what
// shape stopped the beam, only where it stopped -- except a Pin lying on its
// side, which is 6.5 in long and publishes as three circles down its axis so a
// beam crossing it broadside stops where the pin really is.
//
// loadDefault() is Figure FO-2, the starting configuration: 36 Cups and 37
// Pins. Positions were read off that figure with the four Alliance Goals as
// calibration points, then snapped to the same tile grid kFieldPosts uses. The
// other 20 Cups and 26 Pins are Match Loads and Preloads, which start off the
// Field and so are not here.
//
// What this is for: the routines in src/gamespecific/autons all follow the same
// shape -- drive at a pin, close the claw, lift, drive at a goal, open. None of
// that could be watched before, because nothing was there to pick up and the
// claw's own sensor was never written, so clawSees() read true on an empty
// field and auto-grab fired the moment it was switched on.
//
// Deliberately not modelled: mass, tipping, stacks more than one deep, and
// anything an opposing robot does to a piece. A piece is where it was put until
// the claw takes it or the robot drives into it.
#pragma once

#include <vector>

#include "chad/util/util.hpp"

namespace sim {

enum class PieceKind : int { Pin = 0, Cup = 1 };

/// The four Pin colourings the manual lists. Nothing in the physics reads this;
/// it decides what a routine may legally score, and how the piece draws.
enum class PinColor : int {
    RedYellow = 0, BlueYellow = 1, YellowYellow = 2, RedBlue = 3
};

/// Which of the nine Goals an index refers to. The order matches kFieldPosts in
/// gamespecific/config.cpp, which happens to group cleanly: 0-1 red, 2-3 blue,
/// 4 the tall centre Goal, 5-8 the short neutrals.
enum class GoalKind : int {
    RedAlliance = 0, BlueAlliance = 1, NeutralShort = 2, NeutralTall = 3
};
GoalKind goalKind(int index);

struct Piece {
    int   id{0};
    PieceKind kind{PieceKind::Pin};

    float x{0.0f}, y{0.0f};     // inches, field frame
    float theta{0.0f};          // radians; which way a lying pin points
    float radiusIn{0.8f};       // cross-section the beams see

    PinColor color{PinColor::YellowYellow};

    /// Pins only. Upright is the scoring orientation; the sixteen Pins around
    /// the Autonomous Line start lying down.
    bool  upright{true};
    /// Cups only, and cosmetic here: a Pin nested in the transparent half
    /// scores, one nested in the opaque half does not.
    bool  clearUp{false};

    /// In the claw right now. Exactly one piece can be.
    bool  held{false};

    /// Nested in a Goal, or -1 when it is not. `level` is the lift preset it
    /// was placed at, which is what a routine's liftToPin(n) means.
    int   goal{-1};
    int   level{0};

    /// Nested in this Cup's id, or -1. A nested Pin rides its Cup.
    int   inCup{-1};
};

class Pieces {
public:
    /// Figure FO-2 -- what is on the Field when the Match starts.
    void loadDefault();
    void clear();

    int  add(PieceKind kind, float x, float y, float thetaRad);
    bool remove(int id);
    Piece*       get(int id);
    const Piece* get(int id) const;

    const std::vector<Piece>& all() const { return mPieces; }
    std::vector<Piece>&       all()       { return mPieces; }

    /// Claw position in the robot frame: x forward, y left, theta pointing
    /// angle -- the same convention chad::sensorMounts uses.
    void setClawMount(const chad::Pose& mount) { mClawMount = mount; }
    const chad::Pose& clawMount() const { return mClawMount; }

    /// Where the jaws are in the field frame, and which way they face.
    void clawPoint(const chad::Pose& robot, float& cx, float& cy, float& cdir) const;

    /// Carry, take, drop and shove. `liftPin` is the lift preset the robot is
    /// closest to, used to decide which level a piece is nested at when it is
    /// released against a Goal. The body half-extents are what plows loose
    /// pieces aside -- <SG6> allows plowing, and a sim where a robot drives
    /// through a Cup without touching it is not showing the route that runs.
    void step(const chad::Pose& robot, bool clawClosed, int liftPin,
              float halfLenIn, float halfWidIn, double dtMs);

    /// True when Rapier is driving the pieces. The browser and the test
    /// harness install it; anything that does not gets the kinematic model
    /// below, which is the same rules with a cruder shove.
    bool externalPhysics() const;

    /// Tell the solver its body list is out of date. Anything that adds,
    /// removes, or moves a piece by hand has to say so -- Rapier owns where a
    /// piece is, so a pose written behind its back is one it will undo.
    void markDirty() { mDirty = true; }

    /// Hand the pieces to the field as dynamic circles, so the beams stop on
    /// them. Kept apart from the actors' segments so neither erases the other.
    void publish() const;

    int heldId() const { return mHeld; }

    /// Put a piece in the jaws outright, without the claw having to close on
    /// it. This is how a Preload starts: the robot is placed on the field with
    /// one already gripped, so it arrives held rather than being found.
    void hold(int id);

    /// Let go of whatever is in the jaws without nesting it. Needed by the
    /// browser's drag: moving a held piece by hand only looked like it worked,
    /// because the claw still claimed it and the next tick snapped it back.
    void release();

    /// Which Goal a point is nearest, and how far, using the same nine circles
    /// gamespecific/config.cpp registers. -1 when there are none loaded.
    static int nearestGoal(float x, float y, float& distIn);

    // --- tuning ------------------------------------------------------------
    /// How close a piece has to be to the jaws to be taken. Matches
    /// kClawGrabRangeIn, which is what the robot's own auto-grab uses.
    float grabRangeIn{4.5f};
    /// And how far off the claw's axis it may sit. A claw is not a magnet.
    float grabConeDeg{45.0f};
    /// Released within this of a Goal centre, a piece is nested on it rather
    /// than left on the floor.
    float seatRangeIn{5.0f};

private:
    void shove(const chad::Pose& robot, float halfLenIn, float halfWidIn);
    void pushToSolver();
    void pumpSolver(const chad::Pose& robot, double dtMs,
                    float halfLenIn, float halfWidIn,
                    float heldX, float heldY, float heldTheta);
    void carryNested();
    void seat(Piece& p, int liftPin);

    std::vector<Piece> mPieces;
    int  mNextId{1};
    int  mHeld{-1};
    bool mDirty{true};
    chad::Pose mClawMount{-8.0f, 0.0f, chad::PI};
};

Pieces& pieces();

/// True diameters, from the manual. A Pin is 1.6 in across and 6.5 in tall; a
/// Cup is 3.16 in across and 6.48 in tall. Both stand well above a distance
/// sensor, so both stop beams.
constexpr float kPinRadiusIn  = 0.80f;
constexpr float kPinLengthIn  = 6.50f;
constexpr float kCupRadiusIn  = 1.58f;
/// The octagonal Goal, measured across flats off Figure FO-2 at 5.7 in.
constexpr float kGoalRadiusIn = 2.85f;

}  // namespace sim
