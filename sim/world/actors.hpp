// chad-sim -- the other robots on the field.
//
// Four robots play a match and only one of them is yours. The other three park
// against walls, cross in front of a sensor and sit in the corner you were
// going to range off, and a wall-ranging filter has no way to tell any of that
// from the wall itself. Testing RCL against an empty field tests the easy case.
//
// These are deliberately NOT simulated robots. They have no drivetrain, no
// dynamics and no collision of their own: they are rectangles that follow a
// script, because what matters downstream is only that a beam stops on them.
// Anything more would be a second robot to debug.
//
// They reach the sensors through Field::setDynamic(), so they are real geometry
// to the raycaster and to the body collision test, and invisible to the
// geometry editor and to save/load.
#pragma once

#include <vector>

namespace sim {

struct Actor {
    int   id{0};
    bool  enabled{true};
    /// 0 = alliance partner, 1 = opponent. Colour only; the physics is the same.
    int   team{0};

    float x{0.0f}, y{0.0f};     // inches
    float theta{0.0f};          // radians, standard math convention
    float lengthIn{15.0f};
    float widthIn{15.0f};

    // --- patrol ----------------------------------------------------------
    /// Waypoints, interleaved x, y. Fewer than two and it just sits there.
    std::vector<float> path;
    float speedIn{24.0f};
    /// Closed circuit. Off is an open path walked there and back -- a patrol
    /// that drives to the last point and parks looks exactly like one that has
    /// stopped working, with the button still lit saying it is running.
    bool  loop{true};
    /// Which leg of the path it is on, how far along that leg, and which way
    /// along it. `dir` only matters on an open path.
    int   leg{0};
    float legT{0.0f};
    int   dir{1};
    /// Patrol is per-actor so one can be parked while another circles.
    bool  patrol{false};
    /// Point the body along the leg it is driving. Off keeps `theta` fixed,
    /// which is what you want for a robot parked square against a wall.
    bool  faceTravel{true};
};

class Actors {
public:
    int  add(float x, float y, float thetaRad, int team);
    bool remove(int id);
    void clear();

    Actor*       get(int id);
    const Actor* get(int id) const;

    const std::vector<Actor>& all() const { return mActors; }
    std::vector<Actor>&       all()       { return mActors; }

    /// Advance every patrolling actor along its path.
    void step(double dtMs);

    /// Hand the current footprints to the field as dynamic geometry. Call it
    /// after step() and before anything casts a ray.
    void publish() const;

    /// Park every actor back on the first waypoint of its path. Called from a
    /// world reset so a re-run faces the same traffic as the run before it --
    /// without which a stress test measures the RNG rather than the filter.
    void rewind();

    /// Snapshot for save/restore across a reset that should not move them.
    std::vector<Actor> snapshot() const { return mActors; }
    void restore(std::vector<Actor> a) { mActors = std::move(a); }

private:
    std::vector<Actor> mActors;
    int mNextId{1};
};

Actors& actors();

}  // namespace sim
