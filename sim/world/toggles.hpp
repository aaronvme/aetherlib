// chad-sim -- the four Override Toggles.
//
// A Toggle is a roller on brackets bolted to the middle of each Field wall. It
// spins and rides up and down in the bracket slots, so a Robot driving into it
// rolls it onto its next face; <SC4> counts three of those faces as scored
// states -- red, blue, and neutral -- and the one showing at the end of the
// Match decides who owns every yellow Pin Placed in that Quadrant. That is a
// 10 point swing per Pin, which is why it is worth being able to try it here.
//
// 26.8 in long and centred on the wall, measured off Figure FO-2 with the same
// calibration the piece layout uses.
//
// What is modelled: which face is showing, and that ramming the front of the
// robot into that stretch of wall advances it by one. What is not: the roller's
// height. It sits on the top rail, so a real robot needs something tall enough
// to reach it, and this cannot tell you whether yours is.
#pragma once

#include <vector>

#include "chad/util/util.hpp"

namespace sim {

/// The three faces <SC4> scores.
///
/// Which one comes up next depends on the wall. The Field has 180 degree
/// rotational symmetry, so the roller on red's side is the same assembly as
/// blue's turned half a turn -- and a robot ramming it from inside the Field
/// therefore walks the colours the other way round. Blue's two walls go yellow,
/// red, blue; red's two go yellow, blue, red.
enum class ToggleState : int { Neutral = 0, Red = 1, Blue = 2 };

struct Toggle {
    int id{0};
    /// 0 = north (+y), 1 = east (+x), 2 = south (-y), 3 = west (-x).
    int wall{0};
    float x{0.0f}, y{0.0f};         // centre of the bar, on the wall
    float lengthIn{26.8f};
    ToggleState state{ToggleState::Neutral};
    /// True while the robot is up against it. Held so one ram is one flip --
    /// without it a robot parked on a toggle would spin it every tick.
    bool contact{false};
};

class Toggles {
public:
    /// One on the middle of each wall.
    void loadDefault();
    void clear();
    void resetStates();

    const std::vector<Toggle>& all() const { return mToggles; }
    Toggle* get(int id);

    void setState(int id, ToggleState s);

    /// One face on, the way a ram would turn it. The browser's click uses this
    /// so setting one by hand and driving into it agree.
    void cycle(int id);

    /// Advance any toggle the robot is currently ramming. Returns the id it
    /// flipped this tick, or -1. `driveIn` is commanded speed, not measured:
    /// a robot pressed against a wall has stopped moving and is still pushing.
    int step(const chad::Pose& robot, float halfLenIn, float halfWidIn,
             float driveIn);

    // --- tuning -------------------------------------------------------------
    /// How close the front of the robot has to get to the wall to be on it.
    float reachIn{2.5f};
    /// And how far it has to come off before the same toggle can flip again.
    float releaseIn{5.0f};
    /// Commanded speed that counts as pushing rather than drifting past.
    float minPushIn{3.0f};

private:
    std::vector<Toggle> mToggles;
};

Toggles& toggles();

/// Wall-to-wall interior, matching sim::Field's perimeter.
constexpr float kToggleWallIn = 70.2f;

}  // namespace sim
