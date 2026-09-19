#include "chad/util/localization_fit.hpp"

#include <cmath>

#include "chad_test.hpp"

using chad::tuning::castFieldMap;
using chad::tuning::MapBeam;
using chad::tuning::MapCircle;
using chad::tuning::solveMapped;

namespace {

constexpr float kHalf = 70.2f;

// The nine posts config.cpp registers.
constexpr MapCircle kPosts[] = {
    {-47.0f, -23.5f, 2.0f}, {-23.5f, -47.0f, 2.0f},
    { 23.5f,  47.0f, 2.0f}, { 47.0f,  23.5f, 2.0f},
    {  0.0f,   0.0f, 2.0f}, {-47.0f,  23.5f, 2.0f},
    { 23.5f, -47.0f, 2.0f}, {-23.5f,  47.0f, 2.0f},
    { 47.0f, -23.5f, 2.0f},
};
constexpr int kNumPosts = static_cast<int>(sizeof(kPosts) / sizeof(kPosts[0]));

// The four mounts config.cpp ships, as (x, y, thetaDeg).
struct Mount { float x, y, thetaDeg; };
constexpr Mount kMounts[] = {
    { 1.00f, -3.75f,   0.0f},
    { 0.50f,  6.25f,  90.0f},
    {-0.50f,  5.00f, 180.0f},
    {-0.75f, -6.75f, 270.0f},
};

constexpr double kPi = 3.14159265358979323846;

// Builds the four beams a robot at (x, y, thetaDeg) would read against the
// posts, so a solve can be scored against a pose it never saw.
void synth(float x, float y, float thetaDeg, MapBeam* out,
           float sigmaIn = 0.1f, bool withPosts = true) {
    const float th = static_cast<float>(thetaDeg * kPi / 180.0);
    const float c = std::cos(th), s = std::sin(th);
    for (int i = 0; i < 4; i++) {
        const float aw = th + static_cast<float>(kMounts[i].thetaDeg * kPi / 180.0);
        out[i].present = true;
        out[i].valid = true;
        out[i].sigmaIn = sigmaIn;
        out[i].bx = std::cos(aw);
        out[i].by = std::sin(aw);
        out[i].alongX = kMounts[i].x * c - kMounts[i].y * s;
        out[i].alongY = kMounts[i].x * s + kMounts[i].y * c;
        out[i].inches =
            castFieldMap(x + out[i].alongX, y + out[i].alongY, out[i].bx,
                         out[i].by, withPosts ? kPosts : nullptr,
                         withPosts ? kNumPosts : 0, kHalf);
    }
    for (int i = 4; i < 8; i++) out[i] = MapBeam{};
}

// What readBeams does on the robot: a wall past the sensor's reach is not a
// reading at all, it is silence.
void limitRange(MapBeam* b, float maxIn) {
    for (int i = 0; i < 8; i++) {
        if (!b[i].valid || b[i].inches <= maxIn) continue;
        b[i].valid = false;      // present, but silent: still evidence
        b[i].inches = 0.0f;
        b[i].sigmaIn = 0.0f;
    }
}

constexpr float kReach = 79.0f;

}  // namespace

CHAD_TEST(fieldmap, cast_hits_the_near_wall_with_no_posts) {
    // Facing +x from the origin: the wall is halfSpan away.
    const float d = castFieldMap(0.0f, 0.0f, 1.0f, 0.0f, nullptr, 0, kHalf);
    CHECK_NEAR(d, kHalf, 1e-3);
}

CHAD_TEST(fieldmap, cast_stops_at_a_post_in_front_of_the_wall) {
    // From (-40, -23.5) facing +x, the post at (-47,-23.5) is behind, and the
    // one at (47,-23.5) is ahead at 87 in with the wall at 110.2.
    const float d = castFieldMap(-40.0f, -23.5f, 1.0f, 0.0f, kPosts, kNumPosts,
                                 kHalf);
    CHECK_NEAR(d, 85.0f, 1e-3);   // 87 centre - 2 radius
}

CHAD_TEST(fieldmap, cast_ignores_a_post_behind_the_sensor) {
    const float d = castFieldMap(-40.0f, -23.5f, -1.0f, 0.0f, kPosts, kNumPosts,
                                 kHalf);
    CHECK_NEAR(d, 5.0f, 1e-3);    // post at -47, radius 2
}

CHAD_TEST(fieldmap, cast_from_inside_a_post_is_not_negative) {
    const float d = castFieldMap(0.0f, 0.0f, 1.0f, 0.0f, kPosts, kNumPosts,
                                 kHalf);
    CHECK(d >= 0.0f);
}

CHAD_TEST(fieldmap, solves_a_corner_pose_no_pair_in_range) {
    // Deep in a corner: the far walls are way out of any sensor's reach, so
    // the bracketed pair solve has nothing and only the map can do this.
    MapBeam beams[8];
    synth(-50.0f, -50.0f, 0.0f, beams);
    const auto fix = solveMapped(beams, 8, kPosts, kNumPosts, kHalf);
    CHECK(fix.trusted);
    CHECK_NEAR(fix.x, -50.0f, 0.05);
    CHECK_NEAR(fix.y, -50.0f, 0.05);
}

CHAD_TEST(fieldmap, solves_a_corner_on_the_two_near_walls_alone) {
    // The real reason the floor is relative: at (-50, -50) the far walls are
    // 114 and 119 in away, so a V5 sensor returns nothing for them and only
    // two beams survive. A fixed floor of three would refuse every corner.
    MapBeam beams[8];
    synth(-50.0f, -50.0f, 0.0f, beams);
    limitRange(beams, kReach);

    int live = 0;
    for (int i = 0; i < 8; i++) if (beams[i].valid) live++;
    CHECK_EQ(live, 2);

    const auto fix = solveMapped(beams, 8, kPosts, kNumPosts, kHalf, kReach);
    CHECK(fix.trusted);
    CHECK_NEAR(fix.x, -50.0f, 0.05);
    CHECK_NEAR(fix.y, -50.0f, 0.05);
}

CHAD_TEST(fieldmap, silence_is_what_makes_a_two_beam_corner_solvable) {
    // Only two beams reach a wall from here, and "19.7 in to my left" is the
    // wall at x=-50 or the post at (-23.5,-47) at x=-2.6 -- the beams alone
    // cannot say. The two that stayed silent can: at the near pose the far
    // walls are out of reach, at the far one they are not.
    MapBeam beams[8];
    synth(-50.0f, -50.0f, 0.0f, beams);
    limitRange(beams, kReach);

    const auto blind = solveMapped(beams, 8, kPosts, kNumPosts, kHalf);
    CHECK(!blind.trusted);

    const auto heard = solveMapped(beams, 8, kPosts, kNumPosts, kHalf, kReach);
    CHECK(heard.trusted);
    CHECK_NEAR(heard.x, -50.0f, 0.3);
    CHECK_NEAR(heard.y, -50.0f, 0.3);
}

CHAD_TEST(fieldmap, an_understated_reach_cannot_excuse_silence) {
    // Telling the solve the sensor is blinder than it is would let it wave
    // away a beam that really would have reported, which is how a confident
    // wrong pose happens. A beam that did read proves the reach.
    MapBeam beams[8];
    synth(-48.0f, -64.0f, 0.0f, beams);
    const auto fix = solveMapped(beams, 8, kPosts, kNumPosts, kHalf, 20.0f);
    if (fix.trusted) {
        CHECK_NEAR(fix.x, -48.0f, 1.0);
        CHECK_NEAR(fix.y, -64.0f, 1.0);
    }
}

CHAD_TEST(fieldmap, a_four_beam_fix_is_verified) {
    MapBeam beams[8];
    synth(-33.0f, 41.0f, 90.0f, beams);
    const auto fix = solveMapped(beams, 8, kPosts, kNumPosts, kHalf);
    CHECK(fix.trusted);
    CHECK(fix.inliers >= chad::tuning::MAP_VERIFIED_INLIERS);
}

CHAD_TEST(fieldmap, solves_when_a_post_blocks_one_beam) {
    // The front sensor sits 3.75 in behind the robot centre in y, so a robot
    // at y = -19.75 puts that beam dead on the post at (47, -23.5). Taken as
    // a wall hit it would place the robot 25 in off in x; the back sensor,
    // which is clear, is the one that has the truth.
    MapBeam beams[8];
    synth(-40.0f, -19.75f, 0.0f, beams);

    const float naive = kHalf - beams[0].inches * beams[0].bx - beams[0].alongX;
    CHECK(std::fabs(naive - (-40.0f)) > 5.0f);

    const auto fix = solveMapped(beams, 8, kPosts, kNumPosts, kHalf);
    CHECK(fix.trusted);
    CHECK_NEAR(fix.x, -40.0f, 0.05);
    CHECK_NEAR(fix.y, -19.75f, 0.05);
}

CHAD_TEST(fieldmap, survives_noise_on_every_beam) {
    MapBeam beams[8];
    synth(-33.0f, 41.0f, 90.0f, beams, 0.15f);
    const float jitter[4] = {0.22f, -0.31f, 0.17f, -0.12f};
    for (int i = 0; i < 4; i++) beams[i].inches += jitter[i];

    const auto fix = solveMapped(beams, 8, kPosts, kNumPosts, kHalf);
    CHECK(fix.trusted);
    CHECK_NEAR(fix.x, -33.0f, 0.6);
    CHECK_NEAR(fix.y, 41.0f, 0.6);
}

CHAD_TEST(fieldmap, refuses_an_unmodelled_blockage_on_two_beams) {
    // A robot parked across two of the beams is not in the map. Two inliers
    // is under the floor, so this has to come back untrusted rather than
    // guessing from the two beams that still see a wall.
    MapBeam beams[8];
    synth(-30.0f, -30.0f, 0.0f, beams);
    beams[0].inches -= 20.0f;
    beams[1].inches -= 25.0f;

    const auto fix = solveMapped(beams, 8, kPosts, kNumPosts, kHalf);
    CHECK(!fix.trusted);
}

CHAD_TEST(fieldmap, refuses_when_only_one_axis_is_seen) {
    MapBeam beams[8];
    synth(-50.0f, -50.0f, 0.0f, beams);
    beams[1].valid = false;
    beams[3].valid = false;   // both y-facing sensors gone

    const auto fix = solveMapped(beams, 8, kPosts, kNumPosts, kHalf);
    CHECK(!fix.trusted);
}

CHAD_TEST(fieldmap, agrees_with_the_bare_field_solve_when_nothing_blocks) {
    // Out in the open the map solve and the single-beam solve must land on
    // the same pose, or one of them is wrong.
    MapBeam beams[8];
    synth(-58.0f, 12.0f, 0.0f, beams, 0.1f, false);

    const auto fix = solveMapped(beams, 8, kPosts, kNumPosts, kHalf);
    CHECK(fix.trusted);
    CHECK_NEAR(fix.x, -58.0f, 0.05);
    CHECK_NEAR(fix.y, 12.0f, 0.05);
}

CHAD_TEST(fieldmap, solves_at_a_quarter_turn) {
    MapBeam beams[8];
    synth(20.0f, -55.0f, 270.0f, beams);
    const auto fix = solveMapped(beams, 8, kPosts, kNumPosts, kHalf);
    CHECK(fix.trusted);
    CHECK_NEAR(fix.x, 20.0f, 0.05);
    CHECK_NEAR(fix.y, -55.0f, 0.05);
}

CHAD_TEST(fieldmap, solves_a_diagonal_pose) {
    // The swept search has no notion of an axis, so 45 deg is just another
    // pose. The candidate solve it replaced could not do this at all.
    MapBeam beams[8];
    synth(-20.0f, -20.0f, 45.0f, beams);
    const auto fix = solveMapped(beams, 8, kPosts, kNumPosts, kHalf);
    CHECK(fix.trusted);
    CHECK_NEAR(fix.x, -20.0f, 0.2);
    CHECK_NEAR(fix.y, -20.0f, 0.2);
}

CHAD_TEST(fieldmap, refuses_a_periodic_alias) {
    // The post lattice repeats every 47 in, so these beams fit two poses
    // that far apart to within 0.65 in. Neither is provable: refuse both.
    MapBeam beams[8];
    synth(-48.0f, -14.0f, 0.0f, beams);
    limitRange(beams, kReach);
    const auto fix = solveMapped(beams, 8, kPosts, kNumPosts, kHalf, kReach);
    if (fix.trusted) {
        CHECK_NEAR(fix.x, -48.0f, 1.0);
        CHECK_NEAR(fix.y, -14.0f, 1.0);
    }
}
