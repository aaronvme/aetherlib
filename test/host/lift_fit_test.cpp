#include "chad/util/lift_fit.hpp"

#include <cmath>
#include <vector>

#include "chad_test.hpp"

using chad::tuning::combineLiftSweeps;
using chad::tuning::fitLiftGravity;
using chad::tuning::LiftGravityFit;
using chad::tuning::LiftHoldSample;
using chad::tuning::LiftSweepBin;

namespace {

constexpr float kRatio = 7.0f;        // motor degrees per arm degree
constexpr float kG = 18.0f;
constexpr float kHorizontal = 140.0f; // motor degrees at arm level

/// Samples of a perfect arm from `loArm` to `hiArm` degrees off horizontal.
std::vector<LiftHoldSample> arc(float loArm, float hiArm, int n,
                                float gain = kG, float horiz = kHorizontal) {
    std::vector<LiftHoldSample> out;
    for (int i = 0; i < n; ++i) {
        const float t = (n == 1) ? 0.0f : static_cast<float>(i) /
                                              static_cast<float>(n - 1);
        const float armDeg = loArm + (hiArm - loArm) * t;
        out.push_back({horiz + armDeg * kRatio,
                       gain * std::cos(armDeg * chad::DEG_TO_RAD)});
    }
    return out;
}

}  // namespace

CHAD_TEST(liftfit, recovers_the_gain_and_the_horizontal_position) {
    const auto s = arc(-20.0f, 70.0f, 12);
    const LiftGravityFit f = fitLiftGravity(s.data(), (int)s.size(), kRatio);

    CHECK(f.valid);
    CHECK_NEAR(f.kG, kG, 1e-2);
    CHECK_NEAR(f.horizontalMotorDeg, kHorizontal, 0.5);
    CHECK_NEAR(f.rms, 0.0, 1e-3);
    CHECK(f.samples == 12);
    CHECK_NEAR(f.spanArmDeg, 90.0, 1e-2);
}

CHAD_TEST(liftfit, the_horizontal_is_found_without_the_arm_ever_being_level) {
    // Every sample is above horizontal, so no reading is the answer and the
    // phase has to come out of the curvature. This is the whole point of
    // fitting rather than parking the arm on a level.
    const auto s = arc(25.0f, 80.0f, 10);
    const LiftGravityFit f = fitLiftGravity(s.data(), (int)s.size(), kRatio);

    CHECK(f.valid);
    CHECK_NEAR(f.kG, kG, 1e-2);
    CHECK_NEAR(f.horizontalMotorDeg, kHorizontal, 1.0);
}

CHAD_TEST(liftfit, a_short_arc_is_refused_rather_than_guessed) {
    // 10 degrees of arm: a cosine over that arc is a straight line, so gain and
    // phase trade off freely and any answer fits.
    const auto s = arc(0.0f, 10.0f, 8);
    const LiftGravityFit f = fitLiftGravity(s.data(), (int)s.size(), kRatio);

    CHECK(!f.valid);
    CHECK(f.spanArmDeg < 25.0f);
    CHECK(f.samples == 8);   // refused for span, not for want of data
}

CHAD_TEST(liftfit, an_encoder_tared_a_revolution_out_still_reports_a_usable_value) {
    // atan2 answers on one branch; raw, this came back 2520 motor degrees off
    // and could not be pasted into HORIZONTAL_MOTOR_DEG.
    const float shifted = kHorizontal + 360.0f * kRatio;
    const auto s = arc(-20.0f, 70.0f, 12, kG, shifted);
    const LiftGravityFit f = fitLiftGravity(s.data(), (int)s.size(), kRatio);

    CHECK(f.valid);
    CHECK_NEAR(f.horizontalMotorDeg, shifted, 1.0);
}

CHAD_TEST(liftfit, noise_shows_up_in_rms_without_moving_the_gain) {
    auto s = arc(-20.0f, 70.0f, 24);
    // Deterministic zero-mean wobble, +-1.5 units.
    for (std::size_t i = 0; i < s.size(); ++i) {
        s[i].holdPower += (i % 2 == 0) ? 1.5f : -1.5f;
    }
    const LiftGravityFit f = fitLiftGravity(s.data(), (int)s.size(), kRatio);

    CHECK(f.valid);
    CHECK_NEAR(f.kG, kG, 0.6);
    CHECK(f.rms > 1.0f);
    CHECK(f.rms < 2.0f);
}

CHAD_TEST(liftfit, a_wrong_ratio_shows_up_as_residual) {
    // The failure mode the routine has to be able to name: geometry wrong, not
    // gain wrong. Fitting at 4:1 data that was taken at 7:1 cannot be absorbed
    // by kG, so it has to leave a residual behind.
    const auto s = arc(-20.0f, 70.0f, 16);
    const LiftGravityFit good = fitLiftGravity(s.data(), (int)s.size(), kRatio);
    const LiftGravityFit bad = fitLiftGravity(s.data(), (int)s.size(), 4.0f);

    CHECK_NEAR(good.rms, 0.0, 1e-3);
    CHECK(bad.rms > 1.0f);
}

CHAD_TEST(liftfit, sweeping_both_ways_cancels_friction_out_of_the_gain) {
    const float friction = 6.0f;
    std::vector<LiftSweepBin> bins;
    for (int i = 0; i < 10; ++i) {
        const float armDeg = -20.0f + 10.0f * static_cast<float>(i);
        const float gravity = kG * std::cos(armDeg * chad::DEG_TO_RAD);
        LiftSweepBin b;
        b.motorDeg = kHorizontal + armDeg * kRatio;
        b.sumUp = gravity + friction;    // friction opposes the climb
        b.sumDown = gravity - friction;  // and opposes the descent
        b.nUp = 1;
        b.nDown = 1;
        bins.push_back(b);
    }

    std::vector<LiftHoldSample> held(bins.size());
    float measuredFriction = -1.0f;
    const int m = combineLiftSweeps(bins.data(), (int)bins.size(), held.data(),
                                    &measuredFriction);

    CHECK(m == (int)bins.size());
    CHECK_NEAR(measuredFriction, friction, 1e-3);

    const LiftGravityFit f = fitLiftGravity(held.data(), m, kRatio);
    CHECK(f.valid);
    CHECK_NEAR(f.kG, kG, 1e-2);
    CHECK_NEAR(f.horizontalMotorDeg, kHorizontal, 0.5);
}

CHAD_TEST(liftfit, a_bin_seen_in_only_one_direction_is_dropped) {
    const float friction = 6.0f;
    std::vector<LiftSweepBin> bins;
    for (int i = 0; i < 10; ++i) {
        const float armDeg = -20.0f + 10.0f * static_cast<float>(i);
        const float gravity = kG * std::cos(armDeg * chad::DEG_TO_RAD);
        LiftSweepBin b;
        b.motorDeg = kHorizontal + armDeg * kRatio;
        b.sumUp = gravity + friction;
        b.sumDown = gravity - friction;
        b.nUp = 1;
        b.nDown = (i == 3) ? 0 : 1;   // the down sweep stopped short of this one
        bins.push_back(b);
    }

    std::vector<LiftHoldSample> held(bins.size());
    float measuredFriction = -1.0f;
    const int m = combineLiftSweeps(bins.data(), (int)bins.size(), held.data(),
                                    &measuredFriction);

    CHECK(m == 9);
    CHECK_NEAR(measuredFriction, friction, 1e-3);
    // Kept, that bin would have carried the full friction band into the gain.
    for (int i = 0; i < m; ++i) {
        CHECK(std::fabs(held[i].holdPower -
                        kG * std::cos((held[i].motorDeg - kHorizontal) /
                                      kRatio * chad::DEG_TO_RAD)) < 1e-3f);
    }
}

CHAD_TEST(liftfit, degenerate_input_is_refused_rather_than_returning_nonsense) {
    const auto s = arc(-20.0f, 70.0f, 12);

    CHECK(!fitLiftGravity(nullptr, 12, kRatio).valid);
    CHECK(!fitLiftGravity(s.data(), 2, kRatio).valid);
    CHECK(!fitLiftGravity(s.data(), (int)s.size(), 0.0f).valid);

    // A ratio of zero used to divide straight into an infinity.
    const LiftGravityFit z = fitLiftGravity(s.data(), (int)s.size(), 0.0f);
    CHECK(std::isfinite(z.kG));
    CHECK(std::isfinite(z.horizontalMotorDeg));
}

// --- constant-load lifts -------------------------------------------------

CHAD_TEST(liftfit, a_pulley_lift_reads_as_flat_not_as_a_cosine) {
    // Same load at every height, which is what a spool does. The cosine fit
    // has nothing to find here, and must not beat the flat one.
    const float hold = 20.2f;
    std::vector<LiftHoldSample> s;
    for (int i = 0; i < 20; ++i) {
        s.push_back({static_cast<float>(i) * 35.0f, hold});
    }

    const auto flat = chad::tuning::fitLiftConstant(s.data(), (int)s.size());
    CHECK(flat.valid);
    CHECK_NEAR(flat.meanPower, hold, 1e-3);
    CHECK_NEAR(flat.rmsConstant, 0.0, 1e-3);
    CHECK_NEAR(flat.perMotorDeg, 0.0, 1e-6);
}

CHAD_TEST(liftfit, elastic_assist_shows_up_as_a_slope_the_mean_cannot_match) {
    // Banding that unloads the lift as it rises: linear in height, so the mean
    // alone leaves a residual and the slope removes it.
    const float base = 24.0f, slope = -0.004f;
    std::vector<LiftHoldSample> s;
    for (int i = 0; i < 20; ++i) {
        const float x = static_cast<float>(i) * 35.0f;
        s.push_back({x, base + slope * x});
    }

    const auto f = chad::tuning::fitLiftConstant(s.data(), (int)s.size());
    CHECK(f.valid);
    CHECK_NEAR(f.base, base, 1e-2);
    CHECK_NEAR(f.perMotorDeg, slope, 1e-5);
    CHECK_NEAR(f.rms, 0.0, 1e-3);
    // The whole point: the slope earns its keep here and did not above.
    CHECK(f.rmsConstant > 0.5f);
    CHECK(f.rms < 0.8f * f.rmsConstant);
}

CHAD_TEST(liftfit, a_real_pivoted_arm_still_prefers_the_cosine) {
    // The routine chooses between the two models, so the flat fit must lose
    // when there genuinely is a pivot -- otherwise every arm reads as a pulley.
    const auto s = arc(-40.0f, 60.0f, 20);
    const auto flat = chad::tuning::fitLiftConstant(s.data(), (int)s.size());
    const auto cos = fitLiftGravity(s.data(), (int)s.size(), kRatio);

    CHECK(cos.valid);
    CHECK(cos.rms < 0.8f * std::min(flat.rms, flat.rmsConstant));
}

CHAD_TEST(liftfit, one_sample_height_cannot_produce_a_slope) {
    // Every bin at the same position: the normal equations are singular, and
    // this used to divide by a zero determinant.
    std::vector<LiftHoldSample> s(5, LiftHoldSample{500.0f, 18.0f});
    const auto f = chad::tuning::fitLiftConstant(s.data(), (int)s.size());

    CHECK(f.valid);
    CHECK_NEAR(f.meanPower, 18.0, 1e-3);
    CHECK_NEAR(f.perMotorDeg, 0.0, 1e-9);
    CHECK(std::isfinite(f.base));
}

// --- friction vs back-EMF ------------------------------------------------

CHAD_TEST(liftfit, two_speeds_separate_friction_from_back_emf) {
    // The artifact this exists for: at 25 rpm a 200 rpm cartridge carries
    // ~16 units of back-EMF, which lands in the up/down gap and reads as
    // friction that is not there.
    const float friction = 13.0f, kv = 0.635f;   // units per rpm
    const float gap1 = friction + kv * 25.0f;
    const float gap2 = friction + kv * 50.0f;

    const auto s = chad::tuning::splitFrictionAndEmf(gap1, 25.0f, gap2, 50.0f);
    CHECK(s.valid);
    CHECK_NEAR(s.frictionUnits, friction, 1e-2);
    CHECK_NEAR(s.emfPerRpm, kv, 1e-4);
    // And the single-speed number really was that far out.
    CHECK(gap1 > 2.0f * friction);
}

CHAD_TEST(liftfit, a_frictionless_drive_puts_the_whole_gap_in_back_emf) {
    const float kv = 0.6f;
    const auto s = chad::tuning::splitFrictionAndEmf(kv * 25.0f, 25.0f,
                                                     kv * 50.0f, 50.0f);
    CHECK(s.valid);
    CHECK_NEAR(s.frictionUnits, 0.0, 1e-3);
    CHECK_NEAR(s.emfPerRpm, kv, 1e-5);
}

CHAD_TEST(liftfit, a_gap_that_shrinks_with_speed_is_refused) {
    // No back-EMF falls with speed, so this is the mechanism changing between
    // passes -- string reseating, a stage binding. Better refused than fitted.
    const auto s = chad::tuning::splitFrictionAndEmf(40.0f, 25.0f, 30.0f, 50.0f);
    CHECK(!s.valid);
}

CHAD_TEST(liftfit, one_speed_cannot_split_anything) {
    const auto s = chad::tuning::splitFrictionAndEmf(29.3f, 25.0f, 29.3f, 25.0f);
    CHECK(!s.valid);
    CHECK(std::isfinite(s.frictionUnits));
}
