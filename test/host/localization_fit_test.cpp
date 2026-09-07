#include "chad/util/localization_fit.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include "chad_test.hpp"

using namespace chad::tuning;

namespace {

constexpr float kSpan = BEAM_SPAN_SIGMAS;

struct Rng {
    std::uint32_t s;
    explicit Rng(std::uint32_t seed) : s(seed ? seed : 1u) {}
    float uniform() {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        return static_cast<float>(s % 1000000u) / 1000000.0f;
    }
    float gauss() {
        const float u1 = 0.000001f + uniform() * 0.999998f;
        const float u2 = uniform();
        return std::sqrt(-2.0f * std::log(u1)) * std::cos(6.2831853f * u2);
    }
    /// One-sided exponential on (-span, 0), decay lambda. Inverse transform on
    /// the truncated cdf so the sample really is the density EM assumes.
    float shortTail(float lambda, float span) {
        const float e = 1.0f - std::exp(-span / lambda);
        const float v = -lambda * std::log(1.0f - uniform() * e);
        return -v;
    }
};

// Draw from the mixture EM is meant to recover.
std::vector<float> drawMixture(Rng& rng, int n, float pHit, float pShort,
                               float lambda) {
    std::vector<float> out;
    out.reserve(static_cast<std::size_t>(n));
    while (static_cast<int>(out.size()) < n) {
        const float r = rng.uniform();
        float u;
        if (r < pHit) {
            u = rng.gauss();
        } else if (r < pHit + pShort) {
            u = rng.shortTail(lambda, kSpan);
        } else {
            u = -kSpan + 2.0f * kSpan * rng.uniform();
        }
        if (u > -kSpan && u < kSpan) out.push_back(u);
    }
    return out;
}

// Evaluate the mixture exactly as MclTracking::buildGaussianLUT does, so the
// test is tied to the filter's arithmetic rather than to a restatement of it.
float filterLogLik(const BeamMixture& m, float u) {
    float p = m.hitWeight * std::exp(-0.5f * u * u) + m.randWeight;
    if (u < 0.0f) {
        p += m.shortWeight * std::exp(u / (m.shortDecaySigmas > 0.05f
                                               ? m.shortDecaySigmas
                                               : 0.05f));
    }
    return std::log(p > 1e-30f ? p : 1e-30f);
}

}  // namespace

CHAD_TEST(localizationFit, opposingBeamsSumToAConstantWhateverThePose) {
    // Front mount (1.0, -3.75) facing +x, back (-0.5, 5.0) facing -x, on a
    // 144 in field: the sum is 142.5 no matter where the robot sits.
    const float span = 144.0f;
    const float oFront = mountAlongAxis(1.0f, -3.75f, 0.0f, true);
    const float oBack = mountAlongAxis(-0.5f, 5.0f, 0.0f, true);
    CHECK_NEAR(oFront, 1.0, 1e-5);
    CHECK_NEAR(oBack, -0.5, 1e-5);
    const float expected = opposingPairSum(span, oFront, oBack);
    CHECK_NEAR(expected, 142.5, 1e-4);

    // Verify against the geometry directly, from several positions.
    for (float cx : {-60.0f, -20.0f, 0.0f, 15.0f, 55.0f}) {
        const float dFront = span * 0.5f - cx - oFront;
        const float dBack = span * 0.5f + cx + oBack;
        CHECK_NEAR(dFront + dBack, expected, 1e-3);
    }
}

CHAD_TEST(localizationFit, mountAlongAxisRotatesWithTheRobot) {
    // Turned 90 degrees, the front mount's x arm now lies along y.
    const float t = 1.5707963f;
    CHECK_NEAR(mountAlongAxis(1.0f, 0.0f, t, true), 0.0, 1e-4);
    CHECK_NEAR(mountAlongAxis(1.0f, 0.0f, t, false), 1.0, 1e-4);
    // And the pair constant still holds once the offsets are resolved.
    const float oa = mountAlongAxis(1.0f, -3.75f, t, false);
    const float ob = mountAlongAxis(-0.5f, 5.0f, t, false);
    const float expected = opposingPairSum(144.0f, oa, ob);
    for (float cy : {-40.0f, 0.0f, 30.0f}) {
        const float da = 72.0f - cy - oa;
        const float db = 72.0f + cy + ob;
        CHECK_NEAR(da + db, expected, 1e-3);
    }
}

CHAD_TEST(localizationFit, beamSigmaBaseSwitchesAt200mm) {
    // 200 mm is 7.874 in.
    CHECK_NEAR(beamSigmaBase(5.0f, 0.068f, 0.05f), 0.068, 1e-6);
    CHECK_NEAR(beamSigmaBase(7.0f, 0.068f, 0.05f), 0.068, 1e-6);
    CHECK_NEAR(beamSigmaBase(40.0f, 0.068f, 0.05f), 2.0, 1e-5);
    CHECK_NEAR(beamSigmaBase(100.0f, 0.068f, 0.05f), 5.0, 1e-5);
}

CHAD_TEST(localizationFit, emRecoversPlantedMixtureParameters) {
    Rng rng(4242u);
    const float pHit = 0.80f, pShort = 0.17f, lambda = 1.5f;
    const auto u = drawMixture(rng, 4000, pHit, pShort, lambda);

    const BeamMixtureFit f = fitBeamMixture(u.data(), static_cast<int>(u.size()));
    CHECK(f.valid);
    CHECK(f.samples > 3500);
    CHECK(f.iterations > 1);

    // Proportions, which are what the data actually constrains.
    CHECK_NEAR(f.hitFraction, pHit, 0.05);
    CHECK_NEAR(f.shortFraction, pShort, 0.05);
    CHECK_NEAR(f.randFraction, 1.0f - pHit - pShort, 0.03);
    CHECK_NEAR(f.params.shortDecaySigmas, lambda, 0.35);

    // Sum to one, and hitWeight stays pinned so the filter's scale is unchanged.
    CHECK_NEAR(f.hitFraction + f.shortFraction + f.randFraction, 1.0, 1e-3);
    CHECK(f.params.hitWeight == 1.0f);
    CHECK(f.params.shortWeight > 0.0f);
    CHECK(f.params.randWeight > 0.0f);
}

CHAD_TEST(localizationFit, theFittedWeightsBeatTheShippedGuessOnItsOwnData) {
    // The point of fitting: the filter's own log likelihood, summed over the
    // data, must improve. If it does not, the fit is not worth running.
    Rng rng(99u);
    const auto u = drawMixture(rng, 3000, 0.72f, 0.25f, 2.2f);
    const BeamMixtureFit f = fitBeamMixture(u.data(), static_cast<int>(u.size()));
    CHECK(f.valid);

    const BeamMixture shipped;   // hitWeight 1, shortWeight 0.5, decay 1.2, rand 0.0008
    double llShipped = 0.0, llFitted = 0.0;
    for (float x : u) {
        llShipped += filterLogLik(shipped, x);
        llFitted += filterLogLik(f.params, x);
    }
    CHECK(llFitted > llShipped);
}

CHAD_TEST(localizationFit, aCleanGaussianDatasetReportsNoTail) {
    // No obstacle events at all: the tail and floor must collapse rather than
    // absorbing Gaussian samples and inventing a tail.
    Rng rng(7u);
    std::vector<float> u;
    for (int i = 0; i < 3000; i++) {
        const float g = rng.gauss();
        if (g > -kSpan && g < kSpan) u.push_back(g);
    }
    const BeamMixtureFit f = fitBeamMixture(u.data(), static_cast<int>(u.size()));
    CHECK(f.valid);
    CHECK(f.hitFraction > 0.85f);
    CHECK(f.shortFraction < 0.15f);
    CHECK(f.randFraction < 0.02f);
}

CHAD_TEST(localizationFit, aHeavyTailIsNotMistakenForAWideGaussian) {
    // Half the readings blocked. The whole reason for EM: a moment fit would
    // report one wide Gaussian and no tail.
    Rng rng(31337u);
    const auto u = drawMixture(rng, 4000, 0.50f, 0.49f, 1.0f);
    const BeamMixtureFit f = fitBeamMixture(u.data(), static_cast<int>(u.size()));
    CHECK(f.valid);
    CHECK(f.shortFraction > 0.35f);
    CHECK(f.params.shortWeight > 0.2f);
}

CHAD_TEST(localizationFit, mixtureFitRejectsUnusableInput) {
    const float few[] = {0.1f, -0.2f, 0.3f};
    CHECK(!fitBeamMixture(few, 3).valid);
    CHECK(!fitBeamMixture(nullptr, 500).valid);

    std::vector<float> ok(200, 0.0f);
    CHECK(!fitBeamMixture(ok.data(), 200, 0.5f).valid);   // span too small

    // All out of range, so nothing is usable even though n is large.
    std::vector<float> far(200, 500.0f);
    CHECK(!fitBeamMixture(far.data(), 200).valid);

    // Non-finite entries are skipped, not propagated.
    Rng rng(5u);
    auto mixed = drawMixture(rng, 400, 0.8f, 0.15f, 1.3f);
    mixed[10] = std::nanf("");
    mixed[20] = INFINITY;
    const BeamMixtureFit f = fitBeamMixture(mixed.data(),
                                            static_cast<int>(mixed.size()));
    CHECK(f.valid);
    CHECK(f.samples <= static_cast<int>(mixed.size()) - 2);
    CHECK(std::isfinite(f.params.shortWeight));
    CHECK(std::isfinite(f.params.randWeight));
}

CHAD_TEST(localizationFit, pairToPerBeamWidensOnlyTheDecay) {
    BeamMixture pair;
    pair.shortWeight = 0.4f;
    pair.shortDecaySigmas = 2.0f;
    pair.randWeight = 0.002f;
    const BeamMixture one = perBeamFromPair(pair);
    CHECK_NEAR(one.shortDecaySigmas, 2.0 * 1.41421356, 1e-4);
    CHECK(one.shortWeight == pair.shortWeight);
    CHECK(one.randWeight == pair.randWeight);
    CHECK(one.hitWeight == pair.hitWeight);
}

CHAD_TEST(localizationFit, sigmaCalibrationFindsTheUnderreportingFactor) {
    // MCL claims 0.5 in while really making 1.5 in of error: NEES is 9, so the
    // scale wanted is 3.
    std::vector<float> rep(200, 0.5f), err;
    Rng rng(17u);
    for (int i = 0; i < 200; i++) err.push_back(1.5f * rng.gauss());

    const SigmaCalibration c =
        calibrateMclSigma(rep.data(), err.data(), 200);
    CHECK(c.valid);
    CHECK(c.samples == 200);
    CHECK_NEAR(c.scale, 3.0, 0.5);
    CHECK(c.meanNees > 4.0f);
    // The floor is a high quantile of real error, so it exceeds the typical one.
    CHECK(c.floorIn > 1.5f);
    CHECK(c.floorIn < 6.0f);
}

CHAD_TEST(localizationFit, sigmaCalibrationNeverClaimsBetterThanReported) {
    // Already conservative: reported 4 in, actual error 0.2 in. Scaling down
    // would make the EKF trust MCL more than MCL has earned.
    std::vector<float> rep(100, 4.0f), err(100, 0.2f);
    const SigmaCalibration c = calibrateMclSigma(rep.data(), err.data(), 100);
    CHECK(c.valid);
    CHECK(c.scale == 1.0f);
    CHECK(c.meanNees < 1.0f);
}

CHAD_TEST(localizationFit, sigmaCalibrationIgnoresCollapsedSigmas) {
    std::vector<float> rep, err;
    for (int i = 0; i < 40; i++) { rep.push_back(1.0f); err.push_back(2.0f); }
    for (int i = 0; i < 40; i++) { rep.push_back(0.0f); err.push_back(2.0f); }
    const SigmaCalibration c =
        calibrateMclSigma(rep.data(), err.data(), static_cast<int>(rep.size()));
    CHECK(c.valid);
    CHECK(c.samples == 40);           // the zero sigmas contributed nothing
    CHECK_NEAR(c.scale, 2.0, 0.01);   // and did not skew the answer

    std::vector<float> tooFew(4, 1.0f);
    CHECK(!calibrateMclSigma(tooFew.data(), tooFew.data(), 4).valid);
    CHECK(!calibrateMclSigma(nullptr, err.data(), 40).valid);
}

CHAD_TEST(localizationFit, recoveryScoreUsesMediansAndCountsFailures) {
    const RecoveryTrial t[] = {
        {800.0f, 0.9f, true},
        {1200.0f, 1.1f, true},
        {900.0f, 0.8f, true},
        {0.0f, 40.0f, false},     // never came back
        {1000.0f, 1.0f, true},
    };
    const RecoveryScore s = scoreRecovery(t, 5);
    CHECK(s.valid);
    CHECK(s.trials == 5);
    CHECK(s.converged == 4);
    CHECK_NEAR(s.medianMs, 950.0, 1e-3);   // (900 + 1000)/2, failure excluded
    CHECK_NEAR(s.worstMs, 1200.0, 1e-3);
    CHECK_NEAR(s.medianErrorIn, 0.95, 1e-3);

    // A failed trial must not enter as a large time.
    const RecoveryTrial allBad[] = {{0.0f, 30.0f, false}, {0.0f, 30.0f, false}};
    const RecoveryScore bad = scoreRecovery(allBad, 2);
    CHECK(!bad.valid);
    CHECK(bad.trials == 2);
    CHECK(bad.converged == 0);
    CHECK(!scoreRecovery(nullptr, 4).valid);
}

CHAD_TEST(localizationFit, convergenceRateOutranksSpeed) {
    RecoveryScore fastButFlaky;
    fastButFlaky.valid = true; fastButFlaky.trials = 5; fastButFlaky.converged = 3;
    fastButFlaky.medianMs = 400.0f;

    RecoveryScore slowButSure;
    slowButSure.valid = true; slowButSure.trials = 5; slowButSure.converged = 5;
    slowButSure.medianMs = 1500.0f;

    CHECK(recoveryBetterThan(slowButSure, fastButFlaky));
    CHECK(!recoveryBetterThan(fastButFlaky, slowButSure));

    // Same rate: then speed decides.
    RecoveryScore a = slowButSure, b = slowButSure;
    a.medianMs = 900.0f;
    CHECK(recoveryBetterThan(a, b));

    RecoveryScore invalid;
    CHECK(recoveryBetterThan(slowButSure, invalid));
    CHECK(!recoveryBetterThan(invalid, slowButSure));
}
CHAD_TEST(wall_single, agrees_with_the_bracketed_solve_on_a_clean_pair) {
    // With no blockage the pair's two ends ARE the two single-beam answers, so
    // the bracketed midpoint and either single solve have to land together.
    const float half = 70.2f;
    const float truth = -31.5f;
    const float oPlus = 1.0f, oMinus = -0.5f;
    // What each beam would read from that pose.
    const float dPlus = half - truth - oPlus;
    const float dMinus = half + truth + oMinus;

    const float fromPlus = solveAxisSingle(dPlus, 1.0f, oPlus, half);
    const float fromMinus = solveAxisSingle(dMinus, -1.0f, oMinus, half);
    CHECK_NEAR(fromPlus, truth, 1e-3);
    CHECK_NEAR(fromMinus, truth, 1e-3);

    const auto pair = solveAxisBracketed(dPlus, 1.0f, oPlus,
                                         dMinus, -1.0f, oMinus, half, 1.0f);
    CHECK(pair.trusted);
    CHECK_NEAR(pair.value, truth, 1e-3);
}

CHAD_TEST(wall_single, reaches_where_an_opposing_pair_cannot) {
    // The point of the single solve. A V5 distance sensor gives up around
    // 79 in; on a 140.4 in span an opposing pair needs both readings under
    // that, which fails beyond 8.8 in from the centre line. The near wall is
    // always in reach.
    const float half = 70.2f, reach = 79.0f;
    for (const float truth : {-60.0f, -35.0f, 0.0f, 35.0f, 60.0f}) {
        const float dPlus = half - truth;
        const float dMinus = half + truth;
        const bool pairPossible = dPlus <= reach && dMinus <= reach;
        CHECK(pairPossible == (std::fabs(truth) <= 8.9f));

        // Whichever wall is nearer is readable, and solves exactly.
        if (dPlus <= dMinus) {
            CHECK(dPlus <= half);
            CHECK_NEAR(solveAxisSingle(dPlus, 1.0f, 0.0f, half), truth, 1e-3);
        } else {
            CHECK(dMinus <= half);
            CHECK_NEAR(solveAxisSingle(dMinus, -1.0f, 0.0f, half), truth, 1e-3);
        }
    }
}

CHAD_TEST(wall_single, an_off_square_robot_needs_the_projection) {
    // Same reason solveAxisBracketed takes the cosines: at 15 degrees off, a
    // 60 in reading projects to 58 in, and using the raw reading moves the
    // answer by that difference.
    const float half = 70.2f;
    const float theta = 15.0f * 3.14159265f / 180.0f;
    const float dir = std::cos(theta);
    const float reading = 60.0f;
    const float projected = solveAxisSingle(reading, dir, 0.0f, half);
    const float raw = solveAxisSingle(reading, 1.0f, 0.0f, half);
    CHECK_NEAR(projected - raw, reading * (1.0f - dir), 1e-4);
    CHECK(std::fabs(projected - raw) > 2.0f);
}

CHAD_TEST(wall_single, takes_the_mount_offset_off_the_answer) {
    // The reading is from the sensor, the pose is the body centre.
    const float half = 70.2f;
    CHECK_NEAR(solveAxisSingle(40.0f, 1.0f, 0.0f, half), 30.2, 1e-3);
    CHECK_NEAR(solveAxisSingle(40.0f, 1.0f, 6.0f, half), 24.2, 1e-3);
    CHECK_NEAR(solveAxisSingle(40.0f, -1.0f, 0.0f, half), -30.2, 1e-3);
}

CHAD_TEST(wall_single, a_blocked_beam_is_wrong_and_cannot_know_it) {
    // Stated as a test because it is the whole reason the pair exists. An
    // obstacle shortens the reading, so the answer moves by the blockage and
    // nothing in the single solve reports it.
    const float half = 70.2f;
    const float truth = -31.5f;
    const float clean = half - truth;
    const float blocked = clean - 9.0f;
    CHECK_NEAR(solveAxisSingle(clean, 1.0f, 0.0f, half), truth, 1e-3);
    CHECK_NEAR(solveAxisSingle(blocked, 1.0f, 0.0f, half), truth + 9.0, 1e-3);
}
