#include "chad/util/localization_fit.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include "chad_test.hpp"

using namespace chad::tuning;

namespace {

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
};

constexpr float kHalf = 72.0f;

}  // namespace

// ---------------------------------------------------------------------------
// Wall solving with obstacles present
// ---------------------------------------------------------------------------

CHAD_TEST(wallSolve, cleanBeamsPinTheAxis) {
    // Robot at x = 20, front mount 1.0 ahead, back mount 0.5 behind, square.
    const float cx = 20.0f, oPlus = 1.0f, oMinus = -0.5f;
    const float dPlus = kHalf - cx - oPlus;
    const float dMinus = cx + oMinus + kHalf;
    const WallSolve w = solveAxisBracketed(dPlus, 1.0f, oPlus, dMinus, -1.0f,
                                           oMinus, kHalf, 0.5f);
    CHECK(w.trusted);
    CHECK_NEAR(w.value, cx, 1e-3);
    CHECK_NEAR(w.bracketIn, 0.0, 1e-3);
}

// The placement-accuracy case. A robot put down a few degrees off square still
// solves exactly, because the readings are projected onto the axis.
CHAD_TEST(wallSolve, anOffSquareRobotStillSolvesExactly) {
    for (float deg : {0.0f, 5.0f, 10.0f, 15.0f, 25.0f}) {
        const float th = deg * 3.14159265f / 180.0f;
        const float cx = -18.0f, cy = 33.0f;
        // Mounts in robot frame, front facing +x, back facing -x.
        const float mfx = 1.0f, mfy = -3.75f;
        const float mbx = -0.5f, mby = 5.0f;

        const float oPlus = mountAlongAxis(mfx, mfy, th, true);
        const float oMinus = mountAlongAxis(mbx, mby, th, true);
        const float bxPlus = std::cos(th);
        const float bxMinus = std::cos(th + 3.14159265f);

        // Ray length from each sensor to the wall it faces.
        const float dPlus = (kHalf - (cx + oPlus)) / bxPlus;
        const float dMinus = (-kHalf - (cx + oMinus)) / bxMinus;
        CHECK(dPlus > 0.0f);
        CHECK(dMinus > 0.0f);

        const WallSolve w = solveAxisBracketed(dPlus, bxPlus, oPlus, dMinus,
                                               bxMinus, oMinus, kHalf, 0.5f);
        CHECK(w.trusted);
        CHECK_NEAR(w.value, cx, 1e-2);

        // And the pair constraint holds on the projections, for the mixture.
        const float expect = opposingPairSum(2.0f * kHalf, oPlus, oMinus);
        CHECK_NEAR(dPlus * bxPlus - dMinus * bxMinus, expect, 1e-2);
        (void)cy;
    }
}

// What the bug was: dropping the projection on an off-square robot. The
// midpoint error is 0.5*(1-cos)*(dMinus - dPlus), so it CANCELS at the centre
// of the field and only shows up off to one side -- which is why this needs
// checking somewhere other than cx = 0.
CHAD_TEST(wallSolve, ignoringTheProjectionMovesTheAnswerOffCentre) {
    const float th = 15.0f * 3.14159265f / 180.0f;
    const float oPlus = mountAlongAxis(1.0f, -3.75f, th, true);
    const float oMinus = mountAlongAxis(-0.5f, 5.0f, th, true);
    const float bxPlus = std::cos(th);
    const float bxMinus = std::cos(th + 3.14159265f);

    // At the centre the two errors are equal and opposite, so the naive solve
    // looks fine. This is the trap: testing at cx = 0 finds nothing.
    {
        const float dP = (kHalf - oPlus) / bxPlus;
        const float dM = (-kHalf - oMinus) / bxMinus;
        const WallSolve naive =
            solveAxisBracketed(dP, 1.0f, oPlus, dM, -1.0f, oMinus, kHalf, 20.0f);
        CHECK(std::fabs(naive.value) < 0.1f);
    }
    // Off to one side it does not cancel.
    const float cx = 60.0f;
    const float dPlus = (kHalf - (cx + oPlus)) / bxPlus;
    const float dMinus = (-kHalf - (cx + oMinus)) / bxMinus;
    const WallSolve good = solveAxisBracketed(dPlus, bxPlus, oPlus, dMinus,
                                              bxMinus, oMinus, kHalf, 0.5f);
    const WallSolve naive = solveAxisBracketed(dPlus, 1.0f, oPlus, dMinus,
                                               -1.0f, oMinus, kHalf, 20.0f);
    CHECK_NEAR(good.value, cx, 1e-2);
    CHECK(naive.value - cx > 1.5f);   // about +2.1 in at 15 degrees
}

// The effect that does NOT cancel anywhere, and the one that matters most:
// the bracket is offset by -(1-cos)*span whatever the pose, so the blockage
// detector reads an obstacle that is not there -- on a field that cannot be
// cleared, that is the check being poisoned.
CHAD_TEST(wallSolve, ignoringTheProjectionPoisonsTheBlockageDetector) {
    for (float deg : {5.0f, 15.0f, 25.0f}) {
        const float th = deg * 3.14159265f / 180.0f;
        const float oPlus = mountAlongAxis(1.0f, -3.75f, th, true);
        const float oMinus = mountAlongAxis(-0.5f, 5.0f, th, true);
        const float bxPlus = std::cos(th);
        const float bxMinus = std::cos(th + 3.14159265f);
        // Exactly -(sec(th) - 1) * pairSum: the ray lengths are 1/cos longer
        // than the perpendicular distances they are being mistaken for.
        const float pairSum = opposingPairSum(2.0f * kHalf, oPlus, oMinus);
        const float expected = -(1.0f / std::cos(th) - 1.0f) * pairSum;

        for (float cx : {-40.0f, 0.0f, 40.0f}) {
            const float dP = (kHalf - (cx + oPlus)) / bxPlus;
            const float dM = (-kHalf - (cx + oMinus)) / bxMinus;
            const WallSolve good = solveAxisBracketed(dP, bxPlus, oPlus, dM,
                                                      bxMinus, oMinus, kHalf,
                                                      0.5f);
            const WallSolve naive = solveAxisBracketed(dP, 1.0f, oPlus, dM,
                                                       -1.0f, oMinus, kHalf,
                                                       0.5f);
            // No obstacle anywhere, so the real bracket is zero.
            CHECK_NEAR(good.bracketIn, 0.0, 1e-2);
            CHECK(good.trusted);
            // The naive one invents a bracket, and by the same amount
            // regardless of position -- so it cannot be calibrated away.
            CHECK_NEAR(naive.bracketIn, expected, 0.05);
        }
    }
}

CHAD_TEST(wallSolve, theBracketWidthIsTheBlockage) {
    const float cx = -15.0f, oPlus = 1.0f, oMinus = -0.5f;
    // An obstacle 6 in in front of the forward beam.
    const float blockage = 6.0f;
    const float dPlus = kHalf - cx - oPlus - blockage;
    const float dMinus = cx + oMinus + kHalf;

    const WallSolve w = solveAxisBracketed(dPlus, 1.0f, oPlus, dMinus, -1.0f,
                                           oMinus, kHalf, 0.5f);
    CHECK(!w.trusted);
    CHECK_NEAR(w.bracketIn, blockage, 1e-3);
    // And the truth really is inside the bracket, on the low side here.
    const float hi = w.value + 0.5f * w.bracketIn;
    const float lo = w.value - 0.5f * w.bracketIn;
    CHECK(cx >= lo - 1e-3f);
    CHECK(cx <= hi + 1e-3f);
    // Averaging anyway -- what the naive solve does -- is off by half of it.
    CHECK_NEAR(w.value - cx, blockage * 0.5, 1e-3);
}

CHAD_TEST(wallSolve, blockageOnEitherSideIsCaughtAndBracketsTruth) {
    Rng rng(11u);
    int caught = 0, bracketed = 0, trials = 0;
    for (int k = 0; k < 200; k++) {
        const float cx = -50.0f + 100.0f * rng.uniform();
        const float oPlus = 1.0f, oMinus = -0.5f;
        const float bPlus = (k % 3 == 0) ? 4.0f + 20.0f * rng.uniform() : 0.0f;
        const float bMinus = (k % 5 == 0) ? 4.0f + 20.0f * rng.uniform() : 0.0f;
        const float dPlus = kHalf - cx - oPlus - bPlus + 0.05f * rng.gauss();
        const float dMinus = cx + oMinus + kHalf - bMinus + 0.05f * rng.gauss();

        const WallSolve w = solveAxisBracketed(dPlus, 1.0f, oPlus, dMinus,
                                               -1.0f, oMinus, kHalf, 0.5f);
        const bool blocked = (bPlus > 0.0f || bMinus > 0.0f);
        trials++;
        if (blocked && !w.trusted) caught++;
        if (blocked) {
            const float hi = w.value + 0.5f * w.bracketIn;
            const float lo = w.value - 0.5f * w.bracketIn;
            if (cx >= lo - 0.2f && cx <= hi + 0.2f) bracketed++;
        }
        // A clean pair must never be rejected.
        if (!blocked) CHECK(w.trusted);
    }
    CHECK(trials == 200);
    // Every blocked pair is both detected and bracketed -- blockage only ever
    // shortens, so the inequality is not statistical.
    CHECK(caught > 0);
    CHECK(caught == bracketed);
}

// ---------------------------------------------------------------------------
// Odometry bias
// ---------------------------------------------------------------------------

CHAD_TEST(odomBias, recoversThePropItWasGivenAtTheStatedDecorrLength) {
    // Plant: sigma(L) = prop * sqrt(L * decorr), which is what the EKF charges.
    Rng rng(7u);
    const float prop = 0.06f, decorr = 40.0f, slip = 0.05f;
    std::vector<BiasLeg> legs;
    for (int rep = 0; rep < 60; rep++) {
        for (float L : {8.0f, 16.0f, 24.0f}) {
            BiasLeg leg;
            leg.lengthIn = L;
            leg.alongErrIn = prop * std::sqrt(L * decorr) * rng.gauss();
            leg.crossErrIn = slip * std::sqrt(L * decorr) * rng.gauss();
            legs.push_back(leg);
        }
    }
    const BiasFit f = fitOdomBias(legs.data(), static_cast<int>(legs.size()),
                                  decorr);
    CHECK(f.valid);
    CHECK(f.legs == 180);
    CHECK(f.groups == 3);
    CHECK_NEAR(f.vertBiasProp, prop, 0.012);
    CHECK_NEAR(f.slipBiasProp, slip, 0.012);
    CHECK(!f.shapeSuspect);
    CHECK_NEAR(f.growthExponent, 0.5, 0.2);
}

// The reason biasDecorrIn is passed in rather than fitted: only the product is
// observable, so the same data must give the same FILTER behaviour whichever
// convention is chosen.
CHAD_TEST(odomBias, onlyTheProductWithDecorrLengthIsIdentifiable) {
    Rng rng(23u);
    std::vector<BiasLeg> legs;
    for (int rep = 0; rep < 40; rep++) {
        for (float L : {8.0f, 16.0f, 24.0f}) {
            BiasLeg leg;
            leg.lengthIn = L;
            leg.alongErrIn = 0.06f * std::sqrt(L * 40.0f) * rng.gauss();
            leg.crossErrIn = 0.05f * std::sqrt(L * 40.0f) * rng.gauss();
            legs.push_back(leg);
        }
    }
    const int n = static_cast<int>(legs.size());
    const BiasFit at40 = fitOdomBias(legs.data(), n, 40.0f);
    const BiasFit at160 = fitOdomBias(legs.data(), n, 160.0f);
    CHECK(at40.valid);
    CHECK(at160.valid);

    // Quadrupling the decorrelation length halves the prop...
    CHECK_NEAR(at160.vertBiasProp, at40.vertBiasProp * 0.5, 1e-4);
    CHECK_NEAR(at160.slipBiasProp, at40.slipBiasProp * 0.5, 1e-4);
    // ...leaving prop^2 * decorr, which is all the filter ever uses, unchanged.
    CHECK_NEAR(at160.vertBiasProp * at160.vertBiasProp * 160.0f,
               at40.vertBiasProp * at40.vertBiasProp * 40.0f, 1e-5);
    // The measured quantity itself does not depend on the convention at all.
    CHECK_NEAR(at160.alongPerRootIn, at40.alongPerRootIn, 1e-6);
    CHECK_NEAR(at160.crossPerRootIn, at40.crossPerRootIn, 1e-6);
}

CHAD_TEST(odomBias, aPurelySystematicErrorShowsUpAsTheWrongShape) {
    // Error growing linearly in L, not as sqrt(L): the filter's model does not
    // describe this robot, and the fit must say so rather than average it away.
    Rng rng(31u);
    std::vector<BiasLeg> legs;
    for (int rep = 0; rep < 40; rep++) {
        for (float L : {8.0f, 16.0f, 24.0f}) {
            BiasLeg leg;
            leg.lengthIn = L;
            leg.alongErrIn = 0.02f * L + 0.05f * rng.gauss();
            leg.crossErrIn = 0.01f * L + 0.05f * rng.gauss();
            legs.push_back(leg);
        }
    }
    const BiasFit f = fitOdomBias(legs.data(), static_cast<int>(legs.size()),
                                  40.0f);
    CHECK(f.valid);
    CHECK(f.growthExponent > 0.8f);
    CHECK(f.shapeSuspect);
}

CHAD_TEST(odomBias, rmsNotVarianceSoAConsistentOffsetIsNotSubtractedOut) {
    // Every leg errs the same way by the same amount: a pure scale error. Its
    // standard deviation is zero, so a variance-based fit would report no bias
    // at all -- which is the one answer that must not come out.
    std::vector<BiasLeg> legs;
    for (int rep = 0; rep < 30; rep++) {
        BiasLeg leg;
        leg.lengthIn = 24.0f;
        leg.alongErrIn = 1.2f;    // identical every time
        leg.crossErrIn = 0.0f;
        legs.push_back(leg);
    }
    const BiasFit f = fitOdomBias(legs.data(), static_cast<int>(legs.size()),
                                  40.0f);
    CHECK(f.valid);
    CHECK(f.vertBiasProp > 0.0f);
    // rms/sqrt(L) = 1.2/sqrt(24) = 0.2449, then /sqrt(40).
    CHECK_NEAR(f.alongPerRootIn, 1.2 / std::sqrt(24.0), 1e-4);
    CHECK_NEAR(f.vertBiasProp, (1.2 / std::sqrt(24.0)) / std::sqrt(40.0), 1e-5);
    CHECK(f.slipBiasProp == 0.0f);
}

CHAD_TEST(odomBias, signCancellationDoesNotHideTheBias) {
    // Half the legs err one way, half the other -- an out and back. The mean
    // signed error is zero, which is exactly why phase 3's out-and-back cannot
    // see this and a one-way leg can. rms still finds it.
    std::vector<BiasLeg> legs;
    for (int rep = 0; rep < 30; rep++) {
        BiasLeg a, b;
        a.lengthIn = b.lengthIn = 24.0f;
        a.alongErrIn = 1.0f;
        b.alongErrIn = -1.0f;
        legs.push_back(a);
        legs.push_back(b);
    }
    const BiasFit f = fitOdomBias(legs.data(), static_cast<int>(legs.size()),
                                  40.0f);
    CHECK(f.valid);
    CHECK_NEAR(f.alongPerRootIn, 1.0 / std::sqrt(24.0), 1e-4);
}

CHAD_TEST(odomBias, rejectsUnusableInput) {
    const BiasLeg few[] = {{24.0f, 0.5f, 0.1f}, {24.0f, 0.4f, 0.1f}};
    CHECK(!fitOdomBias(few, 2, 40.0f).valid);
    CHECK(!fitOdomBias(nullptr, 40, 40.0f).valid);

    std::vector<BiasLeg> ok(20, BiasLeg{24.0f, 0.5f, 0.1f});
    CHECK(!fitOdomBias(ok.data(), 20, 0.0f).valid);     // no decorr length
    CHECK(!fitOdomBias(ok.data(), 20, -5.0f).valid);

    // Zero-length and non-finite legs are skipped, not propagated.
    std::vector<BiasLeg> messy(ok);
    messy.push_back({0.0f, 1.0f, 1.0f});
    messy.push_back({24.0f, std::nanf(""), 0.0f});
    messy.push_back({24.0f, 0.0f, INFINITY});
    const BiasFit f = fitOdomBias(messy.data(), static_cast<int>(messy.size()),
                                  40.0f);
    CHECK(f.valid);
    CHECK(f.legs == 20);
    CHECK(std::isfinite(f.vertBiasProp));
    CHECK(std::isfinite(f.slipBiasProp));

    // One leg length only: the props still come out, the shape check cannot.
    const BiasFit one = fitOdomBias(ok.data(), 20, 40.0f);
    CHECK(one.valid);
    CHECK(one.groups == 1);
    CHECK(one.growthExponent == 0.0f);
    CHECK(!one.shapeSuspect);
}
