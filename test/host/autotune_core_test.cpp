#include "chad/util/autotune_core.hpp"

#include <cmath>
#include <vector>

#include "chad_test.hpp"

using namespace chad;
using namespace chad::tuning;

namespace {

constexpr float kDt = 0.01f;
constexpr float kTrialS = 1.5f;

// PID -> motor power -> first-order velocity plant. The derivative filter is
// modelled because PID::update filters it, and without that a large kD chatters
// on a 10 ms grid: 54 zero crossings on an otherwise settled trace, which trips
// oscillationLimit and stops the ladder for a reason that does not exist on the
// robot.
struct Plant {
    float kS, kV, kA, derivFilter;
    float sat{127.0f};

    void run(PIDGains g, float target, std::vector<float>& t,
             std::vector<float>& err) const {
        t.clear();
        err.clear();
        float pos = 0.0f, vel = 0.0f, integ = 0.0f, prevE = target, fd = 0.0f;
        const int n = static_cast<int>(kTrialS / kDt);
        for (int k = 0; k < n; k++) {
            const float e = target - pos;
            const float rawD = (e - prevE) / kDt;
            fd = derivFilter * rawD + (1.0f - derivFilter) * fd;
            prevE = e;
            integ += e * kDt;

            float power = g.kP * e + g.kI * integ + g.kD * fd;
            if (power > sat) power = sat;
            if (power < -sat) power = -sat;
            const float volts = power * 12.0f / 127.0f;

            const float fric = vel > 1e-6f ? kS : (vel < -1e-6f ? -kS : 0.0f);
            float acc = 0.0f;
            if (!(std::fabs(vel) < 1e-6f && std::fabs(volts) < kS)) {
                acc = (volts - fric - kV * vel) / kA;
            }
            vel += acc * kDt;
            pos += vel * kDt;
            t.push_back(static_cast<float>(k) * kDt * 1000.0f);
            err.push_back(e);
        }
    }
};

StepWeights weights;

StepMetrics measure(const Plant& p, PIDGains g, float step) {
    std::vector<float> t, e;
    p.run(g, step, t, e);
    return analyseStep(t, e, step, weights);
}

// Run the whole search against a plant, the way the auton does.
struct SearchResult {
    PIDGains gains;
    float cost;
    int trials;
};

SearchResult runSearch(const Plant& p, float step, float kpStart, float seed) {
    GainSearchConfig cfg;
    cfg.kdSeedRatio = seed;
    GainSearch s;
    s.reset(cfg, {kpStart, 0.0f, 0.0f});
    s.beginStep();
    int guard = 0;
    while (!s.stepDone() && guard++ < 200) {
        const TrialRequest r = s.request();
        s.submit(measure(p, r.gains, step));
    }
    const StepMetrics m = measure(p, s.best(), step);
    return {s.best(), m.valid ? m.cost : 1e30f, s.trials()};
}

// Exhaustive grid, for something to be excessive against.
float gridOptimum(const Plant& p, float step, float kpLo, float kpHi) {
    float best = 1e30f;
    for (int i = 0; i < 28; i++) {
        const float f = static_cast<float>(i) / 27.0f;
        const float kp = kpLo * std::pow(kpHi / kpLo, f);
        for (int j = 0; j < 13; j++) {
            const float ratio = 0.6f * static_cast<float>(j) / 12.0f;
            const StepMetrics m = measure(p, {kp, 0.0f, kp * ratio}, step);
            if (m.valid && m.cost < best) best = m.cost;
        }
    }
    return best;
}

const Plant kAngular{1.0f, 0.040f, 0.040f * 0.15f, 0.5f};
const Plant kLateral{0.85f, 0.185f, 0.185f * 0.15f, 0.6f};

}  // namespace

// ---------------------------------------------------------------------------
// The finding this file exists for
// ---------------------------------------------------------------------------

CHAD_TEST(autotuneCore, carryingDerivativeLetsTheKpLadderClimb) {
    // Without the seed the ladder is stopped by the overshoot of an undamped
    // response on its very first rung, so kP never leaves where it started.
    const SearchResult bare = runSearch(kAngular, 90.0f, 20.0f, 0.0f);
    const SearchResult seeded = runSearch(kAngular, 90.0f, 20.0f, 0.075f);

    CHECK(bare.gains.kP <= 20.0f * 1.001f);     // stuck on rung 1
    CHECK(seeded.gains.kP > 20.0f * 1.4f);      // climbed at least one rung
    CHECK(seeded.cost < bare.cost);
}

CHAD_TEST(autotuneCore, theSeedBeatsNoSeedAgainstAnExhaustiveGrid) {
    struct Case { const Plant& p; float step, kpStart, kpLo, kpHi; };
    const Case cases[] = {
        {kAngular, 90.0f, 20.0f, 5.0f, 900.0f},
        {kAngular, 180.0f, 20.0f, 5.0f, 900.0f},
        {kLateral, 24.0f, 4.0f, 1.0f, 250.0f},
        {kLateral, 48.0f, 4.0f, 1.0f, 250.0f},
    };
    int better = 0;
    double sumBare = 0.0, sumSeeded = 0.0;
    for (const Case& c : cases) {
        const float grid = gridOptimum(c.p, c.step, c.kpLo, c.kpHi);
        CHECK(grid < 1e29f);
        const SearchResult bare = runSearch(c.p, c.step, c.kpStart, 0.0f);
        const SearchResult seeded = runSearch(c.p, c.step, c.kpStart, 0.075f);
        const double exBare = 100.0 * (bare.cost - grid) / grid;
        const double exSeeded = 100.0 * (seeded.cost - grid) / grid;
        sumBare += exBare;
        sumSeeded += exSeeded;
        if (exSeeded < exBare) better++;
        // Neither should ever beat an exhaustive grid.
        CHECK(exBare > -1.0);
        CHECK(exSeeded > -1.0);
    }
    CHECK(better >= 3);
    CHECK(sumSeeded < sumBare);
    // Measured across 80 plant/step combinations the means are +120% and +39%;
    // on these four the seeded mean must at least stay under half the bare one.
    CHECK(sumSeeded < 0.5 * sumBare);
}

CHAD_TEST(autotuneCore, aSeedOfZeroReproducesTheOldSearchExactly) {
    // The kP ladder asks for kD = 0, and the kD phase starts at kdRatioStep --
    // so seed 0 is the pre-existing behaviour, not merely similar to it.
    GainSearchConfig cfg;
    cfg.kdSeedRatio = 0.0f;
    GainSearch s;
    s.reset(cfg, {20.0f, 0.0f, 0.0f});
    s.beginStep();
    CHECK(s.request().gains.kD == 0.0f);
    CHECK(s.request().phase == GainPhase::Kp);

    // Force the ladder to stop immediately with a big overshoot.
    StepMetrics m;
    m.valid = true;
    m.overshootPct = 50.0f;
    m.cost = 10.0f;
    s.submit(m);
    CHECK(s.phase() == GainPhase::Kd);
    CHECK_NEAR(s.request().gains.kD, 20.0 * cfg.kdRatioStep, 1e-5);
}

// ---------------------------------------------------------------------------
// Search mechanics
// ---------------------------------------------------------------------------

CHAD_TEST(autotuneCore, kpCarriesForwardAsAFloorAcrossStepSizes) {
    GainSearchConfig cfg;
    GainSearch s;
    s.reset(cfg, {10.0f, 0.0f, 0.0f});

    // First step size: let it climb twice, then stop.
    s.beginStep();
    StepMetrics good;
    good.valid = true;
    good.cost = 5.0f;
    s.submit(good);            // rung 1 accepted, climbs
    good.cost = 4.0f;
    s.submit(good);            // rung 2 accepted, climbs
    StepMetrics stop = good;
    stop.overshootPct = 40.0f;
    stop.cost = 99.0f;
    s.submit(stop);            // rung 3 rejected on overshoot
    const float carried = s.best().kP;
    CHECK(carried > 10.0f);

    // Second step size must not start below what the first established.
    s.beginStep();
    CHECK_NEAR(s.request().gains.kP, carried, 1e-4);
    StepMetrics worse;
    worse.valid = true;
    worse.cost = 1000.0f;
    worse.overshootPct = 90.0f;
    s.submit(worse);           // first rung awful, ladder stops
    CHECK(s.best().kP >= carried - 1e-4f);
}

CHAD_TEST(autotuneCore, bestMetricsIsClearedSoTheKiGateCannotReadTheLastStep) {
    GainSearchConfig cfg;
    cfg.includeIntegral = true;
    cfg.steadyStateTol = 1.0f;
    GainSearch s;
    s.reset(cfg, {10.0f, 0.0f, 0.0f});

    s.beginStep();
    StepMetrics big;
    big.valid = true;
    big.cost = 5.0f;
    big.steadyState = 20.0f;      // huge steady-state error at this amplitude
    big.overshootPct = 40.0f;     // stops the ladder at once
    s.submit(big);
    CHECK(s.bestMetrics().valid);
    CHECK(s.bestMetrics().steadyState == 20.0f);

    s.beginStep();
    // Cleared, so nothing from the previous amplitude can gate anything.
    CHECK(!s.bestMetrics().valid);
    CHECK(s.bestMetrics().steadyState == 0.0f);
}

CHAD_TEST(autotuneCore, anInvalidTrialAdvancesInsteadOfLoopingForever) {
    GainSearchConfig cfg;
    GainSearch s;
    s.reset(cfg, {10.0f, 0.0f, 0.0f});
    s.beginStep();

    const StepMetrics dead;   // valid == false: the trial never ran
    CHECK(s.phase() == GainPhase::Kp);
    s.submit(dead);
    CHECK(s.phase() == GainPhase::Kd);
    s.submit(dead);
    CHECK(s.stepDone());
    // And the search still yields usable gains rather than zeros.
    CHECK(s.best().kP >= 10.0f);
    CHECK(s.best().kD > 0.0f);
}

CHAD_TEST(autotuneCore, theKdPhaseStopsAtTheFirstStepThatDoesNotHelp) {
    GainSearchConfig cfg;
    GainSearch s;
    s.reset(cfg, {10.0f, 0.0f, 0.0f});
    s.beginStep();

    StepMetrics m;
    m.valid = true;
    m.cost = 10.0f;
    m.overshootPct = 40.0f;
    s.submit(m);                       // ladder stops, enter kD
    CHECK(s.phase() == GainPhase::Kd);
    const float seeded = s.best().kD;
    CHECK(seeded > 0.0f);

    m.overshootPct = 0.0f;
    m.cost = 8.0f;
    s.submit(m);                       // helps, kD moves up
    CHECK(s.best().kD > seeded);
    const float improved = s.best().kD;

    m.cost = 9.0f;
    s.submit(m);                       // does not help, stop here
    CHECK(s.stepDone());
    CHECK_NEAR(s.best().kD, improved, 1e-5);
}

CHAD_TEST(autotuneCore, kiIsSkippedUnlessAskedForAndNeeded) {
    // Off by default, even with steady-state error left over.
    {
        GainSearchConfig cfg;
        GainSearch s;
        s.reset(cfg, {10.0f, 0.0f, 0.0f});
        s.beginStep();
        StepMetrics m;
        m.valid = true;
        m.cost = 10.0f;
        m.overshootPct = 40.0f;
        m.steadyState = 50.0f;
        s.submit(m);
        m.cost = 99.0f;
        s.submit(m);          // kD fails to help -> would enter kI if enabled
        CHECK(s.stepDone());
        CHECK(s.best().kI == 0.0f);
    }
    // On, but the error is already inside tolerance: still skipped.
    {
        GainSearchConfig cfg;
        cfg.includeIntegral = true;
        cfg.steadyStateTol = 1.0f;
        GainSearch s;
        s.reset(cfg, {10.0f, 0.0f, 0.0f});
        s.beginStep();
        StepMetrics m;
        m.valid = true;
        m.cost = 10.0f;
        m.overshootPct = 40.0f;
        m.steadyState = 0.2f;
        s.submit(m);
        m.cost = 99.0f;
        s.submit(m);
        CHECK(s.stepDone());
        CHECK(s.best().kI == 0.0f);
    }
    // On, and the error is real: the phase runs.
    {
        GainSearchConfig cfg;
        cfg.includeIntegral = true;
        cfg.steadyStateTol = 1.0f;
        GainSearch s;
        s.reset(cfg, {10.0f, 0.0f, 0.0f});
        s.beginStep();
        StepMetrics m;
        m.valid = true;
        m.cost = 10.0f;
        m.overshootPct = 40.0f;
        m.steadyState = 5.0f;
        s.submit(m);
        m.cost = 99.0f;
        s.submit(m);
        CHECK(s.phase() == GainPhase::Ki);
        CHECK(s.request().gains.kI > 0.0f);
    }
}

CHAD_TEST(autotuneCore, metricsAverageLeavesRejectedRepsOut) {
    MetricsAverage avg;
    avg.reset();

    StepMetrics a;
    a.valid = true;
    a.cost = 10.0f;
    a.overshootPct = 4.0f;
    a.settleMs = 500.0f;
    a.oscillations = 2;
    avg.add(a);

    StepMetrics rejected;      // never ran; initial is the only clue why
    rejected.initial = 3.5f;
    avg.add(rejected);

    StepMetrics b = a;
    b.cost = 20.0f;
    b.overshootPct = 6.0f;
    b.settleMs = 700.0f;
    b.oscillations = 4;
    avg.add(b);

    const StepMetrics r = avg.result();
    CHECK(r.valid);
    CHECK(avg.accepted() == 2);
    CHECK_NEAR(r.cost, 15.0, 1e-4);            // not 10.0 -- the zero is absent
    CHECK_NEAR(r.overshootPct, 5.0, 1e-4);
    CHECK_NEAR(r.settleMs, 600.0, 1e-3);
    CHECK(r.oscillations == 3);
}

CHAD_TEST(autotuneCore, everyRepRejectedKeepsTheFirstOpeningError) {
    MetricsAverage avg;
    avg.reset();
    StepMetrics r1, r2;
    r1.initial = 0.4f;
    r2.initial = 9.9f;
    avg.add(r1);
    avg.add(r2);
    const StepMetrics r = avg.result();
    CHECK(!r.valid);
    CHECK(avg.accepted() == 0);
    CHECK_NEAR(r.initial, 0.4, 1e-5);   // the first, not the last
}

// ---------------------------------------------------------------------------
// analyseStep had no test at all before this
// ---------------------------------------------------------------------------

CHAD_TEST(autotuneCore, analyseStepRejectsATraceThatNeverMoved) {
    // A trial that never ran must not score as a flawless response.
    std::vector<float> t, e;
    for (int i = 0; i < 50; i++) {
        t.push_back(static_cast<float>(i) * 10.0f);
        e.push_back(0.05f);
    }
    const StepMetrics m = analyseStep(t, e, 90.0f, weights);
    CHECK(!m.valid);
    CHECK_NEAR(m.initial, 0.05, 1e-6);   // set before the guard, so it explains

    CHECK(!analyseStep({}, {}, 90.0f, weights).valid);
    CHECK(!analyseStep({0.0f, 1.0f}, {90.0f, 80.0f}, 90.0f, weights).valid);
    // Mismatched lengths and a non-positive step.
    CHECK(!analyseStep({0, 1, 2, 3}, {90.0f, 80.0f, 70.0f}, 90.0f, weights).valid);
    CHECK(!analyseStep({0, 1, 2, 3}, {90.0f, 80.0f, 70.0f, 0.0f}, 0.0f, weights).valid);
}

CHAD_TEST(autotuneCore, analyseStepMeasuresOvershootAndSettleOnAKnownTrace) {
    // Error goes 90 -> -9 (10% overshoot) -> 0 and stays.
    std::vector<float> t, e;
    for (int i = 0; i < 100; i++) t.push_back(static_cast<float>(i) * 10.0f);
    for (int i = 0; i < 100; i++) {
        if (i < 30) e.push_back(90.0f - 3.0f * static_cast<float>(i));
        else if (i < 40) e.push_back(-9.0f);
        else e.push_back(0.0f);
    }
    const StepMetrics m = analyseStep(t, e, 90.0f, weights);
    CHECK(m.valid);
    CHECK_NEAR(m.overshootPct, 10.0, 0.5);
    CHECK_NEAR(m.steadyState, 0.0, 1e-5);
    CHECK_NEAR(m.settleMs, 400.0, 15.0);   // last exit from the 2% band
    CHECK(m.oscillations >= 1);
    CHECK(m.cost > 0.0f);
}
