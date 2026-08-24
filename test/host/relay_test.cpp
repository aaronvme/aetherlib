#include "chad/util/relay_core.hpp"

#include <cmath>
#include <cstdio>

#include "chad_test.hpp"

using chad::tuning::RelayFit;
using chad::tuning::RelayIdentifier;
using chad::tuning::RelayIdentifyConfig;
using chad::tuning::RelayOutcome;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Heading: angular velocity is first order in command, heading integrates it.
// G(s) = K / (s (tau s + 1)), the shape the header describes -- a lag feeding an
// integrator, so the phase never reaches -180 and the plant has no ultimate
// frequency of its own.
struct Heading {
    static constexpr double kK = 0.0494;   // rad/s per motor unit, steady state
    static constexpr double kTau = 0.15;   // s

    double theta = 0.0;
    double omega = 0.0;

    void step(double cmd, double dt) {
        constexpr int kSub = 10;
        const double h = dt / kSub;
        for (int i = 0; i < kSub; i++) {
            omega += ((kK * cmd) - omega) / kTau * h;
            theta += omega * h;
        }
    }
};

// The limit cycle of a hysteresis relay round G(s) = K/(s(tau s + 1)) solves
// sin^2(phi) tan(phi) = pi eps / (4 d K tau), where phi = asin(eps/a) is how far
// the identified point sits from -180 degrees. Everything else follows from phi,
// so this reference comes from the describing function, not from the identifier.
struct Predicted {
    double phiRad, omega, periodSec, amplitude, ku;
};

Predicted predict(double eps, double d) {
    const double target = kPi * eps / (4.0 * d * Heading::kK * Heading::kTau);
    double lo = 1e-6, hi = kPi / 2.0 - 1e-6;
    for (int i = 0; i < 200; i++) {
        const double mid = 0.5 * (lo + hi);
        const double f = std::sin(mid) * std::sin(mid) * std::tan(mid);
        if (f < target) lo = mid; else hi = mid;
    }
    Predicted p;
    p.phiRad = 0.5 * (lo + hi);
    p.omega = (1.0 / std::tan(p.phiRad)) / Heading::kTau;
    p.periodSec = 2.0 * kPi / p.omega;
    p.amplitude = eps / std::sin(p.phiRad);
    p.ku = 4.0 * d / (kPi * p.amplitude);
    return p;
}

struct Result {
    RelayOutcome outcome;
    RelayFit fit;
    int switches;
};

Result identify(const RelayIdentifyConfig& cfg, double timeoutMs = 8000.0,
                double sampleMs = 10.0) {
    RelayIdentifier id(cfg);
    Heading plant;
    double cmd = 0.0;
    for (double t = 0.0; t < timeoutMs; t += sampleMs) {
        plant.step(cmd, sampleMs / 1000.0);
        cmd = id.update(static_cast<float>(t + sampleMs),
                        static_cast<float>(plant.theta));
        if (id.outcome() != RelayOutcome::Cycling) break;
    }
    id.finish();
    return {id.outcome(), id.fit(), id.switches()};
}

RelayIdentifyConfig shipped(double hystDeg) {
    RelayIdentifyConfig c;
    c.hysteresis = static_cast<float>(hystDeg * kPi / 180.0);
    c.amplitude = 40.0f;
    c.abort = static_cast<float>(90.0 * kPi / 180.0);
    c.cycles = 4;
    c.minHalfPeriodMs = 60.0f;
    return c;
}

}  // namespace

CHAD_TEST(relay, identifiesThePointTheDescribingFunctionPredicts) {
    const RelayIdentifyConfig cfg = shipped(0.5);
    const Predicted p = predict(cfg.hysteresis, cfg.amplitude);
    const Result r = identify(cfg);
    CHECK(r.outcome == RelayOutcome::Fitted);
    // Sampling at 10 ms delays every switch, so the cycle runs slower than the
    // continuous prediction rather than matching it.
    CHECK(r.fit.tu > p.periodSec);
    CHECK_NEAR(r.fit.tu, p.periodSec, 0.20 * p.periodSec);
}

// The identifier measures the peak; the describing function is written on the
// first harmonic, which is smaller. Both are right, and the gap is the reason
// kP comes out low -- so pin the gap rather than pretend it is zero.
CHAD_TEST(relay, thePeakExceedsTheFirstHarmonicAmplitude) {
    const RelayIdentifyConfig cfg = shipped(0.5);
    const Predicted p = predict(cfg.hysteresis, cfg.amplitude);
    const Result r = identify(cfg);
    CHECK(r.outcome == RelayOutcome::Fitted);
    const double peakOverFundamental = r.fit.amplitude / p.amplitude;
    // A triangle wave gives pi^2/8 = 1.234; this plant measures 1.28.
    CHECK(peakOverFundamental > 1.15);
    CHECK(peakOverFundamental < 1.40);
    // ku divides by the amplitude, so it is low by the same factor.
    CHECK_NEAR(r.fit.ku * peakOverFundamental, p.ku, 0.02 * p.ku);
    CHECK(r.fit.ku < 0.85 * p.ku);
}

// ku and tu are named for the critical point but are not measured there: the
// dead band puts them asin(hysteresis/amplitude) away from it.
CHAD_TEST(relay, theIdentifiedPointIsNotTheCriticalPoint) {
    std::printf("    %8s %10s %10s %9s %9s %9s\n", "hyst deg", "phase deg",
                "swing/eps", "half ms", "kP", "kD");
    for (double h : {0.25, 0.5, 1.0, 2.0, 4.0}) {
        const RelayIdentifyConfig cfg = shipped(h);
        const Predicted p = predict(cfg.hysteresis, cfg.amplitude);
        const Result r = identify(cfg);
        std::printf("    %8.2f %10.1f %10.2f %9.1f %9.1f %9.3f\n", h,
                    p.phiRad * 180.0 / kPi, r.fit.amplitude / cfg.hysteresis,
                    500.0 * r.fit.tu, r.fit.kP, r.fit.kD);
        // A dead band this wide against a 40 unit relay output falls outside
        // the phase budget, and is refused rather than fitted.
        const bool wide = h >= 2.0;
        CHECK((r.outcome == (wide ? RelayOutcome::Chatter
                                  : RelayOutcome::Fitted)));
        CHECK(p.phiRad > 0.0);
    }
    // Wider dead band, lower excited frequency, longer period, worse phase.
    const Predicted narrow = predict(shipped(0.25).hysteresis, 40.0);
    const Predicted wide = predict(shipped(4.0).hysteresis, 40.0);
    CHECK(wide.phiRad > narrow.phiRad);
    CHECK(wide.periodSec > narrow.periodSec);
}

// minSwingRatio is a phase budget: the swing in dead bands is 1/sin(phi), so
// accepting 1.3 accepts a point 80 degrees from where the rule applies.
CHAD_TEST(relay, theSwingGateBoundsThePhaseError) {
    const double at13 = std::asin(1.28 / 1.3) * 180.0 / kPi;
    const double at30 = std::asin(1.28 / 3.0) * 180.0 / kPi;
    CHECK(at13 > 79.0);
    CHECK(at30 > 24.0 && at30 < 26.0);

    RelayIdentifyConfig cfg = shipped(0.5);
    const double measured = identify(cfg).fit.amplitude / cfg.hysteresis;
    // The shipped dead band and relay output clear the 3.0 default with half
    // again to spare, so tightening from 1.3 does not reject what ships.
    CHECK(measured > 4.5);
    CHECK(identify(cfg).outcome == RelayOutcome::Fitted);
    // A budget the swing cannot meet is refused rather than fitted.
    cfg.minSwingRatio = static_cast<float>(measured * 1.05);
    CHECK(identify(cfg).outcome == RelayOutcome::Chatter);
}

CHAD_TEST(relay, aRunawayIsReportedNotFitted) {
    const RelayIdentifyConfig cfg = shipped(0.5);
    // Positive feedback: the relay reinforces the displacement instead of
    // opposing it, which is the wiring mistake the abort band exists for.
    RelayIdentifier id(cfg);
    Heading plant;
    double cmd = 0.0;
    for (double t = 0.0; t < 8000.0; t += 10.0) {
        plant.step(cmd, 0.01);
        cmd = -id.update(static_cast<float>(t + 10.0),
                         static_cast<float>(plant.theta));
        if (id.outcome() != RelayOutcome::Cycling) break;
    }
    CHECK(id.outcome() == RelayOutcome::RanAway);
    CHECK(!id.fit().valid);
}

CHAD_TEST(relay, aPeriodPinnedAtTheDwellFloorIsRefused) {
    RelayIdentifyConfig cfg = shipped(0.5);
    // The half period here is about 140 ms, so a floor above it makes every
    // switch wait on the dwell and the period measures the floor.
    cfg.minHalfPeriodMs = 400.0f;
    const Result r = identify(cfg);
    CHECK(r.outcome == RelayOutcome::DwellFloor);
    CHECK(!r.fit.valid);
}

CHAD_TEST(relay, theTransientHalfPeriodsAreDiscarded) {
    const RelayIdentifyConfig cfg = shipped(0.5);
    const Result r = identify(cfg);
    CHECK(r.outcome == RelayOutcome::Fitted);
    CHECK(r.fit.cycles == cfg.cycles);
    CHECK(r.switches == cfg.cycles * 2 + 2);
}

CHAD_TEST(relay, gainsFollowTyreusLuyben) {
    const RelayIdentifyConfig cfg = shipped(0.5);
    const Result r = identify(cfg);
    CHECK(r.outcome == RelayOutcome::Fitted);
    // kP = ku/2.2, Td = Tu/6.3, and no integral by default.
    CHECK_NEAR(r.fit.kP, 0.45 * r.fit.ku, 1e-4 * r.fit.ku);
    CHECK_NEAR(r.fit.kD, r.fit.kP * r.fit.tu / 6.3, 1e-5);
    CHECK(r.fit.kI == 0.0f);

    RelayIdentifyConfig withI = cfg;
    withI.includeIntegral = true;
    const Result ri = identify(withI);
    CHECK_NEAR(ri.fit.kI, ri.fit.kP / (2.2 * ri.fit.tu), 1e-5);
}

CHAD_TEST(relay, noLimitCycleIsReportedNotFitted) {
    RelayIdentifyConfig cfg = shipped(0.5);
    // Too little output to cross the dead band inside the window.
    cfg.amplitude = 0.02f;
    const Result r = identify(cfg, 2000.0);
    CHECK(r.outcome == RelayOutcome::NoCycle);
    CHECK(!r.fit.valid);
}
