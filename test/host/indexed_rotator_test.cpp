#include "chad/util/indexed_rotator.hpp"

#include <cmath>
#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr float kDt = 0.01f;

// First-order motor: 5.5 W at 200 rpm free speed, ~50 ms electrical/mechanical
// lag. Zero power coasts down slower, standing in for a loose HOLD.
struct Plant {
    double pos{0.0};
    double vel{0.0};
    bool blocked{false};

    void step(float power, float dt) {
        if (blocked) { vel = 0.0; return; }
        const double target = power / 127.0 * 1200.0;
        const double tau = (power == 0.0f) ? 0.10 : 0.05;
        vel += (target - vel) * (1.0 - std::exp(-dt / tau));
        pos += vel * dt;
    }
};

struct Run {
    float minPower{1e9f};
    double maxPos{-1e9};
    int ticks{0};
};

// Steps rotator and plant until it stops being busy or maxSec runs out.
Run runUntilIdle(IndexedRotator& r, Plant& p, float maxSec) {
    Run out;
    for (float t = 0.0f; t < maxSec; t += kDt) {
        const float power = r.update(p.pos, kDt);
        if (power < out.minPower) out.minPower = power;
        p.step(power, kDt);
        if (p.pos > out.maxPos) out.maxPos = p.pos;
        ++out.ticks;
        if (!r.busy()) break;
    }
    return out;
}
}  // namespace

CHAD_TEST(indexedRotator, startsIdleAtZero) {
    IndexedRotator r;
    CHECK(!r.busy());
    CHECK(!r.stalled());
    CHECK_NEAR(r.target(), 0.0, 1e-9);
    CHECK_NEAR(r.update(0.0, kDt), 0.0, 1e-9);
}

CHAD_TEST(indexedRotator, onePressIsOneHalfTurnAndSettles) {
    IndexedRotator r;
    Plant p;
    r.advance(1);
    CHECK_NEAR(r.target(), 180.0, 1e-9);
    CHECK(r.busy());
    const Run run = runUntilIdle(r, p, 2.0f);
    CHECK(!r.busy());
    CHECK(!r.stalled());
    CHECK(run.ticks * kDt < 0.6f);
    CHECK(run.minPower >= 0.0f);
    CHECK_NEAR(p.pos, 180.0, r.config().toleranceDeg + 15.0);
    // Overshoot stays far from the next index.
    CHECK(run.maxPos - 180.0 < 20.0);
}

CHAD_TEST(indexedRotator, quickPressesAccumulate) {
    IndexedRotator r;
    Plant p;
    r.advance(1);
    for (int i = 0; i < 5; ++i) p.step(r.update(p.pos, kDt), kDt);
    r.advance(1);
    for (int i = 0; i < 5; ++i) p.step(r.update(p.pos, kDt), kDt);
    r.advance(1);
    CHECK_NEAR(r.target(), 540.0, 1e-9);
    const Run run = runUntilIdle(r, p, 3.0f);
    CHECK(!r.busy());
    CHECK(run.minPower >= 0.0f);
    CHECK(p.pos > 540.0 - r.config().toleranceDeg);
    CHECK(run.maxPos - 540.0 < 20.0);
}

CHAD_TEST(indexedRotator, ignoresNonPositiveSteps) {
    IndexedRotator r;
    r.advance(0);
    r.advance(-3);
    CHECK_NEAR(r.target(), 0.0, 1e-9);
    CHECK(!r.busy());
}

CHAD_TEST(indexedRotator, neverCommandsReverseWhenPastTarget) {
    IndexedRotator r;
    r.advance(1);
    // Somehow ended up past the target: it must leave it there.
    CHECK_NEAR(r.update(230.0, kDt), 0.0, 1e-9);
    CHECK_NEAR(r.update(260.0, kDt), 0.0, 1e-9);
    CHECK(r.update(260.0, kDt) >= 0.0f);
}

CHAD_TEST(indexedRotator, manyFlipsKeepExactTargets) {
    IndexedRotator r;
    Plant p;
    for (int i = 1; i <= 50; ++i) {
        r.advance(1);
        CHECK_NEAR(r.target(), 180.0 * i, 1e-9);
        runUntilIdle(r, p, 2.0f);
        CHECK(!r.busy());
    }
    CHECK(std::fabs(p.pos - 9000.0) < 25.0);
}

CHAD_TEST(indexedRotator, instancesAreIndependent) {
    IndexedRotator left, right;
    Plant pl, pr;
    left.advance(2);
    for (int i = 0; i < 100; ++i) {
        pl.step(left.update(pl.pos, kDt), kDt);
        const float rp = right.update(pr.pos, kDt);
        CHECK_NEAR(rp, 0.0, 1e-9);
        pr.step(rp, kDt);
    }
    CHECK_NEAR(right.target(), 0.0, 1e-9);
    CHECK(!right.busy());
    CHECK_NEAR(left.target(), 360.0, 1e-9);
}

CHAD_TEST(indexedRotator, stallStopsDrivingAndNextAdvanceRetries) {
    IndexedRotator r;
    Plant p;
    r.advance(1);
    for (int i = 0; i < 6; ++i) p.step(r.update(p.pos, kDt), kDt);
    p.blocked = true;
    float last = 1.0f;
    float t = 0.0f;
    for (; t < 1.0f && !r.stalled(); t += kDt) {
        last = r.update(p.pos, kDt);
        p.step(last, kDt);
    }
    CHECK(r.stalled());
    CHECK(t <= r.config().stallSec + 0.05f);
    CHECK(!r.busy());
    CHECK_NEAR(r.update(p.pos, kDt), 0.0, 1e-9);
    CHECK_NEAR(r.update(p.pos, kDt), 0.0, 1e-9);

    // The stalled flip is retried, not stacked on top of.
    p.blocked = false;
    r.advance(1);
    CHECK(!r.stalled());
    CHECK_NEAR(r.target(), 180.0, 1e-9);
    runUntilIdle(r, p, 2.0f);
    CHECK(!r.busy());
    CHECK(p.pos > 180.0 - r.config().toleranceDeg);
    CHECK(p.pos < 200.0);
}

CHAD_TEST(indexedRotator, nonFinitePositionOutputsZeroAndKeepsState) {
    IndexedRotator r;
    Plant p;
    r.advance(1);
    p.step(r.update(p.pos, kDt), kDt);
    CHECK_NEAR(r.update(kNaN, kDt), 0.0, 1e-9);
    CHECK_NEAR(r.update(std::numeric_limits<double>::infinity(), kDt), 0.0, 1e-9);
    CHECK_NEAR(r.target(), 180.0, 1e-9);
    CHECK(r.busy());
    // An unplugged motor is not a stalled one.
    for (int i = 0; i < 100; ++i) CHECK_NEAR(r.update(kNaN, kDt), 0.0, 1e-9);
    CHECK(!r.stalled());
    CHECK(r.update(p.pos, kDt) > 0.0f);
    runUntilIdle(r, p, 2.0f);
    CHECK(p.pos > 180.0 - r.config().toleranceDeg);
}

CHAD_TEST(indexedRotator, badDtIsHarmless) {
    IndexedRotator r;
    r.advance(1);
    const float a = r.update(0.0, std::numeric_limits<float>::quiet_NaN());
    const float b = r.update(0.0, -1.0f);
    CHECK(std::isfinite(a) && a >= 0.0f);
    CHECK(std::isfinite(b) && b >= 0.0f);
    CHECK(r.update(0.0, kDt) > 0.0f);
}

CHAD_TEST(indexedRotator, manualOverridesAFlipInProgress) {
    IndexedRotator r;
    Plant p;
    r.advance(1);
    for (int i = 0; i < 8; ++i) p.step(r.update(p.pos, kDt), kDt);
    CHECK(p.pos > 5.0 && p.pos < 175.0);

    const float m = r.manual(p.pos, 100.0f);
    CHECK_NEAR(m, 100.0, 1e-6);
    CHECK(!r.busy());
    // One direction only, even by hand.
    CHECK_NEAR(r.manual(p.pos, -60.0f), 0.0, 1e-9);
    CHECK(r.manual(p.pos, 500.0f) <= r.config().maxPower);
}

CHAD_TEST(indexedRotator, releaseHoldsWherever) {
    IndexedRotator r;
    Plant p;
    for (int i = 0; i < 30; ++i) p.step(r.manual(p.pos, 100.0f), kDt);
    const double released = p.pos;
    CHECK(released > 100.0);
    // Released mid-step: nothing drives it on to an index.
    for (int i = 0; i < 50; ++i) {
        const float power = r.update(p.pos, kDt);
        CHECK_NEAR(power, 0.0, 1e-9);
        p.step(power, kDt);
    }
    CHECK(!r.busy());
    CHECK(!r.stalled());
}

CHAD_TEST(indexedRotator, releaseJustShortOfAnIndexDoesNotCreep) {
    IndexedRotator r;
    r.manual(172.0, 100.0f);
    for (int i = 0; i < 50; ++i) CHECK_NEAR(r.update(172.0, kDt), 0.0, 1e-9);
    CHECK(!r.busy());
    // Same after a cancel mid-flip.
    IndexedRotator c;
    c.advance(1);
    c.update(175.0, kDt);
    c.cancel();
    for (int i = 0; i < 50; ++i) CHECK_NEAR(c.update(175.0, kDt), 0.0, 1e-9);
    CHECK(!c.busy());
}

CHAD_TEST(indexedRotator, manualDropsQueuedFlipsAndResyncs) {
    IndexedRotator r;
    Plant p;
    r.advance(3);
    for (int i = 0; i < 5; ++i) p.step(r.update(p.pos, kDt), kDt);
    p.step(r.manual(p.pos, 80.0f), kDt);
    // The queued 540 is gone; release does not catch up.
    for (int i = 0; i < 30; ++i) {
        const float power = r.update(p.pos, kDt);
        CHECK_NEAR(power, 0.0, 1e-9);
        p.step(power, kDt);
    }

    // Spun well past several indices by hand, then one flip goes to the next
    // index ahead -- not back to the old target, not one step per spun index.
    IndexedRotator q;
    CHECK_NEAR(q.manual(500.0, 100.0f), 100.0, 1e-6);
    q.update(500.0, kDt);
    q.advance(1);
    CHECK_NEAR(q.target(), 540.0, 1e-9);
    // Within tolerance of an index counts as on it.
    IndexedRotator s;
    s.manual(538.0, 100.0f);
    s.advance(1);
    CHECK_NEAR(s.target(), 720.0, 1e-9);
}

CHAD_TEST(indexedRotator, holdingBrakeLandsCloseToTheIndex) {
    // A plant that stops dead on zero power, as HOLD does: where it stops is
    // where the rotator let go, which must be close to the index, not at the
    // edge of the snap band.
    IndexedRotator r;
    Plant p;
    r.advance(1);
    for (int i = 0; i < 200 && r.busy(); ++i) {
        const float power = r.update(p.pos, kDt);
        if (power == 0.0f) p.vel = 0.0;
        p.step(power, kDt);
    }
    CHECK(!r.busy());
    CHECK(!r.stalled());
    CHECK(std::fabs(p.pos - 180.0) <= 3.0);
}

CHAD_TEST(indexedRotator, jammedJustShortOfTheIndexCountsAsThere) {
    IndexedRotator r;
    Plant p;
    r.advance(1);
    for (int i = 0; i < 200 && p.pos < 180.0 - 6.0; ++i) p.step(r.update(p.pos, kDt), kDt);
    p.blocked = true;
    for (int i = 0; i < 100; ++i) p.step(r.update(p.pos, kDt), kDt);
    CHECK(!r.stalled());
    CHECK(!r.busy());
    CHECK_NEAR(r.update(p.pos, kDt), 0.0, 1e-9);
    // And the next flip is the next index, not a redo of this one.
    r.advance(1);
    CHECK_NEAR(r.target(), 360.0, 1e-9);
}

namespace {
// A rotator whose shaft reads positionDeg, with nothing queued.
IndexedRotator at(double positionDeg) {
    IndexedRotator r;
    r.manual(positionDeg, 0.0f);
    r.update(positionDeg, kDt);
    return r;
}
}  // namespace

CHAD_TEST(indexedRotator, homeFromZeroDoesNotMove) {
    IndexedRotator r = at(0.0);
    r.home(2);
    CHECK_NEAR(r.target(), 0.0, 1e-9);
    CHECK_NEAR(r.update(0.0, kDt), 0.0, 1e-9);
    for (int i = 0; i < 10; ++i) r.update(0.0, kDt);
    CHECK(!r.busy());
}

CHAD_TEST(indexedRotator, homeGoesForwardToTheNextFullTurn) {
    IndexedRotator a = at(180.0);
    a.home(2);
    CHECK_NEAR(a.target(), 360.0, 1e-9);
    CHECK(a.update(180.0, kDt) > 0.0f);

    // Short of a full turn by toleranceDeg or less, or past it by
    // indexSnapDeg or less (its own overshoot), counts as home.
    IndexedRotator b = at(359.0);
    b.home(2);
    CHECK_NEAR(b.update(359.0, kDt), 0.0, 1e-9);
    IndexedRotator b2 = at(368.0);
    b2.home(2);
    CHECK_NEAR(b2.target(), 360.0, 1e-9);
    CHECK_NEAR(b2.update(368.0, kDt), 0.0, 1e-9);
    IndexedRotator b3 = at(355.0);
    b3.home(2);
    CHECK_NEAR(b3.target(), 360.0, 1e-9);
    CHECK(b3.update(355.0, kDt) > 0.0f);

    // Further past goes on round, never back.
    IndexedRotator c = at(375.0);
    c.home(2);
    CHECK_NEAR(c.target(), 720.0, 1e-9);
    CHECK(c.update(375.0, kDt) > 0.0f);

    IndexedRotator d = at(-90.0);
    d.home(2);
    CHECK_NEAR(d.target(), 0.0, 1e-9);
}

CHAD_TEST(indexedRotator, homeDrivesThePlantThereAndRepeatsAreIdempotent) {
    IndexedRotator r;
    Plant p;
    r.advance(1);
    runUntilIdle(r, p, 2.0f);
    p.vel = 0.0;  // the wrapper brakes HOLD as soon as it goes idle
    r.home(2);
    const Run run = runUntilIdle(r, p, 2.0f);
    CHECK(run.minPower >= 0.0f);
    CHECK(!r.busy());
    CHECK(p.pos > 360.0 - r.config().toleranceDeg);
    p.vel = 0.0;
    // Nothing drives it once there.
    for (int i = 0; i < 100; ++i) {
        const float power = r.update(p.pos, kDt);
        CHECK_NEAR(power, 0.0, 1e-9);
        p.step(power, kDt);
    }
    CHECK(p.pos < 360.0 + r.config().indexSnapDeg);
    const double homed = p.pos;
    for (int k = 0; k < 3; ++k) {
        r.home(2);
        CHECK_NEAR(r.target(), 360.0, 1e-9);
        for (int i = 0; i < 20; ++i) {
            const float power = r.update(p.pos, kDt);
            CHECK_NEAR(power, 0.0, 1e-9);
            p.step(power, kDt);
        }
    }
    CHECK_NEAR(p.pos, homed, 0.5);
}

CHAD_TEST(indexedRotator, homePowerCapAppliesToThatMoveOnly) {
    IndexedRotator r = at(180.0);
    r.home(2, 63.0f);
    CHECK_NEAR(r.target(), 360.0, 1e-9);
    float peak = 0.0f;
    for (int i = 0; i < 10; ++i) {
        const float u = r.update(180.0 + 5.0 * i, kDt);
        if (u > peak) peak = u;
    }
    CHECK(peak > 0.0f);
    CHECK(peak <= 63.0f);
    // The next flip is back at full power.
    r.advance(1);
    CHECK(r.update(230.0, kDt) > 63.0f);
}

CHAD_TEST(indexedRotator, flipPowerCapAppliesToThatFlipOnly) {
    IndexedRotator r;
    Plant p;
    r.advance(1, 63.0f);
    float peak = 0.0f;
    for (float t = 0.0f; t < 2.0f && r.busy(); t += kDt) {
        const float u = r.update(p.pos, kDt);
        if (u > peak) peak = u;
        p.step(u, kDt);
    }
    CHECK(!r.busy());
    CHECK(peak > 0.0f);
    CHECK(peak <= 63.0f);
    CHECK_NEAR(p.pos, 180.0, r.config().toleranceDeg + 15.0);
    // Zero or a non-positive cap means full power.
    r.advance(1, 0.0f);
    CHECK(r.update(p.pos, kDt) > 63.0f);
}

CHAD_TEST(indexedRotator, homeOverridesQueuedFlips) {
    IndexedRotator r = at(0.0);
    r.advance(3);
    r.home(2);
    CHECK_NEAR(r.target(), 0.0, 1e-9);
}

CHAD_TEST(indexedRotator, manualCancelsAHomeAndHomeWorksAfterManual) {
    IndexedRotator r;
    Plant p;
    r.manual(0.0, 0.0f);
    p.pos = 100.0;
    r.update(p.pos, kDt);
    r.home(2);
    for (int i = 0; i < 5; ++i) p.step(r.update(p.pos, kDt), kDt);
    CHECK(r.busy());
    p.step(r.manual(p.pos, 100.0f), kDt);
    CHECK(!r.busy());
    for (int i = 0; i < 40; ++i) p.step(r.manual(p.pos, 100.0f), kDt);
    const double released = p.pos;
    CHECK(released > 360.0);
    for (int i = 0; i < 20; ++i) CHECK_NEAR(r.update(p.pos, kDt), 0.0, 1e-9);
    r.home(2);
    const double expect = 360.0 * std::ceil((p.pos - r.config().indexSnapDeg) / 360.0);
    CHECK_NEAR(r.target(), expect, 1e-9);
    runUntilIdle(r, p, 2.0f);
    CHECK(p.pos > expect - r.config().toleranceDeg);
    CHECK(p.pos < expect + 20.0);
}

CHAD_TEST(indexedRotator, resetRebases) {
    IndexedRotator r;
    r.advance(4);
    r.reset(1000.0);
    CHECK_NEAR(r.target(), 1000.0, 1e-9);
    CHECK(!r.busy());
    r.advance(1);
    CHECK_NEAR(r.target(), 1180.0, 1e-9);
}
