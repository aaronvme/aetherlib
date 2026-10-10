#include "chad/util/grab_trigger.hpp"

#include <cstdint>
#include <functional>
#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr std::uint32_t kPollMs = 5;
constexpr std::uint32_t kSensorMs = 33;

GrabTrigger::Config config() {
    GrabTrigger::Config c;
    c.grabRangeIn = 4.5f;
    c.leadMs = 50.0f;
    c.samplePeriodMs = 40;
    return c;
}

using Truth = std::function<float(std::uint32_t)>;

// Polls a sensor that refreshes every kSensorMs (first refresh at phaseMs)
// and holds its value in between, the robot closing on the object at
// closing in/s. Returns the poll time the trigger fired at, or -1.
long runUntilFire(GrabTrigger& t, const Truth& truth, float closing,
                  std::uint32_t phaseMs, std::uint32_t endMs,
                  const Truth& noise = [](std::uint32_t) { return 0.0f; }) {
    float held = kNaN;
    std::uint32_t nextSample = phaseMs;
    for (std::uint32_t now = 0; now <= endMs; now += kPollMs) {
        while (nextSample <= now) {
            held = truth(nextSample) + noise(nextSample);
            nextSample += kSensorMs;
        }
        if (t.update(now, held, closing)) return static_cast<long>(now);
    }
    return -1;
}

Truth approach(float startIn, float inPerSec) {
    return [=](std::uint32_t ms) { return startIn - inPerSec * 0.001f * static_cast<float>(ms); };
}
}  // namespace

CHAD_TEST(grabTrigger, oneNearSampleIsNotEnough) {
    GrabTrigger t(config());
    CHECK(!t.update(0, kNaN, 0.0f));
    CHECK(!t.update(5, 4.0f, 0.0f));
    CHECK(!t.update(10, 4.0f, 0.0f));  // the same physical sample read again
}

CHAD_TEST(grabTrigger, objectSittingInRangeFiresOnItsSecondSample) {
    GrabTrigger t(config());
    const long fired = runUntilFire(t, [](std::uint32_t) { return 4.0f; }, 0.0f, 0, 500);
    CHECK(fired >= 0);
    CHECK(fired <= static_cast<long>(config().samplePeriodMs + kPollMs));
}

CHAD_TEST(grabTrigger, objectSittingOutOfRangeNeverFires) {
    GrabTrigger t(config());
    CHECK_EQ(runUntilFire(t, [](std::uint32_t) { return 6.0f; }, 0.0f, 0, 2000), -1L);
}

CHAD_TEST(grabTrigger, oneSpikeInASteadyTrackDoesNotFire) {
    GrabTrigger t(config());
    const Truth truth = [](std::uint32_t ms) { return (ms >= 99 && ms < 132) ? 3.0f : 20.0f; };
    CHECK_EQ(runUntilFire(t, truth, 0.0f, 0, 600), -1L);
}

CHAD_TEST(grabTrigger, spikeAheadOfAnApproachDoesNotFire) {
    // The robot closes at 30 in/s from 20 in; one sample claims 3 in.
    GrabTrigger t(config());
    const Truth base = approach(20.0f, 30.0f);
    const Truth truth = [base](std::uint32_t ms) { return (ms >= 99 && ms < 132) ? 3.0f : base(ms); };
    const long fired = runUntilFire(t, truth, 30.0f, 0, 600);
    CHECK(fired < 0 || base(static_cast<std::uint32_t>(fired)) < 8.0f);
}

CHAD_TEST(grabTrigger, recedingObjectNeverFires) {
    GrabTrigger t(config());
    CHECK_EQ(runUntilFire(t, approach(6.0f, -40.0f), -40.0f, 0, 600), -1L);
}

CHAD_TEST(grabTrigger, fastApproachFiresAheadByTheLead) {
    // About this drive's top speed. Whatever the sensor's phase, the object
    // must reach the jaws when the valve finishes, not inches after.
    const GrabTrigger::Config c = config();
    const Truth truth = approach(30.0f, 60.0f);
    for (std::uint32_t phase = 0; phase < kSensorMs; phase += 3) {
        GrabTrigger t(c);
        const long fired = runUntilFire(t, truth, 60.0f, phase, 1000);
        CHECK(fired >= 0);
        const float atClose = truth(static_cast<std::uint32_t>(fired) + static_cast<std::uint32_t>(c.leadMs));
        CHECK(atClose <= c.grabRangeIn + 0.1f);
        CHECK(atClose >= c.grabRangeIn - 0.75f);
    }
}

CHAD_TEST(grabTrigger, fastApproachToleratesSensorNoise) {
    const GrabTrigger::Config c = config();
    const Truth truth = approach(30.0f, 60.0f);
    const Truth noise = [](std::uint32_t ms) { return ((ms / kSensorMs) % 2 == 0) ? 0.4f : -0.4f; };
    for (std::uint32_t phase = 0; phase < kSensorMs; phase += 4) {
        GrabTrigger t(c);
        const long fired = runUntilFire(t, truth, 60.0f, phase, 1000, noise);
        CHECK(fired >= 0);
        const float atClose = truth(static_cast<std::uint32_t>(fired) + static_cast<std::uint32_t>(c.leadMs));
        CHECK(atClose <= c.grabRangeIn + 0.5f);
        CHECK(atClose >= c.grabRangeIn - 1.25f);
    }
}

CHAD_TEST(grabTrigger, slowApproachClosesJustInsideTheRange) {
    const GrabTrigger::Config c = config();
    GrabTrigger t(c);
    const Truth truth = approach(12.0f, 5.0f);
    const long fired = runUntilFire(t, truth, 5.0f, 7, 3000);
    CHECK(fired >= 0);
    const float atClose = truth(static_cast<std::uint32_t>(fired) + static_cast<std::uint32_t>(c.leadMs));
    CHECK(atClose <= c.grabRangeIn + 0.05f);
    CHECK(atClose >= c.grabRangeIn - 0.5f);
}

CHAD_TEST(grabTrigger, dropoutForgetsTheTrack) {
    GrabTrigger t(config());
    CHECK(!t.update(0, 9.0f, 30.0f));
    CHECK(!t.update(33, 8.0f, 30.0f));
    CHECK(!t.update(66, kNaN, 30.0f));
    CHECK(!t.update(99, 4.0f, 30.0f));  // first sample after the gap: unconfirmed
}

CHAD_TEST(grabTrigger, frozenReadingIsNotProjectedForever) {
    // A sensor that stops refreshing mid-approach is not extrapolated without bound.
    GrabTrigger t(config());
    CHECK(!t.update(0, 22.0f, 60.0f));
    CHECK(!t.update(33, 20.0f, 60.0f));
    for (std::uint32_t now = 38; now < 1000; now += kPollMs) {
        CHECK(!t.update(now, 20.0f, 60.0f));
    }
}

CHAD_TEST(grabTrigger, nonFiniteClosingSpeedFallsBackToTheRange) {
    GrabTrigger t(config());
    CHECK(!t.update(0, 6.0f, kNaN));
    CHECK(!t.update(33, 5.0f, kNaN));
    CHECK(!t.update(66, 4.8f, kNaN));
    CHECK(t.update(99, 4.4f, kNaN));
}

CHAD_TEST(grabTrigger, resetDropsConfirmation) {
    GrabTrigger t(config());
    CHECK(!t.update(0, 5.0f, 0.0f));
    t.reset();
    CHECK(!t.update(33, 4.0f, 0.0f));
}
