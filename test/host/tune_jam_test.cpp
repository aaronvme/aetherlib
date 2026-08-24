#include "chad/util/jam_state_machine.hpp"

#include "chad_test.hpp"

using namespace chad;

namespace {
JamTimingConfig fastConfig() {
    JamTimingConfig c;
    c.jamTriggerMs = 150;
    c.reverseDurationMs = 250;
    c.settleDurationMs = 150;
    c.graceMs = 150;
    return c;
}
}  // namespace

CHAD_TEST(jam, staysNormalWithoutAJamSignal) {
    JamStateMachine m{fastConfig()};
    m.reset(0);
    for (uint32_t t = 0; t <= 1000; t += 10) {
        CHECK(m.update(t, false) == JamState::Normal);
    }
}

CHAD_TEST(jam, sustainedJamTriggersReversal) {
    JamStateMachine m{fastConfig()};
    m.reset(1000);
    // grace ends at 1150; jam must hold jamTriggerMs (150) after that.
    CHECK(m.update(1150, true) == JamState::Normal);
    CHECK(m.update(1290, true) == JamState::Normal);
    CHECK(m.update(1300, true) == JamState::Reversing);
}

CHAD_TEST(jam, standingStartDuringGraceNeverTriggers) {
    // A motor commanded from rest reads a jam-band efficiency while it spins
    // up; the grace window must swallow that even if it lasts jamTriggerMs.
    JamStateMachine m{fastConfig()};
    m.reset(0);
    for (uint32_t t = 0; t < 150; t += 10) {
        CHECK(m.update(t, true) == JamState::Normal);
    }
    // Once out of grace, jammed must still hold jamTriggerMs before firing.
    CHECK(m.update(150, true) == JamState::Normal);
    CHECK(m.update(299, true) == JamState::Normal);
    CHECK(m.update(300, true) == JamState::Reversing);
}

CHAD_TEST(jam, motionRestartReArmsGrace) {
    JamTimingConfig cfg = fastConfig();
    JamStateMachine m{cfg};
    m.reset(0);
    // Long past any startup grace, and not jammed.
    m.update(5000, false);
    // Mechanism stops and restarts -- caller notices the edge and re-arms.
    m.noteMotionStart(5000);
    for (uint32_t t = 5000; t < 5150; t += 10) {
        CHECK(m.update(t, true) == JamState::Normal);
    }
    CHECK(m.update(5150, true) == JamState::Normal);
    CHECK(m.update(5300, true) == JamState::Reversing);
}

CHAD_TEST(jam, intermittentJamSignalNeverAccumulates) {
    JamStateMachine m{fastConfig()};
    m.reset(1000);
    for (uint32_t t = 1150; t < 1150 + 1000; t += 20) {
        // Alternates so the jam timer keeps resetting to 0.
        const bool jammed = ((t / 20) % 2) == 0;
        CHECK(m.update(t, jammed) == JamState::Normal);
    }
}

CHAD_TEST(jam, fullCycleReversesThenSettlesThenResumesWatching) {
    JamTimingConfig cfg = fastConfig();
    JamStateMachine m{cfg};
    m.reset(0);
    for (uint32_t t = 150; t <= 300; t += 10) m.update(t, true);
    CHECK(m.state() == JamState::Reversing);

    CHECK(m.update(549, true) == JamState::Reversing);
    CHECK(m.update(550, true) == JamState::Settling);

    CHECK(m.update(699, true) == JamState::Settling);
    CHECK(m.update(700, true) == JamState::Normal);
}

CHAD_TEST(jam, zeroTimestampIsNotMistakenForTheSentinel) {
    // The internal timer uses 0 as "not accumulating"; a jam signal that
    // starts exactly at t=0 is nudged to 1 so it is still timed, rather than
    // being silently ignored forever (off by the 1 ms nudge).
    JamTimingConfig cfg = fastConfig();
    cfg.graceMs = 0;
    JamStateMachine m{cfg};
    m.reset(0);
    CHECK(m.update(0, true) == JamState::Normal);
    CHECK(m.update(150, true) == JamState::Normal);
    CHECK(m.update(151, true) == JamState::Reversing);
}

CHAD_TEST(jam, survivesMillisWraparound) {
    JamTimingConfig cfg = fastConfig();
    cfg.graceMs = 0;
    JamStateMachine m{cfg};
    const uint32_t start = 0xFFFFFF9Cu;  // start + 149 wraps past UINT32_MAX
    m.reset(start);
    CHECK(m.update(start, true) == JamState::Normal);          // jam timer starts here
    CHECK(m.update(start + 149u, true) == JamState::Normal);   // wrapped, elapsed still 149
    CHECK(m.update(start + 150u, true) == JamState::Reversing);
}
