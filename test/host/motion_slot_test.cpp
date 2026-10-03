#include "chad/util/motion_slot.hpp"

#include <vector>

#include "chad_test.hpp"

using namespace chad;

namespace {
int gCopies = 0;

struct Req {
    int id{0};
    std::vector<int> path;

    Req() = default;
    explicit Req(int i) : id(i), path(64, i) {}
    Req(const Req& o) : id(o.id), path(o.path) { ++gCopies; }
    Req& operator=(const Req& o) {
        id = o.id;
        path = o.path;
        ++gCopies;
        return *this;
    }
    Req(Req&&) = default;
    Req& operator=(Req&&) = default;
};
}  // namespace

CHAD_TEST(motionSlot, idleIsSettled) {
    MotionSlot<Req> s;
    CHECK(s.settled());
    CHECK(!s.hasPending());
    CHECK(!s.runningThrough(0));
}

CHAD_TEST(motionSlot, runsToCompletion) {
    MotionSlot<Req> s;
    const uint32_t seq = s.push(Req(1));
    CHECK(!s.settled());
    CHECK(!s.done(seq));

    Req out;
    uint32_t popped = 0;
    CHECK(s.pop(out, popped));
    CHECK_EQ(popped, seq);
    CHECK_EQ(out.id, 1);
    CHECK(s.runningThrough(seq));
    CHECK(!s.cancelled(seq));

    s.finish(seq);
    CHECK(s.done(seq));
    CHECK(s.settled());
    CHECK(!s.runningThrough(seq));
}

CHAD_TEST(motionSlot, cancelBetweenPopAndExecuteIsHonoured) {
    MotionSlot<Req> s;
    const uint32_t seq = s.push(Req(1));
    Req out;
    uint32_t popped = 0;
    CHECK(s.pop(out, popped));

    s.cancelAll();
    // The executor checks this before its first output and on every tick.
    CHECK(s.cancelled(popped));
    CHECK(s.done(seq));
    // Still held by the executor until it lets go.
    CHECK(s.runningThrough(seq));
    CHECK(!s.settled());

    s.finish(popped);
    CHECK(s.settled());
    CHECK(!s.runningThrough(seq));
}

CHAD_TEST(motionSlot, cancelWhileIdleDoesNotSwallowTheNextMotion) {
    MotionSlot<Req> s;
    s.cancelAll();
    s.cancelAll();
    CHECK(s.settled());

    const uint32_t seq = s.push(Req(2));
    CHECK(!s.cancelled(seq));
    CHECK(!s.done(seq));
    CHECK(!s.settled());

    Req out;
    uint32_t popped = 0;
    CHECK(s.pop(out, popped));
    CHECK(!s.cancelled(popped));
    s.finish(popped);
    CHECK(s.settled());
}

CHAD_TEST(motionSlot, cancelDropsPendingAndSettlesWhenIdle) {
    MotionSlot<Req> s;
    const uint32_t seq = s.push(Req(1));
    s.cancelAll();
    CHECK(!s.hasPending());
    CHECK(s.done(seq));
    CHECK(s.settled());

    Req out;
    uint32_t popped = 0;
    CHECK(!s.pop(out, popped));
}

CHAD_TEST(motionSlot, cancelWithRunningAndPendingSettlesOnFinish) {
    MotionSlot<Req> s;
    const uint32_t a = s.push(Req(1));
    Req out;
    uint32_t popped = 0;
    CHECK(s.pop(out, popped));
    const uint32_t b = s.push(Req(2));

    CHECK_EQ(s.cancelAll(), b);
    CHECK(!s.hasPending());
    CHECK(s.done(b));
    CHECK(!s.settled());

    s.finish(a);
    CHECK(s.settled());
}

CHAD_TEST(motionSlot, motionIssuedAfterCancelSurvivesTheStoppingOne) {
    MotionSlot<Req> s;
    const uint32_t a = s.push(Req(1));
    Req out;
    uint32_t popped = 0;
    CHECK(s.pop(out, popped));
    const uint32_t through = s.cancelAll();
    CHECK_EQ(through, a);

    const uint32_t b = s.push(Req(2));
    CHECK(!s.cancelled(b));
    CHECK(s.cancelledThrough() == through);

    s.finish(a);
    CHECK(!s.settled());
    CHECK(!s.runningThrough(through));

    CHECK(s.pop(out, popped));
    CHECK_EQ(popped, b);
    CHECK_EQ(out.id, 2);
    CHECK(!s.runningThrough(through));
    CHECK(s.runningThrough(b));
    s.finish(b);
    CHECK(s.settled());
}

CHAD_TEST(motionSlot, releaseSettlesAStuckExecutor) {
    MotionSlot<Req> s;
    const uint32_t a = s.push(Req(1));
    Req out;
    uint32_t popped = 0;
    CHECK(s.pop(out, popped));
    const uint32_t through = s.cancelAll();

    s.release(through);
    CHECK(s.settled());

    // A late finish must not move completion backwards.
    const uint32_t b = s.push(Req(2));
    s.finish(a);
    CHECK(!s.settled());
    CHECK(s.pop(out, popped));
    s.finish(b);
    CHECK(s.settled());
}

// cancelMotion falls back to this when the queue lock cannot be had, so it
// may not touch the pending request itself.
CHAD_TEST(motionSlot, cancelWithoutTheLockStillDropsThePendingRequest) {
    MotionSlot<Req> s;
    const uint32_t a = s.push(Req(1));
    Req out;
    uint32_t popped = 0;
    CHECK(s.pop(out, popped));
    const uint32_t b = s.push(Req(2));

    CHECK_EQ(s.markCancelled(), b);
    CHECK(s.cancelled(a));
    CHECK(s.cancelled(b));

    s.finish(a);
    CHECK(!s.pop(out, popped));
    CHECK(!s.hasPending());
    CHECK(s.settled());
}

CHAD_TEST(motionSlot, completedTracksReleaseNotCancel) {
    MotionSlot<Req> s;
    const uint32_t a = s.push(Req(1));
    Req out;
    uint32_t popped = 0;
    CHECK(s.pop(out, popped));
    const uint32_t through = s.cancelAll();
    // Cancelled but still stopping: the executor still owns the motors.
    CHECK(!s.completed(a));
    s.release(through);
    CHECK(s.completed(a));
}

CHAD_TEST(motionSlot, requestsAreMovedNotCopied) {
    MotionSlot<Req> s;
    gCopies = 0;
    Req out;
    uint32_t popped = 0;
    for (int i = 1; i <= 5; ++i) {
        Req r(i);
        const int* data = r.path.data();
        s.push(std::move(r));
        CHECK(s.pop(out, popped));
        CHECK(out.path.data() == data);
        s.finish(popped);
    }
    CHECK_EQ(gCopies, 0);
}
