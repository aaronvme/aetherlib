#include "sim_sched.hpp"
#include "sim_runtime.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <new>

#ifdef __EMSCRIPTEN__
#include <emscripten/fiber.h>
#else
#error "chad-sim currently targets Emscripten only (fibers need -sASYNCIFY)."
#endif

namespace sim {
namespace {

// Per-task stacks. The C stack holds ordinary locals; the Asyncify stack holds
// the unwound call frames when a fiber is swapped out. Chassis/MCL keep their
// big buffers in static storage (the robot config is a global), so these are
// comfortably sized rather than large.
constexpr std::size_t kCStackBytes = 256 * 1024;
constexpr std::size_t kAStackBytes = 128 * 1024;

void* allocStack(std::size_t bytes) {
    void* p = std::malloc(bytes);
    if (p == nullptr) std::abort();
    std::memset(p, 0, bytes);
    return p;
}

}  // namespace

// ============================================================================
// Task
// ============================================================================

struct Scheduler::Task {
    TaskId id{kInvalidTask};
    std::string name;
    std::function<void()> fn;
    TaskState state{TaskState::Ready};

    // Virtual clock time at which a sleeping task becomes runnable.
    double wakeAt{0.0};
    // Sleeping with no deadline (blocked on a mutex/notify with TIMEOUT_MAX).
    bool indefinite{false};

    std::uint32_t notifyCount{0};

    emscripten_fiber_t fiber{};
    void* cstack{nullptr};
    void* astack{nullptr};
    bool  isRoot{false};

    ~Task() {
        std::free(cstack);
        std::free(astack);
    }
};

// ============================================================================
// Scheduler
// ============================================================================

Scheduler& sched() {
    static Scheduler s;
    return s;
}

Scheduler::Scheduler() {
    // Slot 0 is the context we were entered on (the JS call into the bridge).
    auto* root = new Task();
    root->id = kRootTask;
    root->name = "root";
    root->state = TaskState::Running;
    root->isRoot = true;
    root->astack = allocStack(kAStackBytes);
    emscripten_fiber_init_from_current_context(&root->fiber, root->astack, kAStackBytes);
    mTasks.push_back(root);
}

Scheduler::~Scheduler() {
    for (auto* t : mTasks) delete t;
}

TaskId Scheduler::spawn(std::function<void()> fn, const char* name) {
    auto* t = new Task();
    t->id = mNextId++;
    t->name = (name != nullptr) ? name : "task";
    t->fn = std::move(fn);
    t->state = TaskState::Ready;
    t->wakeAt = preciseMillis();
    t->cstack = allocStack(kCStackBytes);
    t->astack = allocStack(kAStackBytes);

    emscripten_fiber_init(&t->fiber, &Scheduler::fiberEntry, t,
                          t->cstack, kCStackBytes,
                          t->astack, kAStackBytes);

    if (static_cast<std::size_t>(t->id) >= mTasks.size()) mTasks.resize(t->id + 1, nullptr);
    mTasks[t->id] = t;
    return t->id;
}

void Scheduler::fiberEntry(void* arg) {
    auto* t = static_cast<Task*>(arg);
    if (t->fn) t->fn();
    t->state = TaskState::Finished;
    // Hand control back for the last time; the scheduler will not resume us.
    sched().yieldToScheduler();
}

void Scheduler::switchTo(Task& t) {
    Task* prev = mTasks[mCurrent];
    mCurrent = t.id;
    if (t.state != TaskState::Finished) t.state = TaskState::Running;
    emscripten_fiber_swap(&prev->fiber, &t.fiber);
}

void Scheduler::yieldToScheduler() {
    Task* cur = mTasks[mCurrent];
    Task* root = mTasks[kRootTask];
    mCurrent = kRootTask;
    root->state = TaskState::Running;
    emscripten_fiber_swap(&cur->fiber, &root->fiber);
}

void Scheduler::delayCurrent(std::uint32_t ms) {
    if (mCurrent == kRootTask) {
        // Sleeping on the root context: just drive the world forward. This is
        // what happens when setup code calls pros::delay() outside any task.
        runFor(static_cast<double>(ms));
        return;
    }
    Task* cur = mTasks[mCurrent];
    cur->state = TaskState::Sleeping;
    cur->indefinite = false;
    cur->wakeAt = preciseMillis() + static_cast<double>(ms);
    yieldToScheduler();
}

bool Scheduler::blockCurrent(std::uint32_t timeoutMs) {
    if (mCurrent == kRootTask) {
        // Root cannot block; step time so whoever holds the resource can run.
        runFor(1.0);
        return true;
    }
    Task* cur = mTasks[mCurrent];
    cur->state = TaskState::Blocked;
    if (timeoutMs == 0xffffffffu) {
        cur->indefinite = true;
        cur->wakeAt = 0.0;
    } else {
        cur->indefinite = false;
        cur->wakeAt = preciseMillis() + static_cast<double>(timeoutMs);
    }
    yieldToScheduler();
    // We were resumed either by unblock() (state set to Ready) or by timeout.
    return !cur->indefinite || cur->state != TaskState::Blocked;
}

void Scheduler::unblock(TaskId id) {
    if (id < 0 || static_cast<std::size_t>(id) >= mTasks.size()) return;
    Task* t = mTasks[id];
    if (t == nullptr) return;
    if (t->state == TaskState::Blocked || t->state == TaskState::Sleeping) {
        t->state = TaskState::Ready;
        t->indefinite = false;
        t->wakeAt = preciseMillis();
    }
}

// ============================================================================
// Notifications
// ============================================================================

std::uint32_t Scheduler::notify(TaskId id) {
    if (id < 0 || static_cast<std::size_t>(id) >= mTasks.size()) return 0;
    Task* t = mTasks[id];
    if (t == nullptr) return 0;
    t->notifyCount++;
    unblock(id);
    return t->notifyCount;
}

std::uint32_t Scheduler::notifyTake(bool clearOnExit, std::uint32_t timeoutMs) {
    if (mCurrent == kRootTask) return 0;
    Task* cur = mTasks[mCurrent];

    if (cur->notifyCount == 0) {
        blockCurrent(timeoutMs);
    }

    std::uint32_t out = cur->notifyCount;
    if (clearOnExit) {
        cur->notifyCount = 0;
    } else if (cur->notifyCount > 0) {
        cur->notifyCount--;
    }
    return out;
}

// ============================================================================
// Task control
// ============================================================================

void Scheduler::kill(TaskId id) {
    if (id <= kRootTask || static_cast<std::size_t>(id) >= mTasks.size()) return;
    Task* t = mTasks[id];
    if (t == nullptr) return;
    // A fiber cannot be unwound from outside, so mark it finished and never
    // schedule it again. Its stacks are freed by reap().
    t->state = TaskState::Finished;
}

void Scheduler::suspend(TaskId id) {
    if (id < 0 || static_cast<std::size_t>(id) >= mTasks.size()) return;
    Task* t = mTasks[id];
    if (t != nullptr && t->state != TaskState::Finished) {
        t->state = TaskState::Blocked;
        t->indefinite = true;
    }
}

void Scheduler::resume(TaskId id) { unblock(id); }

void Scheduler::join(TaskId id) {
    if (id < 0 || static_cast<std::size_t>(id) >= mTasks.size()) return;
    while (mTasks[id] != nullptr && mTasks[id]->state != TaskState::Finished) {
        delayCurrent(2);
    }
}

TaskState Scheduler::stateOf(TaskId id) const {
    if (id < 0 || static_cast<std::size_t>(id) >= mTasks.size()) return TaskState::Finished;
    const Task* t = mTasks[id];
    return (t == nullptr) ? TaskState::Finished : t->state;
}

const char* Scheduler::nameOf(TaskId id) const {
    if (id < 0 || static_cast<std::size_t>(id) >= mTasks.size()) return "invalid";
    const Task* t = mTasks[id];
    return (t == nullptr) ? "invalid" : t->name.c_str();
}

int Scheduler::liveTaskCount() const {
    int n = 0;
    for (std::size_t i = 1; i < mTasks.size(); i++) {
        if (mTasks[i] != nullptr && mTasks[i]->state != TaskState::Finished) n++;
    }
    return n;
}

// ============================================================================
// Time advance
// ============================================================================

double Scheduler::nextWakeTime() const {
    double best = -1.0;
    for (std::size_t i = 1; i < mTasks.size(); i++) {
        const Task* t = mTasks[i];
        if (t == nullptr) continue;
        if (t->state != TaskState::Sleeping && t->state != TaskState::Blocked) continue;
        if (t->indefinite) continue;
        if (best < 0.0 || t->wakeAt < best) best = t->wakeAt;
    }
    return best;
}

bool Scheduler::runDueTasks() {
    bool ranAny = false;
    const double now = preciseMillis();

    // Wake anything whose deadline has passed.
    for (std::size_t i = 1; i < mTasks.size(); i++) {
        Task* t = mTasks[i];
        if (t == nullptr) continue;
        if ((t->state == TaskState::Sleeping || t->state == TaskState::Blocked) &&
            !t->indefinite && t->wakeAt <= now + 1e-9) {
            t->state = TaskState::Ready;
        }
    }

    // Run every ready task once, in id order. Deterministic by construction.
    for (std::size_t i = 1; i < mTasks.size(); i++) {
        Task* t = mTasks[i];
        if (t == nullptr || t->state != TaskState::Ready) continue;
        switchTo(*t);
        ranAny = true;
    }
    return ranAny;
}

void Scheduler::reap() {
    for (std::size_t i = 1; i < mTasks.size(); i++) {
        Task* t = mTasks[i];
        if (t != nullptr && t->state == TaskState::Finished) {
            delete t;
            mTasks[i] = nullptr;
        }
    }
}

void Scheduler::runFor(double dtMs) {
    if (dtMs <= 0.0) return;
    const double target = preciseMillis() + dtMs;

    while (true) {
        // Let everything that is due at the current instant run to its next
        // yield point before time moves again.
        while (runDueTasks()) { /* keep draining */ }
        reap();

        const double now = preciseMillis();
        if (now >= target - 1e-9) break;

        // Advance to whichever comes first: the next task wake-up or the end
        // of the requested slice. Physics is integrated inside stepPhysics().
        double wake = nextWakeTime();
        double until = target;
        if (wake >= 0.0 && wake < until) until = wake;
        if (until <= now) until = now + kPhysicsDtMs;
        if (until > target) until = target;

        stepPhysics(until - now);
    }
}

bool Scheduler::runUntil(const std::function<bool()>& pred, double timeoutMs) {
    const double deadline = preciseMillis() + timeoutMs;
    while (preciseMillis() < deadline) {
        if (pred()) return true;
        runFor(kPhysicsDtMs);
    }
    return pred();
}

void Scheduler::resyncSleepers() {
    const double now = preciseMillis();
    for (std::size_t i = 1; i < mTasks.size(); i++) {
        Task* t = mTasks[i];
        if (t == nullptr || t->state == TaskState::Finished) continue;
        // Only timed sleeps are rephased. A task parked on notify_take with no
        // deadline is waiting for an event, not a time, and waking it here
        // would look to it exactly like that event arriving.
        if (t->state == TaskState::Sleeping && !t->indefinite) {
            t->wakeAt = now;
        }
    }
}

void Scheduler::reset() {
    for (std::size_t i = 1; i < mTasks.size(); i++) {
        delete mTasks[i];
        mTasks[i] = nullptr;
    }
    mTasks.resize(1);
    mCurrent = kRootTask;
    mNextId = 1;
    mTasks[kRootTask]->state = TaskState::Running;
}

}  // namespace sim

// ============================================================================
// pros:: free functions and Mutex, implemented against the scheduler
// ============================================================================

#include "pros/rtos.hpp"

namespace pros {

std::uint32_t millis() { return sim::millis(); }
std::uint64_t micros() { return static_cast<std::uint64_t>(sim::preciseMillis() * 1000.0); }

void delay(std::uint32_t milliseconds) {
    sim::sched().delayCurrent(milliseconds);
}

bool Mutex::lock_impl(std::uint32_t timeout) {
    const sim::TaskId me = sim::sched().currentId();

    if (mDepth > 0 && mOwner == me) {   // recursive take by the same task
        mDepth++;
        return true;
    }

    const double start = sim::preciseMillis();
    while (mDepth > 0) {
        if (timeout == 0) return false;
        if (timeout != TIMEOUT_MAX &&
            sim::preciseMillis() - start >= static_cast<double>(timeout)) {
            return false;
        }
        // Owner holds it; yield so it can make progress and release.
        sim::sched().delayCurrent(1);
    }

    mOwner = me;
    mDepth = 1;
    return true;
}

bool Mutex::unlock_impl() {
    if (mDepth <= 0) return false;
    mDepth--;
    if (mDepth == 0) mOwner = sim::kInvalidTask;
    return true;
}

}  // namespace pros
