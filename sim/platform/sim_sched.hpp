// chad-sim — cooperative fiber scheduler on a virtual clock.
//
// Why fibers: chad runs odometry, MCL, RCL and the chassis motion queue in
// pros::Task background loops, and blocks the caller in waitUntilSettled().
// Those are genuinely concurrent stacks -- you cannot drive them by calling
// update() in a loop. Fibers give each task a real stack while keeping
// execution single-threaded.
//
// Scheduling is strictly deterministic: exactly one fiber runs at a time, the
// next one is always the earliest wake time (ties broken by task id), and the
// clock only advances when nothing is runnable. The same seed replays the same
// run, which is what makes a localization bug reproducible.
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace sim {

using TaskId = int;
inline constexpr TaskId kInvalidTask = -1;
inline constexpr TaskId kRootTask = 0;

enum class TaskState { Running, Ready, Sleeping, Blocked, Finished };

class Scheduler {
public:
    Scheduler();
    ~Scheduler();

    // Create a fiber. It does not run until the scheduler next picks it.
    TaskId spawn(std::function<void()> fn, const char* name);

    // Sleep the calling context for `ms` of virtual time. Called by
    // pros::delay(). From the root context this advances the world directly
    // instead of yielding.
    void delayCurrent(std::uint32_t ms);

    // Advance virtual time by dtMs, running every task that becomes due.
    // Called from the bridge once per UI frame (or per Step button press).
    void runFor(double dtMs);

    // Run until `pred` is true or `timeoutMs` of virtual time elapses.
    // Returns true if pred became true. Used by the bridge to run a route to
    // completion without pinning it to a frame rate.
    bool runUntil(const std::function<bool()>& pred, double timeoutMs);

    // --- task control -------------------------------------------------------
    void kill(TaskId id);
    void suspend(TaskId id);
    void resume(TaskId id);
    void join(TaskId id);
    std::uint32_t notify(TaskId id);
    std::uint32_t notifyTake(bool clearOnExit, std::uint32_t timeoutMs);

    TaskState   stateOf(TaskId id) const;
    const char* nameOf(TaskId id) const;
    TaskId      currentId() const { return mCurrent; }
    bool        inRootContext() const { return mCurrent == kRootTask; }

    // --- mutex support ------------------------------------------------------
    // Block the current task until resumed by unblock(). Returns false on
    // timeout. Only meaningful from a task fiber.
    bool blockCurrent(std::uint32_t timeoutMs);
    void unblock(TaskId id);

    // Tear down every task. Destroys fibers the library may still hold
    // pros::Task pointers to (Chassis owns a long-lived motion task), so only
    // use this when nothing will touch those objects again.
    void reset();

    // Make every timed sleeper due at the current instant, leaving tasks that
    // are blocked indefinitely (a mutex, or notify_take with TIMEOUT_MAX)
    // untouched. Paired with resetClock() this gives a run a reproducible
    // starting phase without destroying any task.
    void resyncSleepers();

    // Number of live (non-finished) tasks, excluding root.
    int liveTaskCount() const;

private:
    struct Task;

    void switchTo(Task& t);
    void yieldToScheduler();
    static void fiberEntry(void* arg);

    // Run all tasks currently due at the present clock time. Returns true if
    // any task ran.
    bool runDueTasks();

    // Virtual time of the next task wake-up, or -1 if nothing is sleeping.
    double nextWakeTime() const;

    void reap();

    std::vector<Task*> mTasks;   // index == TaskId; slot 0 is the root context
    TaskId mCurrent{kRootTask};
    int    mNextId{1};
};

Scheduler& sched();

}  // namespace sim
