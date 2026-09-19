// chad-sim PROS shim — pros/rtos.hpp
//
// Tasks are cooperative fibers driven by a virtual clock (sim/platform/
// sim_sched.hpp). pros::delay() yields to the scheduler, which advances virtual
// time and steps the world. Exactly one fiber runs at a time and the next one
// is chosen by earliest wake time, so a run is fully deterministic and there
// are no real data races -- but the library's own locking still gets exercised
// because Mutex is a real (counting) implementation.
#pragma once

#include <cstdint>
#include <functional>

#include "../sim_sched.hpp"

#define TIMEOUT_MAX ((std::uint32_t)0xffffffffUL)

namespace pros {

using task_t = void*;

enum task_state_e_t {
    E_TASK_STATE_RUNNING = 0,
    E_TASK_STATE_READY,
    E_TASK_STATE_BLOCKED,
    E_TASK_STATE_SUSPENDED,
    E_TASK_STATE_DELETED,
    E_TASK_STATE_INVALID
};

constexpr std::uint32_t TASK_PRIORITY_DEFAULT = 8;
constexpr std::uint16_t TASK_STACK_DEPTH_DEFAULT = 0x2000;

std::uint32_t millis();
std::uint64_t micros();
void delay(std::uint32_t milliseconds);

namespace c {
inline std::uint32_t millis() { return ::pros::millis(); }
inline void delay(std::uint32_t ms) { ::pros::delay(ms); }
}  // namespace c

// ============================================================================
// Task
// ============================================================================

using task_fn_t = void (*)(void*);

class Task {
public:
    // Callable form: pros::Task([]{ ... })
    template <typename F>
    explicit Task(F&& func,
                  std::uint32_t prio = TASK_PRIORITY_DEFAULT,
                  std::uint16_t stack_depth = TASK_STACK_DEPTH_DEFAULT,
                  const char* name = "sim_task")
        : mId(sim::sched().spawn(std::function<void()>(std::forward<F>(func)), name)) {
        (void)prio;
        (void)stack_depth;
    }

    // Callable + name: pros::Task([]{ ... }, "chadLog")
    template <typename F>
    Task(F&& func, const char* name)
        : mId(sim::sched().spawn(std::function<void()>(std::forward<F>(func)), name)) {}

    // C form: pros::Task(trampoline, params, "name")
    Task(task_fn_t function, void* parameters, const char* name = "sim_task")
        : mId(sim::sched().spawn([function, parameters]() { function(parameters); }, name)) {}

    Task(task_fn_t function, void* parameters, std::uint32_t prio,
         std::uint16_t stack_depth, const char* name = "sim_task")
        : mId(sim::sched().spawn([function, parameters]() { function(parameters); }, name)) {
        (void)prio;
        (void)stack_depth;
    }

    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    ~Task() = default;

    std::uint32_t get_state() const {
        switch (sim::sched().stateOf(mId)) {
            case sim::TaskState::Running:   return E_TASK_STATE_RUNNING;
            case sim::TaskState::Ready:     return E_TASK_STATE_READY;
            case sim::TaskState::Sleeping:  return E_TASK_STATE_BLOCKED;
            case sim::TaskState::Blocked:   return E_TASK_STATE_BLOCKED;
            case sim::TaskState::Finished:  return E_TASK_STATE_DELETED;
        }
        return E_TASK_STATE_INVALID;
    }

    void remove() { sim::sched().kill(mId); }
    void suspend() { sim::sched().suspend(mId); }
    void resume() { sim::sched().resume(mId); }
    std::uint32_t notify() { return sim::sched().notify(mId); }
    void join() { sim::sched().join(mId); }

    const char* get_name() const { return sim::sched().nameOf(mId); }

    static std::uint32_t notify_take(bool clear_on_exit, std::uint32_t timeout) {
        return sim::sched().notifyTake(clear_on_exit, timeout);
    }

    static void delay_until(std::uint32_t* const prev_time, const std::uint32_t delta) {
        const std::uint32_t target = *prev_time + delta;
        const std::uint32_t now = ::pros::millis();
        if (static_cast<std::int32_t>(target - now) > 0) {
            ::pros::delay(target - now);
        }
        *prev_time = target;
    }

    static Task current() { return Task(sim::sched().currentId()); }

private:
    explicit Task(sim::TaskId id) : mId(id) {}
    sim::TaskId mId;
};

// ============================================================================
// Mutex — real counting mutex over the cooperative scheduler
// ============================================================================

class Mutex {
public:
    Mutex() = default;
    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;

    bool take(std::uint32_t timeout = TIMEOUT_MAX) { return lock_impl(timeout); }
    bool give() { return unlock_impl(); }

    // std::lock_guard interface -- chad uses std::lock_guard<pros::Mutex>.
    void lock() { lock_impl(TIMEOUT_MAX); }
    void unlock() { unlock_impl(); }
    bool try_lock() { return lock_impl(0); }

private:
    bool lock_impl(std::uint32_t timeout);
    bool unlock_impl();

    sim::TaskId mOwner{sim::kInvalidTask};
    int mDepth{0};
};

}  // namespace pros
