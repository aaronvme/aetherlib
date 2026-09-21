#include <atomic>
#include <cmath>

#include "gamespecific/claw.hpp"
#include "gamespecific/config.hpp"
#include "pros/rtos.hpp"

using namespace chad;

namespace {
// The distance sensor does not refresh every few ms, so polling faster
// than this just reads the same physical sample twice and the streak
// below stops meaning anything.
constexpr int kPollMs = 20;

// Two reads in a row before closing, so one bad sample cannot fire the claw.
constexpr int kGrabSamples = 2;

// what the v5 distance sensor reports when nothing came back
constexpr int kNoReturnMm = 9999;

// Under this the sensor stops resolving and reports a floor value or zero.
// Something that close is against the jaws, so it has to count as present --
// reading it as "nothing there" made the claw blindest right where it grabs.
constexpr int kTooCloseMm = 20;

std::atomic<bool> gAutoGrab{false};
std::atomic<bool> gArmed{true};
pros::Task* gWatch = nullptr;
}

double clawDistance() {
    const int mm = claw_dist.get();
    // A get() that fails means no sensor, which must never read as present.
    if (mm == PROS_ERR || mm >= kNoReturnMm) return NAN;
    if (mm < kTooCloseMm) return static_cast<double>(kTooCloseMm) / 25.4;
    return static_cast<double>(mm) / 25.4;
}

bool clawSees() {
    const double in = clawDistance();
    return std::isfinite(in) && in <= kClawGrabRangeIn;
}

bool clawClosed() {
    return claw.is_extended();
}

void clawClose() {
    claw.extend();
}

void clawOpen() {
    claw.retract();
    gArmed = false;
}

void clawToggle() {
    if (clawClosed()) clawOpen();
    else clawClose();
}

void clawSetAutoGrab(bool enabled) {
    gAutoGrab = enabled;
}

bool clawAutoGrab() {
    return gAutoGrab.load();
}

bool clawGrabWhenClose(double timeoutMs) {
    // Nothing can get into a claw that is already shut, so this would sit out
    // the timeout and then report a grab that never happened.
    if (clawClosed()) return false;

    const uint32_t start = pros::millis();
    const uint32_t limit =
        (std::isfinite(timeoutMs) && timeoutMs > 0.0)
            ? static_cast<uint32_t>(timeoutMs) : 0u;
    int streak = 0;
    while (pros::millis() - start < limit) {
        if (clawSees()) {
            if (++streak >= kGrabSamples) {
                clawClose();
                return true;
            }
        } else {
            streak = 0;
        }
        pros::delay(kPollMs);
    }
    return false;
}

void startClawWatch() {
    if (gWatch != nullptr) return;
    gWatch = new pros::Task([] {
        bool wasClosed = clawClosed();
        int streak = 0;
        while (true) {
            const bool closed = clawClosed();
            // Whatever opened the claw, it has to clear the jaws before
            // auto-grab is allowed to bite again.
            if (wasClosed && !closed) {
                gArmed = false;
                streak = 0;
            }
            wasClosed = closed;

            const double in = clawDistance();
            const bool clear = !std::isfinite(in) || in >= kClawClearRangeIn;

            if (!gArmed.load()) {
                if (clear) gArmed = true;
            } else if (gAutoGrab.load() && !closed) {
                if (std::isfinite(in) && in <= kClawGrabRangeIn) {
                    if (++streak >= kGrabSamples) {
                        clawClose();
                        streak = 0;
                    }
                } else {
                    streak = 0;
                }
            }
            pros::delay(kPollMs);
        }
    }, "ClawWatch");
}
