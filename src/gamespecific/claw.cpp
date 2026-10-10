#include <atomic>
#include <cmath>

#include "chad/util/grab_trigger.hpp"
#include "gamespecific/claw.hpp"
#include "gamespecific/config.hpp"
#include "pros/rtos.hpp"

using namespace chad;

namespace {
// The sensor refreshes about every 33 ms; GrabTrigger counts each physical
// sample once, so polling faster only shortens how long a new one goes unseen.
constexpr int kPollMs = 5;

// what the v5 distance sensor reports when nothing came back
constexpr int kNoReturnMm = 9999;

// Under this the sensor stops resolving and reports a floor value or zero.
// Something that close is against the jaws, so it has to count as present --
// reading it as "nothing there" made the claw blindest right where it grabs.
constexpr int kTooCloseMm = 20;

std::atomic<bool> gAutoGrab{false};
std::atomic<bool> gArmed{true};
pros::Task* gWatch = nullptr;

GrabTrigger::Config triggerConfig() {
    GrabTrigger::Config c;
    c.grabRangeIn = static_cast<float>(kClawGrabRangeIn);
    c.leadMs = static_cast<float>(kClawLeadMs);
    return c;
}

// The claw is on the back, so backing up closes on what it sees.
float closingInPerSec() {
    return -drivetrain.averageVelocityInPerSec();
}

bool triggerUpdate(GrabTrigger& trigger) {
    return trigger.update(pros::millis(), static_cast<float>(clawDistance()),
                          closingInPerSec());
}
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
    return !claw.is_extended();
}

void clawClose() {
    claw.retract();
}

void clawOpen() {
    claw.extend();
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
    GrabTrigger trigger(triggerConfig());
    while (pros::millis() - start < limit) {
        if (triggerUpdate(trigger)) {
            clawClose();
            return true;
        }
        pros::delay(kPollMs);
    }
    return false;
}

void startClawWatch() {
    if (gWatch != nullptr) return;
    gWatch = new pros::Task([] {
        GrabTrigger trigger(triggerConfig());
        bool wasClosed = clawClosed();
        while (true) {
            const bool closed = clawClosed();
            // Whatever opened the claw, it has to clear the jaws before
            // auto-grab is allowed to bite again.
            if (wasClosed && !closed) {
                gArmed = false;
                trigger.reset();
            }
            wasClosed = closed;

            // Fed every pass so a track is already confirmed when auto-grab
            // turns on with something approaching.
            const bool fire = triggerUpdate(trigger);

            if (!gArmed.load()) {
                const double in = clawDistance();
                if (!std::isfinite(in) || in >= kClawClearRangeIn) gArmed = true;
            } else if (gAutoGrab.load() && !closed && fire) {
                clawClose();
            }
            pros::delay(kPollMs);
        }
    }, "ClawWatch");
}
