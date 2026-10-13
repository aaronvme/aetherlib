#include <atomic>
#include <cmath>
#include <cstdint>

#include "gamespecific/sidetoggle.hpp"
#include "gamespecific/config.hpp"
#include "chad/util/indexed_rotator.hpp"
#include "pros/rtos.hpp"

using namespace chad;

// Gains: power (out of 127) per motor degree of error, per degree/second.
static const double kP = 1.0;
static const double kI = 0.0;
static const double kD = 0.04;

// Floor on a positive output, so the last few degrees are not left to friction.
static const double MIN_POWER       = 15.0;
// Autonomous flips and home.
static const double MAX_POWER       = 127.0;
// Stops driving and settles inside this of the target.
static const double TOLERANCE_DEG   = 2.0;
// Within this of an index counts as on it (manual release, jam, home overshoot).
static const double INDEX_SNAP_DEG  = 10.0;
static const double SETTLE_TIME_MS  = 40;
// Stalled: short of the target and moved less than this in STALL_TIME_MS.
static const double STALL_MOVED_DEG = 3.0;
static const double STALL_TIME_MS   = 250;

// Driver control at 50%.
const double kSideToggleManualPower     = 63.0;
const double kSideToggleDriverHomePower = 63.0;

namespace {
constexpr std::uint32_t kPeriodMs = 10;
// Home is the next half-turn index ahead: either toggle state, the shortest way forward.
constexpr int kStepsPerHome = 1;

IndexedRotatorConfig rotatorConfig() {
    IndexedRotatorConfig c;
    c.degPerStep = 180.0;
    c.gains = {static_cast<float>(kP), static_cast<float>(kI), static_cast<float>(kD)};
    c.minPower = static_cast<float>(MIN_POWER);
    c.maxPower = static_cast<float>(MAX_POWER);
    c.toleranceDeg = TOLERANCE_DEG;
    c.indexSnapDeg = INDEX_SNAP_DEG;
    c.settleSec = static_cast<float>(SETTLE_TIME_MS / 1000.0);
    c.stallMovedDeg = STALL_MOVED_DEG;
    c.stallSec = static_cast<float>(STALL_TIME_MS / 1000.0);
    return c;
}

// Requests go through counters so callers never touch the rotators, which
// the task owns. A cancel or tare discards requests up to the drop marks.
struct Channel {
    std::atomic<std::uint32_t> requested{0};
    std::atomic<std::uint32_t> applied{0};
    std::atomic<std::uint32_t> dropUpTo{0};
    std::atomic<std::uint32_t> homeGen{0};
    std::atomic<std::uint32_t> homeApplied{0};
    std::atomic<std::uint32_t> homeDropAt{0};
    std::atomic<std::uint32_t> homeAt{0};      // flips before it are replaced
    std::atomic<float> homePower{0.0f};
    std::atomic<float> flipPower{0.0f};        // cap for queued flips, 0 = full
    std::atomic<float> manualPower{0.0f};
    std::atomic<bool> busy{false};
    std::atomic<bool> stalled{false};
};

Channel gChan[2];
std::atomic<std::uint32_t> gCancelGen{0};
std::atomic<std::uint32_t> gTareGen{0};
pros::Task* gTask = nullptr;
std::function<void(const SideToggleSample&)> gTelemetry;

// Calls f(0) for Left, f(1) for Right, both for Both.
template <class F>
void forSides(Side side, F f) {
    if (side != Side::Right) f(0);
    if (side != Side::Left) f(1);
}

float powerCap(double maxSpeed) {
    return (std::isfinite(maxSpeed) && maxSpeed > 0.0) ? static_cast<float>(maxSpeed) : 0.0f;
}

bool busyAt(int i) {
    const Channel& c = gChan[i];
    // Requests before acknowledgements: once both match, busy is current.
    const std::uint32_t req = c.requested.load();
    const std::uint32_t home = c.homeGen.load();
    if (req != c.applied.load() || home != c.homeApplied.load()) return true;
    return c.busy.load();
}

pros::Motor& motorFor(int i) { return i == 0 ? sideToggleLeft : sideToggleRight; }

void snapshotDrops() {
    for (Channel& c : gChan) {
        c.dropUpTo.store(c.requested.load());
        c.homeDropAt.store(c.homeGen.load());
    }
}

// Wrap-safe "a is after b".
bool after(std::uint32_t a, std::uint32_t b) {
    return static_cast<std::int32_t>(a - b) > 0;
}

void run() {
    IndexedRotator rot[2] = {IndexedRotator(rotatorConfig()),
                             IndexedRotator(rotatorConfig())};
    std::uint32_t applied[2] = {gChan[0].applied.load(), gChan[1].applied.load()};
    std::uint32_t seenHome[2] = {gChan[0].homeApplied.load(),
                                 gChan[1].homeApplied.load()};
    // Index 0 is wherever the first good reading puts it, so a stale or
    // never-tared encoder cannot turn into a catch-up spin.
    bool needBase[2] = {true, true};
    std::uint32_t seenTare = gTareGen.load();
    std::uint32_t seenCancel = gCancelGen.load();
    std::uint32_t last = pros::millis();
    std::uint32_t wake = last;

    while (true) {
        const std::uint32_t now = pros::millis();
        float dt = static_cast<float>(now - last) * 0.001f;
        if (dt > 0.1f) dt = 0.1f;
        last = now;

        const std::uint32_t tare = gTareGen.load();
        const std::uint32_t cancel = gCancelGen.load();
        if (tare != seenTare || cancel != seenCancel) {
            for (int i = 0; i < 2; ++i) {
                if (tare != seenTare) needBase[i] = true;
                rot[i].cancel();
                const std::uint32_t drop = gChan[i].dropUpTo.load();
                if (after(drop, applied[i])) applied[i] = drop;
                const std::uint32_t homeDrop = gChan[i].homeDropAt.load();
                if (after(homeDrop, seenHome[i])) seenHome[i] = homeDrop;
            }
            seenTare = tare;
            seenCancel = cancel;
        }

        for (int i = 0; i < 2; ++i) {
            Channel& c = gChan[i];
            pros::Motor& motor = motorFor(i);
            const double pos = motor.get_position();

            if (needBase[i]) {
                if (std::isfinite(pos)) {
                    rot[i].reset(pos);
                    needBase[i] = false;
                } else {
                    motor.brake();
                    continue;
                }
            }

            const std::uint32_t home = c.homeGen.load();
            const std::uint32_t req = c.requested.load();
            const float manual = c.manualPower.load();
            float power;
            if (manual > 0.0f) {
                // Flips and homes asked for while the driver holds it are dropped.
                applied[i] = req;
                seenHome[i] = home;
                power = rot[i].manual(pos, manual);
            } else {
                if (home != seenHome[i]) {
                    const std::uint32_t at = c.homeAt.load();
                    if (after(at, applied[i])) applied[i] = at;
                    rot[i].update(pos, 0.0f);  // home from where the shaft is now
                    rot[i].home(kStepsPerHome, c.homePower.load());
                    seenHome[i] = home;
                }
                const std::int32_t n = static_cast<std::int32_t>(req - applied[i]);
                applied[i] = req;
                if (n > 0) rot[i].advance(n, c.flipPower.load());
                power = rot[i].update(pos, dt);
            }

            if (power > 0.0f) {
                motor.move(static_cast<std::int32_t>(std::lround(power)));
            } else if (rot[i].busy()) {
                motor.move(0);  // derivative braking: coast toward the index
            } else {
                motor.brake();  // HOLD
            }

            c.busy.store(rot[i].busy());
            c.stalled.store(rot[i].stalled());
            c.applied.store(applied[i]);
            c.homeApplied.store(seenHome[i]);

            if (gTelemetry) {
                gTelemetry({i == 0 ? Side::Left : Side::Right,
                            static_cast<float>(rot[i].target()),
                            static_cast<float>(pos),
                            static_cast<float>(rot[i].error()), power,
                            rot[i].stalled()});
            }
        }

        // After an overrun, sleep a full period rather than run back to back.
        const std::uint32_t done = pros::millis();
        if (done - wake >= kPeriodMs) wake = done;
        pros::Task::delay_until(&wake, kPeriodMs);
    }
}
}  // namespace

void startSideToggle() {
    sideToggleLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    sideToggleRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    if (gTask != nullptr) return;
    gTask = new pros::Task([] { run(); }, "SideToggle");
}

void sideToggleFlip(Side side, int flips, bool wait, std::uint32_t timeoutMs,
                    double maxSpeed) {
    if (flips <= 0) return;
    const float cap = powerCap(maxSpeed);
    forSides(side, [&](int i) {
        // Cap before the request so the task never sees the flip uncapped.
        gChan[i].flipPower.store(cap);
        gChan[i].requested.fetch_add(static_cast<std::uint32_t>(flips));
    });
    if (wait) sideToggleWait(side, timeoutMs);
}

void sideToggleHome(Side side, bool wait, std::uint32_t timeoutMs, double maxSpeed) {
    const float cap = powerCap(maxSpeed);
    forSides(side, [&](int i) {
        Channel& c = gChan[i];
        c.homePower.store(cap);
        c.homeAt.store(c.requested.load());
        c.homeGen.fetch_add(1);
    });
    if (wait) sideToggleWait(side, timeoutMs);
}

bool sideToggleBusy(Side side) {
    bool any = false;
    forSides(side, [&](int i) { any = any || busyAt(i); });
    return any;
}

bool sideToggleStalled(Side side) {
    bool any = false;
    forSides(side, [&](int i) { any = any || gChan[i].stalled.load(); });
    return any;
}

bool sideToggleWait(Side side, std::uint32_t timeoutMs) {
    const std::uint32_t start = pros::millis();
    while (sideToggleBusy(side)) {
        if (pros::millis() - start >= timeoutMs) return false;
        pros::delay(kPeriodMs);
    }
    return !sideToggleStalled(side);
}

void sideToggleManual(Side side, double power) {
    const float p = powerCap(power);
    forSides(side, [&](int i) { gChan[i].manualPower.store(p); });
}

void sideToggleCancel() {
    for (Channel& c : gChan) c.manualPower.store(0.0f);
    snapshotDrops();
    ++gCancelGen;
}

void sideToggleTare() {
    snapshotDrops();
    ++gTareGen;
}

double sideTogglePosition(Side side) {
    double sum = 0.0;
    int n = 0;
    forSides(side, [&](int i) { sum += motorFor(i).get_position(); ++n; });
    return sum / n;
}

void sideToggleSetTelemetry(std::function<void(const SideToggleSample&)> callback) {
    gTelemetry = std::move(callback);
}
