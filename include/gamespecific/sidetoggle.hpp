#pragma once

#include <cstdint>
#include <functional>

// One 5.5 W motor per side, direct drive: a half turn of the motor is one
// toggle flip. Each side only ever spins one way. Gains and powers are at the
// top of sidetoggle.cpp. Side::Both applies a call to both sides.
enum class Side { Left, Right, Both };

// Driver control powers, out of 127: R2 hold, and the Up-button home.
extern const double kSideToggleManualPower;
extern const double kSideToggleDriverHomePower;

// Brake HOLD, index 0 at the current position, starts the 10 ms task.
// Safe to call twice.
void startSideToggle();

// Queue flips (half turns). Queued flips add up and never reverse. With wait,
// blocks until done, stalled or timeoutMs; the timeout only ends the wait.
// maxSpeed (out of 127) caps the move; 0 means full speed.
void sideToggleFlip(Side side, int flips = 1, bool wait = false,
                    std::uint32_t timeoutMs = 1000, double maxSpeed = 0.0);

// Spin forward to the next half-turn index (a multiple of 180 from startup),
// the shortest way since it only turns one way. Replaces any queued flips. Up
// to 10 deg past an index counts as there. maxSpeed 0 means full speed.
void sideToggleHome(Side side, bool wait = false, std::uint32_t timeoutMs = 1500,
                    double maxSpeed = 0.0);

// Both: either side busy.
bool sideToggleBusy(Side side);

// The last move jammed and was dropped; the next flip retries it.
// Both: either side stalled.
bool sideToggleStalled(Side side);

// False on timeout or stall. Both waits for both sides.
bool sideToggleWait(Side side, std::uint32_t timeoutMs = 1000);

// Spin forward at power (0..127) while nonzero; 0 releases and holds where it
// stopped. Overrides and drops any flip or home on that side. The next flip
// goes to the next index ahead of where it was left, so manual travel is
// never caught up.
void sideToggleManual(Side side, double power);

// Drop flips, homes and manual drive on both sides; they hold where they are.
void sideToggleCancel();

// Make the current position index 0 on both sides and drop queued flips.
void sideToggleTare();

// Raw motor degrees, NaN if unplugged. Both: the mean of the two.
double sideTogglePosition(Side side);

struct SideToggleSample {
    Side side;  // Left or Right
    float target, position, error, power;
    bool stalled;
};

// Called from the side-toggle task every tick for each side; set it before
// flipping.
void sideToggleSetTelemetry(std::function<void(const SideToggleSample&)> callback);
