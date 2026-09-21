#pragma once

// The claw grips extended and opens retracted. Everything here says open or
// closed so the polarity lives in one place.

// How close something has to get before auto-grab bites.
constexpr double kClawGrabRangeIn = 4.0;

// How much clear space it has to see afterwards before it is allowed to bite
// again. Without this, releasing an object would just clamp back onto it.
constexpr double kClawClearRangeIn = 4.5;

// inches to whatever the claw sensor sees, NaN when there is no return
double clawDistance();

// something is sitting inside kClawGrabRangeIn right now
bool clawSees();

bool clawClosed();
void clawClose();
void clawOpen();
void clawToggle();

// Starts the watcher task. Safe to call twice.
void startClawWatch();

void clawSetAutoGrab(bool enabled);
bool clawAutoGrab();

// Wait for something to come into range and close on it. Returns false and
// leaves the claw alone if nothing showed up in time. Independent of the
// auto-grab mode -- a routine that calls this is asking for it outright.
bool clawGrabWhenClose(double timeoutMs = 1000.0);
