#pragma once

// The claw grips retracted and opens extended. Everything here says open or
// closed so the polarity lives in one place.

// How close something has to get before auto-grab bites.
constexpr double kClawGrabRangeIn = 3.0;

// How much clear space it has to see afterwards before it is allowed to bite
// again. Without this, releasing an object would just clamp back onto it.
constexpr double kClawClearRangeIn = 4.5;

// How far ahead auto-grab closes on something the robot is backing into:
// distance-sensor latency plus the time the jaws take to shut. Raise it if
// the claw still shuts behind the pin at speed, lower it if it shuts in
// front of the pin.
constexpr double kClawLeadMs = 50.0;

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
