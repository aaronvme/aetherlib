// State shared between the chad-sim bridge translation units.
//
// bridge.cpp owns and defines all of it; the feature-specific bridge_*.cpp
// files only read it. Split out so a feature area is its own file rather than a
// section of one 1500-line file -- which is also what lets two of them be
// worked on at the same time.
#pragma once

#include "chad/movement_control/feedforward.hpp"

namespace simbridge {

/// Ticks spent in contact with field geometry, not a bool: a controller that
/// grazes a post for one tick and one that leans on it all run otherwise read
/// exactly the same.
extern int gCollideTicks;

/// Set while the browser is driving by hand. A controller taking the drive
/// clears it, so the two never fight over the motors.
extern bool gManualActive;

/// The output-stage feedforward gains. Shared rather than per-follower so that
/// comparing two followers compares the controllers and not their output
/// stages.
extern chad::FeedforwardGains gLtvGains;

}  // namespace simbridge
