#pragma once

#include "riku/chassis.hpp"
#include "riku/movement_control/feedforward.hpp"
#include "riku/util/jam_detector.hpp"

namespace riku::tuning {

// Streams a subsystem's telemetry to a CSV on the microSD card.
//
// Each of these installs a telemetry callback that owns a Logger; passing
// false removes the callback and closes the file. Nothing here is compiled
// into a subsystem - the subsystems only know about their sample structs.
//
// All are no-ops without a card inserted, so calls can be left in place.

/// /usd/chas<n>.csv - pose, target, per-axis controller output, settled flag.
/// One file per motion.
void logChassis(Chassis& chassis, bool enabled);

/// /usd/jam<n>.csv - per-motor efficiency, temperature and command, plus
/// state. Enable, deliberately jam the mechanism, then read the efficiency
/// band the stalled motor actually sits in.
void logJamDetector(JamDetector& jam, bool enabled);

/// /usd/ff<n>.csv - commanded and measured per-side velocity, the acceleration
/// the model saw, and the voltage produced. Least-squares fit this to recover
/// kV/kA/kS.
void logFeedforward(FeedforwardController& controller, bool enabled);

}  // namespace riku::tuning
