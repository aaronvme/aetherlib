#pragma once

#include "riku/chassis.hpp"
#include "riku/movement_control/feedforward.hpp"
#include "riku/util/jam_detector.hpp"

namespace riku::tuning {

// Attach a telemetry hook that writes CSV to the SD card. No-op without a card.

/// chas<n>.csv, one file per motion.
void logChassis(Chassis& chassis, bool enabled);

/// jam<n>.csv. Enable, deliberately jam the mechanism, then read off the
/// efficiency band the stalled motor sits in.
void logJamDetector(JamDetector& jam, bool enabled);

/// ff<n>.csv. Least-squares fit this to recover kV/kA/kS.
void logFeedforward(FeedforwardController& controller, bool enabled);

}  // namespace riku::tuning
