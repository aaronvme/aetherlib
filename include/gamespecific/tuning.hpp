#pragma once

// Game-specific tuning logs. The library-side equivalents live in
// riku/util/tuning.hpp.

/// /usd/lift<n>.csv - target, position, error, and the PID and gravity
/// feedforward contributions separately, so kG's effect is visible while
/// tuning it. One file per liftMoveTo. No-op without an SD card.
void logLift(bool enabled);
