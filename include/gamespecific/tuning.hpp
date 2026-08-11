#pragma once

/// lift<n>.csv, one file per liftMoveTo. Logs the PID and gravity terms
/// separately so kG's effect is visible. No-op without an SD card.
void logLift(bool enabled);
