#pragma once

void liftMoveTo(double targetDegrees, double maxSpeed = 127.0,
                double timeoutMs = 3000, bool async = false);

/// Log target/position/error/PID/feedforward/power to /usd/lift<n>.csv for the
/// duration of each liftMoveTo. Off by default; no-op without an SD card.
void liftSetLogging(bool enabled);
