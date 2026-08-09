#pragma once

#include <functional>

void liftMoveTo(double targetDegrees, double maxSpeed = 127.0,
                double timeoutMs = 3000, bool async = false);

/// One lift control tick, for tuning.
struct LiftSample {
    float target, position, error, pid, feedforward, power;
};

/// Called once per lift control tick. Empty by default. See
/// gamespecific/tuning.hpp for a hook that streams this to the SD card.
void liftSetTelemetry(std::function<void(const LiftSample&)> callback);
