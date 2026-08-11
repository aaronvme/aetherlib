#pragma once

#include <functional>

void liftMoveTo(double targetDegrees, double maxSpeed = 127.0,
                double timeoutMs = 3000, bool async = false);

// One lift control tick.
struct LiftSample {
    float target, position, error, pid, feedforward, power;
};

/// Empty by default. gamespecific/tuning.hpp has a hook that logs to the SD card.
void liftSetTelemetry(std::function<void(const LiftSample&)> callback);
