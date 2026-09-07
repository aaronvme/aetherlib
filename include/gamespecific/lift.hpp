#pragma once

#include <functional>

void liftMoveTo(double targetDegrees, double maxSpeed = 127.0,
                double timeoutMs = 3000, bool async = false);

void liftCancel();

bool liftBusy();

double liftPosition();

struct LiftSample {
    float target, position, error, pid, feedforward, power;
};

void liftSetTelemetry(std::function<void(const LiftSample&)> callback);
