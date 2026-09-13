#pragma once

#include <functional>

enum class LiftKind { Cascade, Pivoted };

void liftMoveTo(double targetDegrees, double maxSpeed = 127.0,
                double timeoutMs = 3000, bool async = false);

void liftCancel();

bool liftBusy();

double liftPosition();

// inches above the floor, NaN until the geometry in lift.cpp is measured
double liftHeight();

double liftHeightAt(double motorDegrees);

// motor power that holds the lift against gravity where it is
double liftHoldPower();

// pin 0 is the bottom stop, 1..kLiftPinCount are the scoring pins
constexpr int kLiftPinCount = 5;
constexpr int kLiftPresetCount = kLiftPinCount + 1;

// how far above a pin auton rides so it clears going in and coming out
constexpr double kLiftPinClearanceDeg = 35.0;

// motor degrees at the exact pin height
double liftPinDegrees(int pin);

// the pin sitting closest to a motor position
int liftNearestPin(double motorDegrees);

// drive to the exact pin height
void liftToPin(int pin, double timeoutMs = 2500, double maxSpeed = 100.0,
               bool async = false);

// pin height plus clearance; a positive extra is always physically upward,
// and 0 or less means kLiftPinClearanceDeg
void liftToPinAbove(int pin, double extraDegrees = 0.0,
                    double timeoutMs = 2500, double maxSpeed = 100.0,
                    bool async = false);

struct LiftSample {
    float target, position, error, pid, feedforward, power;
};

void liftSetTelemetry(std::function<void(const LiftSample&)> callback);
