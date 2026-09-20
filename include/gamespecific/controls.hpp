#pragma once

namespace chad {
void startPoseTelemetry();

// The controller has three lines and the telemetry task owns two of them.
// Anything that wants to say something there has to ask for them first.
void setPoseTelemetryEnabled(bool enabled);

void driverControl();
}
