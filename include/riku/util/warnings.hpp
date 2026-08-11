/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// Adapted for riku: devices are passed in rather than read from globals,
// and the temperature task no longer exits after the first overheat.

#pragma once

#include <vector>

#include "pros/distance.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

namespace riku {

// Each spawns a background task. Referenced devices must outlive it.

/// Backs off 10 s after a warning, then resumes.
void startTempWarning(pros::Controller& controller,
                      const std::vector<pros::MotorGroup*>& groups,
                      double tempLimitC = 55.0, double batteryLimitC = 45.0);

void startMotorDisconnectWarning(pros::Controller& controller);

void startDistanceDisconnectWarning(pros::Controller& controller,
                                    const std::vector<pros::Distance*>& sensors);

}  // namespace riku
