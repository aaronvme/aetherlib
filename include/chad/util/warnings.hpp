#pragma once

#include <vector>

#include "pros/distance.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

namespace chad {
void startTempWarning(pros::Controller& controller,
                      const std::vector<pros::MotorGroup*>& groups,
                      double tempLimitC = 55.0, double batteryLimitC = 45.0);

void startMotorDisconnectWarning(pros::Controller& controller);

void startDistanceDisconnectWarning(pros::Controller& controller,
                                    const std::vector<pros::Distance*>& sensors);
}
