/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// Adapted for chad: devices are passed in rather than read from globals,
// and the temperature task no longer exits after the first overheat.

#include "chad/util/warnings.hpp"

#include <string>

#include "pros/rtos.hpp"

namespace chad {

namespace {

constexpr int kControllerDelayMs = 50;
constexpr int kPollDelayMs = 300;
constexpr int kWarnHoldMs = 1000;
constexpr int kOverheatBackoffMs = 10000;

/// Shared disconnect-warning loop for motors and distance sensors.
/// `probe` appends the ports of every disconnected device and returns true if
/// any were found.
template <typename Probe>
void disconnectLoop(pros::Controller* controller, const char* prefix, Probe probe) {
    std::vector<unsigned char> disconnected;
    std::vector<unsigned char> lastDisconnected;
    bool isNotified = false;

    while (true) {
        std::string message = prefix;
        const bool anyDisconnected = probe(disconnected, message);

        if (anyDisconnected) {
            controller->clear_line(0);
            pros::delay(kControllerDelayMs);
            controller->set_text(0, 0, message);
            isNotified = true;
            pros::delay(kControllerDelayMs);
        }

        if (disconnected.size() > lastDisconnected.size()) {
            controller->rumble(". . .");
        } else if (disconnected.size() < lastDisconnected.size()) {
            controller->rumble("-");
        } else if (!anyDisconnected && isNotified) {
            controller->clear_line(0);
            isNotified = false;
        }

        lastDisconnected = disconnected;
        disconnected.clear();
        pros::delay(kPollDelayMs);
    }
}

}  // namespace

void startTempWarning(pros::Controller& controller,
                      const std::vector<pros::MotorGroup*>& groups,
                      double tempLimitC, double batteryLimitC) {
    pros::Controller* ctrl = &controller;
    std::vector<pros::MotorGroup*> monitored = groups;

    pros::Task([ctrl, monitored, tempLimitC, batteryLimitC]() {
        while (true) {
            bool isOverheating = false;

            for (std::size_t g = 0; g < monitored.size() && !isOverheating; ++g) {
                if (monitored[g] == nullptr) continue;
                const std::vector<double> temps = monitored[g]->get_temperature_all();
                for (std::size_t i = 0; i < temps.size(); ++i) {
                    if (temps[i] > tempLimitC) {
                        ctrl->rumble(".-.");
                        ctrl->clear_line(0);
                        ctrl->print(0, 0, "OVERHEAT! G%i-%i", (int)g + 1, (int)i + 1);
                        pros::delay(kWarnHoldMs);
                        ctrl->clear_line(0);
                        isOverheating = true;
                        break;
                    }
                }
            }

            if (pros::battery::get_temperature() >= batteryLimitC) {
                ctrl->rumble(".-.");
                ctrl->clear_line(0);
                ctrl->print(0, 0, "Battery Overheat! %i",
                            (int)pros::battery::get_temperature());
                pros::delay(kWarnHoldMs);
                ctrl->clear_line(0);
                isOverheating = true;
            }

            pros::delay(200);
            // Upstream returned here, permanently ending temperature
            // monitoring after the first overheat. Back off and keep going.
            if (isOverheating) pros::delay(kOverheatBackoffMs);
        }
    });
}

void startMotorDisconnectWarning(pros::Controller& controller) {
    pros::Controller* ctrl = &controller;

    pros::Task([ctrl]() {
        std::vector<pros::Motor> allMotors = pros::Motor::get_all_devices();

        disconnectLoop(ctrl, "MD: ",
                       [&allMotors](std::vector<unsigned char>& disconnected,
                                    std::string& message) {
                           bool any = false;
                           for (pros::Motor& m : allMotors) {
                               if (!m.is_installed()) {
                                   const unsigned char port = m.get_port();
                                   disconnected.push_back(port);
                                   message += " " + std::to_string(port);
                                   any = true;
                               }
                           }
                           return any;
                       });
    });
}

void startDistanceDisconnectWarning(pros::Controller& controller,
                                    const std::vector<pros::Distance*>& sensors) {
    pros::Controller* ctrl = &controller;
    std::vector<pros::Distance*> monitored = sensors;

    pros::Task([ctrl, monitored]() {
        disconnectLoop(ctrl, "DSD: ",
                       [&monitored](std::vector<unsigned char>& disconnected,
                                    std::string& message) {
                           bool any = false;
                           for (pros::Distance* s : monitored) {
                               if (s == nullptr) continue;
                               if (!s->is_installed()) {
                                   const unsigned char port = s->get_port();
                                   disconnected.push_back(port);
                                   message += " " + std::to_string(port);
                                   any = true;
                               }
                           }
                           return any;
                       });
    });
}

}  // namespace chad
