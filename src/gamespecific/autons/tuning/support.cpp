#include "gamespecific/autons/tuning/support.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdio>

#include "liblvgl/lvgl.h"
#include "pros/screen.hpp"
#include "chad/api.hpp"

namespace chad::autons {

Report::Report() {
    lv_obj_t* blank = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(blank, lv_color_hex(0x000000), 0);
    lv_screen_load(blank);
    pros::delay(60);
    pros::screen::erase();
    master.set_text(0, 0, "running-hands off ");
}

Report::~Report() {
    master.set_text(0, 0, "done-ok to touch  ");
    pros::delay(60);
    master.rumble("..");
}

void Report::operator()(const char* fmt, ...) {
    char buf[128];
    va_list a;
    va_start(a, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, a);
    va_end(a);
    printf("%s\n", buf);
    if (!mPage.empty()) return;
    if (mLine > kProgressLines) {
        pros::screen::erase();
        mLine = 1;
    }
    pros::screen::print(pros::E_TEXT_SMALL, mLine++, "%s", buf);
}

void Report::result(const char* fmt, ...) {
    char buf[128];
    va_list a;
    va_start(a, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, a);
    va_end(a);
    printf("%s\n", buf);
    mPage.emplace_back(buf);
    // Medium while the page fits, small once it does not: a result you cannot
    // read is the same as a result that was never printed.
    const bool small = mPage.size() > static_cast<size_t>(kResultLines);
    const auto font = small ? pros::E_TEXT_SMALL : pros::E_TEXT_MEDIUM;
    const int cap = small ? kProgressLines : kResultLines;
    pros::screen::erase();
    int line = 1;
    for (const auto& l : mPage) {
        if (line > cap) break;
        pros::screen::print(font, line++, "%s", l.c_str());
    }
}

void setFilters(bool on) {
    if (on) {
        chassis.mcl().startTracking();
        chassis.ekf().startTracking();
    } else {
        chassis.ekf().stopTracking();
        chassis.mcl().stopTracking();
        chassis.rcl().stopTracking();
    }
    pros::delay(200);
}

bool headingOk(Report& out) {
    if (imu.is_calibrating()) {
        out.result("IMU calibrating. wait, rerun");
        return false;
    }
    if (!std::isfinite(imu.get_heading())) {
        out.result("IMU not reporting");
        out.result("check the port, then rerun");
        return false;
    }
    const uint32_t before = chassis.odom().imuFaults();
    pros::delay(120);
    const uint32_t climbed = chassis.odom().imuFaults() - before;
    if (climbed > 0) {
        out.result("IMU dropping out (%lu/120ms)",
                   static_cast<unsigned long>(climbed));
        out.result("check the cable, then rerun");
        return false;
    }
    return true;
}

std::function<float()> tiltProbe() {
    const float baseP = static_cast<float>(imu.get_pitch());
    const float baseR = static_cast<float>(imu.get_roll());
    return [baseP, baseR]() {
        const float p = std::fabs(static_cast<float>(imu.get_pitch()) - baseP);
        const float r = std::fabs(static_cast<float>(imu.get_roll()) - baseR);
        return std::max(std::isfinite(p) ? p : 0.0f, std::isfinite(r) ? r : 0.0f);
    };
}

void waitForA(Report& out, const char* why) {
    out.result("%s", why);
    out.result("press A to continue");
    master.set_text(0, 0, "measure, press A  ");
    pros::delay(60);
    master.rumble("-");
    const uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < 180000u) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) break;
        pros::delay(20);
    }
    while (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) pros::delay(20);
    master.set_text(0, 0, "running-hands off ");
}

bool solvePark(const tuning::RangeNoiseFit& beams, float theta,
               float& cx, float& cy, int& pinned) {
    bool gotX = false, gotY = false;
    const float c = std::cos(theta), s = std::sin(theta);
    for (int i = 0; i < MAX_DISTANCE_SENSORS; ++i) {
        if (beams.samples[i] <= 8) continue;
        const Pose& m = sensorMounts[i];
        const float ox = m.x * c - m.y * s;
        const float oy = m.x * s + m.y * c;
        const float aw = theta + m.theta;
        const float bx = std::cos(aw), by = std::sin(aw);
        const float d = beams.meanIn[i];
        if (std::fabs(bx) > 0.7f) {
            cx = (bx > 0.0f ? Field::FIELD_HALF_WALL : -Field::FIELD_HALF_WALL) -
                 d * bx - ox;
            gotX = true;
        } else if (std::fabs(by) > 0.7f) {
            cy = (by > 0.0f ? Field::FIELD_HALF_WALL : -Field::FIELD_HALF_WALL) -
                 d * by - oy;
            gotY = true;
        }
    }
    pinned = (gotX ? 1 : 0) + (gotY ? 1 : 0);
    return gotX && gotY;
}

tuning::Characterizer makeCharacterizer(Report& out) {
    tuning::CharacterizeConfig cfg;
    Report* sink = &out;
    cfg.onProgress = [sink](const char* stage, const char* detail) {
        (*sink)("[%s] %s", stage, detail);
    };
    return tuning::Characterizer(chassis, drivetrain, cfg);
}

}  // namespace chad::autons
