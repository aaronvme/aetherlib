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
            cx = tuning::solveAxisSingle(d, bx, ox, Field::FIELD_HALF_WALL);
            gotX = true;
        } else if (std::fabs(by) > 0.7f) {
            cy = tuning::solveAxisSingle(d, by, oy, Field::FIELD_HALF_WALL);
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

int readBeams(BeamRead* out, float robotThetaRad) {
    if (out == nullptr) return 0;
    const auto cfg = chassis.mcl().config();
    int n = 0;
    for (int i = 0; i < MAX_DISTANCE_SENSORS; ++i) {
        out[i] = BeamRead{};
        if (distSensors[i] == nullptr) continue;
        const int mm = distSensors[i]->get();
        if (mm <= static_cast<int>(cfg.minSensorRange) || mm > 9000) continue;
        if (distSensors[i]->get_confidence() < cfg.confidenceThreshold) continue;
        const float inches = static_cast<float>(mm) / 25.4f;
        if (inches > cfg.maxSensorRange) continue;

        const Pose& m = sensorMounts[i];
        const float aw = robotThetaRad + m.theta;
        out[i].valid = true;
        out[i].inches = inches;
        out[i].sigmaIn = tuning::beamSigmaBase(inches, cfg.rangeSigmaCloseIn,
                                               cfg.rangeSigmaProp);
        out[i].bx = std::cos(aw);
        out[i].by = std::sin(aw);
        out[i].alongX = tuning::mountAlongAxis(m.x, m.y, robotThetaRad, true);
        out[i].alongY = tuning::mountAlongAxis(m.x, m.y, robotThetaRad, false);
        ++n;
    }
    return n;
}

int findBeamPairs(const BeamRead* beams, BeamPair* out) {
    if (beams == nullptr || out == nullptr) return 0;
    bool haveX = false, haveY = false;
    int n = 0;
    for (int i = 0; i < MAX_DISTANCE_SENSORS && n < 2; ++i) {
        if (!beams[i].valid) continue;
        for (int j = i + 1; j < MAX_DISTANCE_SENSORS && n < 2; ++j) {
            if (!beams[j].valid) continue;
            const float dot = beams[i].bx * beams[j].bx + beams[i].by * beams[j].by;
            if (dot > -0.98f) continue;

            BeamPair p;
            if (std::fabs(beams[i].bx) > 0.7f) {
                if (haveX) continue;
                p.xAxis = true;
                p.plus = beams[i].bx > 0.0f ? i : j;
                p.minus = beams[i].bx > 0.0f ? j : i;
                haveX = true;
            } else if (std::fabs(beams[i].by) > 0.7f) {
                if (haveY) continue;
                p.xAxis = false;
                p.plus = beams[i].by > 0.0f ? i : j;
                p.minus = beams[i].by > 0.0f ? j : i;
                haveY = true;
            } else {
                continue;
            }
            out[n++] = p;
        }
    }
    return n;
}

bool solveWallTruthClear(const BeamRead* beams, float& cx, float& cy) {
    if (beams == nullptr) return false;
    float bestX = 1e9f, bestY = 1e9f;

    for (int i = 0; i < MAX_DISTANCE_SENSORS; ++i) {
        const BeamRead& b = beams[i];
        if (!b.valid) continue;

        if (std::fabs(b.bx) > 0.7f) {
            if (b.inches >= bestX) continue;
            bestX = b.inches;
            cx = tuning::solveAxisSingle(b.inches, b.bx, b.alongX,
                                        Field::FIELD_HALF_WALL);
        } else if (std::fabs(b.by) > 0.7f) {
            if (b.inches >= bestY) continue;
            bestY = b.inches;
            cy = tuning::solveAxisSingle(b.inches, b.by, b.alongY,
                                        Field::FIELD_HALF_WALL);
        }
    }
    return bestX < 1e9f && bestY < 1e9f;
}

bool solveTruth(const BeamRead* beams, const BeamPair* pairs, int nPairs,
                float kSigmas, float& cx, float& cy, float& worstBracketIn) {
    worstBracketIn = 0.0f;
    if (kFieldIsClear) return solveWallTruthClear(beams, cx, cy);
    return solveWallTruth(beams, pairs, nPairs, kSigmas, cx, cy,
                          worstBracketIn);
}

bool solveWallTruth(const BeamRead* beams, const BeamPair* pairs, int nPairs,
                    float kSigmas, float& cx, float& cy,
                    float& worstBracketIn) {
    worstBracketIn = 0.0f;
    if (beams == nullptr || pairs == nullptr) return false;
    bool gotX = false, gotY = false;

    for (int k = 0; k < nPairs; ++k) {
        const BeamPair& p = pairs[k];
        if (p.plus < 0 || p.minus < 0) continue;
        const BeamRead& bp = beams[p.plus];
        const BeamRead& bm = beams[p.minus];
        if (!bp.valid || !bm.valid) continue;

        const float oPlus = p.xAxis ? bp.alongX : bp.alongY;
        const float oMinus = p.xAxis ? bm.alongX : bm.alongY;
        const float dPlus = p.xAxis ? bp.bx : bp.by;
        const float dMinus = p.xAxis ? bm.bx : bm.by;

        const float sp = bp.sigmaIn * dPlus;
        const float sm = bm.sigmaIn * dMinus;
        const float tol = kSigmas * std::sqrt(sp * sp + sm * sm);
        const auto w = tuning::solveAxisBracketed(
            bp.inches, dPlus, oPlus, bm.inches, dMinus, oMinus,
            Field::FIELD_HALF_WALL, tol);
        const float width = std::fabs(w.bracketIn);
        if (width > worstBracketIn) worstBracketIn = width;
        if (!w.trusted) continue;
        if (p.xAxis) { cx = w.value; gotX = true; }
        else { cy = w.value; gotY = true; }
    }
    return gotX && gotY;
}
}
