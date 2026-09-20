#include "gamespecific/autons/tuning/support.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <string>

#include "liblvgl/lvgl.h"
#include "pros/screen.hpp"
#include "chad/api.hpp"
#include "gamespecific/controls.hpp"

namespace chad::autons {
Report::Report() {
    // The telemetry task rewrites lines 0 and 1 every 60 ms, so a routine
    // that does not claim the controller is talking over itself.
    setPoseTelemetryEnabled(false);
    lv_obj_t* blank = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(blank, lv_color_hex(0x000000), 0);
    lv_screen_load(blank);
    pros::delay(60);
    pros::screen::erase();
    ctlLine(0, "run: hands off");
}

Report::~Report() {
    ctlLine(0, "done: touch ok");
    pros::delay(60);
    master.rumble("..");
    setPoseTelemetryEnabled(true);
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

void Report::clear() {
    if (!mPage.empty()) return;
    pros::screen::erase();
    mLine = 1;
}

void ctlLine(int line, const char* fmt, ...) {
    char buf[64];
    va_list a;
    va_start(a, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, a);
    va_end(a);
    master.print(line, 0, "%-15.15s", buf);
}

void setFilters(bool on) {
    // Starting the filters by hand skipped what Chassis::startTracking does
    // to keep them out of each other's way: with EKF running, MCL's autoSync
    // has to be off, or both of them write corrections into the same odom
    // pose and it jitters between them. Anything that then drives to a field
    // coordinate -- moveDistance included -- chases that.
    if (on) {
        chassis.startTracking();
    } else {
        chassis.stopTracking();
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
    ctlLine(0, "measure-press A");
    pros::delay(60);
    master.rumble("-");
    const uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < 180000u) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) break;
        pros::delay(20);
    }
    while (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) pros::delay(20);
    ctlLine(0, "run: hands off");
}

void waitForStart(Report& out, const char* what) {
    out("%s", what);
    out("A starts. live beams below:");
    ctlLine(0, "place, press A");
    pros::delay(60);
    master.rumble("-");

    const uint32_t t0 = pros::millis();
    uint32_t nextDraw = 0;
    int page = 0;
    while (pros::millis() - t0 < 180000u) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) break;
        if (pros::millis() >= nextDraw) {
            nextDraw = pros::millis() + 250;

            float in[MAX_DISTANCE_SENSORS] = {};
            int row = kBeamRow;
            for (int i = 0; i < MAX_DISTANCE_SENSORS; ++i) {
                if (distSensors[i] == nullptr) continue;
                const int mm = distSensors[i]->get();
                const int conf = distSensors[i]->get_confidence();
                in[i] = static_cast<float>(mm) / 25.4f;
                pros::screen::print(pros::E_TEXT_SMALL, row++,
                                    "s%d %6d mm  c%3d  %6.1f in   ", i, mm,
                                    conf, static_cast<double>(in[i]));
            }
            // The brain is on the robot and the robot is against a wall, so
            // put it on the controller too.
            if (page == 0) {
                ctlLine(1, "F%-5.1f L%-5.1f", static_cast<double>(in[0]),
                        static_cast<double>(in[1]));
            } else {
                ctlLine(2, "B%-5.1f R%-5.1f", static_cast<double>(in[2]),
                        static_cast<double>(in[3]));
            }
            page ^= 1;
        }
        pros::delay(20);
    }
    while (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) pros::delay(20);
    ctlLine(0, "run: hands off");
}

bool solvePark(const tuning::RangeNoiseFit& beams, float theta,
               float& cx, float& cy, int& pinned, bool* mapDisagrees) {
    if (mapDisagrees != nullptr) *mapDisagrees = false;

    BeamRead reads[MAX_DISTANCE_SENSORS];
    bool gotX = false, gotY = false;
    const auto cfg = chassis.mcl().config();
    for (int i = 0; i < MAX_DISTANCE_SENSORS; ++i) {
        reads[i] = BeamRead{};
        if (distSensors[i] == nullptr) continue;
        const Pose& m = sensorMounts[i];
        const float aw = theta + m.theta;
        reads[i].present = true;
        reads[i].bx = std::cos(aw);
        reads[i].by = std::sin(aw);
        reads[i].alongX = tuning::mountAlongAxis(m.x, m.y, theta, true);
        reads[i].alongY = tuning::mountAlongAxis(m.x, m.y, theta, false);
        if (beams.samples[i] <= 8) continue;

        reads[i].valid = true;
        reads[i].inches = beams.meanIn[i];
        reads[i].sigmaIn = beams.sigmaIn[i] > 0.0f
                               ? beams.sigmaIn[i]
                               : tuning::beamSigmaBase(beams.meanIn[i],
                                                       cfg.rangeSigmaCloseIn,
                                                       cfg.rangeSigmaProp);
        if (std::fabs(reads[i].bx) > 0.7f) gotX = true;
        else if (std::fabs(reads[i].by) > 0.7f) gotY = true;
    }

    pinned = (gotX ? 1 : 0) + (gotY ? 1 : 0);
    if (pinned < 2) return false;

    if (kFieldIsClear) return solveWallTruthClear(reads, cx, cy);

    int inliers = 0;
    if (solveWallTruthMapped(reads, cx, cy, inliers)) return true;
    if (mapDisagrees != nullptr) *mapDisagrees = true;
    return false;
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

        // Geometry is filled for every fitted sensor, reading or not: a
        // sensor that stayed quiet has still told the solve something.
        const Pose& m = sensorMounts[i];
        const float aw = robotThetaRad + m.theta;
        out[i].present = true;
        out[i].bx = std::cos(aw);
        out[i].by = std::sin(aw);
        out[i].alongX = tuning::mountAlongAxis(m.x, m.y, robotThetaRad, true);
        out[i].alongY = tuning::mountAlongAxis(m.x, m.y, robotThetaRad, false);

        const int mm = distSensors[i]->get();
        if (mm <= static_cast<int>(cfg.minSensorRange) || mm > 9000) continue;
        if (distSensors[i]->get_confidence() < cfg.confidenceThreshold) continue;
        const float inches = static_cast<float>(mm) / 25.4f;
        if (inches > cfg.maxSensorRange) continue;

        out[i].valid = true;
        out[i].inches = inches;
        out[i].sigmaIn = tuning::beamSigmaBase(inches, cfg.rangeSigmaCloseIn,
                                               cfg.rangeSigmaProp);
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

namespace {
int loadFieldCircles(tuning::MapCircle* out, int cap) {
    int n = 0;
    for (const auto& c : fieldCircleObstacles()) {
        if (n >= cap) break;
        out[n++] = {c.x, c.y, c.radius};
    }
    return n;
}
}

bool solveWallTruthMapped(const BeamRead* beams, float& cx, float& cy,
                          int& inliers) {
    inliers = 0;
    if (beams == nullptr) return false;

    tuning::MapCircle circles[kMaxFieldCircles];
    const int n = loadFieldCircles(circles, kMaxFieldCircles);

    const auto fix = tuning::solveMapped(beams, MAX_DISTANCE_SENSORS, circles,
                                         n, Field::FIELD_HALF_WALL,
                                         chassis.mcl().config().maxSensorRange);
    if (!fix.trusted) return false;
    cx = fix.x;
    cy = fix.y;
    inliers = fix.inliers;
    return true;
}

bool solveTruth(const BeamRead* beams, const BeamPair* pairs, int nPairs,
                float kSigmas, float& cx, float& cy, float& worstBracketIn,
                int* inliers) {
    worstBracketIn = 0.0f;
    if (inliers != nullptr) *inliers = 0;
    if (kFieldIsClear) {
        return solveWallTruthClear(beams, cx, cy);
    }
    if (solveWallTruth(beams, pairs, nPairs, kSigmas, cx, cy,
                       worstBracketIn)) {
        if (inliers != nullptr) *inliers = kBracketedFix;
        return true;
    }
    int n = 0;
    if (!solveWallTruthMapped(beams, cx, cy, n)) return false;
    if (inliers != nullptr) *inliers = n;
    return true;
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
