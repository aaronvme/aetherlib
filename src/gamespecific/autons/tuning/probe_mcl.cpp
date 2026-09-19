#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include <cmath>

#include "pros/screen.hpp"
#include "chad/api.hpp"

namespace chad {
using namespace autons;

namespace {
constexpr int kRow = 1;
constexpr uint32_t kDrawMs = 150;

// The beam solve sweeps the whole field twice, so it is far too slow to run
// inside the drive loop. It is worth a stop anyway: a fix taken while the
// robot is rolling is a fix of where it used to be.
bool truthNow(float& tx, float& ty, int& inliers) {
    BeamRead beams[MAX_DISTANCE_SENSORS];
    readBeams(beams, chassis.getPose().theta);
    BeamPair pairs[2];
    const int n = findBeamPairs(beams, pairs);
    float bracket = 0.0f;
    return solveTruth(beams, pairs, n, 3.0f, tx, ty, bracket, &inliers);
}
}

void autonProbeMcl() {
    Report out;
    out("mcl live check");
    if (!headingOk(out)) return;
    out("park with 2 walls in view");
    waitForStart(out, "seeding from the beams");

    setFilters(false);
    auto ch = makeCharacterizer(out);
    const auto beams = ch.measureRangeNoise(distSensors);
    float px = 0.0f, py = 0.0f;
    int pinned = 0;
    bool disagree = false;
    if (!solvePark(beams, chassis.getPose().theta, px, py, pinned, &disagree)) {
        out.result("no fix to start from.");
        out.result("park nearer two walls");
        return;
    }
    const float th = chassis.getPose().theta;
    chassis.setPose({px, py, th});
    chassis.startTracking();

    pros::screen::erase();
    float worst = 0.0f;
    float tx = px, ty = py, err = 0.0f;
    bool haveTruth = true;
    int inliers = 0, checks = 0;
    uint32_t nextDraw = 0;

    while (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        if (chassis.isSettled()) {
            chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                           master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            chassis.arcade(0, 0);
            pros::delay(250);
            haveTruth = truthNow(tx, ty, inliers);
            if (haveTruth) {
                const Pose p = chassis.getPose();
                err = std::sqrt((p.x - tx) * (p.x - tx) +
                                (p.y - ty) * (p.y - ty));
                if (err > worst) worst = err;
                checks++;
            }
        }

        if (pros::millis() >= nextDraw) {
            nextDraw = pros::millis() + kDrawMs;
            const Pose p = chassis.getPose();
            const auto est = chassis.mcl().estimate();
            int row = kRow;
            pros::screen::print(pros::E_TEXT_SMALL, row++,
                                "pose  %+6.1f %+6.1f  h%4.0f  ",
                                static_cast<double>(p.x),
                                static_cast<double>(p.y),
                                static_cast<double>(stdRadToVexDeg(p.theta)));
            pros::screen::print(pros::E_TEXT_SMALL, row++,
                                "mcl   %+6.1f %+6.1f  s%4.1f ",
                                static_cast<double>(est.pose.x),
                                static_cast<double>(est.pose.y),
                                static_cast<double>(std::sqrt(est.covXX)));
            pros::screen::print(pros::E_TEXT_SMALL, row++,
                                "nEff %4.0f  gate %d  reloc %d   ",
                                static_cast<double>(est.nEff),
                                chassis.ekf().mclRejectStreak(),
                                chassis.ekf().relocateCount());
            row++;
            if (haveTruth) {
                pros::screen::print(pros::E_TEXT_SMALL, row++,
                                    "truth %+6.1f %+6.1f  n%d    ",
                                    static_cast<double>(tx),
                                    static_cast<double>(ty), inliers);
                pros::screen::print(pros::E_TEXT_SMALL, row++,
                                    "err %5.1f in   worst %5.1f  ",
                                    static_cast<double>(err),
                                    static_cast<double>(worst));
            } else {
                pros::screen::print(pros::E_TEXT_SMALL, row++,
                                    "truth: no fix there      ");
                row++;
            }
            row++;
            pros::screen::print(pros::E_TEXT_SMALL, row++,
                                "A = check vs beams, B = end");
        }
        pros::delay(20);
    }

    chassis.arcade(0, 0);
    chassis.stopTracking();
    out.result("worst error %.1f in", worst);
    out.result("last %.1f in over %d checks", err, checks);
    out.result("gate streak %d  reloc %d",
               chassis.ekf().mclRejectStreak(),
               chassis.ekf().relocateCount());
}
}
