#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include <cmath>
#include <cstdio>

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
        out.result("no fix: %d/2 axes seen", pinned);
        out.result(disagree ? "beams fight the post map"
                                : "need a wall on each axis");
        for (int i = 0; i < MAX_DISTANCE_SENSORS; ++i) {
            if (beams.samples[i] > 8) {
                out.result("  s%d %.1f in", i, beams.meanIn[i]);
            }
        }
        return;
    }
    const float th = chassis.getPose().theta;
    chassis.setPose({px, py, th});
    chassis.startTracking();

    pros::screen::erase();
    // Fully buffered stdout holds short lines until 4KB accumulates, which
    // for a 100-byte telemetry line is most of a minute and for a two-line
    // failure message is forever.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    // The screen rows go nowhere a log can reach. These do: printf is what
    // `pros terminal` streams, so the run can be read back off the wire
    // instead of off the brain.
    std::printf("mclhdr,ms,px,py,ph,mx,my,sx,sy,neff,inj,gate,reloc,"
                "rate,held,r0,r1,r2,r3\n");
    const uint32_t tStart = pros::millis();
    float worst = 0.0f;
    float tx = px, ty = py, err = 0.0f;
    bool haveTruth = true;
    int inliers = 0, checks = 0;
    uint32_t nextDraw = 0;
    int ctlPage = 0;

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
                std::printf("mclchk,%lu,%.2f,%.2f,%d,%.2f\n",
                            static_cast<unsigned long>(pros::millis() - tStart),
                            static_cast<double>(tx), static_cast<double>(ty),
                            inliers, static_cast<double>(err));
            } else {
                std::printf("mclchk,%lu,nofix\n",
                            static_cast<unsigned long>(pros::millis() - tStart));
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

            // Report holds the controller now, so this routine has to put
            // its own numbers there. One line per pass: master.print drops
            // anything sent inside 50 ms.
            if (ctlPage == 0) {
                ctlLine(0, "X%-6.1f Y%-6.1f", static_cast<double>(p.x),
                        static_cast<double>(p.y));
            } else if (ctlPage == 1) {
                ctlLine(1, "H%-4.0f nEff%-5.0f",
                        static_cast<double>(stdRadToVexDeg(p.theta)),
                        static_cast<double>(est.nEff));
            } else if (haveTruth) {
                ctlLine(2, "err%-5.1f wst%-5.1f", static_cast<double>(err),
                        static_cast<double>(worst));
            } else {
                ctlLine(2, "A=check  B=end");
            }
            ctlPage = (ctlPage + 1) % 3;

            const auto& rd = chassis.mcl().sensorReadings();
            const auto& ok = chassis.mcl().validSensors();
            std::printf("mcl,%lu,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,"
                        "%.0f,%.3f,%d,%d",
                        static_cast<unsigned long>(pros::millis() - tStart),
                        static_cast<double>(p.x), static_cast<double>(p.y),
                        static_cast<double>(stdRadToVexDeg(p.theta)),
                        static_cast<double>(est.pose.x),
                        static_cast<double>(est.pose.y),
                        static_cast<double>(std::sqrt(est.covXX)),
                        static_cast<double>(std::sqrt(est.covYY)),
                        static_cast<double>(est.nEff),
                        static_cast<double>(chassis.mcl().randomInjectFraction()),
                        chassis.ekf().mclRejectStreak(),
                        chassis.ekf().relocateCount());
            std::printf(",%.0f,%d",
                        static_cast<double>(
                            chassis.mcl().latestTurnRateRadS() * RAD_TO_DEG),
                        chassis.mcl().sensorsHeld() ? 1 : 0);
            for (int i = 0; i < 4; ++i) {
                if (ok[i]) std::printf(",%.2f", static_cast<double>(rd[i]));
                else std::printf(",");
            }
            std::printf("\n");
            std::fflush(stdout);
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
