#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include <cmath>
#include <cstdio>

#include "pros/screen.hpp"
#include "chad/api.hpp"

namespace chad {
using namespace autons;

namespace {
constexpr int kWin = 16;
constexpr uint32_t kDrawMs = 150;
constexpr float kDriveScale = 0.40f;

// Front, left, back, right. The CSV columns are fixed at four, so anything
// mounted past those is out of scope for this probe rather than silently
// widening every row.
constexpr int kBeams = 4;
static_assert(kBeams <= MAX_DISTANCE_SENSORS, "kBeams exceeds the sensor array");

struct Beam {
    float win[kWin]{};
    int   fill{0};
    int   idx{0};
    double sum{0.0};
    double sumSq{0.0};
    int   n{0};
    float last{0.0f};
    int   conf{0};
    bool  ok{false};

    void push(float in) {
        last = in;
        win[idx] = in;
        idx = (idx + 1) % kWin;
        if (fill < kWin) ++fill;
        sum += in;
        sumSq += static_cast<double>(in) * static_cast<double>(in);
        ++n;
    }

    bool settled() const { return fill >= kWin; }

    float windowMean() const {
        if (fill == 0) return 0.0f;
        float s = 0.0f;
        for (int i = 0; i < fill; ++i) s += win[i];
        return s / static_cast<float>(fill);
    }

    float mean() const {
        return n > 0 ? static_cast<float>(sum / n) : 0.0f;
    }

    float sd() const {
        if (n < 2) return 0.0f;
        const double m = sum / n;
        const double v = sumSq / n - m * m;
        return v > 0.0 ? static_cast<float>(std::sqrt(v)) : 0.0f;
    }

    void reset() { fill = 0; idx = 0; sum = 0.0; sumSq = 0.0; n = 0; }
};

// Two sensors facing opposite ways read the wall separation minus the span
// between their own mounts, wherever the robot sits. That makes the sum a
// measurement of the map alone, with the pose divided out.
float predictedSum(float spanIn) {
    return 2.0f * Field::FIELD_HALF_WALL - spanIn;
}
}

void autonProbeGeometry() {
    Report out;
    out("beam geometry probe");
    if (!headingOk(out)) return;
    out("park mid-lane with one");
    out("opposing pair on two walls");
    waitForStart(out, "then wiggle slow, watch min");

    setFilters(false);

    const int confMin = chassis.mcl().config().confidenceThreshold;
    const float spanLR = sensorMounts[1].y - sensorMounts[3].y;
    const float spanFB = sensorMounts[0].x - sensorMounts[2].x;
    const float predLR = predictedSum(spanLR);
    const float predFB = predictedSum(spanFB);

    Beam b[kBeams];
    float minLR = 0.0f, minFB = 0.0f;
    bool haveMinLR = false, haveMinFB = false;

    pros::screen::erase();
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::printf("geohdr,wall,%.2f,spanLR,%.2f,predLR,%.2f,spanFB,%.2f,predFB,%.2f\n",
                static_cast<double>(2.0f * Field::FIELD_HALF_WALL),
                static_cast<double>(spanLR), static_cast<double>(predLR),
                static_cast<double>(spanFB), static_cast<double>(predFB));
    std::printf("geocols,ms,h,s0,s1,s2,s3,c0,c1,c2,c3,lr,dlr,fb,dfb,minlr,minfb,px,py\n");
    std::printf("geomarkcols,ms,mark,m0,sd0,n0,m1,sd1,n1,m2,sd2,n2,m3,sd3,n3,"
                "lr,dlr,fb,dfb\n");

    const uint32_t tStart = pros::millis();
    uint32_t nextDraw = 0;
    int marks = 0;
    int ctlPage = 0;

    while (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        if (chassis.isSettled()) {
            chassis.arcade(
                kDriveScale * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                kDriveScale * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        }

        for (int i = 0; i < kBeams; ++i) {
            if (distSensors[i] == nullptr) { b[i].ok = false; continue; }
            const int mm = distSensors[i]->get();
            b[i].conf = distSensors[i]->get_confidence();
            // Looser than the MCL gate on purpose. This probe is measuring the
            // map, so a reading past the map limit is the finding rather than
            // an outlier to drop.
            b[i].ok = mm > 25 && mm <= 2800 && b[i].conf >= confMin;
            if (b[i].ok) b[i].push(static_cast<float>(mm) / 25.4f);
        }

        const bool liveLR = b[1].ok && b[3].ok && b[1].settled() && b[3].settled();
        const bool liveFB = b[0].ok && b[2].ok && b[0].settled() && b[2].settled();
        const float lr = b[1].windowMean() + b[3].windowMean();
        const float fb = b[0].windowMean() + b[2].windowMean();
        // A pair sum bottoms out where the beams are square to the walls, so
        // the running minimum is the perpendicular sum without ever having to
        // trust that the robot was placed square.
        if (liveLR && (!haveMinLR || lr < minLR)) { minLR = lr; haveMinLR = true; }
        if (liveFB && (!haveMinFB || fb < minFB)) { minFB = fb; haveMinFB = true; }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            chassis.arcade(0, 0);
            ++marks;
            std::printf("geomark,%lu,%d",
                        static_cast<unsigned long>(pros::millis() - tStart), marks);
            for (int i = 0; i < kBeams; ++i) {
                std::printf(",%.3f,%.3f,%d", static_cast<double>(b[i].mean()),
                            static_cast<double>(b[i].sd()), b[i].n);
            }
            const float mLR = b[1].mean() + b[3].mean();
            const float mFB = b[0].mean() + b[2].mean();
            std::printf(",%.3f,%.3f,%.3f,%.3f\n",
                        static_cast<double>(mLR),
                        static_cast<double>(mLR - predLR),
                        static_cast<double>(mFB),
                        static_cast<double>(mFB - predFB));
            for (int i = 0; i < kBeams; ++i) b[i].reset();
            haveMinLR = false;
            haveMinFB = false;
            master.rumble(".");
        }

        if (pros::millis() >= nextDraw) {
            nextDraw = pros::millis() + kDrawMs;
            const Pose p = chassis.getPose();
            const float h = stdRadToVexDeg(p.theta);
            int row = 1;
            for (int i = 0; i < kBeams; ++i) {
                if (distSensors[i] == nullptr) continue;
                pros::screen::print(pros::E_TEXT_SMALL, row++,
                                    "s%d %7.2f c%3d n%5d %s", i,
                                    static_cast<double>(b[i].last), b[i].conf,
                                    b[i].n, b[i].ok ? "  " : "??");
            }
            row++;
            pros::screen::print(pros::E_TEXT_SMALL, row++,
                                "LR %7.2f  d %+6.2f    ",
                                static_cast<double>(lr),
                                static_cast<double>(lr - predLR));
            if (haveMinLR) {
                pros::screen::print(pros::E_TEXT_SMALL, row++,
                                    " min %7.2f  d %+6.2f  ",
                                    static_cast<double>(minLR),
                                    static_cast<double>(minLR - predLR));
            } else {
                pros::screen::print(pros::E_TEXT_SMALL, row++,
                                    " min --  pair not seen  ");
            }
            pros::screen::print(pros::E_TEXT_SMALL, row++,
                                "FB %7.2f  d %+6.2f    ",
                                static_cast<double>(fb),
                                static_cast<double>(fb - predFB));
            if (haveMinFB) {
                pros::screen::print(pros::E_TEXT_SMALL, row++,
                                    " min %7.2f  d %+6.2f  ",
                                    static_cast<double>(minFB),
                                    static_cast<double>(minFB - predFB));
            } else {
                pros::screen::print(pros::E_TEXT_SMALL, row++,
                                    " min --  pair not seen  ");
            }
            row++;
            pros::screen::print(pros::E_TEXT_SMALL, row++,
                                "pred LR %.2f  FB %.2f  ",
                                static_cast<double>(predLR),
                                static_cast<double>(predFB));
            pros::screen::print(pros::E_TEXT_SMALL, row++,
                                "h %5.1f   marks %d     ",
                                static_cast<double>(h), marks);
            row++;
            pros::screen::print(pros::E_TEXT_SMALL, row++,
                                "A = log+reset, B = end");

            // The brain is on the robot and the robot is against a wall.
            // One line per pass: master.print drops anything inside 50 ms.
            switch (ctlPage) {
                case 0: master.print(0, 0, "F%-5.1f L%-5.1f ",
                                     static_cast<double>(b[0].last),
                                     static_cast<double>(b[1].last)); break;
                case 1: master.print(1, 0, "B%-5.1f R%-5.1f ",
                                     static_cast<double>(b[2].last),
                                     static_cast<double>(b[3].last)); break;
                default: master.print(2, 0, "LR%-6.2f %+5.2f ",
                                      static_cast<double>(lr),
                                      static_cast<double>(lr - predLR)); break;
            }
            ctlPage = (ctlPage + 1) % 3;

            std::printf("geo,%lu,%.1f",
                        static_cast<unsigned long>(pros::millis() - tStart),
                        static_cast<double>(h));
            for (int i = 0; i < kBeams; ++i) {
                if (b[i].ok) std::printf(",%.2f", static_cast<double>(b[i].last));
                else std::printf(",");
            }
            for (int i = 0; i < kBeams; ++i) {
                std::printf(",%d", b[i].conf);
            }
            std::printf(",%.2f,%.2f,%.2f,%.2f",
                        static_cast<double>(lr),
                        static_cast<double>(lr - predLR),
                        static_cast<double>(fb),
                        static_cast<double>(fb - predFB));
            if (haveMinLR) std::printf(",%.2f", static_cast<double>(minLR));
            else std::printf(",");
            if (haveMinFB) std::printf(",%.2f", static_cast<double>(minFB));
            else std::printf(",");
            std::printf(",%.2f,%.2f\n", static_cast<double>(p.x),
                        static_cast<double>(p.y));
            std::fflush(stdout);
        }
        pros::delay(20);
    }

    chassis.arcade(0, 0);
    out.result("pred LR %.2f FB %.2f", static_cast<double>(predLR),
               static_cast<double>(predFB));
    if (haveMinLR) {
        out.result("LR min %.2f  d %+.2f", static_cast<double>(minLR),
                   static_cast<double>(minLR - predLR));
    } else {
        out.result("LR: pair never seen");
    }
    if (haveMinFB) {
        out.result("FB min %.2f  d %+.2f", static_cast<double>(minFB),
                   static_cast<double>(minFB - predFB));
    } else {
        out.result("FB: pair never seen");
    }
    out.result("marks logged %d", marks);
}
}
