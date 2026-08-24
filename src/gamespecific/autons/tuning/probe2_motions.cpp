#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include <cmath>

#include "chad/api.hpp"

namespace chad {

using namespace autons;

void autonProbe2Motions() {
    Report out;
    out("probe: 2 turns, 2 drives");
    if (!headingOk(out)) return;
    setFilters(false);

    {
        // CCW must drop the IMU heading and raise pose theta.
        const float imu0 = static_cast<float>(imu.get_heading());
        const float th0 = chassis.getPose().theta;
        drivetrain.setVoltage(-30.0f, 30.0f);
        pros::delay(400);
        drivetrain.setVoltage(0.0f, 0.0f);
        pros::delay(300);
        const float dImu = static_cast<float>(imu.get_heading()) - imu0;
        const float dTh = radToDeg(angleDiffRad(th0, chassis.getPose().theta));
        out.result("CCW kick: imu %+.1f pose %+.1f", dImu, dTh);
        out.result(dImu < 0.0f && dTh > 0.0f ? "  signs OK" : "  SIGNS WRONG");
    }

    for (const float to : {45.0f, 0.0f}) {
        chassis.setPose({0.0f, 0.0f, vexToStdRad(to == 45.0f ? 0.0f : 45.0f)});
        const float before = stdRadToVexDeg(chassis.getPose().theta);
        const uint32_t t0 = pros::millis();
        chassis.turnToHeading(to, 2000, {.targetTolerance = 2.0f});
        const uint32_t ms = pros::millis() - t0;
        // The outcome, not the duration: a motion can now end early on a stall
        // as well as on arrival, so a short time is not evidence of success.
        out.result("turn %.0f: %.1f->%.1f %ums %s", to, before,
                   stdRadToVexDeg(chassis.getPose().theta), ms,
                   outcomeName(chassis.lastOutcome()));
    }

    for (const float d : {8.0f, -8.0f}) {
        chassis.setPose({0.0f, 0.0f, chassis.getPose().theta});
        const float h0 = chassis.getPose().theta;
        const uint32_t t0 = pros::millis();
        chassis.moveDistance(d, 2000, {.targetTolerance = 1.0f});
        const uint32_t ms = pros::millis() - t0;
        const Pose p = chassis.getPose();
        const float along = p.x * std::cos(h0) + p.y * std::sin(h0);
        out.result("drive %.0f: %.1f off %.1f %ums %s", d, along,
                   -p.x * std::sin(h0) + p.y * std::cos(h0), ms,
                   outcomeName(chassis.lastOutcome()));
        out.result("   turned %+.0f deg", radToDeg(angleDiffRad(h0, p.theta)));
    }
    out.result("want arrived; timeout/stalled = bad");
}

}  // namespace chad
