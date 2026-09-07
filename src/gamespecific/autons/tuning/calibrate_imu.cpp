#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include <cmath>
#include <cstdio>

#include "chad/api.hpp"

namespace chad {
using namespace autons;

void autonCalibrateImu() {
    Report out;
    out("imu scale: spin it by hand");
    if (!headingOk(out)) return;
    setFilters(false);
    drivetrain.setVoltage(0.0f, 0.0f);

    const double factorWas = imu.scaleFactor();
    if (!(factorWas > 0.0)) {
        out.result("scale factor is %.4f", factorWas);
        return;
    }

    out.result("turn the robot by hand:");
    out.result("5+ full turns, either way,");
    out.result("then press A. count on the");
    out.result("controller.");
    master.set_text(0, 0, "spin it, press A  ");
    pros::delay(60);
    master.rumble("-");

    double raw = 0.0;
    char buf[24];
    const uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < 300000u) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) break;
        const double scaled = imu.get_rotation();
        if (std::isfinite(scaled)) raw = scaled / factorWas;
        std::snprintf(buf, sizeof(buf), "turns %+.2f     ", raw / 360.0);
        master.set_text(1, 0, buf);
        pros::delay(250);
    }
    while (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) pros::delay(20);
    master.set_text(0, 0, "running-hands off ");

    const double turns = std::round(raw / 360.0);
    if (std::fabs(turns) < 1.0) {
        out.result("only %.2f turns. spin more", raw / 360.0);
        return;
    }

    const double perRev = raw / turns;
    if (perRev < 330.0 || perRev > 380.0) {
        out.result("%.1f deg/turn is not credible", perRev);
        out.result("miscount or a bad IMU");
        return;
    }

    imu.setScaleFactor(360.0, perRev);
    out.result("APPLIED. copy to config.cpp:");
    out.result("ScaledIMU imu(%d, 360.0, %.3f);",
               static_cast<int>(imu.get_port()), perRev);
    out.result("%.0f turns, factor %.5f", std::fabs(turns), 360.0 / perRev);
    out.result("was %.5f", factorWas);
    if (std::fabs(turns) < 5.0) {
        out.result("under 5 turns: rerun with");
        out.result("more for real precision");
    }
    out.result("rerun phase 1 after this");
}
}
