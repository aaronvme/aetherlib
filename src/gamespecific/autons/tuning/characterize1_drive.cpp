#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include "chad/api.hpp"
#include "chad/util/characterize.hpp"

namespace chad {

using namespace autons;

void autonCharacterize1Drive() {
    Report out;
    out("phase 1: track width + feedforward");
    if (!headingOk(out)) return;
    out("spins, then shuttles ~30 in");
    setFilters(false);
    auto ch = makeCharacterizer(out);

    const float widthWas = drivetrain.trackWidth;
    const float width = ch.trackWidth();

    const auto str = ch.feedforward(false);
    const auto turn = ch.feedforward(true);
    drivetrain.setVoltage(0.0f, 0.0f);

    if (!str.valid || str.r2 < 0.9f) {
        out.result("STOP r2=%.3f want>=0.9", str.r2);
        out.result("lower rampVoltsPerSec, rerun");
        return;
    }

    drivetrain.trackWidth = width;
    ramseteFf.kV = str.kV;
    ramseteFf.ksStraight = str.kS;
    // An unidentified kA fits near zero with r2 still at 0.999, so writing it
    // unconditionally is how kaStraight ends up at 0 and stays there.
    ramseteFf.kaStraight = str.kaValid ? str.kA : 0.0f;
    ramseteFf.ksTurn = turn.kS;
    ramseteFf.kaTurn = turn.kaValid
                           ? turn.kA * width * INCH_TO_METER * 0.5f
                           : 0.0f;

    constexpr float kLoopSpeedup = 3.0f;
    const float tau = str.kV > 0.0f ? str.kA / str.kV : 0.0f;
    ramseteFf.kP = (kLoopSpeedup - 1.0f) * str.kV;

    out.result("copy to config.cpp:");
    out.result("trackWidth = %.3ff (was %.3f)", width, widthWas);
    out.result("kV=%.4ff kaStraight=%.4ff", str.kV, ramseteFf.kaStraight);
    out.result("ksStraight=%.4ff ksTurn=%.4ff", str.kS, turn.kS);
    out.result("kaTurn=%.4ff kP=%.2ff", ramseteFf.kaTurn, ramseteFf.kP);
    if (!str.kaValid || !turn.kaValid) {
        out.result("kA UNFIT t=%.1f/%.1f want>=25", str.kaT, turn.kaT);
        out.result("raise rampVoltsPerSecFast or accelSpanTicks");
    }
    out.result("r2=%.3f tau=%.3fs", str.r2, tau);
    // Free speed the fit implies, against the rpm the RAMSETE profile cruises at.
    const float ffTop =
        str.kV > 0.0f ? (12.0f - str.kS) / str.kV / INCH_TO_METER : 0.0f;
    const float cfgTop = drivetrain.rpm / 60.0f * PI * drivetrain.wheelDiameter;
    out.result("top ff %.0f cfg %.0f in/s", ffTop, cfgTop);
    if (cfgTop > 1.0f && ffTop > 1.0f) {
        out.result("rpm = %.0ff (is %.0f)", drivetrain.rpm * ffTop / cfgTop,
                   drivetrain.rpm);
    }
    if (tau < 0.05f) {
        out.result("tau under 50ms: kA looks");
        out.result("wrong, do not trust kP");
    }
}

}  // namespace chad
