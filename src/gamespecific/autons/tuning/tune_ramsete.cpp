#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include <cmath>
#include <functional>
#include <vector>

#include "chad/api.hpp"

namespace chad {
using namespace autons;

namespace {
std::vector<Pose> weavePath(int n) {
    std::vector<Pose> out;
    for (int i = 0; i <= n; ++i) {
        const float u = static_cast<float>(i) / static_cast<float>(n);
        out.push_back({-40.0f - 12.0f * std::cos(TWO_PI * u),
                       -48.0f + 48.0f * u, 0.0f});
    }
    for (size_t i = 0; i < out.size(); ++i) {
        const Pose& a = out[i == 0 ? 0 : i - 1];
        const Pose& b = out[std::min(i + 1, out.size() - 1)];
        out[i].theta = std::atan2(b.y - a.y, b.x - a.x);
    }
    return out;
}

struct PathScore {
    float xte{0.0f};
    float end{0.0f};
    float tilt{0.0f};
    float secs{0.0f};
    float cost{1e9f};
};

PathScore runRamsetePath(const std::vector<Pose>& path, const RAMSETEParams& p,
                         const std::function<float()>& tilt) {
    PathScore sc;
    double sum = 0.0;
    int n = 0;
    float peak = 0.0f;
    const std::vector<Pose>* pts = &path;

    chassis.setTelemetry([&sum, &n, &peak, pts, &tilt](const ChassisSample& s) {
        float best = 1e9f;
        for (const Pose& q : *pts) {
            best = std::min(best, std::hypot(s.x - q.x, s.y - q.y));
        }
        sum += best;
        n++;
        const float t = tilt();
        if (std::isfinite(t) && t > peak) peak = t;
    });

    const uint32_t t0 = pros::millis();
    chassis.moveRAMSETE(path, 6000, p);
    sc.secs = static_cast<float>(pros::millis() - t0) / 1000.0f;
    chassis.setTelemetry(nullptr);

    const Pose end = chassis.getPose();
    const Pose& last = path.back();
    sc.xte = static_cast<float>(sum / (n > 0 ? n : 1));
    sc.end = std::hypot(end.x - last.x, end.y - last.y);
    sc.tilt = peak;
    sc.cost = sc.xte + 0.5f * sc.end + 0.2f * sc.secs;
    return sc;
}
}

void autonTuneRamsete() {
    Report out;
    out("ramsete: b then zeta");
    if (!headingOk(out)) return;
    out("place near -52,-48 facing 0");

    setFilters(false);
    auto ch = makeCharacterizer(out);
    const auto beams = ch.measureRangeNoise(distSensors);
    float px = 0.0f, py = 0.0f;
    int pinned = 0;
    bool mapDisagrees = false;
    if (!solvePark(beams, chassis.getPose().theta, px, py, pinned,
                   &mapDisagrees)) {
        if (mapDisagrees) {
            out.result("beams fight the post map;");
            out.result("park clear of a post");
        } else {
            out.result("cannot fix position (%d/2 axes)", pinned);
            out.result("move nearer two walls");
        }
        return;
    }
    chassis.setPose({px, py, chassis.getPose().theta});
    out.result("started at %.1f,%.1f", px, py);
    setFilters(true);
    pros::delay(600);

    const auto path = weavePath(32);
    const Pose& first = path.front();
    const float startHeading = stdRadToVexDeg(first.theta);
    const auto tilt = tiltProbe();

    auto toStart = [&]() {
        chassis.moveToPoint(first.x, first.y, 5000);
        chassis.turnToHeading(startHeading, 2000);
    };
    toStart();

    constexpr bool kUseVelocityController = true;

    RAMSETEParams p;
    p.useVelocityController = kUseVelocityController;
    p.ffGains = ramseteFf;
    if (kUseVelocityController && ramseteFf.kV <= 0.0f) {
        out.result("ffGains empty: run phase 1");
        out.result("first, or set the flag false");
        return;
    }

    float bestCost = 1e9f;
    float bestB = p.b, bestZeta = p.zeta;

    for (const float b : {24.0f, 48.0f, 96.0f}) {
        p.b = b;
        p.zeta = bestZeta;
        const PathScore sc = runRamsetePath(path, p, tilt);
        out("b=%.0f xte=%.2f end=%.2f t=%.1f", b, sc.xte, sc.end, sc.tilt);
        if (sc.cost < bestCost) { bestCost = sc.cost; bestB = b; }
        toStart();
    }

    for (const float z : {0.70f, 0.85f, 1.00f}) {
        p.b = bestB;
        p.zeta = z;
        const PathScore sc = runRamsetePath(path, p, tilt);
        out("zeta=%.2f xte=%.2f end=%.2f", z, sc.xte, sc.end);
        if (sc.cost < bestCost) { bestCost = sc.cost; bestZeta = z; }
        toStart();
    }

    out.result("best b=%.0f zeta=%.2f", bestB, bestZeta);
    out.result("cost %.2f = xte + end/2", bestCost);
    out.result("pass in RAMSETEParams; there");
    out.result("is no config default for b");
}
}
