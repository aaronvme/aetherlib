#include "sim_runtime.hpp"

#include <random>

namespace sim {
namespace {

DeviceBus gBus;
double    gClockMs = 0.0;
double    gPhysicsAccum = 0.0;

std::function<void(double)> gPhysics;

std::mt19937 gGen{0xC0FFEEu};
std::normal_distribution<float>       gNormal{0.0f, 1.0f};
std::uniform_real_distribution<float> gUniform{0.0f, 1.0f};

// A second stream, for noise that is NOT part of what a seeded run is
// reproducing. Every draw from gGen shifts the whole sequence after it, so
// adding one new consumer anywhere silently changes what every localization
// test measures -- synthesising the claw sensor moved the MCL disturbance
// battery's worst case from 28 in to 74 without a line of filter code
// changing. Devices the estimators never read belong here instead.
std::mt19937 gAuxGen{0x5EED1234u};
std::normal_distribution<float> gAuxNormal{0.0f, 1.0f};

}  // namespace

void DeviceBus::reset() { *this = DeviceBus{}; }

DeviceBus& bus() { return gBus; }

std::uint32_t millis() { return static_cast<std::uint32_t>(gClockMs); }
double        preciseMillis() { return gClockMs; }

void resetClock() {
    gClockMs = 0.0;
    gPhysicsAccum = 0.0;
}

void setPhysicsFn(std::function<void(double)> fn) { gPhysics = std::move(fn); }

void stepPhysics(double dtMs) {
    if (dtMs <= 0.0) return;

    double remaining = dtMs;
    while (remaining > 0.0) {
        double slice = kPhysicsDtMs - gPhysicsAccum;
        if (slice > remaining) slice = remaining;

        gClockMs += slice;
        gPhysicsAccum += slice;
        remaining -= slice;

        if (gPhysicsAccum >= kPhysicsDtMs - 1e-9) {
            if (gPhysics) gPhysics(kPhysicsDtMs);
            gPhysicsAccum = 0.0;
        }
    }
}

void seedRng(std::uint32_t seed) {
    gGen.seed(seed);
    gNormal.reset();
    gUniform.reset();
    gAuxGen.seed(seed ^ 0x5EED1234u);
    gAuxNormal.reset();
}

float randUniform() { return gUniform(gGen); }
float randNormal()  { return gNormal(gGen); }
float randNormal(float mean, float stddev) { return mean + gNormal(gGen) * stddev; }

float randNormalAux() { return gAuxNormal(gAuxGen); }

}  // namespace sim
