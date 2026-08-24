// chad-sim — virtual runtime backing the PROS shim.
//
// The shim headers under sim/platform/pros/ are thin: every device just reads
// or writes a slot in DeviceBus. The robot model owns the other side of that
// bus. This keeps the fake hardware layer free of simulation logic.
//
// Time is virtual. millis() reads a counter that only advances when the
// scheduler steps, so a run is deterministic and reproducible for a given seed.
#pragma once

#include <cstdint>
#include <functional>

namespace sim {

// V5 smart ports are 1-21; index 0 is unused so port numbers index directly.
inline constexpr int kMaxSmartPorts = 22;
// ADI ports 'A'-'H'.
inline constexpr int kMaxAdiPorts = 8;

// PROS reports INT32_MAX from a disconnected device.
inline constexpr std::int32_t kProsErr = INT32_MAX;

// Physics/integration granularity. The kinematic model is integrated at this
// step regardless of how large a slice the scheduler asks for.
inline constexpr double kPhysicsDtMs = 1.0;

// ============================================================================
// DeviceBus — the boundary between fake hardware and the robot model
// ============================================================================

struct MotorState {
    bool present{false};
    int gearsetRpm{200};        // cartridge RPM: 100 red / 200 green / 600 blue
    bool reversed{false};       // negative port number in the config
    double positionDeg{0.0};    // integrated encoder, motor shaft degrees
    double velocityRpm{0.0};    // motor shaft RPM
    double commandedMv{0.0};    // PHYSICAL millivolts at the motor, -12000..12000
    int brakeMode{0};           // 0 coast, 1 brake, 2 hold
    double temperatureC{25.0};
    double currentMa{0.0};
};

struct ImuState {
    bool present{false};
    // Continuous rotation in VEX convention (degrees, 0 = forward, CW positive)
    // as the PHYSICAL sensor would report it -- already including under-report
    // error, drift and noise. ScaledIMU's correction is applied on top by the
    // library, which is exactly the path we want under test.
    double rotationDeg{0.0};
    bool calibrating{false};
};

struct DistanceState {
    bool present{false};
    std::int32_t distanceMm{9999};   // 9999 = nothing in range
    std::int32_t confidence{63};     // 0-63
    std::int32_t objectSize{100};
};

struct RotationState {
    bool present{false};
    std::int32_t positionCentideg{0};
};

struct AdiEncoderState {
    bool present{false};
    std::int32_t ticks{0};           // 360 ticks/rev
};

struct DeviceBus {
    MotorState      motors[kMaxSmartPorts];
    ImuState        imus[kMaxSmartPorts];
    DistanceState   distances[kMaxSmartPorts];
    RotationState   rotations[kMaxSmartPorts];
    AdiEncoderState adiEncoders[kMaxAdiPorts];

    void reset();
};

DeviceBus& bus();

// ============================================================================
// Virtual clock
// ============================================================================

std::uint32_t millis();
double        preciseMillis();
void          resetClock();

// Advance virtual time by dtMs, integrating the robot model in kPhysicsDtMs
// slices. Called only by the scheduler.
void stepPhysics(double dtMs);

// Installed by the world at startup.
void setPhysicsFn(std::function<void(double dtMs)> fn);

// ============================================================================
// Deterministic RNG
// ============================================================================
//
// Every noise source in the sim pulls from here so a given seed replays
// exactly. NOTE: chad's MclTracking seeds itself from std::random_device, so
// its particle jitter is not reproducible across runs -- only the world
// (sensor noise, drift) is. See sim/README.md.

void  seedRng(std::uint32_t seed);
float randUniform();                      // [0, 1)
float randNormal();                       // mean 0, stddev 1
float randNormal(float mean, float stddev);

}  // namespace sim
