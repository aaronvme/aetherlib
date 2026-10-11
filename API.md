# chad API reference

Use `#include "chad/api.hpp"`, `#include "gamespecific/gamespecific_api.hpp"` to include chad library.


- [Conventions](#conventions)
- [Chassis](#chassis) — the main API
- [Motion parameters](#motion-parameters)
- [Pose and math](#pose-and-math)
- [PID and exit conditions](#pid-and-exit-conditions)
- [Drivetrain](#drivetrain)
- [Tracking](#tracking) — odometry, RCL, MCL, EKF
- [Auton selector](#auton-selector)
- [PID auto-tuner](#pid-auto-tuner)
- [Characterisation](#characterisation)
- [Simulator and sweeps](#simulator-and-sweeps)
- [Logging](#logging)
- [Jam detection](#jam-detection)
- [Controller warnings](#controller-warnings)
- [ScaledIMU](#scaledimu)
- [Sensor calibration](#sensor-calibration)
- [Feedforward and LTV](#feedforward-and-ltv)
- [Velocity profiles](#velocity-profiles)
- [FastTrig](#fasttrig)
- [Game-specific layer](#game-specific-layer)
- [Numbers that bite](#numbers-that-bite)

---

## Conventions


**Two angle conventions coexist.** Every `Chassis` method and every `*Params`
tolerance takes **VEX degrees** — 0 = forward, clockwise positive, `[0, 360)`.
`Pose::theta` is **standard math radians** — 0 = +X, counter-clockwise positive.
Convert with `vexToStdRad()` / `stdRadToVexDeg()`.

**Units.** Chassis, tracking and motion are all **inches**. The feedforward
controller and LTV follower work in **metres** — their gains are only meaningful
in the units they were fitted in. `Drivetrain::setVoltage()` takes **-127..127**;
`DrivetrainVoltages` carries **volts**.

**Timeouts** are milliseconds; `0` means no timeout.

---

## Chassis

`chad/chassis.hpp`

```cpp
Chassis(Drivetrain& drivetrain, const OdomSensors& sensors,
        const ChassisConfig& config = {});
```

### Calibration and pose

| Method | Notes |
|---|---|
| `void calibrate()` | Robot must be stationary ~2 s |
| `void setPose(const Pose&)` | Fans out to odom, MCL and RCL |
| `void setPosition(float x, float y)` | Heading unchanged |
| `void setHeading(float thetaDeg)` | VEX degrees; position unchanged |
| `Pose getPose() const` | `theta` in standard math radians |

### Driver control

```cpp
void tank(float left, float right);        // -127..127
void arcade(float throttle, float steer);
void curvature(float throttle, float steer);
```

### Autonomous movement

Blocking unless `async = true`, VEX degrees.

```cpp
void moveDistance(float target, float timeout = 0,
                  const MoveDistanceParams& = {}, bool async = false);
void moveToPoint(float x, float y, float timeout = 0,
                 const MoveToPointParams& = {}, bool async = false);
void turnToHeading(float thetaDeg, float timeout = 0,
                   const TurntoHeadingParams& = {}, bool async = false);
void turnToPoint(float x, float y, float timeout = 0,
                 const TurnToPointParams& = {}, bool async = false);
void moveArc(float x, float y, float radius, float timeout = 0,
             const ArcParams& = {}, bool async = false);
void moveBoomerang(float x, float y, float thetaDeg, float timeout = 0,
                   const BoomerangParams& = {}, bool async = false);
void moveRAMSETE(const std::vector<Pose>& path, float timeout = 0,
                 const RAMSETEParams& = {}, bool async = false);
void swingToHeading(float thetaDeg, SwingSide side, float timeout = 0,
                    const SwingParams& = {}, bool async = false);
```

All motions run on one shared motion task, so motors and PIDs are only ever
touched by a single thread. A blocking call waits for *its own* motion to
finish, not merely for the queue to drain.

### Motion status

```cpp
bool isSettled() const;
void cancelMotion();
void abortMotion();
void waitUntilSettled();
void waitUntilDist(float dist);   // inches travelled
```

`cancelMotion` returns once the motion has stopped and the drive is off, so it
cannot be called from the motion task. It waits at most about 100 ms for the
loop to see the cancel, then cuts the drive regardless. `setTelemetry`'s
callback runs on that task; use `abortMotion` there, which stops the motion
and returns immediately.

A motion also ends by itself when the competition status changes. PROS
deletes the autonomous or driver task on a mode change without unwinding it,
so the task that queued the motion may no longer exist to cancel it.

### Tracking access

```cpp
OdomTracking& odom();
RclTracking&  rcl();
MclTracking&  mcl();
void startTracking();
void stopTracking();
```

### Tuning

```cpp
void setLateralGains(const PIDGains&);
void setAngularGains(const PIDGains&);
void setThrottleCurve(const DriveCurve&);
void setSteerCurve(const DriveCurve&);

// Settle tolerances every motion falls back to when its own is negative,
// plus the moveToPoint steering hold radius. A negative argument leaves that
// one alone. setPidTuning does NOT touch these -- damping and tolerance are
// not the same knob, and a caller changing gains should not silently move
// where its motions stop.
void setSettleTolerances(float toleranceIn, float toleranceDeg,
                         float bearingHoldDist = -1.0f);
void setLateralSlew(float unitsPerSec);
void setBrakeMode(pros::motor_brake_mode_e);
void setDistanceSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>&);
```

### Dry run

Used for selector path preview.

```cpp
void setDryRun(bool enabled);
bool isDryRun() const;
const std::vector<Pose>& dryRunPath() const;
void resetDryRunPath();
```

### Telemetry

```cpp
void setTelemetry(std::function<void(const ChassisSample&)> callback);
```

```cpp
struct ChassisSample {
    int type;                        // MotionType of the running motion
    float x, y, thetaDeg;            // thetaDeg in VEX degrees
    float targetX, targetY;
    float lateralOut, angularOut;
    bool settled;
};
```

Set it before starting a motion, motion task reads each tick. Empty by
default.

### ChassisConfig

| Field | Default | Meaning |
|---|---|---|
| `lateralGains` | `{10, 0, 0.3}` | Distance PID |
| `angularGains` | `{100, 0, 5}` | Turn PID |
| `lateralWindupRange` / `angularWindupRange` | `0` | Integral only accumulates inside this error band |
| `lateralFlipReset` / `angularFlipReset` | `false` | Zero the integral when error changes sign |
| `throttleCurve` / `steerCurve` | — | Driver-control input shaping |
| `defaultMoveDistance` etc. | — | Per-motion-type parameter defaults |
| `distanceSensors` | `{}` | For MCL/RCL |
| `sensorMounts` | `{}` | Robot-frame mount poses, index-matched to the above |
| `settleTimeMs` | `0` | Settle condition must hold this long before exiting. `0` exits on the first in-tolerance sample, which can fire while still carrying velocity through the target. `config.cpp` ships `110`. |
| `useMclTracking` / `useRclTracking` | `false` | |
| `mclConfig` / `rclConfig` | — | See [Tracking](#tracking) |

---

## Motion parameters

Every field is optional — use designated initializers for the ones you care
about. Tolerances are inches for lateral motions, VEX degrees for turns.

```cpp
struct MoveDistanceParams {
    bool  forwards{true};
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{-1.0};     // inches, negative = ChassisConfig
    float earlyExitRange{0.0};       // exit early within this range; 0 = off
};

struct MoveToPointParams {           // + lead
    bool  forwards{true};
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{-1.0}, earlyExitRange{0.0};
    float lead{0.0};                 // declared, not read by the controller
};

struct TurntoHeadingParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{-1.0};     // VEX degrees, negative = ChassisConfig
    float earlyExitRange{0.0};       // VEX degrees of error left; 0 = off
};

struct TurnToPointParams {           // + offset
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{-1.0};
    float earlyExitRange{0.0};       // VEX degrees of error left; 0 = off
    float offset{0.0};               // angular offset, VEX degrees
};

struct SwingParams {                 // same shape as TurntoHeadingParams
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{-1.0};
};

struct ArcParams {
    bool  forwards{true};          // not read by the controller
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{-1.0};     // inches, negative = ChassisConfig
    float lead{0.0};               // not read by the controller
};

struct BoomerangParams {
    bool  forwards{true};          // not read by the controller
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{-1.0};     // inches, negative = ChassisConfig
    float headingTolerance{2.0};     // VEX degrees
    float lead{0.5}, leadDecay{0.5};
};

On every motion but boomerang and RAMSETE, `minSpeed` is a floor on the
magnitude of the output, applied after `maxSpeed` and before the lateral slew,
and only while the motion is outside its tolerance -- held through the
tolerance band it would stop the motion settling at all. `earlyExitRange` ends
the motion as soon as the remaining error is inside it, so the next queued
motion takes over while the robot is still carrying speed. A chained
`moveToPoint` (`minSpeed` not 0) also ends once the robot crosses the line
through the target square to the direction it started from, so one that passes
beside its target hands off there instead of turning back until it times out.
Motions that reach the radius first, and unchained ones, are unaffected. Both were declared
but unread until they were wired up; an auton tuned against the old behaviour
will move faster and hand off sooner.

`minSpeed` on a boomerang is not a speed floor. The controller scales its
along-track output by `min(distToTarget / 6, 1)`, so inside 6 in the approach
is throttled quadratically and crawls; `minSpeed / maxSpeed` is the floor on
that SCALE. Raising it restores terminal authority without stopping the motion
from converging, because the output is still proportional to the error.

Setting it equal to `maxSpeed` switches the throttle off entirely. That is
worth knowing because the throttle is there to prevent an overshoot a
well-damped lateral loop does not have: on the simulator, off it, the same
boomerang finishes 0.18 in out in 3.0 s instead of 0.22 in out in 4.5 s, with
zero overshoot either way. It is what makes boomerang twice as slow as
`moveToPoint` over the same distance. Whether your lateral loop is damped
enough to do without it is a hardware question.

Arc and boomerang were the only two motions that ignored
`ChassisConfig::targetToleranceIn` -- both shipped a hard `1.0`, so raising or
lowering the robot-wide tolerance silently did nothing to them. They now take
the same `-1.0` fallback as everything else. The default is unchanged for a
caller that leaves `ChassisConfig` alone, since that is 1.0 too.

struct RAMSETEParams {
    float b{24.0};                    // aggressiveness, b > 0
    float zeta{2.10};                 // damping; tuned on the robot, see below
    float smoothSpacingIn{2.0};      // resample the path; 0 = take as given
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{-1.0};     // inches, negative = ChassisConfig
    float headingTolerance{2.0};     // VEX degrees
    float lookaheadDist{10.0};       // inches
    bool  useVelocityProfile{true};
    float maxAccel{50.0};            // in/s²
};
```

`b` and `zeta` feed the standard RAMSETE gain, with `b` scaled by
`INCH_TO_METER` squared before use. The defaults were measured on the robot
(retuned from 48 / 0.85); `zeta` above 1 is deliberately overdamped.

**The path you pass is control points, not the trajectory.** `moveRAMSETE`
resamples it along a Catmull-Rom spline every `smoothSpacingIn` inches before
the controller sees it, so three corner points describe a curve:

```cpp
std::vector<Pose> path = {{-24.0f, -24.0f, 0.0f},
                          {0.0f, -40.0f, 0.0f},
                          {24.0f, -24.0f, 0.0f}};
chassis.moveRAMSETE(path, 5000);        // drives the curve, not the polyline
```

Without it those three poses are a polyline: RAMSETE cuts the corners, and
the profile has no curvature to slow for because consecutive headings barely
differ. The resampling also computes tangent headings, which is what
`maxLateralAccel` reads — a hand-written list of zero headings is how the
cornering limit gets switched off by accident.

Set `smoothSpacingIn = 0` to drive a path exactly as given. An already-dense
path resampled at 2 in comes back as very nearly itself, so the default is
safe for one.

`chad::splinePath(controls, spacingIn)` in
`chad/movement_control/spline.hpp` is the same function, if you want the
poses in hand. It is header-only and PROS-free; `test/host/spline_test.cpp`
covers endpoints, interior interpolation, spacing, tangents, coincident
controls, degenerate input and a closed loop. The browser preview's
`buildPath()` matches it deliberately, down to the minimum of two samples per
segment, so the sim draws the curve it drives.

```cpp
enum class TurnDirection { CCW, CW, SHORTEST };
enum class SwingSide { Left, Right };   // Left = left side stationary
```

### Geometry helpers

```cpp
// turntopoint.hpp
float resolveTurnError(float rawErrorRad, TurnDirection);
float headingToPoint(const Pose& current, float targetX, float targetY);
float headingToPointOffset(const Pose& current, float x, float y, float offsetDeg);
bool  headingWithinTolerance(float headingErrorRad, float toleranceRad);

// arc.hpp — positive radius = center to the left (CCW)
Pose  computeArcCenter(const Pose& current, float radius);
float chordDistance(const Pose& current, float x, float y);
float arcAngle(const Pose& current, float x, float y, float radius);
float arcEndHeading(const Pose& current, float x, float y, float radius);

// ramsete.hpp
int closestPathIndex(const std::vector<Pose>& path, const Pose&);
int lookaheadIndex(const std::vector<Pose>& path, const Pose&, float lookaheadDist);
```

---

## Pose and math

`chad/util/util.hpp` — header-only.

```cpp
struct Pose {
    float x{0}, y{0};      // inches
    float theta{0};        // standard math radians

    Pose operator+(const Pose&) const;
    Pose operator-(const Pose&) const;
    Pose operator*(float) const;
    bool equals(const Pose&, float linTol = 0.001f, float angTol = 0.0001f) const;
};
```

### Angles

```cpp
float wrapRad(float);            // -> [-PI, PI]
float wrapRadPositive(float);    // -> [0, 2PI)
float wrapDeg(float);            // -> [0, 360)
float angleDiffRad(float from, float to);   // shortest signed
float angleDistRad(float from, float to);   // shortest absolute
```

All of these — and `botHeadingToTrig` below — guard non-finite input and return
`0`. That matters: IMU errors surface as `PROS_ERR_F` (`INFINITY`), and a
subtractive wrap loop on infinity never terminates. It hard-freezes the brain.

They also reduce in **one step** rather than by repeated subtraction, which is a
second freeze and not the same one. Past about 2^27 the gap between adjacent
floats exceeds `2*PI`, so `rad -= TWO_PI` rounds straight back to `rad` and the
loop never ends — and `1e9` is a perfectly finite value that the non-finite
guard waves through. Above that magnitude a `float` no longer resolves the angle
anyway, so the reduction is only exact over the range where it is meaningful.

### Conversions

```cpp
constexpr float degToRad(float), radToDeg(float);
constexpr float mmToInch(float), inchToMm(float);
constexpr float inchToMeter(float), meterToInch(float);

float vexToStdRad(float vexDeg);     // VEX heading -> math radians
float stdRadToVexDeg(float stdRad);  // math radians -> VEX heading
float botHeadingToTrig(float botDeg);
```

Constants: `PI`, `HALF_PI`, `QUARTER_PI`, `TWO_PI`, `INV_PI`, `RAD_TO_DEG`,
`DEG_TO_RAD`, `MM_TO_INCH`, `INCH_TO_MM`, `INCH_TO_METER`, `METER_TO_INCH`.

### General

```cpp
template<typename T> T clamp(T val, T min, T max);
template<typename T> T sign(T val);           // +1, 0, -1
template<typename T> T sq(T val);
bool  nearlyEqual(float a, float b, float epsilon = 1e-5f);
float lerp(float a, float b, float t);
float distance(float x1, float y1, float x2, float y2);
float distanceToPoint(const Pose&, float x, float y);
float bearingToPoint(const Pose& from, float toX, float toY);
```

### Field constants

`Field::FIELD_LENGTH` `FIELD_WIDTH` (144), `FIELD_HALF` (72),
`FIELD_WALL_TO_WALL` (140.4), `FIELD_HALF_WALL` (70.2), `TILE_SIZE` (24),
`MAX_OBSTACLE_DURATION_MS`, plus `Field::fieldToScreenX/Y()` for path preview.

`MAX_DISTANCE_SENSORS` (8) sizes every sensor array library-wide. Changing it
requires a full `make clean`.

```cpp
struct OdomDelta {
    float dVert{0};    // inches, forward+
    float dHoriz{0};   // inches, left+
    float dTheta{0};   // radians, CCW+
};
```

---

## PID and exit conditions

```cpp
struct PIDGains { float kP{0}, kI{0}, kD{0}; };

class PID {
    PID(float kP, float kI, float kD, float windupRange = 0, bool flipReset = false);
    PID(const PIDGains&, float windupRange = 0, bool flipReset = false);

    float update(float error);
    void  reset();

    PIDGains getGains() const;
    void setGains(const PIDGains&);
    void setFlipReset(bool);
    void setWindupRange(float);
    float getWindupRange() const;
    void setFixedDt(float seconds);   // 0 = measure real elapsed time
};
```

```cpp
class ExitCondition {              // from LemLib
    ExitCondition(float range, int time);
    bool update(float input);      // true once error held inside range for time ms
    bool getExit() const;
    void reset();
};
```

---

## Drivetrain

```cpp
Drivetrain(pros::MotorGroup* left, pros::MotorGroup* right,
           float trackWidth, float wheelDiameter,
           float rpm = 450.0f, float horizontalDrift = 2.0f);
```

`rpm` is **wheel** RPM, not cartridge RPM. `trackWidth` is centre-to-centre, in
inches.

```cpp
void  setLeftVoltage(float);       // -127..127
void  setRightVoltage(float);
void  setVoltage(float left, float right);
void  setBrakeMode(pros::motor_brake_mode_e);
float averagePositionDeg() const;  // ignores PROS_ERR from dead motors
float externalGearRatio() const;   // wheel revs per motor rev; 1.0 until resolved

// Per-motor positions into out[0..max); PROS_ERR_F left for an unreadable
// motor. Returns the count written. No allocation.
static int readPositionsDeg(const pros::MotorGroup*, double* out, int max);
```

Public fields: `leftMotors`, `rightMotors`, `trackWidth`, `wheelDiameter`,
`rpm`, `horizontalDrift`.

```cpp
struct DrivetrainVoltages {
    float left{0}, right{0};                    // VOLTS
    float leftMillivolts() const;               // for pros::Motor::move_voltage()
    float rightMillivolts() const;
    float leftPower(float maxVoltage = 12) const;   // -127..127
    float rightPower(float maxVoltage = 12) const;
};

struct DriveCurve {
    float deadband{15}, minOutput{20}, curve{1.0};  // >1 exponential, <1 logarithmic
    float apply(float input) const;
};
```

---

## Tracking

### TrackingWheel

```cpp
TrackingWheel(pros::adi::Encoder&, float wheelDiamIn, float offsetIn, float gearRatio = 1);
TrackingWheel(pros::Rotation&,     float wheelDiamIn, float offsetIn, float gearRatio = 1);

float distanceIn() const;      // holds last good value on sensor error
float offsetIn() const;        // +right / +forward; odom removes offset * dTheta
float wheelDiamIn() const;
float gearRatio() const;
SensorType sensorType() const; // ADI | Rotation
const pros::Rotation* rotation() const; // nullptr for ADI
void  reset();
float rawTicks() const;
float rawDegrees() const;
```

### OdomSensors

```cpp
struct OdomSensors {
    std::vector<TrackingWheel*> vertWheelCollection;
    std::vector<TrackingWheel*> horizWheelCollection;
    std::vector<pros::Imu*>     imuCollection;   // first is primary
    Drivetrain* drivetrain{nullptr};             // fallback when no dead wheels
};
```

IMU scale correction is [`ScaledIMU`](#scaledimu)'s job alone; odometry takes
the heading as the IMU object reports it.

### OdomTracking

```cpp
void setSensors(const OdomSensors&);
void update(bool force = false);
void setBackgroundUpdatesEnabled(bool);
bool backgroundUpdatesEnabled() const;
void startBackgroundOdom();
void stopBackgroundOdom();
bool isBackgroundOdomRunning() const;

OdomDelta getLastDelta() const;   // single tick — intermediate deltas are LOST
OdomDelta consumeDelta();         // accumulated since last call, thread-safe

bool hasHorizontalTracking() const;
bool hasVerticalTrackingWheel() const;

void setPose(const Pose&);
Pose getPose() const;
void reset();
void calibrate();

const OdomSensors& sensors() const;
float imuHeadingDeg() const;      // VEX degrees, guarded heading as of the last tick
float imuHeadingRad() const;      // standard math radians, same source
uint32_t imuFaults() const;       // unreadable reads + re-anchors after a gap or step
```

Prefer `consumeDelta()` when more than one task drives `update()` — the
background odom task does.

The heading getters return the guarded heading as of the last `update()`,
`setPose()` or `reset()`, not a fresh IMU read, so they cost no device access
and agree with the pose's theta.

`setSensors()` is safe while the background task runs: it re-baselines against
the new set, and a tick that was already reading the old one is discarded. No
lock is held across a device read, so a task deleted mid-`setPose()` on a mode
change cannot leave odometry blocked.

### RclTracking

Range-corrected localization: intersects distance-sensor rays with known field
geometry for an absolute position fix.

```cpp
RclTracking(OdomTracking&, const Config&);

void setSensor(int index, pros::Distance*, const Pose& mountOffset);
void configureSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>&,
                      const std::array<Pose, MAX_DISTANCE_SENSORS>&);
void enableSensor(int), disableSensor(int);
void disableSensorFor(int index, float durationMs);  // <= 0: until enableSensor

void addLineObstacle(float x1, float y1, float x2, float y2, float lifetimeMs = 0);
void addCircleObstacle(float x, float y, float radius, float lifetimeMs = 0);
void clearObstacles();            // lifetimeMs 0 = permanent

void update();
void updateSensorPoses(const Pose& robotPose);
bool isValidReading(int index) const;
std::pair<CoordType, float> getBotCoordFromSensor(int index) const;
std::pair<Pose, int> computePose() const;    // {pose, sensorsUsed}

void startTracking(), stopTracking();
bool isTracking() const;

void syncUpdate();                           // blend estimate into odometry
bool updateBotPoseFromBestSensor();
void hardReset(const Pose&);                 // drop residual, recovery, samples

void setFarSideGate(float nx, float ny, float d, float lifetimeMs = 0);
void setFarSideGateFromLine(float x1, float y1, float x2, float y2,
                            float lifetimeMs = 0);
void clearFarSideGate();
bool  farSideGateActive() const;
float farSideGateRemainingMs() const;

const std::array<RclSensor, MAX_DISTANCE_SENSORS>& sensors() const;
Pose latestEstimate() const;
int  activeSensorCount() const;
std::uint32_t faultedMask() const;           // bit i: slot i answered PROS_ERR
float lastDtMs() const;                      // measured period of the last update
```

The far-side gate drops any return that lands beyond a half-plane (the auton
line). `setFarSideGateFromLine` keeps the half the robot is standing in, read
from the pose when it is called. If no pose was declared in the 500 ms before
arming, the first `hardReset()` (a `setPose`) within 500 ms after arming picks
the side again, so a routine may arm it just before its start `setPose`; later
resets and beam solves never flip it. A lifetime is a `pros::millis()`
deadline: it runs out whether or not the tracking task is running.
`clearFarSideGate()` removes it outright; if the mutex cannot be taken within
20 ms (a task deleted holding it) the clear is flagged and the tracking task
applies it. Obstacle lifetimes are deadlines too.

A device answering `PROS_ERR` (unplugged, the wrong device on the port, still
booting) is faulted: never fresh, classed `Invalid`, no displacement evidence,
and flagged in `faultedMask()` and `RclSensor::faulted`. Only a working sensor's
9999 "no object" counts as a missing return. Readings under the V5's 20 mm
minimum are `Invalid` too.

`update()` measures its own period rather than trusting `updatePeriodMs`, which
only sets the loop's target; velocity, the stale hold and the `maxSyncRate`
step use the measured value. A cycle more than 10 x `updatePeriodMs` after the
last is treated as a gap and differences no velocity.

`enum class CoordType { X, Y, INVALID }` — a single distance sensor constrains
one axis only.

| `Config` field | Default | |
|---|---|---|
| `sensorCount` | `MAX_DISTANCE_SENSORS` | |
| `maxSyncDist` | `5.0` | Max inches to sync per update |
| `syncProp` | `0.15` | Fraction of the residual applied per sync. `config.cpp` ships `0.25` for RCL: gated sampling leaves about one cycle in three with a new reading. |
| `maxSyncRate` | `8.0` | Caps one healthy sync step at this × that cycle's dt. Only cycles with a new sample sync (about one in three), so the sustained rate is about a third of it. `config.cpp` ships `12.0`. |
| `accumulationAlpha` | `0.3` | EMA factor |
| `updatePeriodMs` | `10.0` | Loop target; timing uses the measured period |
| `angleTolerance` | `10.0` | Ray must be within this of a wall axis |
| `confidenceThreshold` | `30` | Min V5 sensor confidence (0–63). `config.cpp` ships `20`. |
| `autoSync` | `true` | |
| `useAccumulation` | `false` | |
| `maxCorrectionRate` | `30.0` | Pose-jump guard, in/s. Every automatic sync (normal and recovery) passes a token bucket (`chad/util/correction_limiter.hpp`): over any T seconds the pose moves at most `maxCorrectionBurst + maxCorrectionRate × T`. Normal syncs (≤ `maxSyncRate`) never reach it; recovery, otherwise up to `maxSyncDist` per reading (~150 in/s), glides instead of jumping. `hardSync`, `hardReset`/`setPose` and `updateBotPoseFromBestSensor()` bypass it; `hardReset` refills it. ≤ 0 disables. |
| `maxCorrectionBurst` | `2.0` | Pose-jump guard: inches that may be applied at once from a full bucket. ≤ 0 disables. |
| `trustTolIn` | `2.0` | Per-sensor reading trust (`chad/util/reading_trust.hpp`, `RclSensor::trust` in [0, 1]). Each fresh sample is judged against the predicted wall: consistent if \|innovation\| ≤ `trustTolIn + trustTolFrac × range`; short beyond that (something in the beam: a game element, a robot); long beyond `maxWallInnovation` (a beam miss). Long inside the gate is neutral (a plausible pose correction). Pair-verified readings are consistent. Frozen while displaced. |
| `trustTolFrac` | `0.05` | Range-proportional part of the consistent band; 5 % is the V5's rated accuracy past 200 mm. |
| `trustGraceMs` | `40.0` | Disagreement (ms of real sample spacing, a gap charged ≤ 60 ms) absorbed before trust falls, so one stray sample costs nothing. |
| `trustRampMs` | `90.0` | Accrual over which trust then falls from 1 to 0. A sensor reading 5 in short at 20 in is excluded on its third sample (~90 ms). |
| `trustShortSeverityIn` | `8.0` | Short accrual is × (1 + excess / this), at most × 4, so a gross short (a robot 40 in in front) drops the sensor on its first sample, before it can count as displacement evidence. ≤ 0: flat rate. |
| `trustLongWeight` | `0.25` | Long-beyond-the-gate accrual relative to short: four times slower. |
| `trustHoldMs` | `150.0` | Unbroken consistent readings required before trust is paid back (hysteresis); any short or long sample restarts it. |
| `trustRecoverMs` | `300.0` | Time to pay back a full accrual once the hold is met; from 0 a sensor votes again ~250 ms after the obstruction clears, full trust ~360 ms. |
| `trustFloor` | `0.5` | Below this a sensor does not vote in any tier, recovery included (pair-verified readings excepted), except that a `TooFar` reading votes in recovery: a long reading cannot be an obstruction. Its disagreement still counts as displacement evidence for its first `displaceCycles` distrusted samples (a pose running away from a stalled body looks the same at onset), then stops unless it reads long: a robot parked in front of a sensor can cause at most one recovery, with nothing from that sensor to move toward (~140 ms in `sim/test/rcl_trust.mjs`). ≤ 0 never excludes. `hardReset`/`setPose` and (re)configuring a slot restore full trust. |
| `rangeWeighting` | `true` | Within a tier, average the surviving votes weighted by `trust / σ(range)²`, σ = max(15 mm, 5 % of range); off: weighted by trust only. One vote per axis (the usual case) is unchanged. |

### MclTracking

Monte Carlo localization, 1024 particles, ~40 Hz.

```cpp
MclTracking(OdomTracking&,
            std::array<pros::Distance*, SENSOR_COUNT>,
            const Pose& startPose,
            const Config&);

void startTracking(), stopTracking();
bool isTracking() const;

void predict();          // propagate particles from odometry delta
void updateWeights();    // weight by sensor match
void resample();         // systematic low-variance
Pose update();           // one full iteration; returns new estimate
std::pair<Pose, float> getEstimate() const;    // {pose, N_eff}

void setPose(const Pose&);       // reinitialize particles around this
void uniformReset();             // scatter across the field
void syncToOdometry();

void setDistanceSensors(const std::array<pros::Distance*, SENSOR_COUNT>&);
void setSensorMounts(const std::array<Pose, SENSOR_COUNT>&);
void enableSensor(int), disableSensor(int);
void disableSensorFor(int index, float durationMs);

void setObstacles(const std::vector<LineObstacle>* = nullptr,
                  const std::vector<CircleObstacle>* = nullptr);
void setDrift(float verticalDrift, float horizontalDrift);

Pose getRawEstimate() const;
const std::array<float, SENSOR_COUNT>& sensorReadings() const;   // inches
const std::array<bool,  SENSOR_COUNT>& validSensors() const;
void setDistSyncProp(float);
```

`setObstacles()` stores **pointers** — the vectors must outlive the tracker.

| `Config` field | Default | |
|---|---|---|
| `particleCount` | `1024` | |
| `resampleThreshold` | `512` | Resample when N_eff drops below |
| `maxSensorRange` | `100.0` | inches `config.cpp` ships `78`. |
| `distResampleVariance` | `1.5` | inches of jitter per resample `config.cpp` ships `1.0`. |
| `thetaResampleVariance` | `0.01` | radians |
| `maxThetaDeviation` | `0.08` | radians, clamp vs IMU |
| `trackingWheelVariance` | `0.08` | dead wheels |
| `imeVariance` | `0.12` | motor encoders — more slip `config.cpp` ships `0.004`. |
| `imuVariance` | `0.005` | |
| `rangeSigmaCloseIn` | `0.787` | beam sigma at or below 200 mm `config.cpp` ships `0.0872`. |
| `rangeSigmaProp` | `0.05` | fraction of the reading beyond that |
| `horizDependentVarianceProp` | `0.03` | × \|dVert\|, lateral slip `config.cpp` ships `0.004`. |
| `horizConstantNoise` | `0.03` | inches/update baseline |
| `distSyncProp` / `thetaSyncProp` | `0.06` / `0.001` | Lerp toward chassis; inert when the EKF is on |
| `updatePeriodMs` | `25.0` | |
| `confidenceThreshold` | `10` | |
| `minSensorRange` | `1.0` | mm; raise for recessed sensors. Readings under 20 mm, the V5 sensor's floor, are dropped whatever it is set to. `config.cpp` ships `20`. |
| `maxRandomFrac` | `0.0` | Cap on the injected fraction when the weights collapse. `config.cpp` ships `0.25`. |
| `injectResampleFloor` | `0.03` | Injection below this is ignored. `config.cpp` ships `0.20`. |
| `injectRadiusIn` | `0.0` | inches around the current estimate to place injected candidates; `0` searches the whole field. `config.cpp` ships `25`. |
| `autoSync` | `true` | |

Injection is the augmented-MCL recovery for a filter whose weights have
collapsed. `injectRadiusIn` is what keeps it a recovery rather than a
relocation. Searching the whole field means every wall and post that looks
like the one in front is a live candidate: a robot squaring on a wall
integrates travel it never made, the weights collapse, and the filter
re-converges on a mirror of itself 70 in away. Bounding the search to a
radius the robot could plausibly have been displaced by keeps the prior.
`sim/test/disturbance.mjs` gates all of this.

Season-specific field targets are `static constexpr` arrays inside
`mcl_tracking.hpp` (`kFieldTargets`, `kFieldCircles`, `kDisablingLines`) and are
currently empty — fill them in per game. `kFieldWalls` never changes.

### EkfTracking

`chad/tracking/ekf_tracking.hpp` — EKF over SE(2). State is `[x, y, theta]` with
a full 3x3 covariance; odometry predicts, and three families correct: a distance
reading against a known wall, IMU heading, and MCL position.

Unimodal, so it cannot represent "in one of two places" and cannot recover from a
kidnap alone. MCL can, and is wired in as a measurement to supply exactly that.
Run them together.

```cpp
EkfTracking(OdomTracking&, std::array<pros::Distance*, SENSOR_COUNT>,
            const Pose& startPose, const Config&);

void startTracking();
void stopTracking();
Pose update();                  // one full predict-then-correct cycle
Pose getEstimate() const;
Uncertainty uncertainty() const;   // sigmaX, sigmaY, sigmaTheta
void setPose(const Pose&);
void setPose(const Pose&, float posSigma, float thetaSigma);
void attachMcl(MclTracking*);
int  beamsUsed() const;
int  mclRejectStreak() const;
int  relocateCount() const;
const std::array<BeamState, SENSOR_COUNT>& beamStates() const;
```

`BeamState` per beam: `Absent`, `NoReturn`, `Deadzone`, `Grazing`, `Rejected`,
`Used`.

| `Config` field | Default | |
|---|---|---|
| `vertNoiseProp` | `0.045` | sigma per inch of along-track travel `config.cpp` ships `0.004`. |
| `horizNoiseProp` | `0.045` | per inch of cross-track |
| `slipNoiseProp` | `0.035` | lateral sigma per inch driven `config.cpp` ships `0.004`. |
| `slipNoiseFloor` | `0.010` | lateral sigma per update |
| `vertBiasProp` | `0.06` | systematic along-track scale error `config.cpp` ships `0.0459`. |
| `slipBiasProp` | `0.05` | systematic cross-track error `config.cpp` ships `0.0159`. |
| `biasDecorrIn` | `40.0` | inches over which that bias renews |
| `thetaNoiseProp` | `0.020` | sigma per radian turned |
| `thetaNoisePerInch` | `0.0015` | heading sigma per inch driven |
| `posNoiseFloor` | `0.015` | inches per update |
| `thetaNoiseFloor` | `0.0004` | radians per update |
| `useRangeUpdates` | `true` | |
| `rangeSigmaBase` | `0.60` | inches, fixed part `config.cpp` ships `0.0872`. |
| `rangeSigmaProp` | `0.030` | fraction of the reading |
| `rangeGateChi2` | `9.0` | 1 dof, 3 sigma |
| `maxIncidenceRad` | `1.15` | past 66 deg off normal a ToF return is junk |
| `maxRangeIn` | `100.0` | inches; past this a beam is reporting its own ceiling, not a wall `config.cpp` ships `78`. |
| `obstacleConeFrac` | `0.22` | tan(12.5 deg), the V5 half-cone |
| `obstacleRangeTol` | `6.0` | inches |
| `obstacleSigmaPad` | `2.0` | extra tolerance, in position sigmas |
| `useImuHeading` | `true` | |
| `imuSigmaRad` | `0.006` | |
| `useMclUpdates` | `true` | |
| `mclSigmaScale` | `1.6` | the cloud understates its own error |
| `mclSigmaFloor` | `1.2` | inches |
| `mclGateChi2` | `11.8` | 2 dof, 3 sigma |
| `mclResetCycles` | `10` | consecutive gate failures before relocating |
| `mclMinNeffFrac` | `0.20` | cloud health required to be believed |
| `autoSync` | `true` | write the fused pose into odometry |
| `syncProp` | `0.12` | fraction of the correction applied per update |
| `thetaSyncProp` | `0.20` | |
| `updatePeriodMs` | `10.0` | |

`syncProp` below 1 is what keeps the pose continuous. A hard write steps the
pose every cycle and every motion controller downstream differentiates it; the
filter's own state is exact either way. A relocation bypasses it and lands in
one write. Measure the noise it is smoothing with
`Characterizer::localizationNoise()`.

Phase 4 calls it **twice**, and the difference matters. The first call runs
with the filters off, so its `sigmaThetaDeg` is the IMU on its own — that is
what `mcl.imuVariance` and `ekf.imuSigmaRad` are, both sigmas in radians
despite `imuVariance`'s name. The second runs with the filters on and measures
the fused pose, which is the right thing for `suggestTolerance` and the wrong
thing for the heading sigmas: feeding the EKF's own output back as its
measurement noise would be circular. The written value is floored at 0.01 deg,
the IMU's own resolution, because a zero sigma would give the filter infinite
confidence and a singular update.

---

## Auton selector

LVGL 9 touch list on the brain screen.

```cpp
using AutonFunc = std::function<void()>;

class AutonSelector {
    void registerAuton(const std::string& name, AutonFunc);   // before init()
    void init();
    void select(int index);
    int  selectedIndex() const;
    std::string selectedName() const;
    std::size_t count() const;
    void runSelected();
};
```

---

## PID auto-tuner

`chad/util/autotune.hpp`. Drives repeated step responses and fits gains from the
traces. **The robot moves on its own — give it clear space.**

```cpp
tuning::PidAutoTuner tuner(chassis);            // AutoTuneConfig optional
PIDGains angular = tuner.tuneAngular();         // 0 -> step -> 0, repeatedly
PIDGains lateral = tuner.tuneLateral();         // forward and back
tuner.cancel();
bool cancelled = tuner.wasCancelled();
const StepMetrics& m = tuner.lastMetrics();
```

Each axis runs kP (grow until an overshoot or oscillation limit, keep best cost)
→ kD (upward from the seed, stop when cost stops improving) → kI (off by
default). kP carries forward between step sizes as a floor; kD and kI are
re-searched, because the amplitude they were chosen at has changed. Results are
applied to the chassis and returned; **nothing is persisted** — copy them into
`ChassisConfig` yourself.

### Why the ladder carries derivative

The kP ladder runs with `kdSeedRatio * kP` of derivative already in place. That
is not a detail — it is what makes the ladder work at all. Undamped, a good kP
overshoots and oscillates, the cost function punishes exactly that, so a weak kP
outscores a strong one and the ladder stops on its first rung. The kD phase then
searches around the wrong kP and cannot recover: kD trades overshoot for settle
time, it does not raise the proportional gain it was handed.

Swept over 80 plant/step combinations (five response time constants × four
friction levels × two axes × two amplitudes) against an exhaustive grid over kP
and kD:

| `kdSeedRatio` | mean excess | median | 90th pct | worst | ladder stuck on rung 1 | trials |
|---|---|---|---|---|---|---|
| `0` | +120% | +55% | +268% | +879% | 39/80 | 5.4 |
| `0.05` | +43% | +28% | +96% | +234% | 16/80 | 6.3 |
| **`0.075`** | **+39%** | **+24%** | **+84%** | +234% | **7/80** | 6.8 |
| `0.10` | +46% | +29% | +96% | +237% | 2/80 | 7.0 |

The plateau from 0.035 to 0.10 all lands near +40%, so the default is a
mid-plateau choice rather than a knife edge. It buys that for 6.8 trials per
step size against 5.4.

The plant model this was swept on includes the derivative filter
`PID::update` applies, and it has to: unfiltered, a large kD chatters on a 10 ms
grid — 54 zero crossings on an otherwise settled trace — which trips
`oscillationLimit` and stops the ladder for a reason that does not exist on the
robot. Sweeping without it inverts the result and makes the seed look harmful.

| `AutoTuneConfig` field | Default | |
|---|---|---|
| `angleStepsDeg` | `{90, 180}` | Turn sizes, in order |
| `distanceStepsIn` | `{24, 48}` | |
| `trialMs` | `1500` | Each trial runs the full window |
| `reps` | `2` | Out-and-back pairs per candidate, averaged |
| `kpStart` / `kpStartLateral` | `20` / `4` | Floors: the ladder only climbs |
| `kpGrowth` | `1.5` | |
| `overshootLimitPct` | `20` | Stop growing kP past this |
| `oscillationLimit` | `3` | |
| `kdSeedRatio` | `0.075` | kD carried while kP climbs — a derivative time in seconds |
| `kdRatioStep` | `0.05` | kD then searched upward from the seed, in multiples of kP |
| `steadyStateTolDeg` / `steadyStateTolIn` | `1.5` / `0.75` | kI only attempted above this |
| `kiRatioStep` | `0.5` | kI as a multiple of kP, so per second. Live only with `includeIntegral` |
| `includeIntegral` | `false` | The kI phase. Off: no standing load, and no windup range |
| `maxIterations` | `6` | Per stage |
| `weights` | see below | Cost function |
| `tiltDeg` | empty | `float()` returning absolute chassis tilt |
| `tiltAbortDeg` | `0.0` | Cut the trial when tilt passes this. 0 disables |
| `onProgress` | empty | `void(const char* stage, PIDGains, const StepMetrics&)` |

`tiltDeg` alone only records the peak, which `lastPeakTilt()` reports after
the trial. That is enough to stop a kP *ladder* from climbing into a tip,
because the next candidate is lower. It is not enough for a single trial at
gains already too hot for the step size — and the step size is what is worth
raising, since the linear region is always the last `127/kP` inches whatever
the step, so a longer one enters it faster. `tiltAbortDeg` calls `abortMotion`
from inside the telemetry callback, which the motion loop calls on its own
task, so the flag it sets is the one the loop is about to test. `cancelMotion`
is the wrong call there and deadlocks the motion task — see Motion status.

An aborted trial's metrics describe a motion that did not finish, so check
`lastTiltAborted()` **before** `StepMetrics::valid` — otherwise a tip reads as
"no trace" and a refinement loop stops instead of backing the gain off.

Each trial forces `targetTolerance = 0` so the motion cannot settle early. Without
that the motion exits the instant it reaches tolerance and the overshoot is never
observed.

Budget about 6.8 trials per step size, so roughly 14 per axis at the two default
amplitudes — each trial being `reps` out-and-back pairs of `trialMs`.

### The search itself

`chad/util/autotune_core.hpp` — PROS-free and allocation-free, so a whole tune
runs on the host against a plant model rather than only on a robot. That is how
the table above was measured.

```cpp
GainSearch search;
search.reset(GainSearchConfig{}, {kpStart, 0, 0});
for (float step : steps) {
    search.beginStep();                      // kP carries, kD/kI re-search
    while (!search.stepDone()) {
        TrialRequest t = search.request();   // gains and phase to run next
        search.submit(measure(t.gains, step));
    }
}
PIDGains best = search.best();
const StepMetrics& m = search.bestMetrics();  // the measurement of best()
int trials = search.trials();
```

A request/submit state machine rather than a loop with a callback, because the
caller owns the motion — the same search drives a real drivetrain, a simulated
one, or an exhaustive grid comparison, with no duplicated logic.

`bestMetrics()` is written in the same branch as `best()` and cleared by
`beginStep`, so the kI gate can only read a measurement of the gains in hand at
the current step size — never the previous amplitude. An invalid trial advances
the phase instead of retrying, so a drivetrain that stops responding ends the
search rather than hanging it.

`MetricsAverage` averages the reps: a rejected rep is left out of the mean
rather than contributing a zero, and if every rep is rejected the result is
invalid but still carries the first rejection's opening error — which is the
only clue to why a trial produced nothing.

### Step-response analysis

`chad/util/step_response.hpp` — pure, no PROS, unit-testable on the host.

```cpp
StepMetrics analyseStep(const std::vector<float>& t,
                        const std::vector<float>& error,
                        float step, const StepWeights&);

struct StepWeights {   // cost per unit
    float overshoot{1.0};      // per percent
    float settle{8.0};         // per second
    float steadyState{6.0};    // per degree or inch
    float oscillation{4.0};    // per crossing
};

struct StepMetrics {
    float overshootPct, steadyState, settleMs;
    int   oscillations;
    float cost;
    bool  valid;      // false for empty, <4 samples, mismatched lengths, step <= 0
};
```

---

## Localization autotuning

`chad/util/localization_fit.hpp` — fits the MCL and EKF settings that used to
be numbers off a simulator sweep. Driven by `autonCalibrateLocalization`
(phase 5), which never drives the robot: you hand place it, it measures.

**What is on the field decides how it measures, and what it can measure at
all.** `kFieldIsClear` in `gamespecific/autons/tuning/support.hpp` says which.

`solveTruth` has three ways to get it, and tries them in that order.

Wall truth from an opposing beam pair brackets an obstacle, and pays for that
by needing both opposing walls in reach at once: on a 140.4 in span against a
V5 sensor's roughly 79 in, only within 8.8 in of a centre line. On a bare
field `solveAxisSingle` takes one beam per axis and the near wall is never
more than half the span away, so the whole field is usable — at the cost of
no blockage detection, which is why it is declared and not detected.

`solveMapped` is what covers the rest of a field that has posts on it. It
**sweeps** the pose over a 1 in grid of the field, scoring every cell by how
well the walls and the registered posts explain all the beams at once, then
descends onto the winner. It costs about 3 ms on a host and well under 0.1 s
on a brain, which a hand-supervised routine can afford.

It sweeps rather than solving because proposing candidates has a blind spot
that cannot be seen from inside. Reading each beam as a clean wall hit and
solving gives one candidate per beam -- but where *every* beam on an axis is
looking at a post, the true pose is never proposed, so the solve returns an
alias and reports it as verified. The post lattice is periodic, so those
aliases are real: (-48,-14) and (-48,-61.7) agree on every beam to within
0.65 in. A sweep finds both and refuses.

Three things make the objective usable. A ray either hits a 2 in circle or it
does not, and at a graze that flips the prediction by the length of the field,
so a near miss is admitted with its cost growing from zero at the rim rather
than being free or fatal. Nothing behind a solid blocker may explain a
reading. And the inlier window is capped at `MAP_TOL_MAX_IN`, because the
shipped `rangeSigmaProp` of 0.05 asks for a 9 in window at 60 in -- wide
enough to call a post a wall.

**Silence is a measurement.** A configured sensor that returned nothing says
whatever it pointed at was out of reach, and that is often the only thing
separating a corner from its alias: at (-50,-50) two beams reach a wall and
"19.7 in to my left" is equally the wall at x=-50 or the post at (-23.5,-47)
at x=-2.6, but at the second pose the silent front sensor would have seen a
wall at 71.8 in. So `MapBeam::present` is set for every fitted sensor,
reading or not, and `solveMapped` takes the sensor's reach.

That reach has to be honest, and it is not symmetric. Overstating it is safe:
silence stops carrying information and the solve degrades to what it was
without it. Understating it excuses silence that should not be excused, which
is how a confident wrong pose happens -- measured at 10 verified-wrong fixes
across the field. `solveMapped` therefore raises the reach it was given to the
largest reading it can actually see, since a beam that returned 117 in proves
the sensor reaches that far. **`config.cpp` ships `maxSensorRange = 140`,
which is past what a V5 sensor really does; phase 4 measures the truth.**

The inlier floor is relative, `nValid - MAP_MAX_OUTLIERS`, because how many
beams *can* agree is set by where the robot is: in a corner the far walls are
past a V5 sensor's reach and only two beams read at all, and a fixed floor of
three would refuse every corner on the field.

That leaves one thing the solve cannot do, and it reports rather than hides
it. A two-beam fix is one beam per axis, and a beam that reads short simply
moves the candidate to where it explains itself — there is no third beam to
contradict it. Such a fix comes back with `inliers` under
`MAP_VERIFIED_INLIERS`, and phases 5 and 6 count them, so a run that leaned
on unchecked truth says so on the results page instead of quietly biasing a
fit. An opposing pair reports as `kBracketedFix` and is always verified: it
detects blockage outright.

The beam mixture is the one fit a bare field **cannot** do. Its short term
describes an occluded reading; with nothing on the field there is no tail, EM
drives `shortWeight` to nearly zero, and a filter with no occlusion allowance
is what the first robot to stand in front of a sensor annihilates. Phase 5
declines to touch `shortWeight`, `shortDecaySigmas` and `randWeight` when the
field is declared clear, and warns if it sees a tail anyway — that is the
field disagreeing with you. The resample-variance choice and MCL's sigma
calibration do not care what is on the field and still run.

A registered obstacle that is not physically there is worse than none:
`clearFieldObstacles()` exists because MCL deadzones any beam a particle can
point at one, so every phantom post is somewhere a wrong particle hides
instead of explaining its reading. `solveMapped` raises the stakes on that,
because it now believes the post list closely enough to place the robot from
it. A wrong `kFieldPosts` is a wrong tuning run, silently.

```cpp
struct BeamMixture {
    float hitWeight{1.0f};
    float shortWeight{0.50f};
    float shortDecaySigmas{1.2f};
    float randWeight{0.0008f};
};

struct BeamMixtureFit {
    BeamMixture params;
    float hitFraction, shortFraction, randFraction;   // sum to 1
    float meanLogLik;      // per sample
    int   samples, iterations;
    bool  valid;
};

BeamMixtureFit fitBeamMixture(const float* u, int n,
                              float span = BEAM_SPAN_SIGMAS,
                              int maxIters = 200, float tol = 1e-7f);
BeamMixture    perBeamFromPair(const BeamMixture&);
float          opposingPairSum(float fieldSpanIn, float alongA, float alongB);
float          mountAlongAxis(float mx, float my, float thetaRad, bool xAxis);
float          beamSigmaBase(float readingIn, float closeIn, float prop);

struct MapCircle { float x, y, radius; };
struct MapBeam   { bool present, valid;
                   float inches, sigmaIn, bx, by, alongX, alongY; };
struct MapSolve  { float x, y; int inliers; bool trusted; };

// castFieldMap predicts a reading; mapBeamResidual asks how far a reading is
// from anything that could have produced it. They are not the same model --
// only the second forgives a graze.
float    castFieldMap(float sx, float sy, float dx, float dy,
                      const MapCircle*, int nCircles,
                      float halfSpanIn, float farIn = MAP_FAR_IN);
float    mapBeamResidual(float sx, float sy, float dx, float dy,
                         float readingIn, const MapCircle*, int nCircles,
                         float halfSpanIn);
MapSolve solveMapped(const MapBeam*, int nBeams,
                     const MapCircle*, int nCircles, float halfSpanIn,
                     float maxRangeIn = 0.0f);
```

### The truth signal needs no pose

Reading off the `+axis` wall gives `c = half - d_a - o_a`; off the `-axis` wall,
`c = -half + d_b - o_b`. Setting them equal cancels the pose entirely:

```
d_a + d_b = span - o_a + o_b
```

So two opposing beams must sum to a constant fixed by the field span and the two
mount offsets resolved along that axis — **wherever the robot happens to be**.
No tape measure, no IMU position, and placement error cannot contaminate it,
because placement never enters. That is what makes a hand-placed routine as
good as a driven one for this particular fit.

### Why EM and not a moment fit

The mixture `MclTracking::buildGaussianLUT` evaluates, on the standardised
residual `u = (measured - expected) / sigma`:

```
p(u) = hitWeight * exp(-u^2/2) + randWeight
                 + [u < 0] * shortWeight * exp(u / shortDecaySigmas)
```

A short reading and an underestimated sigma both fatten the residual spread, so
no moment separates them; the uniform floor is invisible in any moment low
enough to estimate from a few hundred samples. Expectation maximisation instead
assigns each residual a responsibility per component, so the tail is identified
by the residuals *only* the tail can explain. Against planted parameters it
recovers the mixture proportions to within 0.05 and the decay to within 0.35
sigmas, and the fitted weights beat the shipped guess on the filter's own log
likelihood — which is the only comparison that matters, since it is the filter
that has to use them.

The filter's form is unnormalised, because it only ranks particles against each
other. Fitting needs comparable responsibilities, so the components are
normalised over `[-span, span]` and converted back with `hitWeight` pinned at 1.
`hitWeight` is therefore never reported: it sets the scale the other three are
measured against.

`perBeamFromPair` converts a fit made on pair sums to a single beam. The pair
residual is `e_a + e_b`, so its Gaussian core is `sqrt(2)` wider while the tail
keeps its own scale — an obstacle blocks one beam, essentially never both at
once. The decay converts; the weights do not.

### EKF trust in MCL, and recovery

```cpp
SigmaCalibration calibrateMclSigma(const float* reported, const float* errorIn,
                                   int n, float floorQuantile = 0.9f);
RecoveryScore    scoreRecovery(const RecoveryTrial*, int n);
bool             recoveryBetterThan(const RecoveryScore&, const RecoveryScore&);
```

Particle spread is not a covariance: it measures agreement *between particles*,
which collapses as they converge whether or not they converged on the truth. So
`mean(err^2 / sigma^2)` comes out well above the 1.0 a calibrated filter would
give, and `mclSigmaScale` is the square root of it. It never goes below 1 —
scaling down would have the EKF trust MCL more than MCL has earned.

`mclSigmaFloor` cannot come from the same average: a sigma collapsing toward
zero makes individual terms enormous while barely moving the mean. It is a high
quantile of the observed error instead.

Recovery is scored on medians, and a trial that never reconverges counts as a
failure rather than as a large time — letting it contribute one drags the
comparison toward whichever setting failed most interestingly. `recoveryBetterThan`
ranks convergence *rate* first: a setting that recovers slowly still recovers.

### What one pass measures

The hand move **is** the kidnap — MCL is still at the previous placement — so
one pass of placements yields all three groups at once, and the two resample
settings are rotated across placements so the run is a search rather than a
single measurement.

| Setting | How |
|---|---|
| `mcl.shortWeight`, `mcl.shortDecaySigmas`, `mcl.randWeight` | EM on pair residuals, filters off |
| `ekf.mclSigmaScale`, `ekf.mclSigmaFloor` | reported sigma vs actual error while reconverging |
| `mcl.distResampleVariance`, `mcl.thetaResampleVariance` | best of 3 candidates by recovery rate, then speed |

Park square-ish with two walls in view, or no opposing pair exists and the
placement is skipped. The tail fraction is only as good as the number of
placements: each contributes the same sample count, so it estimates the fraction
of *placements* with a blocked beam, and 15 of them resolve that to about ±0.11.

`ekf.vertBiasProp`, `ekf.slipBiasProp` and `ekf.biasDecorrIn` are not fitted
here — they are per inch driven, so a parked robot produces no signal. They are
`autonCalibrateOdomBias`'s job, below.

### Obstacles are assumed present

Nothing in either routine assumes a clear field, because the field cannot be
cleared. The truth solve uses `solveAxisBracketed`, which rejects an axis whose
opposing pair disagrees by more than 3 sigma of combined beam noise — the
signature of a blocked beam. Tolerance comes from the readings' own sigmas
rather than a constant, since at 40 in a beam's sigma is already 2 in and a
fixed threshold would either reject every clean pair up close or miss every
obstacle far away.

Blocked placements still contribute to the **beam mixture** — a blocked beam is
precisely the tail the mixture exists to describe. They are excluded only from
the truth-dependent parts.

---

## Odometry bias

`autonCalibrateOdomBias` (phase 6). Fits the systematic part of odometry error
from one-way legs with a wall solve at both ends. Needs **24 in** and nothing
more.

```cpp
struct BiasLeg  { float lengthIn, alongErrIn, crossErrIn; };
struct BiasFit {
    float vertBiasProp, slipBiasProp;
    float alongPerRootIn, crossPerRootIn;   // what is actually observable
    float growthExponent;                   // 0.5 is the assumed shape
    int   legs, groups;
    bool  valid, shapeSuspect;
};
BiasFit fitOdomBias(const BiasLeg*, int n, float biasDecorrIn);
```

### biasDecorrIn is not identifiable, at any distance

The EKF charges `vertBiasProp^2 * |ds| * biasDecorrIn` of variance per tick,
accumulating to `vertBiasProp^2 * L * biasDecorrIn` over a leg of length `L`.
`biasDecorrIn` occurs **nowhere else in the filter** — one line, multiplying both
bias props. So double it, divide both props by `sqrt(2)`, and the filter behaves
identically. Three settings, two degrees of freedom.

This is why `fitOdomBias` takes `biasDecorrIn` as an argument rather than
fitting it: only the products `vertBiasProp^2 * biasDecorrIn` and
`slipBiasProp^2 * biasDecorrIn` are observable. Pass the convention you intend
to keep — the routine passes whatever `config.cpp` already has — and the props
come out consistent with it. **A fitted prop is meaningless without the
`biasDecorrIn` it was fitted against**, which is why phase 6 prints both.

A useful consequence: the 24 in ceiling costs nothing. The quantity that would
have wanted long legs cannot be measured over any distance.

### Root mean square, not variance

The term exists to cover *systematic* error, and `P` has to cover the total
error including its mean. So the fit uses `rms` of the signed error;
subtracting the sample mean, as a variance would, removes exactly the part
being measured. A pure wheel-scale error — identical on every leg, zero
standard deviation — is the case that makes this concrete: a variance-based fit
reports no bias at all, which is the one answer that cannot be right.

`E[e^2 / L]` is the same for every leg length under the model, so its mean is
the estimator. Regressing `e^2` on `L` would overweight the long legs, whose
`e^2` has the largest spread.

### Why one-way, and what the three lengths are for

Out-and-back cancels a systematic error by construction: a 1% scale error
over-reports both directions, the signed errors cancel, and the endpoint says
nothing. That is exactly why `Characterizer::measureOdomDrift` (phase 3)
measures the *random walk* and not this. A one-way leg with truth at both ends
does not cancel.

The three lengths (8, 16, 24 in) are a **model check**, not an identification.
`growthExponent` regresses `log(rms)` on `log(L)`; the filter's `sqrt(L)` shape
is 0.5, while an error that never decorrelates over this range would be 1.0.
Outside 0.25 of 0.5 the fit sets `shapeSuspect` and the routine says so rather
than quietly reporting a number fitted to the wrong shape.

Every drive is a measurement and the direction alternates, so the robot
oscillates inside its 24 in rather than walking away. A leg whose wall-solved
length disagrees with the commanded one by more than half is thrown out, as is
any leg where either end could not be pinned.

---

## Characterisation

`chad/util/characterize.hpp` — measures constants of this robot from quantities
it can observe itself. Run before `PidAutoTuner`; a feedforward that matches the
drivetrain leaves the PID less to do.

```cpp
struct CharacterizeConfig {
    int   staircaseLevels{11};          // quasistatic dwells, kS and kV
    float maxVolts{9.0f};
    float breakawayVoltsPerSec{2.0f};   // creep rate, breakawayVolts()
    float dwellMs{900.0f};
    float holdMs{400.0f};               // averaged, at the tail of each dwell
    float stepVolts{7.0f};              // steps, kA
    float stepMs{550.0f};
    float slopeWindowMs{200.0f};
    int   stepReps{6};
    float sampleMs{10.0f};
    float minSpeedInPerSec{3.0f};  // below this the fit reads stiction as slope
    float maxTravelIn{60.0f};      // room needed either side of the start
    float minR2{0.9f};
    float maxKaSpread{0.5f};
    float freeSpeedLo{0.55f};
    float freeSpeedHi{1.15f};
    int   spinRotations{6};
    float parkSeconds{6.0f};
    std::function<void(const char* stage, const char* detail)> onProgress;
};

struct FeedforwardFit {
    float kV, kS, kA;        // volts per m/s, volts, volts per m/s^2
    float r2;                // straightness of the staircase
    float kaSpread;          // (max - min) / kA across step reps
    float freeSpeedRatio;    // fitted no-load speed / what rpm implies
    float speedSpanInPerSec;
    int   samples;           // staircase levels that held steady
    int   stepReps;          // step reps that produced a kA
    bool  valid;             // kS and kV usable
    bool  kaValid;           // kA usable; kA is left as fitted either way
    bool  drivetrainSuspect;
};

struct NoiseFit {
    float sigmaIn, sigmaThetaDeg, peakIn;
    int   samples;
    float suggestTolerance;    // targetTolerance
    float suggestBearingHold;  // MoveToPointParams::bearingHoldDist
};

struct OdomDriftFit {
    float alongPerIn, lateralPerIn;   // sigma per inch driven
    float thetaPerIn;                 // heading sigma, radians per inch
    float distanceIn;
    int   reps;
    bool  valid;
};

struct RangeNoiseFit {
    float sigmaIn[MAX_DISTANCE_SENSORS];
    float meanIn[MAX_DISTANCE_SENSORS];
    int   samples[MAX_DISTANCE_SENSORS];
    float worstSigmaIn;
};

class Characterizer {
public:
    Characterizer(Chassis&, Drivetrain&, const CharacterizeConfig& = {});
    FeedforwardFit feedforward(bool spinInPlace = false);
    float trackWidth();
    float measureScale(float trueDistanceIn, float commandDistanceIn = 24.0f);
    NoiseFit localizationNoise();
    OdomDriftFit measureOdomDrift(float legIn = 36.0f, int reps = 3);
    RangeNoiseFit measureRangeNoise(
        const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors);
};
```

All three `OdomDriftFit` residuals are **sigmas**. The out-and-back sums
`|residual|` because a signed sum would cancel, but every filter field they
feed expects a standard deviation, and `E|X| = sigma * sqrt(2/pi)` for a
zero-mean Gaussian — so `sigmaFromMeanAbs` multiplies by `sqrt(pi/2)`. Without
it the figures handed to the filter are 20% low, which is the direction that
makes a filter gate out the corrections it needs.

`thetaPerIn` is per inch **driven**, not per radian turned: the legs are
straight, so there is no rotation to divide by. It feeds
`ekf.thetaNoisePerInch`. Nothing measures `ekf.thetaNoiseProp`, which is per
radian actually turned and would need a routine that turns.

The robot drives itself. `feedforward()` runs two phases, and the split is not
an accident of history -- a single voltage ramp **cannot** identify `kA` at all.
Solving `kS + kV*v + kA*a = c*t` for a ramp rate `c` gives `a = c/kV` once the
opening transient has passed, so the acceleration column is constant, its only
variation is the noise in a differenced encoder velocity, and least squares
attenuates `kA` toward zero while `kS` absorbs the `kA*c/kV` it dropped. Against
a first-order plant with 200 rpm cartridge quantisation the joint ramp fit
returned `kA` **with the wrong sign in 100 of 100 trials** and `kS` 32% high --
at `r2 = 0.997`, which is why `r2` is not the only gate.

So:

1. **Quasistatic staircase** for `kS` and `kV`. Dwell at each of
   `staircaseLevels` voltages for `dwellMs`, average the last `holdMs`, and fit
   `volts = kS + kV*speed` over the dwells that reached steady state. `a` is
   zero by construction rather than fitted. The lowest level sits just above
   the measured `breakawayVolts()`, which matters more than it sounds: `kS` is
   an intercept, and starting from a fixed voltage instead makes it an
   extrapolation, worth 15-25% of error against 8% when anchored.
2. **Steps** for `kA`. `stepReps` runs from rest at `stepVolts`, each giving one
   `kA = (volts - kS - kV*speed) / accel`, with `accel` a least squares slope
   over `slopeWindowMs` starting where the robot actually broke away. The
   median is taken, so a rep that slipped moves it one place instead of
   dragging a mean. In simulation this lands `kA` within 2%.

Direction alternates so the robot oscillates about its start rather than
walking away, but a high level still covers `dwellMs` at full speed: give it
`maxTravelIn` either side. Levels that cannot fit their hold window inside that
are skipped, and `samples` reports how many survived. `measureScale()` and
`measureOdomDrift()` need a clear lane of 24 and 36 in respectively. The rest
spin on the spot.

Three gates, because they fail independently:

| Gate | Covers | Fails when |
|---|---|---|
| `r2` vs `minR2` | staircase straightness | a motor stalls mid-run, the surface changes |
| `kaSpread` vs `maxKaSpread` | agreement between step reps | one rep slipped or never broke away |
| `freeSpeedRatio` in `[freeSpeedLo, freeSpeedHi]` | the drivetrain itself | a dead motor, wrong `externalGearRatio`, wrong `wheelDiameter` |

The third is the one worth having. A drivetrain that is merely *weaker* than
configured still produces a perfectly straight line: with half the motors
pulling, `r2` stays at 0.98 and `kaSpread` at 0.05 while every constant is 70-85%
wrong. Only comparing the fitted no-load speed `(12 - kS)/kV` against what
`rpm` and `wheelDiameter` claim catches it -- measured 0.66-1.00 across healthy
plants against 0.39-0.40 for a halved drivetrain or a doubled wheel diameter.

`kA` is returned as fitted even when `kaValid` is false, so you can see what it
was; `autonCharacterize1Drive` writes `0` into `ffGains` in that case, since a
baseless `kA` feeds the velocity loop a voltage proportional to commanded
acceleration and the old ramp fit could make it negative.

One thing none of the gates catch, because it is not an error: on a drivetrain
with real quadratic drag, the fit returns the best *linear* model over the speed
range it saw. `FeedforwardGains` has no quadratic term to hold anything else.

The maths is PROS-free in `chad/util/characterize_fit.hpp` (`fitVelocity`,
`fitAccel`, `slopeOverWindow`, `freeSpeedRatio`), so a whole fit runs on the
host against a plant model rather than only on a robot.

```cpp
chad::tuning::CharacterizeConfig cfg;
cfg.onProgress = [](const char* stage, const char* detail) {
    printf("[%s] %s\n", stage, detail);
};
chad::tuning::Characterizer ch(chassis, drivetrain, cfg);

float width = ch.trackWidth();          // spins ~6 turns
auto  ff    = ch.feedforward(false);    // staircase + steps, kV/kS/kA
auto  turn  = ch.feedforward(true);     // turning friction
auto  noise = ch.localizationNoise();   // parked, with your race config

drivetrain.trackWidth = width;

chad::RAMSETEParams p;
p.useVelocityController = true;
p.ffGains.kV            = ff.kV;
p.ffGains.ksStraight    = ff.kS;
p.ffGains.kaStraight    = ff.kaValid ? ff.kA : 0.0f;
p.ffGains.ksTurn        = turn.kS;
p.ffGains.kP            = 12.0f;    // then sweep in the sim
p.ffGains.integralClamp = 2.0f;
p.targetTolerance       = noise.suggestTolerance;
```

`ffGains` is per-motion, so the fitted values need somewhere to live between
runs. `config.cpp` holds one `FeedforwardGains ramseteFf` for that; write the
phase 1 numbers there and pass it to every motion that follows a path:

```cpp
p.useVelocityController = true;
p.ffGains               = ramseteFf;
```

`ffGains` is in metres and volts, unlike the rest of the API. `kV` comes out the
same whether the ramp drives straight or spins, because it is fitted against
per-side wheel speed either way -- which is what lets phase 1 skip the straight
ramp when there is no room for one.

`measureScale()` needs a tape measure; it returns the factor to multiply
`Drivetrain::wheelDiameter` by.

```cpp
drivetrain.wheelDiameter *= ch.measureScale(24.5f, 24.0f);  // true, commanded
```

`trackWidth()` divides the wheels' differential travel by the heading the IMU
reports turning through, so it is independent of wheel diameter.
`localizationNoise()` measures scatter while parked, which is noise by
definition; accuracy is a different question and needs a known field position.

### Filter noise

Most of what looks like MCL tuning is characterisation. These two measure the
physical quantities the filters model, so the numbers go straight into config
rather than being searched for.

```cpp
auto drift = ch.measureOdomDrift(36.0f, 3);            // out and back, 3 reps
auto beams = ch.measureRangeNoise(config.distanceSensors);  // parked at a wall
```

`measureOdomDrift()` drives out and back, so the physical start and end are the
same point and whatever odometry reports instead is its own error, split along
and across the starting heading. Run it with localization off.

```cpp
config.mclConfig.imeVariance                = drift.alongPerIn;
config.mclConfig.horizConstantNoise         = drift.lateralPerIn;
config.mclConfig.rangeSigmaCloseIn          = beams.worstSigmaIn;
config.ekfConfig.vertNoiseProp              = drift.alongPerIn;
config.ekfConfig.slipNoiseProp              = drift.lateralPerIn;
config.ekfConfig.rangeSigmaBase             = beams.worstSigmaIn;
```

What is left over genuinely cannot be measured on a robot, because scoring it
needs a known true pose: `distResampleVariance`, `distSyncProp`, the gate
thresholds. Those are what `sim/test/autotune.mjs` searches.

---

## Simulator and sweeps

The browser simulator lives on the **`sim` branch**, not here. It compiles this
library to WebAssembly and runs it against fake hardware, so a gain fitted there
is a gain you can paste into the robot.

```sh
git checkout sim
./sim/build.sh
python sim/serve.py       # http://localhost:8080
./sim/test/run.sh         # headless regression suite
```

`sim/README.md` on that branch documents the route editor that runs real auton
C++, the parameter sweeps in `sim/test/tune.mjs`, and the coordinate-descent
search in `sim/test/autotune.mjs` that fits the gains above no robot can score.

Two caveats travel back here. The sim models no traction, torque, current limit
or voltage sag — differential-drive kinematics are exact, so the robot cannot
slide and `maxLateralAccel` is a profile-speed knob rather than a grip limit.
And `encoderSlip`, `imuDriftDegPerSec` and `imuNoiseDeg` default to zero, so a
localization comparison left at defaults is measuring dead reckoning against
itself.

Because the sim sits on its own branch, a change merged here is not checked
against the wasm bridge until it reaches `sim`. Merge `main` into `sim` and run
`./sim/test/run.sh` before trusting anything fitted there.

---

## Logging

`chad/util/logger.hpp` — buffered CSV to the microSD card. `row()` never touches
the card; a background task does the writes.

```cpp
Logger(std::string name, std::vector<std::string> columns,
       std::size_t capacityRows = 256);

bool start();                                  // false if no card / can't open
void stop();
void row(std::initializer_list<float> values); // no-op if start() failed
bool isRunning() const;
uint32_t droppedRows() const;                  // non-zero = writer fell behind
const std::string& path() const;
static bool sdInserted();

static constexpr std::size_t kMaxColumns = 15; // t_ms prepended on top
```

Writes `/usd/<name><n>.csv` at the first free `n`, so runs never overwrite each
other. Flushes every 50 ms. On overflow the *newest* row is dropped, so the start
of a motion survives.

### Prebuilt hooks

Attach a logger to a subsystem's telemetry callback. All off by default, all
no-ops without a card.

```cpp
// chad/util/tuning.hpp
chad::tuning::logChassis(Chassis&, bool enabled);                 // chas<n>.csv
chad::tuning::logJamDetector(JamDetector&, bool enabled);         // jam<n>.csv
chad::tuning::logFeedforward(FeedforwardController&, bool);       // ff<n>.csv

// gamespecific/tuning.hpp
logLift(bool enabled);                                            // lift<n>.csv
```

Plot with `python tools/plot_log.py <file>.csv [--cols a b] [--xy]`.

---

## Jam detection

Generic N-motor stall detection with automatic reversal. `update()` never blocks.

```cpp
JamDetector(const Config& = {});
JamDetector(const std::vector<pros::Motor*>&, const Config& = {});

void addMotor(pros::Motor*, float reverseScale = -1.0f, bool monitored = true);
void clearMotors();
std::size_t motorCount() const;

void setVelocities(const std::vector<float>&);   // latched, reapplied after reversal
void setVelocity(std::size_t index, float);
void setAllVelocities(float);
void stop();
float commandedVelocity(std::size_t index) const;

void update();                                   // call every 10-20 ms

void enable(bool);
bool isEnabled() const;
void setSuppressCondition(std::function<bool()>);
bool isUnjamming() const;
bool isOverTemp() const;
State state() const;                             // Normal | Reversing | Settling
uint32_t jamCount() const;
void resetStats();
void reset();

const Config& config() const;
void setConfig(const Config&);
void setTelemetry(std::function<void(const JamSample&)>);
```

```cpp
struct JamSample {              // covers the first 3 registered motors
    float efficiency[3];        // percent; inf if disconnected
    float temperature[3];       // celsius
    float command[3];           // latched rpm
    int state;
    uint32_t jamCount;
};
```

`reverseScale` is the command multiplier while unjamming: `-1` reverses, `0`
holds, `+1` keeps driving forward.

| `JamDetectorConfig` field | Default | |
|---|---|---|
| `jamTriggerMs` | `150` | Continuous stall before unjamming |
| `reverseDurationMs` | `250` | |
| `settleDurationMs` | `150` | Detection pause after a reversal |
| `maxTempC` | `55.0` | Above this, motors held at zero |
| `jamEfficiencyLowPct` / `HighPct` | `7` / `20` | Efficiency band identifying a jam |
| `suppress` | never | Return true to skip detection and reset the timer |

The efficiency band is **empirical, from one robot's intake**. Log
`get_efficiency()` while deliberately jamming your mechanism before trusting it —
that's what `logJamDetector()` is for.

---

## Controller warnings

Each spawns a background task; the referenced devices must outlive it.

```cpp
void startTempWarning(pros::Controller&, const std::vector<pros::MotorGroup*>&,
                      double tempLimitC = 55.0, double batteryLimitC = 45.0);
void startMotorDisconnectWarning(pros::Controller&);
void startDistanceDisconnectWarning(pros::Controller&,
                                    const std::vector<pros::Distance*>&);
```

Temperature warnings back off 10 s after firing, then resume.

---

## ScaledIMU

`pros::Imu` subclass correcting the under-reporting typical of V5 IMUs.
`get_rotation()` and `get_heading()` are virtual overrides, so the scaling
applies even through a `pros::Imu*`.

```cpp
ScaledIMU(std::uint8_t port,
          double expectedRotation = 360.0,
          double actualReading = 355.0);

double scaleFactor() const;
void   setScaleFactor(double expectedRotation, double actualReading);
```

To measure: rotate the robot exactly 10 full turns, read
`imu.get_rotation() / 10.0`, pass that as `actualReading`.

---

## Sensor calibration

### SensorCalibrator

Least-squares solver for distance-sensor **lateral** mount offsets. Park facing a
flat wall, sweep a few headings, fit a line. **Blocking, and it moves the robot.**

```cpp
SensorCalibrator(Chassis&, const Config& = {});

std::vector<CalibrationResult> calibrateAll(const std::vector<CalibrationConfig>&);
CalibrationResult calibrate(const CalibrationConfig&);   // does NOT restore heading

const Config& config() const;
void setConfig(const Config&);
```

```cpp
struct CalibrationConfig {
    std::string name;
    pros::Distance* sensor{nullptr};
    float baseHeadingDeg{0};     // VEX heading where this sensor faces the wall head-on
};

struct CalibrationResult {
    std::string name;
    float offsetIn{0};           // -> ChassisConfig::sensorMounts[i].y, LEFT POSITIVE
    float confidence{CALIBRATION_FAILED_SCORE};
    int   samplesUsed{0};
    bool  valid{false};
};
```

Two traps in `confidence`: **lower is better** — it's a mean squared residual, not
a confidence score. And with exactly 2 samples the line passes through both
points, so the residual is identically 0 and the number is meaningless. Check
`samplesUsed >= 3`.

Only the lateral offset is identifiable from this sweep; measure the forward
offset with a ruler.

| `SensorCalibratorConfig` field | Default | |
|---|---|---|
| `sweepAnglesDeg` | `{0, 30, 35}` | Relative to `baseHeadingDeg`; needs ≥2 distinct |
| `turnMaxSpeed` | `60` | |
| `turnTimeoutMs` | `1000` | |
| `settleDelayMs` | `500` | |
| `postTurnDelayMs` | `600` | A still-rocking robot smears the reading |
| `maxValidReadingMm` | `9990` | Discards the "nothing in range" sentinel |
| `verbose` | `true` | printf to the PROS terminal |

### estimateWallAngle

Absolute heading fix from two forward sensors against a flat wall — drift-free,
which the IMU cannot be.

```cpp
float estimateWallAngle(pros::Distance& sensorA,   // robot's RIGHT
                        pros::Distance& sensorB,   // robot's LEFT
                        float robotHeadingDeg,     // VEX degrees, picks the quadrant
                        const WallAngleConfig& = {});

bool wallAngleValid(float wallAngleDeg);
constexpr float WALL_ANGLE_INVALID = -1000.0f;
```

```cpp
float wall = estimateWallAngle(rightDist, leftDist,
                               stdRadToVexDeg(chassis.getPose().theta));
if (wallAngleValid(wall)) chassis.setHeading(wall);
```

`setHeading()` is the right entry point — it fans the correction out to odometry,
MCL and RCL together. Do it stationary and square to a known-clear wall.

**Never validated on hardware.** In the donor repo this was dead code referencing
an undefined global, so it could not have linked. If the correction pushes your
heading the wrong way, swap the two sensor arguments.

| `WallAngleConfig` field | Default | |
|---|---|---|
| `sensorBaselineIn` | `10.75` | Between the lenses, not the housings |
| `sensorBiasMm` | `10.0` | Added to sensorB, for a mounting-depth mismatch |
| `maxValidReadingMm` | `1800` | Beyond ~1.8 m the noise swamps the measurement |

---

## Feedforward and LTV

Both work in **metres**, not inches.

### FeedforwardController

Converts a desired chassis twist into motor voltages via `V = kV·v + kA·a +
kS·sign(v)`, plus per-side PI correction, then desaturates proportionally so a
saturated side doesn't distort the commanded curvature.

```cpp
FeedforwardController(const FeedforwardGains&, float trackWidth);

DrivetrainVoltages update(float targetLinearVel, float targetAngularVel,
                          float measuredLeftVel, float measuredRightVel,
                          float dt);            // seconds — pass the REAL elapsed time
void reset();                                   // before each new motion

void setGains(const FeedforwardGains&);
const FeedforwardGains& getGains() const;
void  setTrackWidth(float);
float getTrackWidth() const;
void  setMaxVoltage(float volts);               // default 12.0
float getMaxVoltage() const;
void  setTelemetry(std::function<void(const FeedforwardSample&)>);
```

```cpp
struct FeedforwardGains {
    float kV{0};                  // volts per velocity
    float kaStraight{0}, kaTurn{0};
    float ksStraight{0}, ksTurn{0};
    float kP{0}, kI{0};           // on velocity error
    float integralErrorGate{0};   // only integrate below this error; <=0 disables
    float integralClamp{0};       // hard cap on the accumulated integral; <=0 disables
};

struct FeedforwardSample {
    float vTarget, wTarget;
    float vLeftTarget, vRightTarget, vLeftMeasured, vRightMeasured;
    float aLinear, aAngular;
    float voltLeft, voltRight;
    float dt;
};
```

Output is in **volts**. Fit the gains by logging with `logFeedforward()` and
least-squares fitting the CSV offline. Set `integralClamp` — without it the
integrator grows without bound whenever the drivetrain can't reach its target
(pushing a wall, or another robot).

### LTVPathFollower

LQR-unicycle trajectory follower over precomputed paths.

```cpp
LTVPathFollower(Chassis&, Drivetrain&, const FeedforwardGains&);

void followPath(const std::string& path_name, const ltvConfig&);
double getPathLength(const std::string& path_name);
void waitUntilDone();
void waitUntil(float dist_inches);
void waitUntil(float x_inch, float y_inch, float radius_inch = 2.0f);
void cancel();
bool isRunning();
void precompute_paths(const std::vector<std::string>& path_names);
void setAbortCondition(std::function<bool()>);       // empty = never abort
FeedforwardController& velocityController();         // for logFeedforward()
```

`ltvConfig` carries forward/backward LQR weight sets (`q_x`, `q_y`, `q_theta`,
`r_ang`, `r_vel` and `_b` backward variants) plus `backwards`, `turnFirst`,
`end_correction`, `mpose_lead`, `max_lin_correction`, `max_ang_correction`,
`exit_points`, `q_scalar`, `log`, `test`, `path_index`.

```cpp
struct State { float x, y, heading, linear_vel, angular_vel; };   // METRES, math radians
```

---

## Velocity profiles

```cpp
std::vector<ProfilePoint> generateTrapezoidal(float distance, float maxVel,
                                              float maxAccel, float dt = 0.01f);
std::vector<ProfilePoint> generateSCurve(float distance, float maxVel,
                                         float maxAccel, float maxJerk,
                                         float dt = 0.01f);

struct ProfilePoint { float time{0}, velocity{0}, position{0}; };
struct MotionProfile { float maxVelocity, maxAccel, maxJerk, distance, duration; };
```

---

## FastTrig

LUT-based sin/cos for high-frequency localization loops. 1024 entries per
quadrant (~0.088° resolution), 4 KB table, no interpolation.

```cpp
float FastTrig::sin(float rad);
float FastTrig::cos(float rad);
```

---

## Game-specific layer

`include/gamespecific/` — the robot-specific half. Edit these freely; the `chad`
namespace below them doesn't depend on any of it.

### config.hpp

Declares the globals defined in `config.cpp`: `master`, `leftMotors`,
`rightMotors`, `frontMotor`, `leverMotor`, `armMotor`, `imu`, `frontOptic`, the
pneumatics (`matchLoadGate`, `lift`, `leftDescoreArm`, `trapDoor`, `intakeLift`),
`front_dist` / `left_dist` / `back_dist` / `right_dist`, `distSensors`,
`sensorMounts`, `drivetrain`, `odomSensors`, `chassisConfig`, `chassis`,
`selector`.

```cpp
void registerFieldObstacles();
void registerAutonDisableLine(float lifetimeMs = 16000.0f);
void clearAutonDisableLine();
```

`registerFieldObstacles()` feeds the alliance studs and neutral goals to both RCL
and MCL, and `initialize()` calls it, so the map is live for every run.
`fieldCircleObstacles()` hands the same list to the tuning autons, which solve
wall truth against it. `registerAutonDisableLine()` arms RCL's far-side gate on
the diagonal auton line, so returns from the half where the opponents are moving
are not taken for walls. It keeps the half the robot starts in, whichever side
of the routine's `setPose` it is called on, and expires 16 s after arming on the
wall clock; `clearAutonDisableLine()` drops it at once.

### autons.hpp / controls.hpp

```cpp
void autonExample();   // exercises all eight motion types
void autonSkills();
void autonRedLeft();
void autonBlueLeft();

void startPoseTelemetry();
void driverControl();
```

### lift.hpp

```cpp
void liftMoveTo(double targetDegrees, double maxSpeed = 127.0,
                double timeoutMs = 3000, bool async = false);
void liftSetTelemetry(std::function<void(const LiftSample&)>);

struct LiftSample { float target, position, error, pid, feedforward, power; };
```

Gravity feedforward is available but **disabled** — `kG` is `0.0` in
`lift.cpp`, alongside `MOTOR_DEG_PER_ARM_DEG` and `HORIZONTAL_MOTOR_DEG`. Set all
three to enable it; `logLift()` logs the PID and gravity terms separately so
you can see what `kG` is contributing.

---

## Numbers that bite

`targetTolerance` must exceed localization error, or settle never fires and the
motion times out. Under MCL: 1 in against 1.5 in error failed 3/3, 3 in passed.
Pick it with `localizationNoise()`.

`lateralGains.kP` has a ceiling that has nothing to do with stability margin.
`moveDistance` and `moveToPoint` have **no velocity profile** -- they are P+D
against the drivetrain, so the only thing that stops them is the output leaving
saturation in time to decelerate. `127/kP` is how far from the target that
happens; the stopping distance is roughly speed times the drivetrain's response
time. Once `127/kP` is the smaller of the two the robot sails through its
waypoint and reverses out, and no tolerance or debounce can fix it. On the
simulator's 70 ms lag the turn is sharp: 0.00 in of overshoot up to kP 14,
0.40 in at 15, 1.22 in with seven reversals at 16, 3.5 in at 20. Your lag is
not 70 ms, so measure it -- but the shape of the limit is real, and a final
pose alone cannot see it. Watch the robot, or record the peak distance past
the target.

`bearingHoldDist` stays small. `atan2` to a target at shrinking range is mostly
noise, which a tuned angular loop chases at 591 deg/s. Holding the last good
bearing fixes that, but too large a radius holds a stale one: 3 failures in 15
at 1.5 in, 12 in 15 at 6 in.

It is the boomerang's radius too. Near the target the carrot has collapsed
onto it, so the carrot bearing is noise; its weight in the blended heading is
only 2%, but 2% of a random angle is a degree and a half of final heading
error that no tolerance can tighten away. Holding the bearing inside
`bearingHoldDist` took the same motion from 1.65 deg to 0.31 deg. Dropping the
carrot term entirely instead does NOT work: it is the only lateral authority a
boomerang has once it is locked to the final heading, and position then
plateaus 0.30 in out.

Arc and boomerang both used an unsigned distance as the along-track error, so
the term could only grow and an overshoot drove away from the target rather
than backing up. A boomerang aimed 34 in away left the target at 1.3 in and
kept going to 9 in and 161 deg off before the timeout stopped it. Both now
project onto the heading the way `moveToPoint` does. On an arc the magnitude
is kept and only the sign is taken, because partway round the heading is up to
half the sweep off the chord and a projection would throttle the term to
nothing on a near-semicircle.

`distResampleVariance` is not a smoothness knob. Lowering it improves every
tracking number and removes kidnap recovery entirely — 3 recoveries in 4 at the
default 1.5, 0 in 4 at 1.0 and below.

`ffGains.kP` corrects actuator lag, so it depends on your motors -- and it
depends on them more than one number can cover. Re-measured on the
simulator's routing battery, the value that works tracks `responseTauMs`
directly: at 140 ms, `kP` 12 is the best of the four tested; at the sim's
default 70 ms it stalls the robot dead 0.36 in short of the last path point
and burns the timeout, where 2 and 6 finish; at 30 ms every `kP` tested times
out and only the open loop completes. So the earlier claim that 12 held from
20 to 200 ms does not reproduce. Leave `useVelocityController` off until you
have measured your own lag, which is what
`autonCharacterize1Drive` is for.

`integralClamp` defaults to 0, meaning disabled. Set it whenever you set `kI`,
or a drivetrain held short of its target winds up without bound.

Angular `kP` near 100 is normal here. It does not cause endpoint spin; bearing
geometry did.

Run MCL and the EKF together, not MCL alone. Hairpin, realistic odometry error:

| mode | estimate error | pose jerk | kidnap recovery |
|---|---|---|---|
| dead reckoning | 4.38 in | 0.0069 | never |
| MCL only | 5.84 in | 0.4602 | 2.4 s |
| MCL + EKF | 2.17 in | 0.0109 | 1.7 s |

Dead reckoning is smoother than either filter alone and never recovers from
displacement.
