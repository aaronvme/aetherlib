# riku API reference

Everything public, by module. `#include "riku/api.hpp"` pulls in all of it;
`#include "gamespecific/gamespecific_api.hpp"` adds the robot-specific layer.

- [Conventions](#conventions)
- [Chassis](#chassis) — the main API
- [Motion parameters](#motion-parameters)
- [Pose and math](#pose-and-math)
- [PID and exit conditions](#pid-and-exit-conditions)
- [Drivetrain](#drivetrain)
- [Tracking](#tracking) — odometry, RCL, MCL
- [Auton selector](#auton-selector)
- [PID auto-tuner](#pid-auto-tuner)
- [Logging](#logging)
- [Jam detection](#jam-detection)
- [Controller warnings](#controller-warnings)
- [ScaledIMU](#scaledimu)
- [Sensor calibration](#sensor-calibration)
- [Feedforward and LTV](#feedforward-and-ltv)
- [Velocity profiles](#velocity-profiles)
- [FastTrig](#fasttrig)
- [Game-specific layer](#game-specific-layer)

---

## Conventions

Getting these wrong is the most common source of confusion, so they're up front.

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

`riku/chassis.hpp` — composes tracking, motion and driver control.

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

Blocking unless `async = true`. All headings in VEX degrees.

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
void waitUntilSettled();
void waitUntilDist(float dist);   // inches travelled
```

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
void setBrakeMode(pros::motor_brake_mode_e);
void setDistanceSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>&);
```

### Dry run

Movement commands record a path without driving motors — used for selector path
preview.

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

Set it before starting a motion — the motion task reads it each tick. Empty by
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
| `settleTimeMs` | `0` | Settle condition must hold this long before exiting. `0` exits on the first in-tolerance sample, which can fire while still carrying velocity through the target |
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
    float targetTolerance{1.0};      // inches
    float earlyExitRange{0.0};       // exit early within this range; 0 = off
};

struct MoveToPointParams {           // + lead
    bool  forwards{true};
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{1.0}, earlyExitRange{0.0};
    float lead{0.0};                 // carrot-point lead for curved approaches
};

struct TurntoHeadingParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{2.0};      // VEX degrees
};

struct TurnToPointParams {           // + offset
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{2.0};
    float offset{0.0};               // angular offset, VEX degrees
};

struct SwingParams {                 // same shape as TurntoHeadingParams
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{2.0};
};

struct ArcParams {
    bool  forwards{true};
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{1.0};
    float lead{0.0};
};

struct BoomerangParams {
    bool  forwards{true};
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{1.0};
    float headingTolerance{2.0};     // VEX degrees
    float lead{0.5}, leadDecay{0.5};
};

struct RAMSETEParams {
    float b{2.0};                    // aggressiveness, b > 0
    float zeta{0.7};                 // damping, 0 < zeta < 1
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{1.0};      // inches
    float headingTolerance{2.0};     // VEX degrees
    float lookaheadDist{10.0};       // inches
    bool  useVelocityProfile{true};
    float maxAccel{50.0};            // in/s²
};
```

`b` and `zeta` use the conventional metre-based RAMSETE parameterization; the
controller rescales to inches internally, so the textbook defaults work as-is.

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

`riku/util/util.hpp` — header-only.

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

All four guard non-finite input and return `0`. That matters: IMU errors surface
as `PROS_ERR_F` (`INFINITY`), and a subtractive wrap loop on infinity never
terminates — it hard-freezes the brain.

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
float offsetIn() const;        // +right / +forward
float wheelDiamIn() const;
float gearRatio() const;
SensorType sensorType() const; // ADI | Rotation
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
    float imuScaleFactor{1.0f};
};
```

Use **either** `imuScaleFactor` **or** [`ScaledIMU`](#scaledimu) — never both on
the same IMU.

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
float imuHeadingDeg() const;      // VEX degrees
float imuHeadingRad() const;      // standard math radians
```

Prefer `consumeDelta()` when more than one task drives `update()` — the
background odom task does.

### RclTracking

Range-corrected localization: intersects distance-sensor rays with known field
geometry for an absolute position fix.

```cpp
RclTracking(OdomTracking&, const Config&);

void setSensor(int index, pros::Distance*, const Pose& mountOffset);
void configureSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>&,
                      const std::array<Pose, MAX_DISTANCE_SENSORS>&);
void enableSensor(int), disableSensor(int);
void disableSensorFor(int index, float durationMs);

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
void setRclPose(const Pose&);

const std::array<RclSensor, MAX_DISTANCE_SENSORS>& sensors() const;
Pose latestEstimate() const;
int  activeSensorCount() const;
```

`enum class CoordType { X, Y, INVALID }` — a single distance sensor constrains
one axis only.

| `Config` field | Default | |
|---|---|---|
| `sensorCount` | `MAX_DISTANCE_SENSORS` | |
| `maxSyncDist` | `5.0` | Max inches to sync per update |
| `accumulationAlpha` | `0.3` | EMA factor |
| `updatePeriodMs` | `10.0` | |
| `angleTolerance` | `10.0` | Ray must be within this of a wall axis |
| `confidenceThreshold` | `30` | Min V5 sensor confidence (0–63) |
| `autoSync` | `true` | |
| `useAccumulation` | `false` | |

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
| `maxSensorRange` | `100.0` | inches |
| `distResampleVariance` | `1.5` | inches of jitter per resample |
| `thetaResampleVariance` | `0.01` | radians |
| `maxThetaDeviation` | `0.08` | radians, clamp vs IMU |
| `trackingWheelVariance` | `0.08` | dead wheels |
| `imeVariance` | `0.12` | motor encoders — more slip |
| `imuVariance` | `0.005` | |
| `horizDependentVarianceProp` | `0.03` | × \|dVert\|, lateral slip |
| `horizConstantNoise` | `0.03` | inches/update baseline |
| `distSyncProp` / `thetaSyncProp` | `0.10` / `0.001` | Lerp toward chassis |
| `updatePeriodMs` | `25.0` | |
| `confidenceThreshold` | `10` | |
| `minSensorRange` | `1.0` | mm; raise for recessed sensors |
| `autoSync` | `true` | |

Season-specific field targets are `static constexpr` arrays inside
`mcl_tracking.hpp` (`kFieldTargets`, `kFieldCircles`, `kDisablingLines`) and are
currently empty — fill them in per game. `kFieldWalls` never changes.

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

`riku/util/autotune.hpp`. Drives repeated step responses and fits gains from the
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
→ kD (as a multiple of kP, stop when cost stops improving) → kI (only if
steady-state error remains). Gains carry forward between step sizes. Results are
applied to the chassis and returned; **nothing is persisted** — copy them into
`ChassisConfig` yourself.

| `AutoTuneConfig` field | Default | |
|---|---|---|
| `angleStepsDeg` | `{90, 180}` | Turn sizes, in order |
| `distanceStepsIn` | `{24, 48}` | |
| `trialMs` | `1500` | Each trial runs the full window |
| `reps` | `2` | Out-and-back pairs per candidate, averaged |
| `kpStart` / `kpStartLateral` | `20` / `4` | |
| `kpGrowth` | `1.5` | |
| `overshootLimitPct` | `20` | Stop growing kP past this |
| `oscillationLimit` | `3` | |
| `kdRatioStep` | `0.05` | kD searched as a multiple of kP |
| `steadyStateTolDeg` / `steadyStateTolIn` | `1.5` / `0.75` | kI only attempted above this |
| `kiStep` | `0.01` | |
| `maxIterations` | `6` | Per stage |
| `weights` | see below | Cost function |
| `onProgress` | empty | `void(const char* stage, PIDGains, float cost)` |

Each trial forces `targetTolerance = 0` so the motion cannot settle early. Without
that the motion exits the instant it reaches tolerance and the overshoot is never
observed.

Budget roughly 3.5 minutes per axis at defaults.

### Step-response analysis

`riku/util/step_response.hpp` — pure, no PROS, unit-testable on the host.

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

## Logging

`riku/util/logger.hpp` — buffered CSV to the microSD card. `row()` never touches
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
// riku/util/tuning.hpp
riku::tuning::logChassis(Chassis&, bool enabled);                 // chas<n>.csv
riku::tuning::logJamDetector(JamDetector&, bool enabled);         // jam<n>.csv
riku::tuning::logFeedforward(FeedforwardController&, bool);       // ff<n>.csv

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

`include/gamespecific/` — the robot-specific half. Edit these freely; the `riku`
namespace below them doesn't depend on any of it.

### robot_config.hpp

Declares the globals defined in `robot_config.cpp`: `master`, `leftMotors`,
`rightMotors`, `frontMotor`, `leverMotor`, `armMotor`, `imu`, `frontOptic`, the
pneumatics (`matchLoadGate`, `lift`, `leftDescoreArm`, `trapDoor`, `intakeLift`),
`front_dist` / `left_dist` / `back_dist` / `right_dist`, `distSensors`,
`sensorMounts`, `drivetrain`, `odomSensors`, `chassisConfig`, `chassis`,
`selector`.

```cpp
void registerFieldObstacles();
void registerAutonDisableLine(float lifetimeMs = 16000.0f);
```

`registerFieldObstacles()` feeds the alliance studs and neutral goals to both RCL
and MCL. `registerAutonDisableLine()` marks the diagonal auton line as a
temporary obstacle so readings across it aren't mistaken for walls.

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
