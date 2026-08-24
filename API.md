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
void followTrajectory(const Trajectory&, float timeout = 0,
                      const TrajectoryFollowParams& = {}, bool async = false);
```

All motions run on one shared motion task, so motors and PIDs are only ever
touched by a single thread. A blocking call waits for *its own* motion to
finish, not merely for the queue to drain.

### Motion status

```cpp
bool isSettled() const;
MotionOutcome lastOutcome() const;
bool lastMotionArrived() const;
void cancelMotion();
void waitUntilSettled();
void waitUntilDist(float dist);   // inches travelled
```

Every motion exits through one `SettleMonitor`, so how it ended is a value
rather than something to infer from the pose:

```cpp
enum class MotionOutcome {
    None,        // nothing has run yet
    Arrived,     // inside tolerance, held for the settle time
    ChainedOut,  // inside earlyExitRange; the drive was left running
    TimedOut,
    Stalled,     // the drivetrain stopped short of the target
    Cancelled,
    Invalid,     // the request itself was unusable
};
```

`lastMotionArrived()` collapses that to the question most routines actually
ask — did it reach what it was asked to reach, or give up. Branch on it before
committing to a scoring action:

```cpp
chassis.moveToPoint(24, 36);
chassis.waitUntilSettled();
if (!chassis.lastMotionArrived()) {
    // Stalled, timed out or cancelled. Do not run the score.
}
```

**Stall detection is on by default.** Below `stallSpeedIn` (0.5 in/s, measured
on the wheels rather than the pose, so rotating in place does not count) for
`stallTimeMs` (500), a motion gives up with `Stalled` instead of holding voltage
into a wall for the rest of its timeout — which stalls the motors and browns out
the brain. It is suppressed for `stallGraceMs` (300) after the start, or every
standing start would read as a stall. Set `stallSpeedIn` to 0 to disable.

A **turn** is tested on its turn rate against `stallSpeedDeg` (2.0 deg/s), not
on wheel surface speed. `stallSpeedIn` of 0.5 in/s over an 11.9 inch track is
4.81 deg/s, and a turn converging at that rate covers its last 4 degrees in
0.83 s — it was being called stopped and ended `Stalled`. `stallSpeedIn = 0`
still disables stall detection everywhere, turns included.

A motion that ends `Stalled` has genuinely failed to meet its criterion. A
curved path that runs its reference to the end and comes to rest inside its
position tolerance reports `Arrived`, even though RAMSETE leaves 5-12 degrees of
heading residual it has no authority to remove — that residual is expected, not
a failure.

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
    float lead{0.0};                 // carrot-point lead for curved approaches
};

struct TurntoHeadingParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{-1.0};     // VEX degrees, negative = ChassisConfig
};

struct TurnToPointParams {           // + offset
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{-1.0};
    float offset{0.0};               // angular offset, VEX degrees
};

struct SwingParams {                 // same shape as TurntoHeadingParams
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{-1.0};
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
    float b{48.0};                   // aggressiveness, b > 0
    float zeta{0.85};                // damping, 0 < zeta < 1
    float maxSpeed{127}, minSpeed{0};
    float targetTolerance{3.0};      // inches, negative = ChassisConfig
    float headingTolerance{2.0};     // VEX degrees
    float lookaheadDist{10.0};       // inches
    bool  useVelocityProfile{true};
    float maxAccel{50.0};            // in/s²
};
```

`b` is in rad²/m² — the conventional metre-based RAMSETE parameterization — and
the controller rescales it by `INCH_TO_METER²` internally, so a `b` quoted for a
metre-unit robot can be used unchanged. The textbook `b = 2`, `zeta = 0.7` are
*not* the defaults here: a VEX field's curves are much tighter relative to the
robot, and `b = 2` leaves 3.4–12.5 in of cross-track where 48 leaves 1.2–2.5 in
on the same paths. See `RAMSETEParams` for the sweep.

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
EncoderAverage averagePositionDegLive() const;   // value + live/faulted counts
EncoderAverage leftPositionInLive() const;
EncoderAverage rightPositionInLive() const;
WheelReading   wheelVelocitiesLive() const;      // both sides, one sweep
```

The scalar forms return `0.0f` when every encoder on that group is faulted,
which a consumer cannot tell from standing still — so odometry and the motion
loop's rest test use the `Live` forms, hold the last good value, and count the
outage. Reading the scalar form instead freezes the pose while the robot drives,
with one rejected jump and nothing after it to say why.

```cpp
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
bool  faulted() const;         // the sensor errored on the last read
uint32_t faultCount() const;
```

A faulted wheel is dropped from the average rather than averaged in as a held
value — averaging a frozen reading with live ones scales the whole distance by
`live / total`, which is not a jump and so slips past every jump test.

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

Sensor faults are counted, not just survived: `distanceFaults()`,
`distanceOutages()` and `imuFaults()`. An outage is every source of one kind
failing at once, which is the case that would otherwise freeze the pose while
the robot keeps driving.

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
| `maxSyncDist` | `5.0` | Inches per update, but `min(maxSyncDist, maxSyncRate * updatePeriodMs/1000)` is what applies, and at the defaults that is `0.08` in — so this cannot bind unless `updatePeriodMs` exceeds 625 |
| `hardSync` | `false` | Write the measured axes outright, ignoring `syncProp` and `maxSyncRate` |
| `syncProp` | `0.15` | Fraction of the RCL/odometry error applied per update |
| `maxSyncRate` | `8.0` | in/s ceiling on ordinary correction. The displacement path is exempt |
| `accumulationAlpha` | `0.3` | EMA factor |
| `updatePeriodMs` | `10.0` | |
| `angleTolerance` | `10.0` | Degrees; the ray must be within this of a wall axis. The four shipped mounts are 90° apart, so `fmod(heading, pi/2)` is the same for all of them and they pass or fail together: RCL contributes nothing outside a 20°-wide window per 90°, i.e. 22% of headings |
| `maxWallInnovation` | `8.0` | Inches, and it does two jobs: within `±` this a reading is *accepted* as the wall, beyond `+` this it is *proof* of displacement. As an acceptance window it is ~6 sigma of the modelled range noise at 40 in, so an object 7 in in front of the wall is taken for the wall |
| `recoverySyncProp` | `0.9` | Sync fraction once readings prove the pose wrong. With the rate limit also lifted this is 99% of the correction in two updates |
| `displaceCycles` | `3` | Consecutive cycles of evidence before believing the robot moved: 30 ms at the default period, against the EKF's 100 ms plus a separate evidence test |
| `noReturnWallLimit` | `35.0` | Inches; a predicted wall closer than this must produce a return, so silence contradicts the pose. The V5 is specified to 2 m, so this uses 44% of the available range |
| `confidenceThreshold` | `30` | Min V5 sensor confidence (0–63). The EKF's `minConfidence` is `10` against the same hardware |
| `obstacleConeFrac` | `0.22` | `tan(12.5°)`, the V5 half-cone |
| `obstacleRangeTol` | `6.0` | Inches; reading vs post surface |
| `autoSync` | `true` | |
| `useAccumulation` | `false` | Average multiple readings for stability |
| `minReadingMm` | `1.0` | Below this the read faulted; it is not a short measurement |
| `maxReadingMm` | `2000.0` | Past the V5's specified range a reading is treated as silence |
| `confidenceRangeMm` | `200.0` | Range above which `confidenceThreshold` applies at all — the V5 switches ranging mode here |
| `displaceEvidence` | `2` | Weighted points in one cycle before `displaceCycles` starts counting |
| `displaceWeightTooFar` | `2` | A reading through a wall is conclusive on its own |
| `displaceWeightSilent` | `1` | Silence where a wall should be is weaker, so two are needed |

Every one of these is a gate on whether a reading is believed, and the EKF's
equivalents are all exposed and documented; these were not, which is why
`maxSyncDist` sat inert at 62x the value that actually applies. RCL is off in
`config.cpp` (`useRclTracking = false`), so none of it is live — read the table
as defaults to be re-judged before switching it on, not as tuned values.

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
| `resampleThreshold` | `512` | N_eff floor, but it only gates the case where NO beam scored: a tick that took a measurement resamples regardless |
| `maxSensorRange` | `100.0` | inches. `config.cpp` ships `140`. |
| `distResampleVariance` | `0.15` | inches of steady-state roughening per resample. `config.cpp` ships `1.5`, where the trade was settled over 648 paired kidnaps: `0.15` tracks 2.4x better (0.65 in mean / 1.7 in worst vs 1.56 / 7.2) and recovers worse on the fused pose (528/648 vs 550, McNemar p = 0.0013). `1.0` and `0.6` lose the same way at p = 0.0030 and 0.024 |
| `thetaResampleVariance` | `0.001` | radians. `config.cpp` overrides this to `0.01` |
| `distSearchVariance` | `12.0` | inches; extra spread applied only while the ranges stop fitting. Over 648 paired kidnaps (8 starts x 16 directions x 8 seed bases), `6.0` recovers 628 raw / 515 fused and `12.0` recovers 637 / 550 — McNemar one-sided p = 0.039 and p < 1e-4, for 0.005 in of mean fused error and a smaller raw worst case (7.2 in vs 7.7) |
| `thetaSearchVariance` | `0.01` | radians, same trigger |
| `searchMismatchFloor` | `0.5` | `fitSigmas()/lostFitSigmas` at which the search spread starts coming on |
| `searchMismatchFull` | `0.9` | and where it is fully on. With `lostFitSigmas` 8 the band is 4 to 7.2 sigmas of range residual |
| `lostFitSigmas` | `8.0` | range residual, in sigmas, that counts as "lost". Scales the two thresholds above. Swept 4 to 24 over 648 paired kidnaps: `8` recovers 550/648 fused against `12`'s 534 (McNemar p = 0.0035); raw MCL does not move (p = 0.31). `4` adds 3 more, which is noise, for half the margin to a real calibration residual |
| `lostFitCapProp` | `0.8` | scales the per-beam residual cap, which is `lostFitCapProp * lostFitSigmas * searchMismatchFloor / alphaFast`. Measured safe over 0.4 to 1.4; at 0 and at 3.0 one outlier tick fires the ramp |
| `acquireVariance` | `6.0` | inches of spread `setPose()` seeds. Was `distSearchVariance`; split off because raising that to 12 for recovery also widened the cloud at the start of every run, which cost 0.79 in -> 1.76 in of trajectory landing error |
| `acquireThetaVariance` | `0.01` | radians, same role |
| `maxRandomFrac` | `0.0` | augmented-MCL injection ceiling; off, see below |
| `deadzoneLikelihood` | `0.06` | per inch, weighed against a normalised beam density |
| `useNoReturnBeams` | `false` | score a 9999 mm no-return instead of discarding it |
| `maxThetaDeviation` | `0.08` | radians, clamp vs IMU |
| `trackingWheelVariance` | `0.08` | dead wheels |
| `imeVariance` | `0.12` | motor encoders — more slip. `config.cpp` ships `0.004`. |
| `imuVariance` | `0.005` | |
| `rangeSigmaCloseIn` | `0.787` | beam sigma at or below 200 mm. `config.cpp` ships `0.068`. |
| `rangeSigmaProp` | `0.05` | fraction of the reading beyond that |
| `horizDependentVarianceProp` | `0.03` | × \|dVert\|, lateral slip. `config.cpp` ships `0.004`. |
| `horizConstantNoise` | `0.03` | inches/update baseline |
| `distSyncProp` / `thetaSyncProp` | `0.06` / `0.001` | Lerp toward chassis; inert when the EKF is on |
| `updatePeriodMs` | `25.0` | |
| `confidenceThreshold` | `10` | |
| `minSensorRange` | `1.0` | mm; raise for recessed sensors |
| `autoSync` | `true` | |

Field geometry lives in `static constexpr` arrays in
`chad/tracking/mcl_core.hpp`. `mclfield::kSurfaces` holds the four perimeter
walls and never changes; `kCircularSurfaces` and `kBlockers` are empty, to be
filled in per game.

Beyond the fields above the tracker exposes `estimate()`,
`estimateGeneration()`, `snapshotParticles()`, `setConfig()`, `setSeed()`,
`setAutoSync()`, `randomInjectFraction()`, `likelihoodMismatch()`,
`deadzoneFraction()`, `fitSigmas()`, `searchMismatch()` and `resampleCount()`.
`fitSigmas()` is the median per-beam range residual of the published estimate in
sigmas (median, not mean, so one beam on something unmapped cannot declare a
healthy filter lost) and
is what drives the search ramp -- it is an absolute measure of whether the
estimate explains the beams, where `likelihoodMismatch()` is the relative
fast/slow gap on the cloud's mean likelihood and rises merely because the cloud
spread. Only the former separates a displaced filter from a healthy one:
measured 0.25 tracking, 0.26 at 2.5 in out, 6.47 on a 24 in kidnap.
`searchMismatch()` is `fitSigmas()` mapped onto 0..1 through `lostFitSigmas`.
`estimateGeneration()` is how a caller detects a stale
publisher: `estimate()` is lock-free against the worker and, once
`stopTracking()` has run, returns the last published tick rather than blocking.

Augmented-MCL random injection (`maxRandomFrac`) ships **off**. It is
implemented and tested, but an unmapped season post deadzones an injected
particle's beam instead of scoring it, so enabling it needs the real post
geometry in front of a robot. Recovery does not depend on it: the
mismatch-driven search spread recovers a 20 inch displacement with injection
disabled.

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
| `vertNoiseProp` | `0.045` | sigma per inch of along-track travel. `config.cpp` ships `0.004`. |
| `horizNoiseProp` | `0.045` | per inch of cross-track |
| `slipNoiseProp` | `0.035` | lateral sigma per inch driven. `config.cpp` ships `0.004`. |
| `slipNoiseFloor` | `0.010` | lateral sigma per update |
| `vertBiasProp` | `0.010` | systematic along-track scale error |
| `slipBiasProp` | `0.010` | systematic cross-track error |
| `biasDecorrIn` | `40.0` | inches over which that bias renews |
| `thetaNoiseProp` | `0.020` | sigma per radian turned |
| `thetaNoisePerInch` | `0.0015` | heading sigma per inch driven |
| `posNoiseFloor` | `0.015` | inches per update |
| `thetaNoiseFloor` | `0.0004` | radians per update |
| `useRangeUpdates` | `true` | |
| `rangeSigmaBase` | `0.60` | inches, fixed part. `config.cpp` ships `0.068`. |
| `rangeSigmaProp` | `0.030` | fraction of the reading |
| `rangeGateChi2` | `9.0` | 1 dof, 3 sigma |
| `maxIncidenceRad` | `1.15` | past 66 deg off normal a ToF return is junk |
| `maxRangeIn` | `100.0` | |
| `obstacleConeFrac` | `0.22` | tan(12.5 deg), the V5 half-cone |
| `obstacleRangeTol` | `6.0` | inches, **range match only** |
| `obstacleSigmaPad` | `2.0` | widens the range match, and the cone, by this many position sigmas; capped by `obstacleTolMaxIn` |
| `obstacleTolMaxIn` | `18.0` | ceiling on the padded tolerance |
| `useImuHeading` | `true` | |
| `imuSigmaRad` | `0.006` | |
| `useMclUpdates` | `true` | |
| `mclSigmaScale` | `1.6` | the cloud understates its own error |
| `mclSigmaFloor` | `1.2` | inches |
| `mclGateChi2` | `11.8` | 2 dof, 3 sigma |
| `mclResetCycles` | `10` | consecutive gate failures before relocating |
| `mclRelocEvidenceRatio` | `12.0` | MCL must fit the beams this many times better than the filter's own state before a relocation is allowed. Swept against BOTH rates over 864 geometries per arm: at the modelled range noise, 4 relocates falsely on 0.8% and 12 on 0.2%, for 7.4% vs 7.6% missed against a 6.1% geometric floor. At 5 mm range noise every value from 1 to 40 is identical |
| `autoSync` | `true` | write the fused pose into odometry |
| `syncProp` | `0.12` | fraction of the correction applied per update |
| `thetaSyncProp` | `0.20` | **inert as shipped** — see below |
| `updatePeriodMs` | `10.0` | |

A post inside a beam's cone is not on its own enough to discard the reading —
the reading has to match the post surface too. Dropping it on the cone alone
blinded a ~19.6 inch strip of wall per post at 40 inches, whatever the
tolerance said.

`thetaSyncProp` is **inert in the shipped configuration**:
`OdomCore::Config::headingFromImu` defaults true and overwrites the blended
heading within one tick. Set it false through `OdomTracking::setCoreConfig` to
make the blend live. `EkfTracking::getEstimate()` carries the fused heading
either way.

Reject reasons are counted separately, because a filter that silently discards
every measurement looks identical from the outside to a healthy one:
`rangeRejects()` / `rangeRejectStreak()` (gated), `rangeNoReturns()`,
`rangeDeadzoned()`, `rangeGrazed()`, `rangeBlindStreak()` and
`covarianceRepairs()`. `rangeBlindStreak()` is the one that catches a permanent
deadzone. Confidence outside `[0, EkfCore::CONFIDENCE_MAX]` (63) refuses the
beam rather than fusing it, and confidence can only ever inflate the
bench-measured `rangeSigmaBase`, never shrink it.

`syncProp` below 1 is what keeps the pose continuous. A hard write steps the
pose every cycle and every motion controller downstream differentiates it; the
filter's own state is exact either way. A relocation bypasses it and lands in
one write. Measure the noise it is smoothing with
`Characterizer::localizationNoise()`.

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
| `kiRatioStep` | `0.5` | kI searched as a multiple of kP, so per second |
| `maxIterations` | `6` | Per stage |
| `weights` | see below | Cost function |
| `onProgress` | empty | `void(const char* stage, PIDGains, float cost)` |

Each trial forces `targetTolerance = 0` so the motion cannot settle early. Without
that the motion exits the instant it reaches tolerance and the overshoot is never
observed.

Budget roughly 3.5 minutes per axis at defaults.

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

## Relay auto-tuner

`chad/util/relay_tune.hpp`. Switches a fixed output across the setpoint and
reads a point on the frequency response off the resulting limit cycle. Seconds
per axis at one output, where `PidAutoTuner` takes minutes and climbs `kP` to
find the limit; `PidAutoTuner` remains the option for an axis that will not
oscillate. **The robot moves on its own — give it clear space.**

```cpp
struct RelayTuneConfig {
    float amplitudeAngular{40.0f};   // relay output, motor units
    float amplitudeLateral{80.0f};
    float hysteresisDeg{0.5f};       // dead band, sets the excited frequency
    float hysteresisIn{0.5f};
    int   cycles{4};                 // full cycles averaged, after the transient
    float minHalfPeriodMs{60.0f};
    float settleMs{300.0f};
    float timeoutMs{8000.0f};
    float sampleMs{10.0f};
    float abortDeg{90.0f};           // give up if it runs away
    float abortIn{12.0f};
    float kpFactorAngular{0.45f};    // Tyreus-Luyben
    float kpFactorLateral{0.45f};
    float tdDivisor{6.3f};
    float tiFactor{2.2f};
    bool  includeIntegral{false};
    float minSwingRatio{3.0f};       // phase budget, see below
    float dwellSlack{1.2f};
    std::function<void(const char* stage, const char* detail)> onProgress;
};

struct RelayFit {
    float ku;          // gain at the excited frequency, output per error unit
    float tu;          // period of the limit cycle, seconds
    float amplitude;   // measured peak of the limit cycle
    float kP, kI, kD;
    int   cycles;
    bool  valid;
};

tuning::RelayAutoTuner tuner(chassis, drivetrain);   // RelayTuneConfig optional
const tuning::RelayFit heading = tuner.tuneAngular();
const tuning::RelayFit drive = tuner.tuneLateral();
```

`ku` and `tu` are named for the ultimate gain and period but are not measured
there. A lag feeding an integrator never reaches −180° of phase, so neither axis
has an ultimate frequency of its own, and the dead band is what selects the
frequency that gets excited. The identified point sits `asin(eps/a1)` away from
−180°, where `eps` is the dead band and `a1` the first-harmonic amplitude — 16°
at the shipped 0.5° dead band and 40-unit output, rising with a wider band.

`minSwingRatio` is therefore a phase budget rather than a noise margin: the peak
swing in dead bands is about `1.28/sin(phase error)`, so the default 3.0 caps the
error near 25°. A fit outside the budget reports `Chatter` and asks for more
relay output, which is the fix — a wider dead band needs a bigger swing to stay
at the same phase. Raising the band without raising the output walks the
identified point away from where Tyreus-Luyben describes anything.

`amplitude` is the measured peak, while the describing function behind
`ku = 4d/(pi a1)` is written on the first harmonic, which is smaller — 1.28× on
an integrator-plus-lag plant, close to the `pi^2/8` of a triangle wave. So `ku`
and `kP` come out about 22% low. That is left as headroom rather than corrected,
since the peak is what a 100 Hz loop can measure robustly and `PidAutoTuner`
refines from here.

The identification maths is PROS-free in `chad/util/relay_core.hpp`
(`RelayIdentifier`), so it is host-tested against the limit cycle the describing
function predicts rather than only on a robot.

---

## Characterisation

`chad/util/characterize.hpp` — measures constants of this robot from quantities
it can observe itself. Run before `PidAutoTuner`; a feedforward that matches the
drivetrain leaves the PID less to do.

```cpp
struct CharacterizeConfig {
    float rampVoltsPerSec{1.0f};      // first slope
    float rampVoltsPerSecFast{6.0f};  // second slope, above rampSwitchVolts
    float rampSwitchVolts{4.5f};
    float maxVolts{9.0f};
    float sampleMs{10.0f};
    int   accelSpanTicks{40};      // ticks the accel difference spans
    float minSpeedInPerSec{3.0f};  // below this the fit reads stiction as slope
    float kaMinT{25.0f};           // standard errors of kA needed to trust kA
    float minR2{0.9f};
    float maxTravelIn{30.0f};
    int   spinRotations{6};
    float parkSeconds{6.0f};
    std::function<void(const char* stage, const char* detail)> onProgress;
};

struct FeedforwardFit {
    float kV, kS, kA;   // volts per m/s, volts, volts per m/s^2
    float r2;           // under minR2 is unusable
    float kaT;          // |kA| in standard errors
    int   samples;
    bool  valid;        // kS and kV are usable
    bool  kaValid;      // and so is kA
};

struct NoiseFit {
    float sigmaIn, sigmaThetaDeg, peakIn;
    int   samples;
    float suggestTolerance;    // targetTolerance
    float suggestBearingHold;  // MoveToPointParams::bearingHoldDist
};

struct OdomDriftFit {
    float alongPerIn, lateralPerIn;   // residual error per inch driven
    float thetaPerRad;
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

The robot drives itself. `feedforward(false)` shuttles back and forth over
`maxTravelIn` (30 in by default) rather than driving one way: reaching
`maxVolts` at `rampVoltsPerSec` takes about 36 ft of travel, and a field is 12.

The ramp uses two slopes because one cannot identify `kA`. Under a single slope
the acceleration settles to a constant, so `kA` is barely excited: measured
against a synthetic plant with 0.6 in/s velocity quantisation, a one-slope ramp
fits `kA` to within 100% of zero at every window length from 5 to 64 ticks,
while switching slope part way recovers it to 6%. `r2` cannot detect this --
it reads 0.9999 either way, because `kS` and `kV` carry the ramp voltage
between them -- so `kaValid` gates on `kaT`, the size of `kA` in standard
errors, and `feedforward` reports `kA UNFIT` rather than a confident zero.
`measureScale()` and `measureOdomDrift()` need a clear lane of 24 and 36 in
respectively. The rest spin on the spot.

```cpp
chad::tuning::CharacterizeConfig cfg;
cfg.onProgress = [](const char* stage, const char* detail) {
    printf("[%s] %s\n", stage, detail);
};
chad::tuning::Characterizer ch(chassis, drivetrain, cfg);

float width = ch.trackWidth();          // spins ~6 turns
auto  ff    = ch.feedforward(false);    // straight-line kV/kS/kA
auto  turn  = ch.feedforward(true);     // turning friction
auto  noise = ch.localizationNoise();   // parked, with your race config

drivetrain.trackWidth = width;

chad::RAMSETEParams p;
p.useVelocityController = true;
p.ffGains.kV            = ff.kV;
p.ffGains.ksStraight    = ff.kS;
p.ffGains.kaStraight    = ff.kA;
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

```sh
export EMSDK="$HOME/emsdk"; source "$EMSDK/emsdk_env.sh"
./sim/build.sh
python -m http.server -d sim/web 8000    # localhost:8000
```

```sh
bash sim/test/run.sh          # smoke: tracking, sensors, determinism
node sim/test/tune.mjs        # sweeps below
node sim/test/mpc.mjs         # experimental branch only
```

Gains that need a known true pose cannot be scored on hardware, so they are
swept here.

```sh
node sim/test/tune.mjs             # all sections
node sim/test/tune.mjs ramsete     # b / zeta
node sim/test/tune.mjs mcl         # filter noise vs kidnap recovery
node sim/test/tune.mjs tolerance   # settle tolerance vs localization error
```

### Searching, not sweeping

`tune.mjs` prints tables for you to read. `autotune.mjs` searches.

```sh
node sim/test/autotune.mjs                        # 2 passes, 3 seeds
node sim/test/autotune.mjs --passes 3 --seeds 4
```

Coordinate descent over ten MCL/EKF parameters, minimising a weighted sum of
estimate error and jerk. Kidnap recovery is a hard constraint rather than a cost
term: every parameter that tightens the estimate also narrows the cloud's
search, so a plain weighted sum walks to a configuration that tracks well and
can never relocate. Candidates that recover less often than the baseline are
rejected outright.

It searches on one set of seeds and revalidates the winner on a disjoint set,
then prints `[HOLDS]` or `[DOES NOT HOLD]`. A result that only appears on the
seeds it was chosen against is the search fitting noise, and the paste-ready
config is withheld in that case.

Each row prints cross-track, peak, jerk, time, endpoint error and kidnap
recovery. Read all of them; two knobs improve the tracking columns while
breaking something else, and the tool flags where.

First output is a control: the same config twice, which must match.
`sim_reset` does not restore filter or motion config, so every trial rewrites
every key. If the control fails, ignore everything below it.

`sim/web/chad_sim.mjs` and `.wasm` are untracked build output and survive a
branch switch. Rebuild after every checkout or you are running the previous
branch's binary.

No traction, torque, current limit or voltage sag. Exact differential-drive
kinematics, so the robot cannot slide and `maxLateralAccel` is a profile-speed
knob, not a grip limit. Odometry error comes from sensor staleness, not slip.
`encoderSlip`, `imuDriftDegPerSec` and `imuNoiseDeg` default to zero — turn them
on or dead reckoning is ground truth and localization comparisons mean nothing.

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

## Geometry

`chad/util/geometry.hpp`. The SE(2) layer everything else is built on. Position
in inches, headings in standard math radians.

```cpp
struct Rotation2d {                  // stores cos and sin, not an angle
    static Rotation2d fromRadians(float), fromDegrees(float), fromVexDegrees(float);
    static Rotation2d fromVector(float x, float y);
    float radians() const, degrees() const, vexDegrees() const;
    Rotation2d operator+(const Rotation2d&) const;   // compose
    Rotation2d operator-() const;                    // invert
    float distanceTo(const Rotation2d&) const;       // signed, shortest
    Rotation2d interpolate(const Rotation2d&, float t) const;
};

struct Translation2d { float x, y; /* norm, angle, rotateBy, dot, cross, ... */ };
struct Twist2d { float dx, dy, dtheta; };   // robot-frame motion along one arc
struct Transform2d { Translation2d translation; Rotation2d rotation; /* inverse */ };
```

Composing `Rotation2d` never needs wrapping, and a non-finite angle yields
identity rather than NaN — a dead PROS sensor reports `PROS_ERR_F`.

`Pose` gains the exponential map:

```cpp
Pose  exp(const Twist2d&) const;          // integrate a twist along its arc
Twist2d log(const Pose& end) const;       // the twist that gets you there
Pose  relativeTo(const Pose& origin) const;
Transform2d transformTo(const Pose& end) const;
Pose  transformBy(const Transform2d&) const;
Pose  interpolate(const Pose& end, float t) const;   // stays on the arc
```

`exp` is what odometry, the EKF, the dry run and trajectory interpolation all
integrate with. Straight-line integration of the same twist lands short on the
inside of every turn, and that error accumulates one way around a lap.

---

## Kinematics

`chad/util/kinematics.hpp`.

```cpp
enum class WheelType { Omni, Traction, Mixed };
float defaultGripLimit(WheelType);        // inches/s^2

struct ChassisSpeeds {
    float vx, vy, omega;                  // robot frame; vy is slip, not an input
    static ChassisSpeeds fromFieldRelative(float, float, float, const Rotation2d&);
    Translation2d toFieldRelative(const Rotation2d&) const;
    ChassisSpeeds discretize(float dt) const;
};

struct WheelSpeeds { float left, right; float desaturate(float maxSpeed); };

class DifferentialKinematics {
    WheelSpeeds toWheelSpeeds(const ChassisSpeeds&) const;
    ChassisSpeeds toChassisSpeeds(const WheelSpeeds&) const;
    Twist2d toTwist(float leftDelta, float rightDelta) const;
};

struct DriveModel {
    float trackWidth, wheelDiameter, maxWheelSpeed;
    WheelType wheels;  float gripLimitOverride;
    float gripLimit() const;
    float maxSpeedForCurvature(float k) const;
    float maxCurvatureAtSpeed(float v) const;
    bool  isFeasible(const ChassisSpeeds&) const;
    float omegaBudgetAt(float v) const;
};
```

`desaturate` scales **both** sides by one factor. Clipping them independently
changes the ratio between them, and that ratio is the commanded curvature.

`discretize` corrects for holding a velocity across a whole tick: commanding
`vx` and `omega` together traces an arc, not the straight line a naive step
assumes.

`maxSpeedForCurvature` returns the smaller of the grip limit `sqrt(a/|k|)` and
the wheel-speed budget `vmax/(1 + |k|b/2)`. Which one binds depends on how tight
the corner is. `Drivetrain::model()` fills this in from the robot config, so the
wheel type is declared once and every curvature limit follows from it.

---

## Trajectories

`chad/movement_control/{spline,trajectory,trajectory_constraint,trajectory_generator}.hpp`.
A trajectory is a time-parameterized reference: it already knows its speed,
acceleration and curvature at every instant, and it was shaped by the
constraints the drivetrain actually has.

```cpp
struct TrajectoryState { float t, velocity, acceleration; Pose pose; float curvature; };

class Trajectory {
    TrajectoryState sample(float t) const;      // O(log n), no allocation
    float totalTime() const;
    Trajectory transformBy(const Transform2d&) const;
    Trajectory relativeTo(const Pose&) const;
    Trajectory mirrored(MirrorAxis = MirrorAxis::X) const;
    Trajectory rotatedHalfTurn() const;         // the alliance swap on a VEX field
    Trajectory concatenate(const Trajectory&) const;
};

TrajectoryResult generateTrajectory(const std::vector<Pose>& waypoints,
                                    const TrajectoryConfig&);
TrajectoryResult generateCubicTrajectory(const Pose& start,
                                         const std::vector<Translation2d>& interior,
                                         const Pose& end, const TrajectoryConfig&);
TrajectoryResult timeParameterize(const std::vector<PoseWithCurvature>&,
                                  const TrajectoryConfig&);
```

Failure is a return value — `TrajectoryResult::error` — never a truncated path,
which would look to a follower like a valid short one.

Constraints are non-owning pointers in the config, so they must outlive the
generate call. `DrivetrainConstraints` bundles the two every path needs:

```cpp
DrivetrainConstraints limits(drivetrain.model());
TrajectoryConfig config = trajectoryLimits(drivetrain.model(), 1.0f, 60.0f);
config.constraints = limits.list();

auto result = generateTrajectory({{0, 0, 0}, {24, 24, HALF_PI}}, config);
if (result.ok()) chassis.followTrajectory(result.trajectory, 5000);
```

`mirrored()` reflects, which reverses handedness and flips curvature. A VEX
field is normally 180-degree rotationally symmetric, so `rotatedHalfTurn()` is
almost always the one you want for the other alliance.

---

## Constrained MPC

`CondensedMpc` solves a condensed QP over the pose-error dynamics. Box limits on
the input take a fast projected-gradient path; asking for anything more general
switches it to an ADMM solver (`movement_control/qp.hpp`) without the caller
doing anything differently.

```cpp
CondensedMpc mpc;
mpc.setup(DrivetrainPlant::discretize(vRef, dt), weights, horizon);
const MpcResult r = mpc.solve(x0, uMin, uMax, maxIters, tol);
float u[2];
mpc.firstMove(u);
```

Use `setupVarying(stages, count, ...)` when the reference speed changes over the
horizon — `setup` assumes it does not, which cannot see a reference braking into
the end of a path.

### Input-rate (jerk) limits

Bounds the change in the *command*, not in the correction, which is why the
reference deltas are needed:

```cpp
const float duMax[2] = {0.4f, 0.8f};   // per tick, per axis
mpc.setInputRateLimit(duMax);
mpc.setPreviousInput(uLastTick);       // anchors the head of the horizon
mpc.setReferenceDeltas(refDeltas, n);  // ref_k - ref_{k-1}; null bounds the correction instead
```

A non-positive entry leaves that axis unlimited. This is what stops a solution
that is feasible on paper from slipping the wheels or browning out the brain.

### State constraints

```cpp
const float a[3] = {0.0f, 1.0f, 0.0f};       // cross-track error
mpc.addStateConstraint(a, -3.0f, 3.0f);      // stay inside a 3 inch corridor
mpc.clearConstraints();                      // back to the box fast path
```

Stages are 1-based and inclusive; `lastStage <= 0` means the end of the horizon.
Returns false when the constraint budget is spent — **check it**, because a
constraint you believe is active but silently is not is worse than a rejection.
The row budget funds every spec the spec count allows: `MPC_MAX_QP_ROWS` covers
`MPC_MAX_STATE_CONSTRAINTS` full-horizon specs on top of the box and a two-axis
rate limit, exactly. `plannedRows()` reports what the current spec set will
generate, and `MpcResult::droppedRows` is non-zero only if a row could not be
represented at all — which forces `converged = false`.

### Solver state and diagnostics

`shift()` advances the warm start one step for the next tick (the
real-time-iteration pattern); `zero()` clears the iterate but keeps the
constraint spec. `MpcResult` reports `kktResidual`, the ADMM primal/dual
residuals, `restarts`, and three failure flags worth logging rather than
ignoring: `factorizationFailed`, `stateNotFinite` and `solutionNotFinite`. A
capped solve is not a converged one — `certified()` is the question to ask.

`primalResidual` is measured on the general path whether or not
`certifyOptimality` is set. It costs one pass over the row support, about 23% of
an ADMM iteration, and it is worth that because a state corridor cannot be
clamped on the way out — it is the only thing that reports how far outside the
solution is, in the row's own units. `dualResidual` needs an H matvec and stays
gated, so `converged` and `kktResidual` still require certification: test those
for "nothing was certified", not `primalResidual`. `zero()` clears the primal
iterate *and* the ADMM slack and multipliers, but not the rate anchor, which is
caller data rather than solver state.

`shiftWarmStart` is right only when the terminal cost is the infinite-horizon
cost-to-go. `MpcPathFollower` passes the DARE solution, so leave it on there; a
bare `CondensedMpc` built without a terminal `P` should turn it off — measured
3x worse committed-move error at a one-iteration budget with it on and no `P`.
On the box path the two metrics disagree: iterations-to-tolerance favours not
shifting even with `P`, while tracking error at a one-iteration budget favours
shifting, and real-time iteration cares about the second.

`AdmmConfig admm` exposes `rho`, `rhoScale`, `sigma`, `alpha` (OSQP's 1.6) and
`checkEvery`. `rho` is deliberately fixed for the life of a factorization:
adapting it mid-iteration would force a refactorization, which a 100 Hz control
loop cannot afford.

## Filters

`chad/util/filters.hpp`. All fixed-size, no allocation, `dt` passed in.

```cpp
class SlewRateLimiter { SlewRateLimiter(float rise, float fall, float initial = 0);
                        float calculate(float input, float dt); };
template <size_t N> class MovingAverage { float calculate(float); };
template <size_t N> class MedianFilter  { float calculate(float); };
class LowPassFilter  { explicit LowPassFilter(float timeConstant);
                       float calculate(float, float dt); };
class HighPassFilter { /* input minus its low-passed self */ };
class Derivative     { float calculate(float, float dt); };
class Debouncer      { enum class Edge { Rising, Falling, Both };
                       bool calculate(bool, float dt); };
```

`LowPassFilter` recomputes its coefficient from the measured `dt` every sample.
A fixed coefficient silently moves the cutoff whenever a PROS task period slips.

---

## Settle conditions

`chad/util/settle.hpp`. The single authority on when a motion is finished, with
time passed in rather than read, so it is testable off-robot.

```cpp
struct SettleSpec {
    float errorTolerance, speedTolerance;
    uint32_t holdMs, timeoutMs;
    float earlyExitRange;             // exit here to chain into the next motion
    float stallSpeed;  uint32_t stallHoldMs;
    uint32_t graceMs;                 // suppress stall detection after the start
};

enum class SettleState { Running, Settled, EarlyExit, TimedOut, Stalled };

class SettleMonitor {
    void reset(uint32_t nowMs);
    SettleState update(uint32_t nowMs, float error, float speed);
    bool done() const;  bool reachedTarget() const;
};
```

A non-finite error never counts as an arrival, and the millisecond arithmetic is
unsigned throughout, so a wrapping clock is ordered correctly. `ExitCondition`
is a thin wrapper over this against the PROS clock.

---

## Latency-compensated pose fusion

`chad/tracking/pose_estimator.hpp`.

```cpp
template <int Capacity> class PoseBuffer {
    void add(uint32_t timeMs, const Pose&);
    bool sample(uint32_t timeMs, Pose& out) const;    // arc-interpolated
    void prune(uint32_t windowMs);
    bool covers(uint32_t timeMs) const;
};

class LatencyPoseEstimator {
    void reset(const Pose&, uint32_t nowMs);
    void update(uint32_t nowMs, const Twist2d& odomTwist);
    bool addMeasurement(uint32_t stampMs, const Pose&, const float stdDev[3]);
    Pose pose() const;  Pose odometryPose() const;
};
```

A wall-range or vision fix says where the robot **was**. Applying it to the
present pose drags in every inch driven since, which on a moving robot is the
dominant error. `addMeasurement` rewinds to the stamped instant, corrects there
with a gain of `q / (q + sqrt(q r))` per axis, then replays the odometry that
followed. A stamp older than the retained history is rejected rather than
guessed at.

---

## Riccati and state-space solvers

`chad/movement_control/riccati.hpp`. Fixed-size, templated on dimension, no heap.

```cpp
namespace linalg {   // multiply, multiplyTransposedA/B, inverse (pivoted),
                     // identity, symmetrise, maxAbs, spectralRadius
}

template <int N, int M> struct DareSolution { float P[N][N], K[M][N];
                                             int iterations; bool converged;
                                             float residual; };

template <int N, int M> DareSolution<N, M> solveDare(const float A[N][N],
    const float B[N][M], const float Q[N][N], const float R[M][M],
    int maxIterations = 40, float tol = 1e-7f);

template <int N, int M> float dareResidual(...);
template <int N, int M> void  dareGain(...);
template <int N, int M> void  closedLoop(...);
```

Structure-preserving doubling, so each step squares the error: plants that value
iteration needs hundreds of sweeps for converge in under ten. `residual` is the
residual of the equation being solved, not the change between iterates — an
iteration that has stopped moving has not necessarily solved anything. A state
the input cannot reach (cross-track at zero reference speed) reports
`converged == false` and a finite gain instead of an infinity.

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

---

## Numbers that bite

`targetTolerance` must exceed localization error, or settle never fires and the
motion times out. Under MCL: 1 in against 1.5 in error failed 3/3, 3 in passed.
Pick it with `localizationNoise()`.

`bearingHoldDist` stays small. `atan2` to a target at shrinking range is mostly
noise, which a tuned angular loop chases at 591 deg/s. Holding the last good
bearing fixes that, but too large a radius holds a stale one: 3 failures in 15
at 1.5 in, 12 in 15 at 6 in.

Roughening and search width are two jobs, and one number used to do both.
Measured **on the robot**, back when it did: lowering `distResampleVariance`
improved every tracking number and removed kidnap recovery entirely — 3
recoveries in 4 at 1.5, 0 in 4 at 1.0 and below. That trade is what
`distSearchVariance` now exists to break, and the hardware measurement above
should be read as applying to *it*, not to `distResampleVariance`, which is now
steady-state roughening only at 0.15.

The decoupling is worth 3x on tracking error in host tests (0.396 in to 0.136
in at the same acquisition envelope) and recovers a 20 inch jump with injection
off. **None of that was measured on a robot.** Re-run the kidnap trial before
trusting it in a match, and if recovery has regressed, raise
`distSearchVariance` rather than `distResampleVariance`.

`ffGains.kP` corrects actuator lag, so it depends on your motors. A lag sweep
put 12 as the only value stable from 20 to 200 ms. Below 4 the path times out.

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
