# riku-sim

A browser simulator for testing the `riku` library off-robot.

It compiles **your code** to WebAssembly and runs it against fake
hardware. There is no second implementation of your algorithms, so a gain you
tune here is a gain you can paste into the robot, and a bug you reproduce here
is a bug in the code.

```
./sim/build.sh            # build to sim/web/
python sim/serve.py       # http://localhost:8080
./sim/test/run.sh         # headless regression suite
```

### Toolchain

Needs the Emscripten SDK — it is already installed at `~/emsdk` on this
machine, and `build.sh` finds it there with no environment setup. On a fresh
clone:

```
git clone --depth 1 https://github.com/emscripten-core/emsdk.git ~/emsdk
cd ~/emsdk && python emsdk.py install latest && python emsdk.py activate latest
```

Set `EMSDK=/path/to/emsdk` if you keep it somewhere else. Nothing else is
required: no CMake, no MSVC, and the project's ARM toolchain is untouched.

Note that the project builds with `gnu++26`, which the MinGW g++ 6.3.0 on this
machine cannot handle — Emscripten's clang is what makes an off-robot build
possible at all.

---

## What is real and what is faked

**Real**  compiled from `src/`:

| | |
|---|---|
| Localization | `OdomTracking`, `MclTracking`, `RclTracking` |
| Control | `PID`, `ExitCondition`, `FeedforwardController` |
| Motion | `moveToPoint`, `turnToPoint`, `moveDistance`, arc, boomerang, RAMSETE, LTV, swing |
| Support | velocity profiles, autotune, jam detection, sensor calibrator |
| Config | your actual `config.cpp` — same ports, geometry, sensor mounts, MCL/RCL tuning |
| Routines | your actual `autons.cpp` |

**Faked** `sim/platform/`, `sim/world/`:

- **PROS devices.** `Distance`, `Imu`, `Rotation`, `adi::Encoder`, `Motor`,
  `MotorGroup`, `Controller`, `Task`, `Mutex`. Each one reads or writes a slot
  in `DeviceBus`; the world owns the other side.
- **Motion.** Deliberately *kinematic*, not dynamic. Commanded voltage maps to
  a target wheel speed through a first-order lag, and differential-drive
  kinematics are integrated from there. No mass, torque or friction. The lag
  exists only so PID and exit conditions have something to settle against.

Everything else aims at the real hardware, because that is what decides whether
an algorithm behaves the way it will on a field:

- **Devices refresh on their own clocks** and hold a stale value in between:
  distance ~30 ms, IMU and motor encoders ~10 ms. Polling faster returns the
  previous sample, exactly as on a brain, so estimator lag is real lag.
- **Distance sensors integrate a ~25 degree cone**, not a pencil ray. The
  reported range is the *nearest* thing anywhere in the cone, so a 4 in goal
  post is seen well off-axis, and a reading can come from something the centre
  ray misses entirely.
- **Confidence falls with range and incidence**, and the return is lost past a
  grazing threshold (default 75 degrees from the surface normal). Running
  near-parallel to a wall and dropping sensors is the dominant real-world
  failure mode for wall-referenced localization.
- **Encoders and the IMU are quantised** to their real resolutions (0.1 motor
  degrees, 0.01 degrees).
- **Field geometry occludes.** `sim/world/field.cpp` raycasts against the
  perimeter and every goal post and returns a surface normal, so a sensor aimed
  at a post reports the post rather than the wall behind it.

Sensor mount offsets come straight from `riku::sensorMounts` in
`gamespecific/config.cpp`, using the same robot-frame convention the library
hands to MCL (x forward, y left, theta pointing angle). The sim and the filter
cannot disagree about where a sensor is.

## Architecture

```
browser (sim/web/)
    |  cwrap
sim/bridge/bridge.cpp        C API, state marshalling, route VM, recorder
    |
sim/world/                   ground truth + sensor synthesis
sim/platform/                PROS shim + fiber scheduler + virtual clock
    |
src/riku/, src/gamespecific/ THE CODE UNDER TEST — unmodified
```

`-Isim/platform` precedes `-Iinclude`, so `#include "pros/imu.hpp"` resolves to
the shim instead of the real header. The library needs no `#ifdef`s.

### Tasks are real fibers

riku runs odometry, MCL, RCL and the chassis motion queue in `pros::Task`
background loops, and blocks the caller inside `waitUntilSettled()`. Those are
genuinely concurrent stacks — you cannot drive them by calling `update()` in a
loop. `sim/platform/sim_sched.cpp` gives each task an Emscripten Asyncify fiber
with its own stack.

Scheduling is deterministic by construction: exactly one fiber runs at a time,
the next is always the earliest wake time with ties broken by task id, and the
clock only advances when nothing is runnable. `pros::delay(n)` yields; the
scheduler advances virtual time and steps the world. That is what makes
"Step 10 ms" meaningful and a divergence bug reproducible.

### Determinism caveat

The world replays exactly for a given seed. **MCL does not.**
`MclTracking`'s constructor seeds `mGen` from `std::random_device`, so particle
initialisation and resampling jitter differ every run. If you want
bit-reproducible MCL runs, that constructor needs an optional seed parameter.

## Route DSL

The **Route** tab runs a text routine against the real `Chassis` API, on its own
fiber, through the same motion queue and the same `waitUntilSettled()`. Edit and
re-run without recompiling.

```
setPose(x: -48, y: -60, h: 90);
moveTo(x: -24, y: -60, t: 2500);
moveTo(x: -48, y: 24, back: true);
turnTo(heading: 140);          turnTo(x: 10, y: 20);
drive(distance: 12);           drive(distance: -8);
boomerang(x: 10, y: 20, h: 90);
arc(x: 10, y: 20, radius: 24);
swing(heading: 90, side: right);
via(x: -24, y: -24);           followPath(t: 5000);
delay(ms: 300);                waitSettled();
maxSpeed(80);
lateralGains(kp: 5, ki: 0, kd: 5);
angularGains(kp: 50, ki: 0, kd: 6);
```

`via()` points define a Catmull-Rom spline and `followPath()` drives it with
`moveRAMSETE`. The browser samples the spline and uploads the exact points the
controller receives, so the purple curve drawn on the field IS the trajectory --
there is no second implementation to drift. Heading ticks along it make a
reversed or twisted spline obvious before you run it.

`t:` sets a per-motion timeout in ms. Waypoints draw on the field as a numbered
dashed preview and the HUD shows which command is executing. **Shift-click the
field** to append a `moveTo` there. Routes save to `localStorage`.

Opcode numbers are the contract between `sim/web/route_dsl.js` and `RouteOp` in
`sim/bridge/bridge.cpp`. `sim/test/dsl.mjs` reads the C++ enum back out and
fails if the two drift apart.

## Plots

The **Plots** tab graphs a run: odometry error, heading error, velocities and
true/MCL positions. Samples land every 20 ms of virtual time in a flat float
ring the browser reads straight out of the WASM heap. **Export CSV** dumps the
run for offline analysis.

## Error sources you can dial in

With every knob at zero, odometry tracks ground truth to ~0.003 in (pure encoder
quantisation) and nothing interesting happens. The sliders are the point.

| Knob | Models |
|---|---|
| `wheel slip` | fraction of travel the encoders never see, the main odometry drift source |
| `left/right scale` | drive asymmetry, "the robot pulls left" |
| `drive lag ms` | first-order velocity response |
| `IMU drift`, `IMU noise` | gyro bias and jitter |
| `IMU under-report` | cancels `ScaledIMU(15, 360.0, 354.25)` exactly at the default; change either and the mismatch shows up as heading error |
| `dist noise mm` | sensor jitter, scaled up on slanted surfaces |
| `dist range mm` | range ceiling, a real V5 tops out near 2000 mm |
| `cone FOV` | 0 makes it a pencil ray; 25 is realistic |
| `max incidence` | grazing angle past which the return is lost |
| device rate sliders | how stale the estimator's inputs are |

## Controls

- **WASD / arrows** drive the robot
- **Drag the robot** to kidnap it — ground truth teleports, the estimator is
  not told, which is the classic MCL recovery test
- **Shift-click** appends a waypoint to the route
- **Localization** selector enables MCL and/or RCL at runtime, independent of
  `useMclTracking` in your config
- **Run auton** executes a real routine from `autons.cpp` on its own fiber
- The **View** tab has layer toggles, start pose, and a live field-geometry
  editor (JSON in, applied without a rebuild)

---

## Design notes worth knowing

**Obstacles invalidate sensors rather than being ray-cast targets — on purpose.**
`kFieldTargets`, `kFieldCircles` and `kDisablingLines` in
`include/riku/tracking/mcl_tracking.hpp` are empty, and the nine posts
registered by `registerFieldObstacles()` go into `mCircleObstacles`, which
`updateWeights()` uses only to mark a reading invalid.

That is a deliberate choice, not an oversight. A 2 in post grazed at range turns
a small pose error into a large expected-distance error, so discarding those
readings is more robust than trying to match against them. MCL localizes off
walls alone.

The consequence for tuning: **wall visibility is the whole budget.** Anything
that costs you a wall reading — grazing incidence, range ceiling, a post in the
way — directly costs localization quality, and there is no fallback. The Sensor
status panel exists to show exactly that: how many of the four sensors MCL is
actually able to use at any moment.

**`moveRAMSETE` had three separate faults. All are fixed; the suite pins them.**
The symptom that surfaced first: from the path start the robot moved 0.00 in,
while dropped partway along the identical path it drove 56.7 in.

1. *It could not start.* `v_d` is read from the trapezoidal profile by
   **arc-length position**, and a trapezoid is 0 at its start. Every term of the
   law scales with `v_d` — `k = 2*zeta*sqrt(w_d^2 + b*v_d^2)` collapses with it —
   so `v` and `w` both came out 0, progress never advanced and `v_d` stayed 0.
   `RAMSETEParams::minSpeed` (previously declared but never read) now floors
   `v_d`, but only below the profile's peak: past it the decel ramp has to be
   allowed to reach zero or the robot creeps off the end of the path.

2. *It never decelerated.* `generateTrapezoidal()` recomputed `tAccel` and
   `maxVel` for a triangular profile but left `dAccel` at its full-speed value.
   The decel branch writes positions starting from `dAccel`, so on any
   triangular profile the position axis jumped past `distance` and a
   position-keyed lookup returned peak velocity for the entire second half. The
   robot held ~48 in/s to the endpoint and sailed 20 in past it. Only the
   position axis was wrong, so time-indexed users of the profile were unaffected.

3. *It ran roughly 50% fast.* The reference pose was the closest path point
   pushed forward by `lookaheadDist`, which leaves a standing `e_x` of about
   that distance. `k*e_x` then adds speed on top of every point of the profile,
   and since `k` is proportional to `v_d` the inflation is proportional too, so
   the profile could not govern the speed at all. RAMSETE's reference is where
   the robot *should be*, so it is now the projection onto the path.
   `lookaheadDist` is kept in the struct — removing it would break existing
   designated-initializer calls — but nothing reads it.

From the path start the robot now comes within **0.29 in** of the path end.

**The default `b` was mis-scaled and is now 48.**
`b` and `zeta` use the meter-based parameterization and the controller converts
`b` to inches exactly, so the textbook `b = 2.0` is faithfully implemented — it
is just set for FRC-scale paths. A VEX field's curves are far tighter relative
to the robot, which needs much more feedback authority. Swept on this
drivetrain, over three paths, with encoder slip / IMU noise / per-side scale
error enabled:

| `b` | straight | gentle curve | tight curve | peak speed |
|----:|---------:|-------------:|------------:|-----------:|
| 2   | 2.2 in   | 2.6 in       | 6.1 in      | 36–47 in/s |
| 16  | 0.5 in   | 1.6 in       | 3.0 in      | 36–47 in/s |
| 48  | 0.3 in   | 0.8 in       | 0.8 in      | 36–48 in/s |
| 512 | 0.4 in   | 0.5 in       | 0.7 in      | 37–46 in/s |

Error flattens out past ~48 and never destabilises, including at
`responseTauMs` raised to 200 and 350 ms. Peak speed is unchanged throughout,
so this buys tracking accuracy without making the robot faster or twitchier.

**What RAMSETE still cannot do: hit a heading at the end.**
Every term of the law scales with `v_d`, and the profile drives `v_d` to 0 at
the endpoint, so the controller has no authority left there. On curved paths the
robot finishes 5–12 deg off the final tangent and cannot correct it, which means
the default `headingTolerance` of 2 deg is never met and the motion runs out its
timeout instead of settling. Position converges fine. Widen `headingTolerance`
per call, or follow the path with a `turnToHeading`. Left as-is because
loosening an accuracy tolerance is a call about your auton, not a bug fix.

**LTV / LQR was never affected — the deadlock was RAMSETE specifically.**
The two followers advance their reference completely differently, and only one
was vulnerable. `LTVPathFollower` steps `for (int i = 0; i < trajectory_size; ++i)`
with `delay_until(&t, 10)`, i.e. one sample per 10 ms tick: the reference moves
on the clock, not on the robot's progress, so a zero starting velocity is simply
the first sample and the next tick carries on. Verified in the sim from the exact
pose that used to deadlock RAMSETE — it drives and lands 0.42 in from the path end.

Two things about LTV worth knowing, neither of them faults:

- Its trajectory is a **text blob**, not a pose vector: `P = {(x, y, theta), ...}`
  in metres and `V = {(v, omega), ...}` in m/s. `parse_tuples()` keys on
  **parentheses**; a brace-delimited blob parses to nothing and the follower
  bails with "Empty trajectory". `sim/web/ltv_traj.js` generates the format.
- Nothing in the project constructs an `LTVPathFollower`, so it had never run.
  The sim builds one on demand.

`FeedforwardGains::kV` is volts per m/s and it matters a lot. This drivetrain
does 450 rpm on a 3.25 in wheel at 12 V, about 1.95 m/s, so kV ~= 12 / 1.95 =
**6.2**. At kV 0.9 the robot ends 9.1 in short of the path; at 6.2 it ends
0.42 in short. That is a tuning sensitivity rather than a bug, and the suite
pins both numbers so it stays visible.

**`maxSensorRange` is set beyond what the hardware can do.**
`config.cpp` sets `.maxSensorRange = 140.0f` inches (3556 mm). A V5 distance
sensor reports up to about 2000 mm. Particles are weighted against expected
distances the real sensor could never return; the `dist range mm` slider shows
what that costs.

## Adding a season's field

Geometry is data-driven. Edit `Field::loadDefault()` in `sim/world/field.cpp`,
or paste JSON into the **View** tab's geometry box and hit Apply, no rebuild.

Only two things in the default layout are authoritative: the 140.4 in perimeter
(fixed by the VRC spec, and matching `kFieldWalls`) and the nine posts, taken
verbatim from `kFieldPosts` in your own `config.cpp`. Anything else your season
needs — barriers, goal structures, loaders — has to be measured and added. I did
not guess at dimensions I could not verify.

Keep it in step with `kFieldWalls` / `kFieldCircles` in `mcl_tracking.hpp` —
where the two disagree is exactly where MCL will mislocalize on a real field.

## Not built yet

- **Draggable waypoints.** The route preview draws numbered waypoints but they
  are not draggable yet; shift-click appends instead. Dragging would have to
  rewrite the matching line in the editor text.
- **Replay scrubbing.** Runs are recorded and plotted, but you cannot drag a
  time cursor and watch the field rewind.
- Lift, intake and other subsystem simulation — the shims exist and the code
  runs, but nothing physical is modelled.

## Library changes made for the sim

All small, and useful on-robot too:

- `MclTracking::snapshotParticles()` / `config()` — read-only accessors so the
  particle cloud can be drawn or logged.
- `#include <algorithm>` added to `movement_control/arc.cpp`. It calls
  `std::min` without including the header; it happened to compile on
  `arm-none-eabi-gcc` through transitive includes, but that is not portable.
- `gamespecific/robot_config.{hpp,cpp}` renamed to `config.{hpp,cpp}`.
