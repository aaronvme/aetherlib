# chad-sim

A browser simulator for testing the `chad` library off-robot.

It compiles **your code** to WebAssembly and runs it against fake
hardware. There is no second implementation of your algorithms, so a gain you
tune here is a gain you can paste into the robot, and a bug you reproduce here
is a bug in the code.

```
./sim/build.sh            # build to sim/web/
python sim/serve.py       # http://localhost:8080
./sim/test/run.sh         # every headless suite
./sim/test/run.sh smoke   # just one
```

Two environment variables keep concurrent work from colliding, because
`sim/web/` is a single pair of files and two builds at once corrupt each other:
`CHAD_SIM_OUT=<dir> ./sim/build.sh` builds somewhere private, and
`CHAD_SIM_MJS=<dir>/chad_sim.mjs` points the suites at it —
`sim/test/harness.mjs` honours it, and `./sim/test/run.sh --out <dir>` sets both.
Suites are discovered, so a new `sim/test/<name>.mjs` is gated automatically;
`tune` and `autotune` are excluded because they are parameter sweeps with no
pass/fail, not tests.

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

## Numbers below this line: check the date

`World::updateDrive` applied each motor's `reversed` flag to the *command*, and
`config.cpp`'s left ports are all negative, so from commit `a3599ea` until
2026-08-23 the robot **spun in place instead of driving**. Nothing rebuilt the sim
in that window, so it never surfaced. Every figure in this file that involved
driving was measured before it and is stale; the qualitative claims survived
re-measurement, the numbers did not. `docs/sim-audit.md` has the re-measured
values and the rest of what that rebuild turned up. Re-measure before quoting.

## What is real and what is faked

**Real**  compiled from `src/`:

| | |
|---|---|
| Localization | `OdomTracking`, `MclTracking`, `EkfTracking`, `RclTracking` |
| Control | `PID`, `ExitCondition`, `FeedforwardController`, `SettleMonitor` |
| Motion | `moveToPoint`, `turnToPoint`, `moveDistance`, arc, boomerang, RAMSETE, LTV, swing, MPC |
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
- **Distance sensors integrate a ~25 degree cone**, not a pencil ray, so a 4 in
  goal post is seen well off-axis and a reading can come from something the
  centre ray misses entirely. The reported range is the *strongest* return in the
  cone, not the nearest: a flat wall fills the cone, so its nearest ray is always
  the cone edge, and taking that reads 3.95 in short on a 43.02 in range at 30 deg
  off square. Strength is `apodisation * cos(incidence) / d^2`, which still lets
  a near post beat the wall behind it. Both numbers are pinned in `world.mjs`.
- **Confidence falls with range and incidence**, and the return is lost past a
  grazing threshold (default 75 degrees from the surface normal). Running
  near-parallel to a wall and dropping sensors is the dominant real-world
  failure mode for wall-referenced localization.
- **Encoder position and the IMU heading are quantised** to their real
  resolutions (0.1 motor degrees, 0.01 degrees). Motor *velocity* is not; see the
  limits below.
- **Field geometry occludes.** `sim/world/field.cpp` raycasts against the
  perimeter and every goal post and returns a surface normal, so a sensor aimed
  at a post reports the post rather than the wall behind it.

Sensor mount offsets come straight from `chad::sensorMounts` in
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
src/chad/, src/gamespecific/ THE CODE UNDER TEST — unmodified
```

`-Isim/platform` precedes `-Iinclude`, so `#include "pros/imu.hpp"` resolves to
the shim instead of the real header. The library needs no `#ifdef`s.

### Tasks are real fibers

chad runs odometry, MCL, RCL and the chassis motion queue in `pros::Task`
background loops, and blocks the caller inside `waitUntilSettled()`. Those are
genuinely concurrent stacks — you cannot drive them by calling `update()` in a
loop. `sim/platform/sim_sched.cpp` gives each task an Emscripten Asyncify fiber
with its own stack.

Scheduling is deterministic by construction: exactly one fiber runs at a time,
the next is always the earliest wake time with ties broken by task id, and the
clock only advances when nothing is runnable. `pros::delay(n)` yields; the
scheduler advances virtual time and steps the world. That is what makes
"Step 10 ms" meaningful and a divergence bug reproducible.

### Determinism

The world and MCL both replay exactly for a given seed. `MclCore` seeds from
`kDefaultSeed`, so a run is reproducible with no effort; `sim_set_param("seed", n)`
pins the world's noise and the particle jitter together, and `sim_mcl_seed(n)`
reseeds MCL alone, so an independent filter run can be compared against an
identical sensor stream. `sim/test/motion.mjs` pins all three: two default-seed
runs are bit-identical, an explicit seed replays, and a different seed is a
genuinely different run that still localizes.

**The FIRST run used to be the odd one out, and no longer is.** Identical inputs
landed ground truth 0.276 in from where every later identical trial landed, so
`recovery.mjs` discarded a warm-up trial and `motion.mjs`'s determinism checks
only passed because five other sections ran ahead of them. The cause was not the
RNG. `MclTracking::stopTracking()` waits for its task in a `pros::delay(5)` loop,
and from the bridge's root context `pros::delay` advances virtual time -- so a
`sim_reset` with a filter up parked the tasks it had just cancelled BEFORE
returning, and a `sim_reset` with no filter up left them due at t = 0. The
chassis motion task's post-cancel zero write then landed inside the first
`sim_step`, after `applyManualDrive()`, and clobbered the whole first physics
slice: 1 ms of drive lost at `sim_step(1)`, 10 ms at `sim_step(10)`.

`sim_reset` now spends `kResetDrainMs` (one 10 ms control period) unconditionally
after cancelling and before rewinding the clock, so every reset ends in the same
scheduler state. Measured after: the first trial is bit-identical to the second
for MCL-only, MCL+EKF, dead reckoning, and whether or not `sim_set_localization`
is called before or after the first reset. The observable is
`sim_drive_cmd_left()` on the first physics tick after a `sim_set_drive`; it used
to read 0.000 on a cold run and now reads the command in every case.
`sim/test/ui.mjs` asserts both, and a warm-up trial is no longer needed.

### What `sim_reset` restores, and what it does not

`sim_reset` restores **run state** and deliberately **not configuration**:

| restored by `sim_reset` | left alone |
|---|---|
| ground truth, encoders, the IMU, the device clocks | every `sim_set_param` key |
| the injected encoder fault (`sim_kill_drive_encoders`) | the trajectory config (`sim_traj_set`) |
| trails, the recorder, the motion records and telemetry trace | the MPC config (`sim_mpc_set`) |
| the route, trajectory, MPC and LTV followers -- a live one used to keep driving straight through a reset | the field geometry |
| the virtual clock and every task's phase | |

Restoring the parameters inside `sim_reset` is **wrong**, and that is measured
rather than argued. Every suite here writes its knobs and THEN resets, because
`sim_reset` *consumes* them: `World::resetTo()` seeds the IMU through
`imuUnderReport` and synthesises the beams through the distance knobs. A
restoring `sim_reset` therefore removes the ability to configure the world a
reset establishes -- simulated, it fails **18** checks in
`sim/test/world.mjs` (every IMU-calibration, slip and per-side-scale check, plus
the range ceiling and the incidence limit) and **4 of 16** in
`sim/test/recovery.mjs`, whose trial count also moves 19 -> 23 because post
collision comes back on.

So trial independence stays the caller's job, and these make it one call instead
of a hand-written table:

| call | what it does |
|---|---|
| `sim_params_restore()` | puts every restorable key back to what `sim_init()` found |
| `sim_params_dirty_json()` | `{count, of, params:[{k, is, was}]}` -- every key that is not back, with both values |
| `sim_param_baseline(name)` | the value `sim_init()` found |
| `sim_param_count()` / `sim_param_name(i)` | the whole key set, enumerable at runtime |
| `sim_param_restored(name)` | 1 restored, 0 exempt, -1 not a key at all |
| `sim_state_json().diag.params` | `{dirty, of}` -- the same count, for callers who never learn the two names above |

107 keys, 106 of them restorable. **`seed` is the single exemption**: it names the
trial rather than the model, `sim_reset` already re-derives both RNGs from it, and
restoring it would silently re-randomise a sweep that had pinned it. It is
excluded from the dirty count for the same reason -- a moved seed is what every
correct caller does, so counting it would stop `dirty == 0` being an assertion
anyone could use. `sim_param_restored("seed")` reports the exemption at runtime,
and `sim/test/ui.mjs` fails if a second one appears.

The leak this replaces is worth stating exactly, because it is the reason
listing keys by hand is not the fix: a probe set `distMaxRangeMm = 1` to blind
the beams and every later trial ran blind, and `distMaxRangeMm` *was* in that
probe's hand-written baseline -- in the block that ran before the reset. Three
assertions went vacuous. `sim_params_dirty_json()` covers all 106 keys in one
call and does not depend on anyone having thought of the right one.

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

## Motion, Traj and MPC tabs

Three panels over the surfaces the headless suites already drive. The HUD's
`motion` row is live on all of them and says **who owns the motors first** --
`trajectory`, `MPC`, the running `MotionType`, or how the last motion ended --
because "what is driving right now" is the question a stuck run raises.

**Motion** builds a list of motions and runs it through the ordinary *blocking*
`Chassis` API on its own fiber, one record per motion. The Route DSL cannot do
this: its commands carry no `earlyExitRange` or `minSpeed`, and it reports nothing
about how a motion ended. The results table is `MotionOutcome`, time taken, when
the settle test first passed and how long it then held, tick count, peak PID
output, and **the chaining seam** -- the drive speed and command on both sides of
the call. That last column separates the four cases that matter: a `chained`
outcome with the drive still on is a handover, an `arrived` with the drive still
on is a runaway, and a `chained` that stopped is a handover that did not happen.
`toleranceMs` is `—` rather than a number when the settle test never passed,
which is what a boomerang or a path motion legitimately does.

**Traj** generates a constrained trajectory from waypoints and follows it. The
generated states are drawn on the field (light blue, with heading ticks) straight
out of the same buffer `followTrajectory` consumes, so nothing can drift between
what is drawn and what is driven. The audit panel is computed **from physics** by
`sim/web/traj.js`, not by re-calling the constraint classes -- per-wheel voltage
against the battery, differenced on the 10 ms grid the follower samples, plus
each state against independent closed-form grip and wheel-speed ceilings.

Generation is allowed to **fail**, and the panel treats that as an answer rather
than an empty canvas. `config.cpp` ships `kaStraight = 0`, so ticking
*wheel-voltage constraint* returns `unusableConstraint` with zero states -- the
status line says so and names `kA` as the thing to measure. `cuspedPath` is the
other one a human will hit: waypoint headings of 45/0/90/0 give a 0.20 in radius.

**MPC** runs `MpcPathFollower` over the trajectory generated on the Traj tab,
resampled on the MPC's own `dt`. It leads with **whether the solve converged
inside its budget**, because that is what decides feasibility on a V5 -- every
feasible configuration measured in `docs/sim-audit.md` sits under 2.4% of a 10 ms
tick and the only expensive one is infeasible. `worstPrimalResidual` is called out
in its own right: an infeasible corridor combined with a rate limit destabilised
the loop to a 0.514 m peak cross-track while **no boolean diagnostic fired**, so a
residual that never comes down reads as a problem even with `infeasibleTicks` at
zero. *Row budget* runs `sim_mpc_rows_json` and shows planned against installed
rows, because `addStateConstraint` accepting a spec is not the same as its rows
reaching the QP.

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
| `wheel slip` | fraction of travel the encoders never see, the main odometry drift source. **Sign-inverted against real wheel slip** -- `world.cpp` multiplies reported travel by `(1 - encoderSlip)`, so odometry reads SHORT (62.64 in true, 46.98 in reported at 0.25), where a slipping wheel turns more than the robot moves and reads LONG. Right magnitude of encoder-to-ground disagreement, wrong direction; left as-is because flipping it would rebase every drift figure in `docs/sim-audit.md` |
| `left/right scale` | drive asymmetry, "the robot pulls left" |
| `drive lag ms` | first-order velocity response |
| `IMU drift` | gyro bias, deg/s |
| `IMU noise` | a random **walk** in deg/sqrt(s), not jitter on a reading: it is added to the accumulated rotation, so the error never comes back. At 2.0 the heading is 11.9 deg RMS out after 32 s **at rest** and stays there. Pinned in `world.mjs` |
| `IMU under-report` | derived as the inverse of `chad::imu.scaleFactor()`, so it cancels the config's own `ScaledIMU` calibration exactly at the default; dial it away to inject a mismatch. It used to hard-code 354.25/360 against a config declaring `ScaledIMU(13, 360.0, 359.0)`, which left a 1.19 deg heading bias standing still |
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
`include/chad/tracking/mcl_tracking.hpp` are empty, and the nine posts
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

From the path start the robot now comes within **0.2 in** of the path end
(re-measured 2026-08-23, after both the drive-model and IMU-calibration fixes).

**The default `b` was mis-scaled and is now 48.**
`b` and `zeta` use the meter-based parameterization and the controller converts
`b` to inches exactly, so the textbook `b = 2.0` is faithfully implemented — it
is just set for FRC-scale paths. A VEX field's curves are far tighter relative
to the robot, which needs much more feedback authority. Swept on this
drivetrain, over three paths, with encoder slip / IMU noise / per-side scale
error enabled. Re-measured 2026-08-23 after the drive-model fix, as worst
cross-track distance to the uploaded polyline and error at the endpoint:

| `b` | straight | gentle curve | tight curve | peak speed |
|----:|-----------------:|-----------------:|-----------------:|-----------:|
| 2   | 1.6 / 1.8 in | 5.2 / 1.8 in | 6.9 / 3.2 in | 48 in/s |
| 16  | 0.3 / 0.4 in | 2.0 / 1.4 in | 2.5 / 0.7 in | 49 in/s |
| 48  | 0.1 / 0.2 in | 1.1 / 1.1 in | 1.6 / 1.2 in | 49 in/s |
| 128 | 0.1 / 0.1 in | 1.0 / 1.5 in | 1.5 / 1.0 in | 50 in/s |
| 512 | 0.1 / 0.2 in | 1.0 / 1.4 in | 1.3 / 1.1 in | 45 in/s |

Error flattens out past ~48 and never destabilises. Peak speed is unchanged
throughout, so this buys tracking accuracy without making the robot faster or
twitchier. The knee is still at 48, which is why the default has not moved.

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
pose that used to deadlock RAMSETE — it drives and lands 0.57 in from the path end.

Two things about LTV worth knowing, neither of them faults:

- Its trajectory is a **text blob**, not a pose vector: `P = {(x, y, theta), ...}`
  in metres and `V = {(v, omega), ...}` in m/s. `parse_tuples()` keys on
  **parentheses**; a brace-delimited blob parses to nothing and the follower
  bails with "Empty trajectory". `sim/web/ltv_traj.js` generates the format.
- Nothing in the project constructs an `LTVPathFollower`, so it had never run.
  The sim builds one on demand.

`FeedforwardGains::kV` is volts per m/s and it matters a lot. Swept on the LTV
path: 12.09 in of end error at kV 0.9, 1.27 in at 4.0, 0.57 in at the default
6.2, 0.45 in at 7.5, **0.37 in at 9.0**, and 0.69 in at 12. Guessing low costs a
twentyfold along-track deficit.

The default 6.2 is 12 V / 1.95 m/s, the speed of a 3.25 in wheel at 450 rpm.
`config.cpp` measures this drivetrain's wheel at 2.6738 in, i.e. 1.60 m/s, which
implies **7.5** — and 7.5 does track better than 6.2, with the optimum near 9.
Worth knowing how that was established: measured against the broken IMU
calibration, 6.2 looked like the optimum and 7.5 looked worse, which is a clean
example of a fixture fault producing a confident wrong tuning answer. The sim
still has no load model, so settle the final value on hardware — but the
physical derivation and the sim now agree that 6.2 is too low.

**`maxSensorRange` is set beyond what the hardware can do.**
`config.cpp` sets `.maxSensorRange = 140.0f` inches (3556 mm). A V5 distance
sensor reports up to about 2000 mm. Particles are weighted against expected
distances the real sensor could never return; the `dist range mm` slider shows
what that costs.

## Adding a season's field

Geometry is data-driven, for collision as well as for the raycaster. Edit
`Field::loadDefault()` in `sim/world/field.cpp`, or paste JSON into the **View**
tab's geometry box and hit Apply, no rebuild.

Containment is the bounding box of the `kind == 0` segments, so `Field::clear()`
leaves a field with **no walls** and the robot free-runs off it — it used to stop
at x = 64.85 against a file-scope 70.2 with no geometry loaded at all. The `kind`
tag therefore rides through the editor's JSON as a fifth element, and Apply with
no edits is a no-op; dropping it turned Apply into "delete the perimeter".

Only two things in the default layout are authoritative: the 140.4 in perimeter
(fixed by the VRC spec, and matching `kFieldWalls`) and the nine posts, taken
verbatim from `kFieldPosts` in your own `config.cpp`. Anything else your season
needs — barriers, goal structures, loaders — has to be measured and added. I did
not guess at dimensions I could not verify.

Keep it in step with `kFieldWalls` / `kFieldCircles` in `mcl_tracking.hpp` —
where the two disagree is exactly where MCL will mislocalize on a real field.

## Headless suites

`./sim/test/run.sh` runs all of these; each is also `node sim/test/<name>.mjs`.

| suite | what it holds |
|---|---|
| `dsl` | route parser and the opcode contract with `RouteOp` in the bridge |
| `smoke` | localization wiring, device realism, RAMSETE and LTV end-to-end |
| `world` | the world model against closed-form physics, not against itself |
| `motion` | `MotionOutcome`, stall detection, settle timing, chaining seams, encoder outage, MCL determinism |
| `traj` | spline, constrained generation, and feasibility audited from physics |
| `mpc` | the constrained MPC, its solver diagnostics, and its cost against a 10 ms tick |
| `ui` | the browser's own pure logic, the parameter-baseline contract, and every element id `app.js` reaches for |

`tune` and `autotune` are parameter sweeps rather than tests, so `run.sh` skips
them. Note `autotune`'s kidnap trial does not drive the robot, and a stationary
robot cannot relocate — read its `kidnap 0/4` as a property of that trial, not as
the recovery rate.

## Not built yet

- **Draggable waypoints.** The route preview draws numbered waypoints but they
  are not draggable yet; shift-click appends instead. Dragging would have to
  rewrite the matching line in the editor text.
- **Replay scrubbing.** Runs are recorded and plotted, but you cannot drag a
  time cursor and watch the field rewind.
- Lift, intake and other subsystem simulation — the shims exist and the code
  runs, but nothing physical is modelled. **`JamDetector` therefore cannot fire
  here**: `currentMa` is a function of the command alone, so a stalled motor
  draws what a free one does. `sim/platform/pros/motors.hpp` also returns hard
  constants for `get_efficiency()` (100.0, against a 7-20% band),
  `get_power()` (0.0), `get_torque()` (0.0), `is_over_temp()` (0) and
  `is_over_current()` (0) -- anything gated on one of those reads "healthy" no
  matter what the robot is doing, so it cannot be tested here.
- **The panels are not visually verified.** There is no browser on the machine
  they were written on. Everything that decides a number, a colour or a threshold
  is in `sim/web/{motion,traj,mpc,view}_view.js` — `view_panel.js` for the
  transport bar and the geometry editor — and asserted by `sim/test/ui.mjs`; what
  is left in `app.js` is DOM plumbing. Statically, `ui.mjs` checks the id graph
  in BOTH directions (105 ids, including the ones reached through a variable or a
  wrapper), duplicate ids, every listener target, every class selector, the
  element KIND behind every `.checked` / `.value` / `.innerHTML`, every verdict
  class against the stylesheet per container, the palette's WCAG contrast (worst
  pair `--dim` on `--card` at 4.90:1), and the plot/CSV columns against
  `sim_record_stride()`. What still needs eyes: whether anything overflows or
  collides at a real window size, the canvas field drawing and the robot/beam
  overlays, pointer drag behaviour, and whether the colours read well on a real
  panel rather than merely clearing a ratio.
- **The MPC trace is filled by the bench, not by a world run.**
  `sim_mpc_trace_ptr()` is written only inside `sim_mpc_bench_json`, so the MPC
  panel shows `sim_mpc_stats_json` and the row budget; there is no per-tick
  residual plot for a path followed in the world.
- **Collision is a disc, not the footprint.** `robotLengthIn`/`robotWidthIn` are
  used as drawn, but collision uses an orientation-free disc of a quarter of the
  diagonal: 5.31 in measured for the 15x15 placeholder, against a 7.50 in
  half-width, so the drawn robot overlaps a wall by 2.19 in where it stops, and a
  long thin robot would clear geometry it should hit. Left alone because both
  dimensions are guesses and every route suite is tuned against that 5.31 in
  clearance -- pinned in `world.mjs` so a change to it is visible.
  A pose that is already inside geometry (`sim_teleport`, or a structure dropped
  on the robot from the editor) can drive OUT: the move is accepted when it
  strictly reduces penetration depth, which is also what stops it becoming a way
  to drive through walls.
- **No load model, so no slip.** Voltage maps to a target wheel speed through a
  first-order lag; there is no mass, torque or friction. The `wheel slip` knob is
  an *encoder* fault rather than a traction model, and it is sign-inverted
  against real slip -- see the knob table above. A grip-limited
  trajectory tracks perfectly here even if the grip limit is wrong by 3x, so the
  sim can say which constraint *binds in the planner* and never whether the robot
  holds the corner. It also cannot judge a velocity-feedback controller fairly:
  `velocityRpm` is exact and unfiltered, while a real V5 reports it coarse and
  late.

## Library changes made for the sim

All small, and useful on-robot too:

- `MclTracking::snapshotParticles()` / `config()` — read-only accessors so the
  particle cloud can be drawn or logged.
- `#include <algorithm>` added to `movement_control/arc.cpp`. It calls
  `std::min` without including the header; it happened to compile on
  `arm-none-eabi-gcc` through transitive includes, but that is not portable.
- `gamespecific/robot_config.{hpp,cpp}` renamed to `config.{hpp,cpp}`.
