# chad-sim

A browser simulator for testing the `chad` library off-robot.

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
| Motion | `moveToPoint`, `turnToPoint`, `moveDistance`, arc, boomerang, RAMSETE, swing |
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

### Determinism caveat

The world replays exactly for a given seed. **MCL does not.**
`MclTracking`'s constructor seeds `mGen` from `std::random_device`, so particle
initialisation and resampling jitter differ every run. If you want
bit-reproducible MCL runs, that constructor needs an optional seed parameter.

## Routing an auton

The **Route** tab holds **auton source**. Not a DSL that resembles one -- the
same C++ you paste into `src/gamespecific/autons/`, run against the real
`Chassis` API on its own fiber, through the same motion queue and the same
`waitUntilSettled()`. Edit and re-run without recompiling.

```cpp
void autonBlueLeft() {
    chassis.setPose(Pose::fromVex(-48.0f, -60.0f, 90.0f));

    chassis.moveToPoint(-24.0f, -60.0f, 2500);
    chassis.turnToHeading(0.0f, 1200, {.maxSpeed = 90.0f});
    chassis.moveDistance(-12.0f, 1500);
    chassis.turnToPoint(0.0f, 0.0f, 1500);
    chassis.moveArc(24.0f, 24.0f, 20.0f, 3000);
    chassis.moveBoomerang(48.0f, 24.0f, 180.0f, 3500, {.lead = 0.6f});
    chassis.swingToHeading(270.0f, SwingSide::Right, 1500);
    pros::delay(300);

    std::vector<Pose> path = {
        {-24.0f, -24.0f, 0.0f},
        {0.0f, -40.0f, 0.0f},
        {24.0f, -24.0f, 0.0f},
    };
    chassis.moveRAMSETE(path, 5000, {.b = 48.0f, .zeta = 0.85f});
}
```

**Copy as auton** wraps it in the includes and the `namespace chad` and copies
it out; paste and compile. That is the whole point -- what you tuned on screen
is what ships, with no translation step in between to get wrong.

### Running the autons you actually shipped

The dropdown beside **Run auton** lists what `kAutons` in `sim/bridge/bridge.cpp`
exposes, and those entries run the *compiled* function -- not the text in the
editor. Selecting one loads the source it was built from, so its route draws
like any other; your own text parks in the **scratch** slot and comes back when
you pick it again.

That source is generated, never committed. `sim/build.sh` runs
`sim/auton_sources.py` over `src/gamespecific/autons/` and writes
`sim/web/auton_sources.js` next to the wasm -- same tree, same build, so a drawn
path cannot disagree with the routine that runs. Delete it and the app still
boots; the compiled autons just lose their preview.

**Run auton** resets to the pose the auton opens with before starting it.
`chassis.setPose()` moves the estimate and not the robot, so without that the
routine drives its whole route from wherever the last run happened to end.

### setPose places the robot once

Only a `setPose` before the first motion moves ground truth. A later one is a
re-localization: it corrects the estimate and leaves the robot alone, because
moving truth to meet the estimate would erase the error the sim exists to show.

### What it understands, and what it refuses

Designated initializers are honoured, per motion, by name:
`{.forwards = false, .maxSpeed = 90.0f, .lead = 4.0f}`. So is a trailing
`true` for an async motion. A `.lead` you wrote and the sim ignored would make
the preview a different motion from the code, so anything it cannot apply is
counted in the `?` column of the report rather than dropped.

Two mistakes are reported as **errors**, because both are compile errors on the
robot and catching them here is free:

- a field the struct does not have (`.speed` on `MoveToPointParams`)
- designated initializers out of declaration order, which C++20 rejects

Anything it cannot route -- a lift call, a loop, a local variable -- is
**skipped and listed** under the editor. Never silently ignored: a preview that
quietly dropped half your auton would be worse than none. A misspelled
`chassis` method is an error rather than a skip, because that is a typo.

Note the two conventions real code carries, both of which the parser respects
because the library does:

- `Pose::theta` is **standard-math radians**, so `setPose({0, 0, 0})` faces +X
  (VEX 90), not VEX 0. `Pose::fromVex(x, y, deg)` is the readable form and the
  one **Copy as auton** writes.
- `moveDistance(-12.0f)` drives backward. `Chassis::moveDistance` flips
  `forwards` itself for a negative target, so the sign is not a shorthand the
  sim invented.

### Paths: control points, splined for you

A `std::vector<Pose>` is a list of **control points**, not the path. Place
three and the trajectory comes out smooth, without asking, exactly as `via()`
did:

```cpp
std::vector<Pose> path = {
    {-24.0f, -24.0f, 0.0f},
    {0.0f, -40.0f, 0.0f},
    {24.0f, -24.0f, 0.0f},
};
chassis.moveRAMSETE(path, 5000);
```

Drag the purple points on the field and the curve follows. Heading ticks
along it make a reversed or twisted curve obvious before you run it.

**The smoothing is `moveRAMSETE`'s, not the simulator's.** It resamples the
path it is handed along a Catmull-Rom spline every
`RAMSETEParams::smoothSpacingIn` inches -- 2 in by default -- before the
controller sees anything, and computes the tangent headings the velocity
profile needs while it is there. So those three poses are all you write in
the auton as well: paste this straight into
`src/gamespecific/autons/` and the robot generates the same curve at runtime.

Which is why the sim uploads the three control points and nothing else, then
asks the wasm for the curve to draw. One generator
(`chad::movement_control/spline.hpp`), used by the preview, by this run and by
the robot. Set `.smoothSpacingIn = 0.0f` to drive a path exactly as given.

The old `via()` preview sampled the same curve in JavaScript, in
`buildPath()`, and that function is still there for the route DSL --
`sim/test/auton_cpp.mjs` asserts its spacing still matches the header's, so
the two previews cannot disagree.

### Editing on the field

Waypoints draw as a numbered dashed preview and the HUD shows which command is
executing. **Shift-click** appends a `moveToPoint` inside the function body.
**Drag a numbered point** and the x/y literals in the source are rewritten in
place -- keeping the `f` suffix, touching nothing else on the line. Source
saves to `localStorage` under its own key, so a route saved in the old DSL is
left alone rather than failing to parse on load.

### The report is the part that helps

After a run, **Last run** lists each motion: how long it took, whether it was cut
off by its timeout, how far it ended from what it was aimed at, the closest the
footprint came to a wall or post, and the localization error at that moment.
Plus the total against the 15 s auton limit.

The drawn path cannot show any of that. A motion that sat out its timeout looks
identical on the field to one that settled, and a route that passes an inch
from a post looks fine right up until the match where it does not.

Two details worth knowing:

- **Cut off** is a heuristic, and the only one available: nothing in the
  `Chassis` API says why a motion returned, so a blocking motion whose duration
  reached its timeout within one loop tick is reported as cut off.
- **Clearance** is measured from the body's own extent in the direction the
  obstacle actually lies, so it means what it says at any heading rather than
  only when the robot is square. It counts everything loaded, and so does the
  physics now -- the two used to disagree, because the physics stopped only on
  a hardcoded perimeter and the posts and drove clean through goal walls the
  column was warning about.

Opcode numbers are the contract between `sim/web/route_dsl.js` and `RouteOp` in
`sim/bridge/bridge.cpp`; `sim/test/dsl.mjs` reads the C++ enum back out and
fails if they drift. `sim/test/auton_cpp.mjs` does the same for the params
structs -- it reads every field of every struct out of
`include/chad/movement_control/*.hpp` -- and then parses the autons that
actually ship in `src/gamespecific/autons/`, `example.cpp` included, which
exercises every motion type with a full params block on each.

`sim/test/app_boot.mjs` loads `app.js` against a stubbed DOM and drives the
render loop, every button and a run of half-typed source through it. Shallow on
purpose -- it does not check what was drawn. It exists because app.js had no
test at all, and the gap bit once: a rename left `drawPlan()` reading a
variable that no longer existed, `frame()` threw on its first call, and the
page came up with a dead clock and an empty field while every other suite
stayed green.

## The sim has its own tuning

`kSimTuning` in `sim/bridge/bridge.cpp` is applied over
`gamespecific/config.cpp` at `sim_init`. It is deliberately **not** the robot's
numbers, and is not meant to become them. The sim's drivetrain is a first-order
lag with no mass, torque or traction, so a gain that is right here is optimistic
there. What it is for is the one thing the sim is authoritative about: **the
route you drew and the route the sim drives being the same route.**

**None of it reaches the robot.** `bridge.cpp` is a simulator source: the
firmware build never compiles it, so `kSimTuning` cannot alter what the brain
runs. `gamespecific/config.cpp` stays the single source of truth for the robot,
and its values still come from the on-hardware routines --
`autonCharacterize*`, `autonAutotune*`, `autonCalibrate*`. The sim does not
replace any of that and is not evidence about any of it.

Only overrides are listed, so the table doubles as the record of what the sim
disagrees with the robot about. Today that is seven numbers:

| | robot | sim | why |
|---|---|---|---|
| `lateralKp` | 9.5 | 12 | at the robot's gain the nine point motions take 16.5 s; at 12 they take 12.5 s. Not higher -- see below |
| `lateralKd` | 0.4 | 0.8 | it has to be overridden **with** `kP`, not inherited: 12 against the robot's 0.4 is under-damped and every long straight overshoots 0.89 in and reverses. See below |
| `ramseteMaxAccel` | 50 | 30 | RAMSETE's endpoint error IS its overshoot, so arriving slower is the whole fix: 0.48 in past the last point at 50, 0.18 in at 30 |
| `ramseteB` | 48 | 96 | at 48 a path does not merely track worse, it times out: 0.89 in cross-track and 19.2 s over five paths against 13.3 s |
| `boomLead` / `boomLeadDecay` / `boomMinSpeed` | 0.5 / 0.5 / 0 | 0.7 / 0.15 / 127 | boomerang throttles its own approach inside 6 in; `minSpeed` floors that scale, and at 127 it switches off |

`targetToleranceIn` used to be here at 0.30, against a robot that shipped 1.0.
The robot ships 0.2 now, so the override had quietly inverted into *loosening*
the tolerance. Inheriting 0.2 scores better anyway -- cost 1.94 against 2.25,
landing 0.194 in out against 0.209 -- for 1.0 s more over the battery, so the
entry is gone rather than re-pinned.

**A partial override is the trap this table sets.** Two of its rows were written
against robot constants that later moved, and the sim went quietly wrong both
times. Overriding one term of a gain set and inheriting the rest couples the
sim's tuning to a hardware measurement that is *expected* to change.

### The overshoot trap, which caught this exact file

`moveDistance` and `moveToPoint` have **no velocity profile.** They are P+D
against a first-order lag, so the only thing that stops them is the output
leaving saturation in time. The plant needs `v*tau` to stop -- 4.4 in from full
speed -- and `127/kP` is how far from the target the output comes off the rail:
10.6 in at `kP` 12, 7.9 in at 16. Past that the arithmetic stops working.

Measured at the default 70 ms lag, over the nine point motions. **These were
taken at `kD` 1.15**, which is what the robot shipped when they were measured
and is why `lateralKd` is now an override in its own right -- at the robot's
current 0.4 the same `kP` 12 overshoots 0.89 in and reverses once:

| `lateralKp` | overshoot | reversals | total |
|---|---|---|---|
| 9.012 (robot, then) | 0.00 in | 0 | 16.5 s |
| 12 | 0.00 in | 0 | 12.5 s |
| 14 | 0.00 in | 0 | 9.4 s |
| 15 | 0.40 in | 0 | 9.6 s |
| 16 | 1.22 in | 7 | 10.6 s |
| 20 | 3.51 in | 9 | 10.8 s |

`kD` swept the same way at `kP` 12, on the whole battery: 0.4 fails 8 cases,
0.6 passes at worst 0.58, and 0.7 to 1.4 all pass with the cost flat from 2.22
to 2.61. 0.8 sits in the flat part with room above the cliff. It holds from
50 ms to 85 ms of lag; at 100 ms the battery does fail, but on turn overshoot
and RAMSETE endpoints rather than anything along-track, and identically at
`kD` 1.15 -- that ceiling is the angular gains and the follower, not this loop.

14 is faster and this table says 12, because 14 only holds at 70 ms: at 100 ms
it overshoots 0.82 in and reverses seven times, where 12 is still clean. 12 is
the largest gain that survives the plant moving.

**An earlier pass of this tuning shipped `kP` 20.** It scored perfectly, because
the score only looked at the final pose -- and a 24 in drive that sails 3.5 in
past the waypoint, reverses, and creeps back finishes at 0.00 in. It is also the
most obviously badly tuned thing you can watch a robot do. That is why
`accuracy.mjs` has `over in` and `revs` columns, and why they are bounded.

### A tolerance below what a controller can deliver is not accuracy

One tolerance covers every motion, because past 0.30 in it stops buying anything.
Over ten point motions: 0.30 in of tolerance lands 0.11 in out in 12.3 s, and
0.12 in lands 0.05 in out in 14.2 s. The last 0.06 in is not worth 1.9 s of a
15 s auton on a robot that is 15 in wide.

It is also the number RAMSETE and boomerang need. Both have a floor on how close
they can get, and a tolerance under it does not finish closer -- it sits out the
timeout at the same distance. At 0.12 in the 48 in curve burned all 6 s and still
finished 0.19 in out; at 0.30 in it exits in 3.1 s, 0.19 in out. So they inherit
this instead of carrying overrides.

### Re-running it

```sh
node sim/test/accuracy.mjs              # the battery, as a table
node sim/test/accuracy.mjs --gate      # exit 1 if a bound is missed
node sim/test/accuracy.mjs --loc=5    # with MCL+EKF up
node sim/test/accuracy.mjs --tau=140  # against a slower motor response
node sim/test/accuracy.mjs --stock    # revert kSimTuning, i.e. the robot's own
node sim/test/tune_routing.mjs all 2  # sweep everything, twice
```

`sim/test/accuracy.mjs` runs one motion per case -- a case that chains motions
cannot say which one missed -- and measures **ground truth** against what the
command asked for. Not the estimate: the estimate agreeing with itself is what a
localization bug looks like. It is a gate in `sim/test/run.sh`, and the same
module the sweeps import, so a tuning run and the gate cannot disagree about what
accurate means.

`--stock` is the measurement that says whether any of this was worth doing. Same
binary, same battery, same 32.5 s of sim time, only those seven numbers
different:

| | robot's values | sim's values |
|---|---|---|
| bounds missed | 13 of 24 | 0 |
| worst end error | 1.40 in | 0.22 in |
| worst heading error | 11.6 deg | 0.17 deg |
| worst cross-track | 0.96 in | 0.60 in |
| worst overshoot | 0.30 in | 0.18 in |
| reversals | 0 | 0 |
| motions cut off by their timeout | 1 | 0 |

The 0.5 in floor across every point-to-point motion in the left column is the
1 in tolerance, not the controller: the debounce expires while the robot is still
closing. That is the number the robot has to live with and the sim does not.
Neither column overshoots -- the robot's gain is well damped, just slow.

As of this writing every motion the editor can emit finishes within **0.22 in**
and **0.17 deg** of what it was asked for, none of them overshoots its waypoint
by more than 0.18 in, **none reverses direction at all**, and the whole 24-case
battery runs in 32.6 s of sim time. Chained into a six-motion route, the robot
ends 0.05 in and 0.28 deg from the last command in 7.9 s.

`tune_routing.mjs` reads its starting point out of the wasm rather than a list in
the file, so it converges on whatever `kSimTuning` already holds instead of
silently re-testing the library defaults. Two things about reading its output:

- **A timeout costs 25 penalty points, which turns the cost into a cliff.** Swept
  in the wrong order, every RAMSETE gain row reports whether it happened to
  squeeze inside the current tolerance rather than how well it tracked. That is
  why the gain sweeps pin a tolerance the follower can comfortably meet and
  tighten it afterwards.
- **The argmin is not always the answer.** Several knobs improve monotonically in
  a direction the sim cannot charge for. Where the table keeps the robot's value
  against a sweep that preferred otherwise, `kSimTuning` says so and why.

### Two things it is not tuned for

**Localization.** Route in dead reckoning, which is the default. Ticking MCL
and/or the EKF costs 0.35-0.44 in of mean estimate error and up to 3.3 in at
worst, and the controller drives on the estimate, so ground-truth error goes from
0.27 in to 1.5 in and motions start timing out. That is not a filter mistuning to
go and fix: in a sim whose odometry is exact, every correction from a noisy
sensor is a step away from truth. Loosening the tolerances only trades the
timeouts for a larger miss -- at 2.5 in of tolerance nothing times out and the
worst motion finishes 4.7 in from its target. If you want the filters to earn
their keep, give odometry something to be wrong about first: `wheel slip` and
`IMU drift` are the **Drive error** and **Sensor error** sliders on the left, and
`sim/test/tune.mjs` is the sweep that scores them.

**Your motors.** `responseTauMs` is the only thing standing in for motor
dynamics, and this tuning is fitted to its 70 ms default. It passes the whole
battery from 30 to 70 ms. At 100 ms the turns start overshooting their heading
(2.3 deg), and at 140 ms that is 8.7 deg. Nothing about that window is a claim
about the robot.

It does suggest one thing to look at on the robot, though, and it is the robot's
own gains that do it: `angularGains` at 176.607/9.933 are what overshoot above
70 ms, and a real drivetrain with mass in it responds slower than that. If turns
overshoot on hardware, `angularKd` is the term that is short -- but measure it
there. This is a hypothesis from a plant with no mass, not a result.

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

Two more are `sim_set_param` keys only, default 0 (off), used by
`sim/test/near_target.mjs`: `driveDeadband` (motor command units a side needs
before it moves -- static friction) and `pivotOffsetIn` (the body turns about a
point this far ahead of the tracking centre, so a turn in place also moves the
tracked pose). The default plant has neither, which is why it never circles a
`moveToPoint` target the way a real drive can.

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

**`maxSensorRange` is set beyond what the hardware can do.**
`config.cpp` sets `.maxSensorRange = 140.0f` inches (3556 mm). A V5 distance
sensor reports up to about 2000 mm. Particles are weighted against expected
distances the real sensor could never return; the `dist range mm` slider shows
what that costs.

## Adding a season's field

Geometry is data-driven. Edit `Field::loadDefault()` in `sim/world/field.cpp`,
or paste JSON into the **View** tab's geometry box and hit Apply, no rebuild.

The last element of each entry is its `kind`: **0 = perimeter, 1 = structure,
2 = user-added**, and it is not cosmetic. Kind 0 always stops the robot; 1 and 2
are obstacles and honour the `collide` switch. Omit it and you get 2, which is
the safe answer for hand-written geometry — but it does mean that pasting the
perimeter back without its `0` leaves walls you can drive through when
`collide` is off.

Only two things in the default layout are authoritative: the 140.4 in perimeter
(fixed by the VRC spec, and matching `kFieldWalls`) and the nine posts, taken
verbatim from `kFieldPosts` in your own `config.cpp`. Anything else your season
needs — barriers, goal structures, loaders — has to be measured and added. I did
not guess at dimensions I could not verify.

Keep it in step with `kFieldWalls` / `kFieldCircles` in `mcl_tracking.hpp` —
where the two disagree is exactly where MCL will mislocalize on a real field.

### A field image behind it

**View -> Field image** draws a render or photo under the vector geometry,
stretched to the 144 in square; path.jerryio's field images work well. It loads
`sim/web/field.png` on boot, or pick a file to swap one in for the session with
no server involved.

It is decoration. The physics still collides with the geometry drawn over it, so
the image is aligned to the field rather than the other way round -- hence the
scale slider, since most renders include the perimeter wall and the 144 in square
does not. Not committed either: the VEX render is not ours to ship.

## Not built yet

- **A load model.** Motion is kinematic: voltage maps to a target wheel speed
  through a first-order lag, with no mass, torque or friction. So the sim
  rewards aggression whose cost it cannot show — sweep `responseTauMs` before
  trusting any tuning result that came out of it.
- **Contact response.** Collision itself is exact -- the body is an oriented
  rectangle tested against every segment and post actually loaded, rotation
  included. A blocked step is split against the surface it drives deepest
  into (wall normal, or radial for a post): the part into it is lost, the part
  along it slides less Coulomb drag `wallFriction` x the push (default 0.4),
  times `wallSlideFrac` (1 = omni, 0 = stop dead). A corner on a flat surface
  pivots about that corner toward the face its reaction torque favours, at
  `squareUpRate` deg per inch pushed in (default 8), until flush -- so a steep
  hit squares the front up and then stops (the push is all normal), a shallow
  one lays the side along the wall and drives on. A hit faster than 10 in/s
  rebounds at `wallRestitution` x the excess (default 0.8), decaying with
  `responseTauMs`: ~0.6 in at full speed, ~0 at a crawl. A turn a surface
  refuses walks the body off it. The drive encoders count commanded travel;
  the pods and IMU see the real slide, pivot and rebound. All three at 0 is
  the frictionless glide. Not modelled: posts squaring the body up (round),
  torque from a post on a face, and rebound spin.
- Lift, intake and other subsystem simulation — the shims exist and the code
  runs, but nothing physical is modelled. Subsystem calls in a routed auton are
  skipped and listed rather than run.

## Library changes made for the sim

All small, and useful on-robot too:

- `MclTracking::snapshotParticles()` / `config()` — read-only accessors so the
  particle cloud can be drawn or logged.
- `#include <algorithm>` added to `movement_control/arc.cpp`. It calls
  `std::min` without including the header; it happened to compile on
  `arm-none-eabi-gcc` through transitive includes, but that is not portable.
- `gamespecific/robot_config.{hpp,cpp}` renamed to `config.{hpp,cpp}`.
- `Chassis::setSettleTolerances()` / `setLateralSlew()` — runtime
  counterparts of the `ChassisConfig` fields. `setPidTuning` deliberately
  does not touch them: damping and tolerance are not the same knob.
