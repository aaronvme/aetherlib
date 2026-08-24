// Headless physics suite for the chad-sim world model.
//
//   node sim/test/world.mjs
//   CHAD_SIM_MJS=/tmp/mybuild/chad_sim.mjs node sim/test/world.mjs
//
// Every check here compares the world against a closed form worked out in this
// file, never against world.cpp's own arithmetic. Where a check needs a robot
// constant (free wheel speed, track width, the IMU's calibration factor) it
// MEASURES it from one experiment and predicts a second, independent one --
// nothing is copied out of gamespecific/config.cpp, because a constant copied
// from the config is exactly how the sim's IMU under-report silently drifted
// 1.6% away from ScaledIMU and put a 1.19 deg bias on every run.
import { loadSim, wrap, tally } from './harness.mjs';

const M = await loadSim(['sim_field_add_segment', 'sim_get_param']);
const api = wrap(M, {
  init:        [null, []],
  reset:       [null, ['number', 'number', 'number']],
  step:        [null, ['number']],
  setDrive:    [null, ['number', 'number']],
  teleport:    [null, ['number', 'number', 'number']],
  stateJson:   ['string', []],
  fieldJson:   ['string', []],
  setParam:    [null, ['string', 'number']],
  getParam:    ['number', ['string']],
  fieldClear:  [null, []],
  fieldDefault:[null, []],
  fieldAddSegment: [null, ['number', 'number', 'number', 'number']],
  fieldAddCircle:  [null, ['number', 'number', 'number']],
  setLoc:      ['sim_set_localization', null, ['number']],
  collideTicks:['sim_collide_ticks', 'number', []],
  trajModelJson:['sim_traj_model_json', 'string', []],
});

const t = tally('world.mjs');
const st = () => JSON.parse(api.stateJson());
const run = (ms, dt = 10) => { for (let i = 0; i < Math.round(ms / dt); i++) api.step(dt); };
const near = (a, b, tol) => Math.abs(a - b) <= tol;
const rel = (a, b) => Math.abs(a - b) / Math.max(1e-9, Math.abs(b));
const DEG = Math.PI / 180;

// The world's kinematic integration step (sim::kPhysicsDtMs). Only the
// closed-form ramp-distance check depends on it.
const PHYS_DT = 0.001;

api.init();
api.setLoc(0);                       // dead reckoning only: MCL is not the subject

// Snapshot every world knob this suite touches so a section cannot leak into
// the next one.
const KNOBS = ['responseTauMs', 'leftScale', 'rightScale', 'encoderSlip',
               'imuUnderReport', 'imuNoiseDeg', 'imuDriftDegPerSec', 'imuPeriodMs',
               'distNoiseMm', 'distFovDeg', 'distConeRays', 'distConeSigmaFrac',
               'distMaxRangeMm', 'distMinRangeMm', 'distMaxIncidenceDeg',
               'distPeriodMs', 'encoderPeriodMs', 'encoderResolutionDeg',
               'collideObstacles'];
const DEFAULTS = Object.fromEntries(KNOBS.map((k) => [k, api.getParam(k)]));
const restore = () => { for (const k of KNOBS) api.setParam(k, DEFAULTS[k]); };

// ===========================================================================
// Drive model: voltage -> wheel speed
// ===========================================================================
t.section('voltage to wheel speed');

/// Settled wheel speed for a symmetric command, inches/s. Starts at whichever
/// end of the lane the command is about to drive away from: the soak covers
/// ~94 in at 12 V, and a robot pinned against a wall now reports the zero wheel
/// speed of a stalled motor rather than the speed it was asked for.
const settled = (volts, ms = 1500) => {
  api.reset(volts >= 0 ? -58 : 58, -60, 90);
  api.setDrive(volts, volts);
  run(ms);
  const s = st();
  api.setDrive(0, 0);
  return s.vel.left;
};

const vAt = { 3: settled(3), 6: settled(6), 9: settled(9), 12: settled(12) };
const vMax = vAt[12];
// A voltage-to-speed map has to be a straight line through the origin. The
// 1500 ms soak is >20 tau, so each reading is the terminal value to 1e-9.
// Tolerance is 0.1%: Drivetrain::setVoltage routes volts through a +/-127 int,
// which truncates 1 mV off each command (0.025% spread across this set).
const slopes = Object.entries(vAt).map(([v, s]) => s / Number(v));
const slopeSpread = (Math.max(...slopes) - Math.min(...slopes)) / (vMax / 12);
t.check('terminal speed is proportional to commanded volts',
        slopeSpread < 1e-3,
        `${Object.entries(vAt).map(([v, s]) => `${v}V=${s.toFixed(2)}`).join(' ')}` +
        `  slope spread ${(100 * slopeSpread).toFixed(3)}%`);

t.check('command saturates at the 12 V rail',
        rel(settled(20), vMax) < 1e-3,
        `20 V gives ${settled(20).toFixed(2)} in/s vs ${vMax.toFixed(2)} at 12 V`);

t.check('reverse is the exact mirror of forward',
        rel(-settled(-6), vAt[6]) < 1e-4,
        `-6 V = ${settled(-6).toFixed(2)}, +6 V = ${vAt[6].toFixed(2)}`);

// The reported wheel speed has to be the rate the robot actually moves at. A
// dt-units slip or a missing 0.5 in the chassis-speed average shows up here and
// nowhere else in this section.
{
  api.reset(-58, -60, 90);
  api.setDrive(12, 12);
  run(600);                                           // settle
  const x0 = st().true.x, v = st().vel.lin;
  run(500);
  const measured = (st().true.x - x0) / 0.5;
  api.setDrive(0, 0);
  t.check('displacement rate equals the reported chassis speed',
          rel(measured, v) < 1e-3,
          `${measured.toFixed(3)} in/s measured vs ${v.toFixed(2)} reported`);
}

// ---------------------------------------------------------------------------
// The first-order lag against its closed form.
//
// The model updates v <- v + (v_inf - v)(1 - e^(-dt/tau)) once per physics
// step and then advances position with the NEW speed, so after N steps
//   v_N = v_inf (1 - a^N),          a = e^(-dt/tau)
//   d_N = v_inf dt (N - a(1 - a^N)/(1 - a))
// Both are exact, not approximations, so the tolerance only has to cover the
// 0.001 in resolution of the reported pose.
t.section('first-order velocity lag');

const TAU = 120;                     // ms, chosen so 500 ms is a partial ramp
api.setParam('responseTauMs', TAU);
const vInf = settled(9);
const a = Math.exp(-PHYS_DT * 1000 / TAU);

// sim_reset drains the tasks it cancels before returning (kResetDrainMs), so a
// command issued after it is in force on the next physics tick. Still MEASURED
// rather than assumed, so the closed forms below stay right if that changes.
let steps = 0;
api.reset(-58, -60, 90);
api.setDrive(9, 9);
while (steps < 6 && st().vel.left === 0) { api.step(1); steps++; }
api.setDrive(0, 0);
// The first nonzero reading arrives on step `steps` and is one tick's worth of
// ramp, so `steps - 1` ticks were integrated with no command.
const latency = steps - 1;
t.check('a drive command reaches the wheels on the tick it is issued',
        latency === 0, `${latency} tick(s) of 1 ms lost after setDrive`);

for (const T of [60, 120, 360]) {
  api.reset(-58, -60, 90);
  api.setDrive(9, 9);
  run(T, 1);
  const v = st().vel.left;
  api.setDrive(0, 0);
  const N = T / (PHYS_DT * 1000) - latency;
  const pred = vInf * (1 - Math.pow(a, N));
  // One tick of the ramp at 0.5 tau is 1.3% of the speed; a 5% error in tau
  // moves it three times that, so this band still sees a wrong time constant.
  t.check(`lag reaches the exponential at t = ${(T / TAU).toFixed(1)} tau`,
          rel(v, pred) < 3e-3,
          `${v.toFixed(2)} vs ${pred.toFixed(2)} in/s over ${N} ticks`);
}

{
  const T = 500;
  const N = T / (PHYS_DT * 1000) - latency;
  api.reset(-58, -60, 90);
  api.setDrive(9, 9);
  run(T, 1);
  const travelled = st().true.x + 58;
  api.setDrive(0, 0);
  const dDisc = vInf * PHYS_DT * (N - a * (1 - Math.pow(a, N)) / (1 - a));
  const dCont = vInf * (T / 1000 - (TAU / 1000) * (1 - Math.exp(-T / TAU)));
  // The gap between integrating with the post-update and the pre-update speed
  // is v_inf * dt = one whole physics step of travel. Keeping the tolerance
  // under half of that is what makes this check able to see the difference.
  t.check('ramp distance matches the closed-form integral',
          Math.abs(travelled - dDisc) < 0.02,
          `travelled ${travelled.toFixed(3)} in, discrete sum ${dDisc.toFixed(3)}, ` +
          `continuous ${dCont.toFixed(3)}, one step = ${(vInf * PHYS_DT).toFixed(3)}`);
}
restore();

// ===========================================================================
// Differential-drive kinematics
// ===========================================================================
t.section('differential kinematics');

// Track width from a pure spin: v_ang = 2v/b, so b = 2v/omega. This is the
// ONLY place b enters, and everything below predicts an independent experiment
// from it.
//
// (0, 0) is a POST -- the robot sat inside its collision envelope, so "a pure
// spin does not translate" was passing because contact forces dDist to zero,
// not because the kinematics are symmetric. Clear space, and assert that.
const CLEAR = [-40, -40, 90];
api.reset(...CLEAR);
api.setDrive(6, -6);
run(1500);
const spin = st();
api.setDrive(0, 0);
const omegaSpin = Math.abs(spin.vel.ang) * DEG;
const trackWidth = 2 * Math.abs(spin.vel.left) / omegaSpin;

const spinDrift = Math.hypot(spin.true.x - CLEAR[0], spin.true.y - CLEAR[1]);
t.check('a pure spin does not translate',
        spinDrift < 0.01 && api.collideTicks() === 0,
        `drifted ${spinDrift.toFixed(4)} in over 1.5 s at ` +
        `${spin.vel.ang.toFixed(1)} deg/s, ${api.collideTicks()} collide ticks`);

// b = 2v/omega is an IDENTITY: world.cpp computes vAng as (right-left)/b from
// the same b, so for a pure spin this recovers whatever b was used and cannot
// detect a wrong one. A 15% error in world.cpp's track width passed all 59
// checks in this file. Close the loop against the drivetrain's own value, which
// reaches here through a different bridge entry point.
const declaredTrack = JSON.parse(api.trajModelJson()).trackWidth;
t.check('the spin-measured track width matches the drivetrain',
        Math.abs(trackWidth - declaredTrack) < 1e-3 * declaredTrack,
        `measured ${trackWidth.toFixed(4)} in vs declared ${declaredTrack.toFixed(4)} in`);

/// Algebraic circle fit: minimise sum (x^2+y^2 + Ax + By + C)^2.
function fitCircle(pts) {
  let m = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  let r = [0, 0, 0];
  for (const [x, y] of pts) {
    const basis = [x, y, 1];
    const rhs = -(x * x + y * y);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) m[i][j] += basis[i] * basis[j];
      r[i] += basis[i] * rhs;
    }
  }
  for (let c = 0; c < 3; c++) {                       // Gaussian elimination
    let p = c;
    for (let i = c + 1; i < 3; i++) if (Math.abs(m[i][c]) > Math.abs(m[p][c])) p = i;
    [m[c], m[p]] = [m[p], m[c]]; [r[c], r[p]] = [r[p], r[c]];
    for (let i = c + 1; i < 3; i++) {
      const f = m[i][c] / m[c][c];
      for (let j = c; j < 3; j++) m[i][j] -= f * m[c][j];
      r[i] -= f * r[c];
    }
  }
  const sol = [0, 0, 0];
  for (let i = 2; i >= 0; i--) {
    let acc = r[i];
    for (let j = i + 1; j < 3; j++) acc -= m[i][j] * sol[j];
    sol[i] = acc / m[i][i];
  }
  const [A, B, C] = sol;
  const cx = -A / 2, cy = -B / 2;
  const R = Math.sqrt(Math.max(0, cx * cx + cy * cy - C));
  let resid = 0;
  for (const [x, y] of pts) resid = Math.max(resid, Math.abs(Math.hypot(x - cx, y - cy) - R));
  return { cx, cy, R, resid };
}

// Asymmetric command -> a circular arc. Predict its radius from the wheel
// speeds and the b measured above; measure it by fitting the traced path.
api.reset(0, -20, 90);
api.setDrive(9, 4);
run(600);                                             // let the lag settle
const arcPts = [];
let unwrapped = 0, prevH = st().true.h;
for (let i = 0; i < 220; i++) {
  api.step(10);
  const s = st();
  arcPts.push([s.true.x, s.true.y]);
  let dh = s.true.h - prevH;
  while (dh > 180) dh -= 360;
  while (dh < -180) dh += 360;
  unwrapped += dh;
  prevH = s.true.h;
}
const arcEnd = st();
api.setDrive(0, 0);
const vL = arcEnd.vel.left, vR = arcEnd.vel.right;
const rPred = 0.5 * trackWidth * (vR + vL) / (vR - vL);
const fit = fitCircle(arcPts);

t.check('arc radius matches the wheel speeds and the measured track width',
        rel(fit.R, Math.abs(rPred)) < 5e-3,
        `fitted R = ${fit.R.toFixed(3)} in, predicted ${Math.abs(rPred).toFixed(3)} in`);
t.check('the path really is a circle',
        fit.resid < 0.02,
        `worst radial residual ${fit.resid.toFixed(4)} in over ${arcPts.length} samples`);
// Concentric wheel arcs: v_out/v_in = (R + b/2)/(R - b/2). Same two
// measurements, a different identity between them.
{
  const ratioMeasured = Math.max(vL, vR) / Math.min(vL, vR);
  const ratioGeom = (fit.R + trackWidth / 2) / (fit.R - trackWidth / 2);
  t.check('inner and outer wheels run concentric arcs',
          rel(ratioMeasured, ratioGeom) < 5e-3,
          `v ratio ${ratioMeasured.toFixed(4)} vs geometric ${ratioGeom.toFixed(4)}`);
}
// Faster right wheel must turn left, i.e. VEX heading decreases.
t.check('the faster wheel is on the outside of the turn',
        (vR > vL) === (unwrapped < 0),
        `right ${vR.toFixed(1)} > left ${vL.toFixed(1)} in/s, ` +
        `VEX heading changed ${unwrapped.toFixed(1)} deg`);

// A constant-curvature path closes. Integrate until the heading has come all
// the way round and require the robot to be back where it started.
api.reset(0, -20, 90);
api.setDrive(9, 4);
{
  let total = 0;
  run(600);                                           // settle before counting
  let prev = st().true.h;
  const afterSettle = st().true;
  for (let i = 0; i < 4000 && Math.abs(total) < 360; i++) {
    api.step(5);
    const h = st().true.h;
    let dh = h - prev;
    while (dh > 180) dh -= 360;
    while (dh < -180) dh += 360;
    total += dh;
    prev = h;
  }
  const end = st().true;
  const gap = Math.hypot(end.x - afterSettle.x, end.y - afterSettle.y);
  api.setDrive(0, 0);
  t.check('a constant-curvature path closes after 360 deg',
          Math.abs(total) >= 360 && gap < 0.01 * 2 * Math.PI * fit.R,
          `turned ${total.toFixed(1)} deg, closed to ${gap.toFixed(3)} in ` +
          `on a ${(2 * Math.PI * fit.R).toFixed(1)} in circumference`);
}
restore();

// ===========================================================================
// Encoders
// ===========================================================================
t.section('encoders');

/// Drive straight from a known pose and split the odometry error into the
/// component along the drive direction and the component across it.
function straightLeg(volts, ms) {
  api.reset(-58, -60, 90);                            // VEX 90 drives along +X
  api.setDrive(volts, volts);
  run(ms);
  api.setDrive(0, 0);
  run(300);
  const s = st();
  return { travel: s.true.x + 58, along: s.odom.x - s.true.x, cross: s.odom.y - s.true.y, s };
}

{
  const leg = straightLeg(6, 2000);
  // Encoder quantum on this base: 0.1 motor deg is
  //   (0.1/360) * (wheel rev per motor rev) * pi * D
  // = 0.0018 in of wheel travel at 0.75 and D = 2.67. Both sides accumulate
  // identical degrees, so the average carries the same bound. 0.02 in is more
  // than ten times it, and small enough that a 0.03% scale error would fail.
  t.check('along-track distance is right to the encoder quantum',
          Math.abs(leg.along) < 0.02,
          `${leg.travel.toFixed(2)} in travelled, odometry is ` +
          `${leg.along.toFixed(4)} in off (${(100 * leg.along / leg.travel).toFixed(4)}%)`);
  t.check('a straight drive produces no cross-track error',
          Math.abs(leg.cross) < 0.01,
          `cross-track ${leg.cross.toFixed(4)} in`);
}

// Slip is defined as the fraction of travel the encoders never see, so
// odometry must come up exactly (1 - s) short ALONG the path and stay straight.
for (const s of [0.05, 0.10, 0.25]) {
  api.setParam('encoderSlip', s);
  const leg = straightLeg(6, 2000);
  const reported = leg.travel + leg.along;
  t.check(`slip ${s} scales reported travel by exactly ${(1 - s).toFixed(2)}`,
          rel(reported / leg.travel, 1 - s) < 2e-3,
          `reported ${reported.toFixed(2)} of ${leg.travel.toFixed(2)} in ` +
          `= ${(reported / leg.travel).toFixed(4)}`);
}
restore();

// A per-side scale error is a pure curvature: with equal commands and
// vR/vL = k, the arc radius is (b/2)(k+1)/(k-1), so over travel d the heading
// must swing d/R radians. Predicted from the measured b, nothing else.
{
  const k = 1.02;
  api.setParam('rightScale', k);
  api.reset(-30, -60, 90);
  api.setDrive(6, 6);
  run(300);                                           // settle the lag
  const s0 = st();
  const p0 = { x: s0.true.x, y: s0.true.y, h: s0.true.h };
  run(1200);
  const s1 = st();
  api.setDrive(0, 0);
  const arcLen = Math.hypot(s1.true.x - p0.x, s1.true.y - p0.y);   // chord
  let dh = s1.true.h - p0.h;
  while (dh > 180) dh -= 360;
  while (dh < -180) dh += 360;
  const R = 0.5 * trackWidth * (k + 1) / (k - 1);
  // chord = 2R sin(theta/2) for a circular arc.
  const chordPred = 2 * R * Math.sin(Math.abs(dh) * DEG / 2);
  t.check('a 2% per-side scale error bends the path by the predicted radius',
          rel(arcLen, chordPred) < 1e-2,
          `turned ${dh.toFixed(2)} deg over a ${arcLen.toFixed(2)} in chord; ` +
          `R = ${R.toFixed(1)} in predicts ${chordPred.toFixed(2)} in`);
}
restore();

// ===========================================================================
// IMU
// ===========================================================================
t.section('IMU');

// The under-report default has to be the inverse of config.cpp's ScaledIMU
// correction, or the library reads a heading bias before the robot has moved --
// and a constant bias b turns every inch driven into 2 sin(b/2) inches of
// cross-track error, which is what a stale hard-coded factor cost here.
for (const h of [0, 45, 90, 137, 180, -90]) {
  api.reset(-20, -20, h);
  run(50);
  const s = st();
  let err = s.odom.h - s.true.h;
  while (err > 180) err -= 360;
  while (err < -180) err += 360;
  t.check(`a reset at heading ${h} has no heading error`,
          Math.abs(err) < 0.02, `off by ${err.toFixed(3)} deg`);
}

// Measure the library's correction factor with the sensor made perfect, then
// predict what a deliberate miscalibration reads. Two experiments, one number.
api.setParam('imuUnderReport', 1.0);
api.reset(0, 0, 100);
run(50);
const scaleFactor = st().odom.h / 100;
t.check('an unscaled sensor exposes the ScaledIMU correction',
        scaleFactor > 0.9 && scaleFactor < 1.1,
        `ScaledIMU factor = ${scaleFactor.toFixed(6)}`);

for (const k of [0.95, 1.03]) {
  api.setParam('imuUnderReport', k);
  api.reset(0, 0, 100);
  run(50);
  t.check(`under-report ${k} reads the predicted heading`,
          near(st().odom.h, 100 * k * scaleFactor, 0.02),
          `${st().odom.h.toFixed(3)} vs ${(100 * k * scaleFactor).toFixed(3)} deg`);
}

// The same factor must apply to a turn, not just to the seeded angle. Both
// headings are unwrapped as they are sampled: a spin crosses +/-180.
{
  const k = 0.95;
  api.setParam('imuUnderReport', k);
  api.reset(0, 0, 90);
  run(50);
  let dTrue = 0, dOdom = 0;
  let pT = st().true.h, pO = st().odom.h;
  const unwrap = (d) => { while (d > 180) d -= 360; while (d < -180) d += 360; return d; };
  api.setDrive(-4, 4);
  for (let i = 0; i < 100; i++) {
    api.step(10);
    const s = st();
    dTrue += unwrap(s.true.h - pT); pT = s.true.h;
    dOdom += unwrap(s.odom.h - pO); pO = s.odom.h;
  }
  api.setDrive(0, 0);
  run(300);
  const s = st();
  dTrue += unwrap(s.true.h - pT);
  dOdom += unwrap(s.odom.h - pO);
  t.check('a turn is scaled by the same factor as the seed',
          rel(dOdom, dTrue * k * scaleFactor) < 3e-3,
          `true ${dTrue.toFixed(2)} deg -> reported ${dOdom.toFixed(2)}, ` +
          `predicted ${(dTrue * k * scaleFactor).toFixed(2)}`);
}
restore();

// Device rate: the bus must hold a stale heading between refreshes, so over a
// steady spin the number of distinct values is the run length over the period.
{
  api.setParam('imuPeriodMs', 50);
  api.reset(0, 0, 90);
  api.setDrive(-6, 6);
  run(200);
  const seen = new Set();
  for (let i = 0; i < 100; i++) { api.step(10); seen.add(st().odom.h.toFixed(2)); }
  api.setDrive(0, 0);
  t.check('IMU holds its value for one device period',
          seen.size >= 18 && seen.size <= 22,
          `${seen.size} distinct headings over 1000 ms at a 50 ms period ` +
          `(expected 20)`);
}
restore();

// ===========================================================================
// Raycaster and the distance-sensor cone
// ===========================================================================
t.section('raycaster geometry');

// Everything below uses beams[0], the front sensor, on geometry built here.
// The sensor's own origin is read back from the beam, so these test the ray
// and the cone rather than re-asserting the mount offsets from config.cpp.
api.setParam('distNoiseMm', 0);

// The mount transform: measure the offset at a heading where the robot frame
// and the world frame coincide, then predict the origin at a rotated heading.
api.setParam('distFovDeg', 0);
api.reset(0, 0, 90);                                  // VEX 90 -> theta = 0
run(60);
const mount = { x: st().beams[0].ox, y: st().beams[0].oy };
api.reset(10, -10, 0);                                // VEX 0 -> theta = +90 deg
run(60);
{
  const b = st().beams[0];
  // Rotating the robot frame by +90 deg: (x, y) -> (-y, x).
  t.check('the mount offset rotates with the robot',
          near(b.ox, 10 - mount.y, 0.02) && near(b.oy, -10 + mount.x, 0.02),
          `mount (${mount.x.toFixed(2)}, ${mount.y.toFixed(2)}) at theta=0 -> ` +
          `(${b.ox.toFixed(2)}, ${b.oy.toFixed(2)}), predicted ` +
          `(${(10 - mount.y).toFixed(2)}, ${(-10 + mount.x).toFixed(2)})`);
}

/// Replace the field with a single vertical wall at x = W.
const oneWall = (W) => {
  api.fieldClear();
  api.fieldAddSegment(W, -68, W, 68);
};

// Head-on against a wall: range is exactly the gap. This is the only place the
// inch/millimetre round trip through the bus is pinned.
oneWall(40);
api.reset(0, 0, 90);
run(120);
{
  const b = st().beams[0];
  t.check('a pencil ray measures a head-on wall exactly',
          near(b.r, 40 - b.ox, 0.05) && near(b.inc, 0, 0.2),
          `sensor at x=${b.ox.toFixed(2)}, reported ${b.r.toFixed(2)} in, ` +
          `expected ${(40 - b.ox).toFixed(2)}, incidence ${b.inc.toFixed(1)} deg`);
}

// Off square by alpha: the slant range is gap/cos(alpha) and the incidence is
// alpha. Both signs, so a flipped surface normal cannot pass.
for (const alpha of [20, -20, 55]) {
  api.reset(0, 0, 90 - alpha);                        // VEX = 90 - alpha_std
  run(120);
  const b = st().beams[0];
  const gap = 40 - b.ox;
  t.check(`a wall at ${alpha} deg incidence measures gap/cos(alpha)`,
          rel(b.r, gap / Math.cos(alpha * DEG)) < 3e-3 &&
          near(b.inc, Math.abs(alpha), 0.3),
          `${b.r.toFixed(2)} in vs ${(gap / Math.cos(alpha * DEG)).toFixed(2)}, ` +
          `incidence ${b.inc.toFixed(1)} deg`);
}

// Range ceiling, in millimetres. A factor-of-25.4 slip in the conversion moves
// this threshold by 25x and both halves of the check fail.
api.setParam('distMaxRangeMm', 1000);
const ceilIn = 1000 / 25.4;
for (const [gap, wantHit] of [[ceilIn - 1.5, true], [ceilIn + 1.5, false]]) {
  api.reset(0, 0, 90);
  run(30);
  oneWall(st().beams[0].ox + gap);
  run(120);
  const b = st().beams[0];
  t.check(`a wall ${gap > ceilIn ? 'past' : 'inside'} the ${ceilIn.toFixed(1)} in ceiling ` +
          `${wantHit ? 'returns' : 'drops out'}`,
          b.hit === wantHit,
          `gap ${gap.toFixed(1)} in -> ${b.hit ? b.r.toFixed(1) + ' in' : 'no return'}`);
}
api.setParam('distMaxRangeMm', DEFAULTS.distMaxRangeMm);

// Incidence dropout. The threshold is a stated model parameter, so bracket it.
oneWall(40);
api.setParam('distMaxIncidenceDeg', 50);
for (const alpha of [44, 56]) {
  api.reset(0, 0, 90 - alpha);
  run(120);
  const b = st().beams[0];
  t.check(`incidence ${alpha} deg ${alpha < 50 ? 'survives' : 'is lost'} at a 50 deg limit`,
          b.hit === (alpha < 50),
          `${b.hit ? 'valid at ' + b.r.toFixed(1) + ' in' : 'dropped'} ` +
          `(incidence ${b.inc.toFixed(1)} deg)`);
}
restore();
api.setParam('distNoiseMm', 0);

// Confidence must fall with range at fixed incidence, and stay in the V5's
// 0-63 register.
{
  const confAt = (gap) => {
    api.reset(0, 0, 90); run(30);
    oneWall(st().beams[0].ox + gap);
    run(120);
    return st().beams[0].conf;
  };
  api.setParam('distFovDeg', 0);
  const cNear = confAt(10), cFar = confAt(60);
  t.check('confidence falls with range and stays inside 0-63',
          cNear > cFar && cFar >= 0 && cNear <= 63,
          `${cNear} at 10 in, ${cFar} at 60 in`);
}

// ---------------------------------------------------------------------------
t.section('cone integration');

// One post, and a sensor aimed straight at its centre from distance L: the
// range is exactly L - r whatever the cone does.
{
  api.fieldClear();
  api.setParam('distFovDeg', 0);
  api.reset(0, 0, 90); run(30);
  const o = st().beams[0];
  api.fieldAddCircle(o.ox + 30, o.oy, 2);
  run(120);
  const b = st().beams[0];
  t.check('a ray aimed at a post centre measures L - r',
          near(b.r, 28, 0.05) && b.obs,
          `${b.r.toFixed(3)} in vs 28.000, isObstacle=${b.obs}`);
}

// A post the axis misses. Hand geometry, with the sensor origin read back:
//   L      distance from the sensor to the post centre
//   phi    angle from the sensor axis to the post centre
//   half   angular half-width of the post, asin(r/L)
// No ray can reach the post unless the cone half-angle exceeds phi - half, and
// any ray that does hit lands between L - r and sqrt(L^2 - r^2).
{
  api.fieldClear();
  api.setParam('distFovDeg', 0);
  api.reset(0, 0, 90); run(30);
  const o = st().beams[0];
  const post = { x: o.ox + 20, y: o.oy - 3.5, r: 2 };
  api.fieldAddCircle(post.x, post.y, post.r);

  const L = Math.hypot(post.x - o.ox, post.y - o.oy);
  const phi = Math.abs(Math.atan2(post.y - o.oy, post.x - o.ox)) / DEG;
  const half = Math.asin(post.r / L) / DEG;
  const need = phi - half;                            // required cone half-angle
  const lo = L - post.r, hi = Math.sqrt(L * L - post.r * post.r);

  const look = (fov) => { api.setParam('distFovDeg', fov); run(120); return st().beams[0]; };

  const pencil = look(0);
  t.check('the axis ray misses a post 3.5 in off centre',
          !pencil.hit,
          `post at ${L.toFixed(2)} in, ${phi.toFixed(2)} deg off axis, ` +
          `half-width ${half.toFixed(2)} deg`);

  const tooNarrow = look(2 * (need - 1.5));
  t.check('a cone narrower than the post\'s angular gap still misses',
          !tooNarrow.hit,
          `FOV ${(2 * (need - 1.5)).toFixed(2)} deg cannot reach ${need.toFixed(2)} deg`);

  const wide = look(40);
  t.check('a wide cone finds the post inside the exact geometric bracket',
          wide.hit && wide.obs && wide.r >= lo - 0.02 && wide.r <= hi + 0.02,
          `FOV 40 -> ${wide.hit ? wide.r.toFixed(2) + ' in' : 'no return'}, ` +
          `bracket [${lo.toFixed(2)}, ${hi.toFixed(2)}]`);
  t.check('the axis range and the timed return disagree, as they must',
          wide.ar > wide.r + 1.0,
          `axis ${wide.ar.toFixed(1)} in, return ${wide.r.toFixed(2)} in`);
}

// Occlusion: a post in front of a wall is what gets timed.
{
  api.fieldClear();
  api.setParam('distFovDeg', 0);
  api.reset(0, 0, 90); run(30);
  const o = st().beams[0];
  api.fieldAddSegment(o.ox + 50, -68, o.ox + 50, 68);
  api.fieldAddCircle(o.ox + 20, o.oy, 2);
  run(120);
  const b = st().beams[0];
  t.check('a post occludes the wall behind it',
          b.obs && near(b.r, 18, 0.05),
          `${b.r.toFixed(2)} in, isObstacle=${b.obs} (wall is at 50 in)`);
}

// Apodisation. A wall fills the whole cone, so the NEAREST ray in it is always
// the one at the cone edge: at incidence alpha with half-FOV h that reads
// gap/cos(alpha - h) instead of gap/cos(alpha). The angular weighting is what
// makes the on-axis return win, and this check is the difference between the
// two -- 10% of the range at 30 deg off square.
{
  oneWall(40);
  const alpha = 30, fov = 25;
  api.setParam('distFovDeg', fov);
  api.reset(0, 0, 90 - alpha);
  run(120);
  const b = st().beams[0];
  const gap = 40 - b.ox;
  const onAxis = gap / Math.cos(alpha * DEG);
  const coneEdge = gap / Math.cos((alpha - fov / 2) * DEG);
  t.check('a slanted wall is measured on axis, not at the nearer cone edge',
          rel(b.r, onAxis) < 5e-3,
          `${b.r.toFixed(2)} in; on-axis ${onAxis.toFixed(2)}, ` +
          `cone edge ${coneEdge.toFixed(2)}`);
}

// Device rate for the distance sensor.
{
  api.fieldDefault();
  restore();
  api.setParam('distNoiseMm', 0);
  api.setParam('distPeriodMs', 50);
  api.reset(20, 0, 90);
  api.setDrive(8, 8);
  run(100);
  const seen = new Set();
  for (let i = 0; i < 100; i++) { api.step(10); seen.add(st().beams[0].r.toFixed(2)); }
  api.setDrive(0, 0);
  t.check('a distance reading is held for one device period',
          seen.size >= 18 && seen.size <= 22,
          `${seen.size} distinct ranges over 1000 ms at a 50 ms period (expected 20)`);
}
restore();
api.fieldDefault();

// ===========================================================================
// Collision
// ===========================================================================
t.section('collision');

/// Drive at full voltage on `heading` until the robot stops, and report how far
/// its centre ended from the wall it hit. Lanes at +/-60 keep the run clear of
/// the goal posts, which sit at 0, +/-23.5 and +/-47.
function wallStop(x0, y0, heading, axis, sign) {
  api.reset(x0, y0, heading);
  api.setDrive(12, 12);
  run(3500);
  api.setDrive(0, 0);
  const s = st();
  const stop = axis === 'x' ? s.true.x : s.true.y;
  return { stop, gap: 70.2 - sign * stop, colliding: s.colliding };
}

// The perimeter EXTENT comes from the field's own geometry export and the disc
// RADIUS from where the robot actually stopped, so the checks below predict one
// experiment from another rather than restating a constant out of world.cpp.
const perimMaxX = Math.max(...JSON.parse(api.fieldJson()).segments
                             .filter((s) => s[4] === 0).flatMap((s) => [s[0], s[2]]));
const discR = perimMaxX - wallStop(-60, -60, 90, 'x', 1).stop;
const postR = JSON.parse(api.fieldJson()).circles[0][2];

{
  const stops = [wallStop(-60, -60,  90, 'x',  1), wallStop( 60, -60, 270, 'x', -1),
                 wallStop(-60, -60,   0, 'y',  1), wallStop(-60,  60, 180, 'y', -1)];
  const gaps = stops.map((s) => s.gap);
  const spread = Math.max(...gaps) - Math.min(...gaps);
  t.check('the robot stops the same distance from all four walls',
          spread < 0.1 && stops.every((s) => s.colliding),
          `gaps ${gaps.map((g) => g.toFixed(3)).join(', ')} in, spread ${spread.toFixed(4)}`);
  // The disc proxy is 0.5*hypot(L, W)*0.5 for the placeholder 15x15 footprint,
  // i.e. 5.303 in. Pinned so a change is visible, not endorsed: it is smaller
  // than the robot's own 7.5 in half-width, so a square robot can put its
  // corners 2.2 in inside a wall. Reported rather than changed, because
  // robotLengthIn/robotWidthIn are guesses, not measurements.
  t.check('the collision proxy is the 5.30 in disc the footprint implies',
          gaps.every((g) => g > 5.25 && g < 5.45),
          `${gaps[0].toFixed(3)} in vs 0.5*hypot(15,15)*0.5 = 5.303`);
}

// A structure added to the geometry has to stop the robot, not just its
// sensors. Before this was wired up, an editor-added barrier blocked every
// distance ray and the robot drove straight through it.
{
  api.fieldDefault();
  api.fieldAddSegment(-20, -68, -20, 68);            // kind 2: user structure
  api.reset(-50, -60, 90);
  api.setDrive(12, 12);
  run(2500);
  api.setDrive(0, 0);
  const blocked = st().true.x;
  t.check('a structure segment stops the robot',
          blocked < -20 && st().colliding,
          `stopped at x = ${blocked.toFixed(2)} with the barrier at -20`);

  api.setParam('collideObstacles', 0);
  api.reset(-50, -60, 90);
  api.setDrive(12, 12);
  run(2500);
  api.setDrive(0, 0);
  t.check('the obstacle switch lets the robot through it',
          st().true.x > -15,
          `reached x = ${st().true.x.toFixed(2)}`);
  restore();
}

// Posts are solid by default and hollow with the switch off.
{
  api.fieldDefault();
  api.reset(-30, 0, 90);                              // aimed at the centre post
  api.setDrive(12, 12);
  run(2500);
  api.setDrive(0, 0);
  const hitPost = st().true.x;
  api.setParam('collideObstacles', 0);
  api.reset(-30, 0, 90);
  api.setDrive(12, 12);
  run(2500);
  api.setDrive(0, 0);
  const through = st().true.x;
  t.check('goal posts are solid unless the switch says otherwise',
          hitPost < -5 && through > 5,
          `stopped at ${hitPost.toFixed(2)} with collision on, ` +
          `${through.toFixed(2)} with it off`);
  restore();
}

// A pose that is ALREADY in penetration has to be able to move out.
// sim_teleport is the kidnap primitive and does not check its destination, and
// the geometry editor can drop a structure on top of the robot. Gating on the
// PROPOSED pose alone rejected every direction forever: the robot was immobile,
// reported zero wheel speed and never recovered.
{
  api.fieldDefault();
  restore();
  api.reset(-40, -40, 90);                            // clear space, facing +X
  run(50);
  api.teleport(0, 0, 90);                             // dead centre of the post
  run(20);
  const pen = postR + discR;                          // depth at the centre
  const inside = st();
  api.setDrive(12, 12);
  const T = 400, tau = DEFAULTS.responseTauMs;
  run(T);                                             // short of the far wall
  api.setDrive(0, 0);
  const out = st();
  const away = Math.hypot(out.true.x, out.true.y);
  // Penetration must not slow the escape either, so the distance covered is the
  // ordinary free-run integral of the lag.
  const free = vMax * (T - tau * (1 - Math.exp(-T / tau))) / 1000;
  t.check('a robot teleported inside a post drives out of it',
          inside.colliding && away > pen && !out.colliding && Math.abs(away - free) < 2,
          `teleported to the post centre, ${pen.toFixed(2)} in deep; clear at ` +
          `${pen.toFixed(2)}, escaped to ${away.toFixed(2)} in in ${T} ms ` +
          `(free run ${free.toFixed(2)} in)`);

  // The same defect outside the perimeter: teleported out of the field, the
  // robot has to be able to drive back in.
  api.reset(-40, -40, 270);
  run(50);
  api.teleport(perimMaxX + 4, -60, 270);              // facing -X
  run(20);
  const depth = (perimMaxX + 4) - (perimMaxX - discR);
  const stranded = st();
  api.setDrive(12, 12);
  run(1500);
  api.setDrive(0, 0);
  const back = st();
  t.check('a robot teleported outside the perimeter drives back in',
          stranded.colliding && back.true.x < perimMaxX - discR && !back.colliding,
          `${depth.toFixed(2)} in outside at x = ${(perimMaxX + 4).toFixed(1)}, ` +
          `recovered to x = ${back.true.x.toFixed(2)} (inside face ` +
          `${(perimMaxX - discR).toFixed(2)})`);
}

// ...and escaping must not become a licence to drive THROUGH geometry. Both of
// these are a max over the whole run, not the final pose: a robot that crossed
// and came back would pass an end-state check.
{
  api.fieldDefault();
  api.reset(-60, -60, 90);
  api.setDrive(12, 12);
  let deepestWall = -Infinity;
  for (let i = 0; i < 400; i++) { api.step(10); deepestWall = Math.max(deepestWall, st().true.x); }
  api.setDrive(0, 0);
  t.check('a clear robot still cannot cross the wall it pins against',
          deepestWall <= perimMaxX - discR + 1e-3,
          `deepest x over 4 s was ${deepestWall.toFixed(4)}, wall face ` +
          `${(perimMaxX - discR).toFixed(4)}`);

  api.reset(-30, 0, 90);                              // head-on at the centre post
  api.setDrive(12, 12);
  let deepestPost = -Infinity;
  for (let i = 0; i < 400; i++) { api.step(10); deepestPost = Math.max(deepestPost, st().true.x); }
  api.setDrive(0, 0);
  t.check('a clear robot still cannot cross a post',
          deepestPost <= -(postR + discR) + 1e-3,
          `deepest x over 4 s was ${deepestPost.toFixed(4)}, post face ` +
          `${(-(postR + discR)).toFixed(4)}`);
}

// Field::clear() has to clear the COLLISION perimeter as well as the drawn one.
// It used to test against a file-scope 70.2, so with no geometry loaded at all
// the robot still stopped dead at x = 64.85.
{
  api.fieldDefault();
  const walled = wallStop(-60, -60, 90, 'x', 1);
  api.fieldClear();
  api.reset(-60, -60, 90);
  api.setDrive(12, 12);
  const T = 4000, tau = DEFAULTS.responseTauMs;
  run(T);
  api.setDrive(0, 0);
  const s = st();
  // Free run: v_inf (T - tau(1 - e^(-T/tau))), off the terminal speed measured
  // at the top of this file.
  const pred = -60 + vMax * (T - tau * (1 - Math.exp(-T / tau))) / 1000;
  t.check('an empty field has no walls, so nothing stops the robot',
          Math.abs(s.true.x - pred) < 1.0 && !s.colliding && api.collideTicks() === 0,
          `free-ran to x = ${s.true.x.toFixed(1)} in vs ${pred.toFixed(1)} predicted; ` +
          `the loaded perimeter stops it at ${walled.stop.toFixed(2)}`);
  api.fieldDefault();
}

// A pinned robot must report what a stalled motor reports -- stopped -- and must
// not fabricate travel from the command. Without this no rest test can tell a
// wall stall from driving, and a motion that grazed a post reported Arrived at a
// pose outside the field.
{
  api.fieldDefault();
  restore();
  api.reset(-60, -60, 90);
  api.setDrive(12, 12);
  run(3500);                                          // pin on the +x wall
  const a = st();
  run(1000);                                          // still commanded 12 V
  const b = st();
  t.check('a pinned robot reports the wheel speed of a stalled motor',
          b.colliding && b.vel.left === 0 && b.vel.right === 0 && b.vel.lin === 0,
          `left ${b.vel.left.toFixed(3)}, right ${b.vel.right.toFixed(3)}, ` +
          `lin ${b.vel.lin.toFixed(3)} in/s at 12 V into the wall`);
  t.check('a pinned robot fabricates no wheel travel',
          Math.abs(b.odom.x - a.odom.x) < 0.02,
          `odometry advanced ${(b.odom.x - a.odom.x).toFixed(4)} in over 1 s; the ` +
          `command alone would have added ${vMax.toFixed(1)} in`);
  // Rotation is what keeps a pinned robot from wedging permanently, so it lives.
  api.setDrive(-12, 12);
  run(400);
  const c = st();
  let dh = c.true.h - b.true.h;
  while (dh > 180) dh -= 360;
  while (dh < -180) dh += 360;
  t.check('a pinned robot can still rotate',
          Math.abs(dh) > 30 && c.vel.left !== 0,
          `turned ${dh.toFixed(1)} deg in 400 ms with the wheels at ` +
          `${c.vel.left.toFixed(1)}/${c.vel.right.toFixed(1)} in/s`);
  api.setDrive(0, 0);
}

// ===========================================================================
// Field geometry
// ===========================================================================
t.section('field geometry');
{
  api.fieldDefault();
  const f = JSON.parse(api.fieldJson());
  const perimeter = f.segments.filter((s) => s[4] === 0);
  const xs = perimeter.flatMap((s) => [s[0], s[2]]);
  const ys = perimeter.flatMap((s) => [s[1], s[3]]);
  const side = Math.max(...xs) - Math.min(...xs);
  // 140.4 in wall to wall is the VRC field spec, and it has to agree with
  // kFieldWalls in mcl_tracking.hpp or MCL mislocalizes against its own map.
  t.check('the perimeter is the 140.4 in VRC square, centred',
          perimeter.length === 4 && near(side, 140.4, 0.05) &&
          near(Math.max(...ys) - Math.min(...ys), 140.4, 0.05) &&
          near(Math.max(...xs) + Math.min(...xs), 0, 0.05),
          `${side.toFixed(2)} x ${(Math.max(...ys) - Math.min(...ys)).toFixed(2)} in, ` +
          `centre (${((Math.max(...xs) + Math.min(...xs)) / 2).toFixed(2)}, ` +
          `${((Math.max(...ys) + Math.min(...ys)) / 2).toFixed(2)})`);
  t.check('every post is a 2 in radius circle inside the perimeter',
          f.circles.length === 9 &&
          f.circles.every((c) => near(c[2], 2, 1e-3) &&
                                 Math.abs(c[0]) < 70.2 && Math.abs(c[1]) < 70.2),
          `${f.circles.length} posts`);
}

// ===========================================================================
// Teleport
// ===========================================================================
t.section('teleport');
{
  api.reset(-40, -40, 90);
  run(200);
  const before = st();
  api.teleport(20, 20, 90);
  run(30);
  const after = st();
  // A kidnap must be invisible to dead reckoning: the encoders and the IMU are
  // untouched, so odometry has to carry on from where it was.
  t.check('a kidnap moves ground truth and not the dead-reckoned pose',
          near(after.true.x, 20, 0.01) && near(after.true.y, 20, 0.01) &&
          Math.hypot(after.odom.x - before.odom.x, after.odom.y - before.odom.y) < 0.05,
          `true -> (${after.true.x.toFixed(1)}, ${after.true.y.toFixed(1)}), ` +
          `odom moved ${Math.hypot(after.odom.x - before.odom.x,
                                   after.odom.y - before.odom.y).toFixed(3)} in`);
}

// ===========================================================================
// Modelled error, held to what the header claims it is
// ===========================================================================
t.section('modelled error, as documented');

// imuNoiseDeg is a random WALK in deg/sqrt(s), not white noise on a reading: it
// is added to the ACCUMULATED rotation, so the error never comes back.
//
// The first check pins the deg/sqrt(s) MAGNITUDE; the second is the one that
// separates a walk from a noisy reading. The increment ratio alone does not:
// measured through odometry rather than the raw bus, white noise on the published
// heading still comes out near 1.9, because the heading-jump rejector filters it.
// So the growth of the error is the discriminator, and it is a factor of 5.5.
{
  const NOISE = 2.0;
  const unwrapDeg = (d) => { while (d > 180) d -= 360; while (d < -180) d += 360; return d; };
  const walk = (windowMs, n = 8) => {
    api.setParam('imuNoiseDeg', NOISE);
    api.reset(-20, -20, 90);
    run(100);
    let prev = unwrapDeg(st().odom.h - st().true.h);
    const inc = [], err = [];
    for (let k = 0; k < n; k++) {
      run(windowMs);
      const e = unwrapDeg(st().odom.h - st().true.h);
      inc.push(e - prev);
      err.push(e);
      prev = e;
    }
    const rms = (a) => Math.sqrt(a.reduce((s, v) => s + v * v, 0) / a.length);
    return { rms: rms(inc), errRms: rms(err), last: prev };
  };
  const w1 = walk(1000), w4 = walk(4000);
  const pred = (ms) => NOISE * scaleFactor * Math.sqrt(ms / 1000);
  const ratio = w4.rms / w1.rms;
  // Eight windows puts the standard error on an RMS at 1/sqrt(16) = 25%, so the
  // band is wide on purpose. White noise on the reading would sit at ratio 1.
  t.check('imuNoiseDeg is a random walk in deg/sqrt(s), not a noisy reading',
          ratio > 1.4 && ratio < 2.7 &&
          w1.rms > 0.6 * pred(1000) && w1.rms < 1.6 * pred(1000),
          `rms increment ${w1.rms.toFixed(2)} deg over 1 s, ${w4.rms.toFixed(2)} over 4 s ` +
          `(predicted ${pred(1000).toFixed(2)} and ${pred(4000).toFixed(2)}); ` +
          `ratio ${ratio.toFixed(2)} against 1.00 for white noise`);
  t.check('the heading error never returns to zero',
          w4.errRms > 3.0,
          `RMS heading error ${w4.errRms.toFixed(2)} deg over 32 s at REST, ` +
          `ending ${w4.last.toFixed(2)} deg out`);
  restore();
}

// encoderSlip is an encoder UNDER-REPORT, not slip: the robot still covers the
// full distance and only odometry comes up short. Real slip on traction wheels
// loses the ground as well, so nothing here stands in for it -- which is why the
// header says so rather than the field name being taken at face value.
{
  api.setParam('encoderSlip', 0);
  const clean = straightLeg(6, 2000);
  api.setParam('encoderSlip', 0.25);
  const slipped = straightLeg(6, 2000);
  const reported = slipped.travel + slipped.along;
  t.check('encoderSlip leaves ground truth untouched',
          Math.abs(slipped.travel - clean.travel) < 0.01 &&
          rel(reported / slipped.travel, 0.75) < 2e-3,
          `true travel ${clean.travel.toFixed(4)} in at slip 0 and ` +
          `${slipped.travel.toFixed(4)} at slip 0.25 (delta ` +
          `${(slipped.travel - clean.travel).toFixed(5)} in); odometry reports ` +
          `${reported.toFixed(2)} in, a factor of ${(reported / slipped.travel).toFixed(4)}`);
  restore();
}

restore();
api.fieldDefault();
process.exit(t.finish());
