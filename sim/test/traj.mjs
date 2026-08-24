// Checks the constrained-trajectory stack: spline -> constraint -> generator ->
// Chassis::followTrajectory. Run with: node sim/test/traj.mjs
//
// The stack had never run outside host unit tests, and the host suite validates
// a generated profile by asking the same constraint objects that shaped it
// whether they are happy (test/host/trajectory_test.cpp, `recheck`). So the
// checks here are written the other way round: reconstruct from physics what
// each state demands of each wheel, and compare it to the battery and to the
// free wheel speed. Everything numeric lives in sim/web/traj.js so the browser
// can run the same audits.
import { loadSim, wrap, tally } from './harness.mjs';
import * as T from '../web/traj.js';

const M = await loadSim(['sim_traj_generate', 'sim_traj_run', 'sim_traj_model_json']);
const api = wrap(M, {
  init: [null, []],
  reset: [null, ['number', 'number', 'number']],
  step: [null, ['number']],
  stateJson: ['string', []],
  setParam: [null, ['string', 'number']],
  collideTicks: ['number', []],

  trajClear: [null, []],
  trajWaypoint: [null, ['number', 'number', 'number', 'number']],
  trajDefaults: [null, []],
  trajSet: [null, ['string', 'number']],
  trajFollowSet: [null, ['string', 'number']],
  trajGenerate: ['number', []],
  trajCount: ['number', []],
  trajStride: ['number', []],
  trajStatesPtr: ['number', []],
  trajResample: ['number', ['number']],
  trajSamplesPtr: ['number', []],
  trajResultJson: ['string', []],
  trajModelJson: ['string', []],
  trajProbeJson: ['string', ['number']],
  trajRun: [null, ['number']],
  trajRunning: ['number', []],
  trajCancel: [null, []],
  trajFollowJson: ['string', []],

  pathClear: [null, []],
  pathPush: [null, ['number', 'number', 'number']],
  routeClear: [null, []],
  routePush: [null, ['number', 'number', 'number', 'number', 'number']],
  routeRun: [null, []],
  routeRunning: ['number', []],
  ltvSetPath: [null, ['string']],
  ltvPathLength: ['number', []],
  ltvRun: [null, ['number', 'number']],
  ltvRunning: ['number', []],
  ltvCancel: [null, []],
  lqrFollow: [null, ['number', 'number', 'number']],
  lqrRunning: ['number', []],
});
api.init();

const t = tally('sim/test/traj.mjs');
const st = () => JSON.parse(api.stateJson());
const model = JSON.parse(api.trajModelJson());

// Physical constants for the audits, taken from the library's own DriveModel so
// the test cannot drift from the robot -- but every FORMULA below is written
// from scratch in sim/web/traj.js.
const BASE = {
  trackWidth: model.trackWidth,
  maxWheelSpeed: model.maxWheelSpeed,
  gripLimit: model.gripLimit,
};
// config.cpp's fitted feedforward. kaStraight is 0 there, so `kA` is the value
// phase 1 is going to measure; the audits sweep it.
const FF = { kS: 0.7020, kV: 6.1287, maxVoltage: 12.0 };

// How far past the rail the generator is allowed to land on its OWN state grid.
// With piecewise-constant acceleration v(t) is linear on each interval, so the
// demand kS*sgn + kV*v(t) + kA*a is monotone there and its supremum is attained
// at an endpoint -- taking the max of the two endpoints is exact, not
// conservative. Measured worst 11.983 / 12.000 / 12.000 V at kA = 0.4 / 0.8 /
// 2.0, so 0.1% is a real bound: pricing the dk/ds term at the state instead of
// over the interval put these at 12.045 and 12.106 V.
const RAIL_SLACK = 0.001;

// Resampled onto a uniform 10 ms grid the bound is looser, and legitimately so.
// A tick that straddles a state boundary has to average two accelerations while
// its kV term is priced at one end of the pair, and neither the generator nor
// the follower is asked for that combination at any instant -- sample() hands
// back the segment's own acceleration. Measured 12.059 V, 15 ticks of 254.
const TICK_SLACK = 0.01;

const PATHS = {
  straight: T.straightWaypoints(-58, -58, -58, -10),
  'gentle S': [
    { x: -58, y: -58, headingDeg: 0 },
    { x: -46, y: -30, headingDeg: 20 },
    { x: -34, y: -6, headingDeg: 0 },
  ],
  'quarter turn': [
    { x: -58, y: -58, headingDeg: 0 },
    { x: -58, y: -24, headingDeg: 0 },
    { x: -30, y: 0, headingDeg: 90 },
  ],
  'wide arc': [
    { x: -50, y: -50, headingDeg: 45 },
    { x: -10, y: -20, headingDeg: 45 },
    { x: 30, y: -50, headingDeg: 135 },
  ],
  // The three above are GENERATION geometry only -- every one of them passes
  // inside the posts' collision envelope (2.0 in post + 5.303 in proxy), 'wide
  // arc' by 1.5 in of the post CENTRE. Harmless for a planner test, fatal for a
  // following one, so anything that drives uses this: up the diagonal gap
  // between (-47,-23.5) and (-23.5,-47), 8.7 in of post margin and 4.9 in of
  // wall margin, and asserted rather than assumed.
  'clear diagonal': [
    { x: -60, y: -52, headingDeg: 40 },
    { x: -34, y: -34, headingDeg: 40 },
    { x: -8, y: -18, headingDeg: 40 },
  ],
};

// The names applyTrajectoryRequest() wants, mapped onto this suite's cwraps.
const trajApi = {
  defaults: api.trajDefaults, clear: api.trajClear, waypoint: api.trajWaypoint,
  set: api.trajSet, followSet: api.trajFollowSet, generate: api.trajGenerate,
};

function gen(waypoints, opts = {}) {
  const req = T.buildTrajectoryRequest(waypoints, opts);
  const code = T.applyTrajectoryRequest(trajApi, req);
  const res = JSON.parse(api.trajResultJson());
  const states = T.readStates(M, api.trajStatesPtr(), api.trajCount(), api.trajStride());
  return { code, res, states };
}

// ===========================================================================
t.section('the drive base the generator reasons about');
// ===========================================================================

// The robot has traction wheels, and the grip term the generator uses is
// derived from that enum by defaultGripLimit(). If config.cpp were switched to
// omni this drops to 60 and every corner speed below changes.
t.check('drivetrain.model() reports traction wheels',
  model.wheelsName === 'traction' && model.wheels === 1,
  `${model.wheelsName} (${model.wheels})`);
t.check('the traction grip limit reaches the generator', model.gripLimit === 200,
  `gripLimit ${model.gripLimit} in/s^2, override ${model.gripLimitOverride}`);
t.check('free wheel speed matches rpm and wheel diameter',
  Math.abs(model.maxWheelSpeed - (model.rpm / 60) * Math.PI * model.wheelDiameter) < 1e-3,
  `${model.maxWheelSpeed.toFixed(4)} in/s`);

// Two independent derivations of the same two ceilings.
{
  let worst = 0;
  for (const k of [0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0]) {
    const p = JSON.parse(api.trajProbeJson(k));
    const mine = T.speedCeilings(k, BASE);
    worst = Math.max(worst,
      Math.abs(p.centripetal - mine.grip) / mine.grip,
      Math.abs(p.kinematics - mine.wheel) / mine.wheel);
  }
  t.check('both constraint classes match a closed form to 1e-5', worst < 1e-5,
    `worst relative ${worst.toExponential(2)}`);
}

// ===========================================================================
t.section('every generated state is physically deliverable');
// ===========================================================================

for (const [name, wps] of Object.entries(PATHS)) {
  const g = gen(wps, { driveConstraints: true, maxAccel: 60 });
  if (!t.check(`${name}: generates`, g.code === 0 && g.states.length >= 10,
      `${g.res.error} n=${g.res.count} T=${g.res.totalTime.toFixed(3)}s`)) continue;

  const a = T.auditWheelVoltages(g.states, { ...BASE, ...FF, kA: 0 });
  t.check(`${name}: no wheel exceeds free speed`,
    a.worstWheelSpeed <= model.maxWheelSpeed * 1.001,
    `peak wheel ${a.worstWheelSpeed.toFixed(2)} of ${model.maxWheelSpeed.toFixed(2)} in/s`);
  t.check(`${name}: kV/kS voltage inside the battery`, a.worstTrue <= 12.0,
    `worst ${a.worstTrue.toFixed(3)} V`);

  const s = T.auditSpeedCeilings(g.states, BASE, g.res.maxVelocity);
  t.check(`${name}: no state above its corner ceiling`, s.worstExcess < 0.05,
    `worst excess ${s.worstExcess.toFixed(4)} in/s`);
}

// generateCubicTrajectory is a separate entry point: only the two end poses
// carry a heading and the interior tangents are solved for.
{
  const g = gen(PATHS['wide arc'], { driveConstraints: true, maxAccel: 60, cubic: true });
  const a = T.auditWheelVoltages(g.states, { ...BASE, ...FF, kA: 0 });
  const c = T.auditCurvature(g.states);
  t.check('the clamped-cubic entry point holds the same physics',
    g.code === 0 && g.states.length >= 10 &&
      a.worstWheelSpeed <= model.maxWheelSpeed * 1.001 &&
      c.worstAbs <= 0.05 * c.peakOmega + 5e-3,
    `n=${g.res.count} T=${g.res.totalTime.toFixed(3)}s peak wheel ` +
    `${a.worstWheelSpeed.toFixed(2)} in/s, curvature residual ${c.worstAbs.toExponential(2)}`);
}

// startVelocity/endVelocity are magnitudes, clamped to maxVelocity.
{
  const rolling = gen(PATHS.straight, { driveConstraints: true, maxAccel: 60, startVel: 20 });
  const clamped = gen(PATHS.straight, { driveConstraints: true, maxAccel: 60, startVel: 500 });
  t.check('startVelocity is honoured and clamped to maxVelocity',
    Math.abs(rolling.states[0].v - 20) < 0.05 &&
      Math.abs(clamped.states[0].v - model.maxWheelSpeed) < 0.05,
    `${rolling.states[0].v.toFixed(3)} in/s, and 500 clamps to ${clamped.states[0].v.toFixed(3)}`);
}

// Without the constraints chassis.hpp tells callers to pass, the same waypoints
// produce states the drivetrain cannot deliver. This is what makes the four
// checks above non-vacuous: the audit CAN reject a trajectory.
{
  const g = gen(PATHS['wide arc'], { driveConstraints: false, maxAccel: 60 });
  const a = T.auditWheelVoltages(g.states, { ...BASE, ...FF, kA: 0 });
  t.check('unconstrained generation IS rejected by the same audit',
    g.code === 0 && a.worstWheelSpeed > model.maxWheelSpeed * 1.05,
    `peak wheel ${a.worstWheelSpeed.toFixed(1)} in/s, ${a.worstTrue.toFixed(1)} V demanded`);
}

// ===========================================================================
t.section('the wheel-voltage constraint against the battery');
// ===========================================================================

// config.cpp ships kaStraight = 0, so this is what a caller adding the
// constraint today actually builds. It used to be a silent no-op -- maxWheelAccelAt()
// returned infinity for kA <= 0, narrowing nothing and warning nobody, which is
// how a trajectory demanding 14.8 V got out. Generation now refuses instead.
{
  const off = gen(PATHS['wide arc'], { driveConstraints: true, maxAccel: 60 });
  const on = gen(PATHS['wide arc'],
    { driveConstraints: true, voltageConstraint: true, kA: 0, maxAccel: 60 });
  t.check('kA = 0 is refused, not silently ignored',
    on.res.error === 'unusableConstraint' && on.res.count === 0,
    `error ${on.res.error}, ${on.res.count} states`);
  t.check('and the refusal is specific to kA, not to the constraint',
    off.res.error === 'none' && off.res.count > 0,
    `without it: ${off.res.error}, ${off.res.count} states, ${off.res.totalTime.toFixed(3)}s`);
}

for (const kA of [0.4, 0.8, 2.0]) {
  const g = gen(PATHS['wide arc'], {
    driveConstraints: true, voltageConstraint: true, kA, maxAccel: 120,
    maxDx: 0.5, maxDy: 0.005, maxDtheta: 0.02,
  });
  if (!t.check(`kA=${kA}: generates`, g.code === 0 && g.states.length >= 50,
      `${g.res.error} n=${g.res.count}`)) continue;
  const a = T.auditWheelVoltages(g.states, { ...BASE, ...FF, kA });

  // Liveness guard for the check below: the profile has to come right up to the
  // rail, or "never over 12 V" proves nothing. It is the TRUE demand that binds
  // now. The rigid model reads HIGHER than 12 here and that is correct -- it
  // drops the v^2 (b/2) dk/ds term, so where dk/ds works against the wheel the
  // rigid figure overstates what the wheel is really asked for.
  t.check(`kA=${kA}: the true demand comes up to the rail (so the next check is live)`,
    a.worstTrue > 11.9 && a.worstTrue <= 12.0 * (1 + RAIL_SLACK),
    `true worst ${a.worstTrue.toFixed(3)} V, rigid model says ${a.worstRigid.toFixed(3)} V`);

  // The demand a real wheel sees. d/dt(v*c) carries a v^2 (b/2) dk/ds term the
  // rigid model drops, and dk/ds is nonzero everywhere on a spline. Before that
  // term was constrained this demanded 14.79 V of a 12 V rail -- 23% over.
  //
  // The bound is the rail plus RAIL_SLACK. The generator prices each interval's
  // dk/ds term at both of its ends, at the speeds the interval actually reaches,
  // so the demand at the far end is the one it was checked for.
  t.check(`kA=${kA}: true wheel demand stays within ${(RAIL_SLACK * 100).toFixed(1)}% of the rail`,
    a.worstTrue <= 12.0 * (1 + RAIL_SLACK),
    `worst ${a.worstTrue.toFixed(3)} V (${((a.worstTrue / 12 - 1) * 100).toFixed(2)}% over), ` +
    `${a.overBudgetTrue} of ${a.rows.length} wheel-states above 12 V, ` +
    `dropped term up to ${a.peakOmitted.toFixed(0)} in/s^2`);

  // The point of the fix: the rigid model, which drops the dk/ds term, is wrong
  // by far more than the sampling slack above. If this stops being true the term
  // has stopped mattering and the constraint is measuring nothing.
  // The gap scales with kA (2.5% at 0.4, 17% at 0.8, 55% at 2.0), so the test is
  // that it is material at all, not that it clears a fixed voltage.
  t.check(`kA=${kA}: the dropped term is what made the difference`,
    a.worstRigid > a.worstTrue * 1.02,
    `rigid ${a.worstRigid.toFixed(2)} V vs true ${a.worstTrue.toFixed(2)} V ` +
    `(${((a.worstRigid / a.worstTrue - 1) * 100).toFixed(1)}% apart)`);

  // Differencing on the state grid uses whatever dt the spline subdivision left
  // behind, so repeat it on a uniform 10 ms grid -- one control tick, the
  // shortest window the drivetrain can be asked to change speed over. If the
  // overage survives that it is not a differentiation artifact.
  if (kA === 0.8) {
    const n = api.trajResample(0.01);
    const rs = T.readStates(M, api.trajSamplesPtr(), n, api.trajStride());
    const a10 = T.auditWheelVoltages(rs, { ...BASE, ...FF, kA });
    t.check(`kA=${kA}: the same holds differenced per 10 ms control tick`,
      a10.worstTrue <= 12.0 * (1 + TICK_SLACK),
      `worst ${a10.worstTrue.toFixed(3)} V over ${n} samples, ` +
      `${a10.overBudgetTrue} wheel-states above 12 V ` +
      `(rigid model says ${a10.worstRigid.toFixed(2)} V)`);
  }
}

// ===========================================================================
t.section('grip is live code, just never the binding term here');
// ===========================================================================

{
  const rows = [];
  for (const grip of [0, 110, 60, 30]) {
    const g = gen(PATHS['gentle S'], { driveConstraints: true, gripLimit: grip, maxAccel: 60 });
    const s = T.auditSpeedCeilings(g.states, { ...BASE, gripLimit: grip || model.gripLimit },
      g.res.maxVelocity);
    rows.push({ grip: grip || model.gripLimit, T: g.res.totalTime, ...s });
  }
  const [own, mixed, omni, low] = rows;
  t.check('at the robot\'s own grip limit, the wheel budget always binds first',
    own.gripBound === 0 && own.wheelBound > 0,
    `grip-bound ${own.gripBound} states, wheel-bound ${own.wheelBound}`);
  t.check('lowering grip makes the centripetal term bind and slows the path',
    omni.gripBound > 0 && omni.T > own.T * 1.05 && low.T > omni.T,
    `T: ${own.T.toFixed(3)}s @${own.grip} -> ${mixed.T.toFixed(3)}s @110 -> ` +
    `${omni.T.toFixed(3)}s @60 -> ${low.T.toFixed(3)}s @30`);
  t.check('peak speed falls monotonically as grip falls',
    own.peakSpeed >= mixed.peakSpeed && mixed.peakSpeed > omni.peakSpeed &&
      omni.peakSpeed > low.peakSpeed,
    rows.map((r) => r.peakSpeed.toFixed(2)).join(' > '));
}

// ===========================================================================
t.section('the time axis agrees with the geometry');
// ===========================================================================

for (const [name, wps] of Object.entries(PATHS)) {
  const g = gen(wps, { driveConstraints: true, maxAccel: 60, maxDx: 1 });
  if (g.states.length < 10) { t.check(`${name}: generates`, false, g.res.error); continue; }

  // Integrating the emitted speeds over the emitted timestamps has to reproduce
  // the arc length of the emitted poses. v is linear in t on a constant-accel
  // segment, so the trapezoid rule is exact and any residual is real.
  const d = T.auditProfileDistance(g.states);
  t.check(`${name}: profile integral matches path arc length`, d.relError < 1e-4,
    `${d.integrated.toFixed(4)} vs ${d.geometric.toFixed(4)} in (${d.relError.toExponential(2)})`);

  const acc = T.auditAcceleration(g.states);
  t.check(`${name}: (dv/dt) matches the stored acceleration`, acc.worst < 0.02,
    `worst ${acc.worst.toExponential(2)} in/s^2`);

  // omega = v * curvature is exactly what followTrajectory feeds forward
  // (chassis.cpp: wRef = vRef * ref.curvature).
  const c = T.auditCurvature(g.states);
  const tol = 0.02 * c.peakOmega + 5e-3;
  t.check(`${name}: heading rate equals v * curvature`, c.worstAbs <= tol,
    `worst ${c.worstAbs.toExponential(2)} rad/s of peak ${c.peakOmega.toFixed(3)}`);
}

// A reversed path negates both velocity and curvature, so the identity has to
// survive the double flip. Waypoint headings say which way the robot FACES.
{
  const wps = [
    { x: -58, y: -20, headingDeg: 0 },
    { x: -46, y: -40, headingDeg: 20 },
    { x: -34, y: -58, headingDeg: 0 },
  ];
  const g = gen(wps, { driveConstraints: true, reversed: true, maxAccel: 60, maxDx: 1 });
  const c = T.auditCurvature(g.states);
  const allBackwards = g.states.every((s) => s.v <= 1e-3);
  t.check('reversed: every velocity is negative', g.states.length >= 10 && allBackwards,
    `n=${g.states.length} vmin ${Math.min(...g.states.map((s) => s.v)).toFixed(2)}`);
  t.check('reversed: heading rate still equals v * curvature',
    c.worstAbs <= 0.02 * c.peakOmega + 5e-3,
    `worst ${c.worstAbs.toExponential(2)} rad/s of peak ${c.peakOmega.toFixed(3)}`);
}

// ===========================================================================
t.section('Trajectory::sample() is what the follower actually sees');
// ===========================================================================

{
  const g = gen(PATHS['gentle S'], { driveConstraints: true, maxAccel: 60 });
  const n = api.trajResample(0.005);
  const rs = T.readStates(M, api.trajSamplesPtr(), n, api.trajStride());
  t.check('resampling produced samples', n > 100, `${n} samples over ${g.res.totalTime.toFixed(3)}s`);

  // interpolate() claims the pose is placed by ARC LENGTH along the connecting
  // arc. Chord placement would be off by the segment sagitta.
  let worstArc = 0;
  for (const s of rs) {
    let j = 0;
    while (j + 1 < g.states.length && g.states[j + 1].t <= s.t) j++;
    if (j + 1 >= g.states.length) continue;
    const dt = s.t - g.states[j].t;
    if (dt <= 1e-9) continue;
    const expect = Math.abs(g.states[j].v * dt + 0.5 * g.states[j].a * dt * dt);
    worstArc = Math.max(worstArc, Math.abs(T.arcLengthBetween(g.states[j], s) - expect));
  }
  t.check('mid-segment samples are placed by arc length', worstArc < 1e-3,
    `worst ${worstArc.toExponential(2)} in`);

  const d = T.auditProfileDistance(rs);
  t.check('the resampled profile covers the same distance', d.relError < 1e-4,
    `${d.integrated.toFixed(3)} vs ${d.geometric.toFixed(3)} in`);
}

// ===========================================================================
t.section('failure modes report rather than truncate');
// ===========================================================================

const failCases = [
  ['no waypoints', [], {}, 'noWaypoints'],
  ['zero acceleration', PATHS.straight, { maxAccel: 0 }, 'badConfig'],
  ['turn in place', [{ x: 0, y: 0, headingDeg: 0 }, { x: 0, y: 0, headingDeg: 90 }], {},
    'degenerateWaypoints'],
  ['17 waypoints', Array.from({ length: 17 }, (_, i) => ({ x: i * 6, y: 0, headingDeg: 0 })), {},
    'tooManyWaypoints'],
  ['no speed allowed', PATHS.straight, { wheelSpeedLimit: 1e-4 }, 'stalledProfile'],
];
for (const [label, wps, opts, expect] of failCases) {
  const g = gen(wps, { driveConstraints: true, ...opts });
  t.check(`${label} -> ${expect}, and an empty trajectory`,
    g.res.error === expect && g.res.count === 0, `got ${g.res.error} n=${g.res.count}`);
}

// Refining the sampling tolerance cannot make a feasible path infeasible: more
// samples of the same geometry is strictly more information. A 48 in straight
// line at 63 in/s and 60 in/s^2 used to generate at maxDx = 0.5 and fail at
// 0.25, because `(v1^2 - v0^2) / (2 ds)` cancels catastrophically in float32:
// its noise floor is about eps*v^2/(2 ds), which at v ~ 53 in/s and ds ~ 0.19 in
// reaches ACCEL_CONVERGE_EPS and pins the fixed-point loop until it burns all
// 32 iterations. Swept, because the failure scales with v^2 and not with the
// geometry, so one tolerance proves nothing.
for (const maxDx of [1, 0.5, 0.25, 0.125]) {
  const g = gen(PATHS.straight, { driveConstraints: true, maxAccel: 60, maxDx });
  t.check(`a straight line generates at maxDx = ${maxDx} in`,
    g.code === 0 && g.res.count > 0, `${g.res.error} n=${g.res.count}`);
}

// ===========================================================================
t.section('the audits themselves can fail');
// ===========================================================================

// Every audit above is only worth its output if it rejects a wrong trajectory.
// These feed each one a deliberately corrupted copy of a good result.
{
  const g = gen(PATHS['gentle S'], { driveConstraints: true, maxAccel: 60, maxDx: 1 });
  const flipK = g.states.map((s) => ({ ...s, k: -s.k }));
  const c = T.auditCurvature(flipK);
  t.check('curvature audit rejects a sign-flipped curvature',
    c.worstAbs > 0.02 * c.peakOmega + 5e-3, `worst ${c.worstAbs.toFixed(4)} rad/s`);

  const fastV = g.states.map((s) => ({ ...s, v: s.v * 1.02 }));
  t.check('distance audit rejects a 2% velocity error',
    T.auditProfileDistance(fastV).relError > 1e-4,
    `rel ${T.auditProfileDistance(fastV).relError.toExponential(2)}`);

  const oneFast = g.states.map((s, i) => (i === (g.states.length >> 1) ? { ...s, v: s.v * 1.2 } : s));
  t.check('wheel-speed audit rejects one over-speed state',
    T.auditWheelVoltages(oneFast, { ...BASE, ...FF, kA: 0 }).worstWheelSpeed >
      model.maxWheelSpeed * 1.001,
    `${T.auditWheelVoltages(oneFast, { ...BASE, ...FF, kA: 0 }).worstWheelSpeed.toFixed(2)} in/s`);

  const shifted = g.states.map((s, i) => (i > 0 ? { ...s, a: s.a + 5 } : s));
  t.check('acceleration audit rejects a 5 in/s^2 offset',
    T.auditAcceleration(shifted).worst > 0.02,
    `worst ${T.auditAcceleration(shifted).worst.toFixed(3)}`);
}

// ===========================================================================
t.section('following the trajectory against the world');
// ===========================================================================

function prep(states) {
  api.trajCancel();
  api.reset(states[0].x, states[0].y, 90 - (states[0].theta * 180) / Math.PI);
  // Obstacles stay ON: with them off, `collideTicks() === 0` asserts nothing,
  // because Field::collides returns after the perimeter test and every path
  // here sits well inside that box.
  api.setParam('collideObstacles', 1);
  for (let i = 0; i < 40; i++) api.step(10);
}

function drive(states, isRunning, limitMs = 9000) {
  let ms = 0, peak = 0, sum = 0, n = 0;
  for (let i = 0; i < 5; i++) { api.step(10); ms += 10; }
  while (isRunning() && ms < limitMs) {
    api.step(10); ms += 10;
    const s = st();
    const e = T.crossTrack(states, s.true.x, s.true.y);
    peak = Math.max(peak, e); sum += e; n++;
  }
  const s = st();
  const last = states[states.length - 1];
  let dh = ((90 - s.true.h) * Math.PI) / 180 - last.theta;
  while (dh > Math.PI) dh -= 2 * Math.PI;
  while (dh < -Math.PI) dh += 2 * Math.PI;
  return {
    s: ms / 1000, peak, mean: n ? sum / n : 0, tasks: s.tasks,
    endErr: Math.hypot(s.true.x - last.x, s.true.y - last.y),
    endHeadDeg: Math.abs(dh) * 180 / Math.PI,
  };
}

const FOLLOW = { ffKv: 6.1287, ffKs: 0.702, ffKp: 13, targetTol: 1.5, headingTol: 6 };
const runs = {};
for (const name of ['clear diagonal']) {
  const g = gen(PATHS[name], { driveConstraints: true, maxAccel: 60, follow: FOLLOW });
  prep(g.states);
  api.trajRun(8000);
  const r = drive(g.states, () => api.trajRunning() === 1);
  const f = JSON.parse(api.trajFollowJson());
  api.trajCancel();
  runs[name] = { plan: g.res.totalTime, ...r, outcome: f.outcome };
  t.check(`${name}: followed to the end`,
    f.outcome === 'arrived' && r.endErr < 1.5 && r.peak < 1.5,
    `mean ${r.mean.toFixed(3)} peak ${r.peak.toFixed(2)} end ${r.endErr.toFixed(2)} in, ` +
    `${r.endHeadDeg.toFixed(1)} deg, ${r.s.toFixed(2)}s`);
  // The reference runs on the clock, so the run should take the planned time.
  t.check(`${name}: ran for the planned duration`,
    Math.abs(r.s - g.res.totalTime) < 0.15,
    `${r.s.toFixed(2)}s vs planned ${g.res.totalTime.toFixed(2)}s`);
  t.check(`${name}: never touched field geometry`, api.collideTicks() === 0,
    `${api.collideTicks()} collide ticks`);
}

// useVelocityController is what makes the trajectory's speeds mean anything.
// Open loop, voltage is treated as speed and the same path tracks worse.
{
  const name = 'clear diagonal';
  const g = gen(PATHS[name], {
    driveConstraints: true, maxAccel: 60, follow: { ...FOLLOW, velCtrl: 0 },
  });
  prep(g.states);
  api.trajRun(8000);
  const r = drive(g.states, () => api.trajRunning() === 1);
  api.trajCancel();
  t.check('the velocity controller measurably improves tracking',
    r.mean > runs[name].mean * 1.5,
    `open loop mean ${r.mean.toFixed(3)} in vs closed ${runs[name].mean.toFixed(3)} in`);
}

// timeScale stretches the same geometry in time without regenerating.
{
  const g = gen(PATHS['clear diagonal'], {
    driveConstraints: true, maxAccel: 60,
    follow: { ...FOLLOW, timeScale: 2, holdAtEnd: 0 },
  });
  prep(g.states);
  api.trajRun(9000);
  const r = drive(g.states, () => api.trajRunning() === 1, 11000);
  api.trajCancel();
  t.check('timeScale 2 takes twice as long over the same path',
    Math.abs(r.s - 2 * g.res.totalTime) < 0.3 && r.peak < 1.5,
    `${r.s.toFixed(2)}s vs 2x${g.res.totalTime.toFixed(2)}s, peak ${r.peak.toFixed(2)} in`);
}

// Reversed, driven.
{
  const wps = [
    { x: -58, y: -20, headingDeg: 0 },
    { x: -46, y: -40, headingDeg: 20 },
    { x: -34, y: -58, headingDeg: 0 },
  ];
  const g = gen(wps, { driveConstraints: true, reversed: true, maxAccel: 60, follow: FOLLOW });
  prep(g.states);
  api.trajRun(8000);
  const r = drive(g.states, () => api.trajRunning() === 1);
  const f = JSON.parse(api.trajFollowJson());
  api.trajCancel();
  t.check('a reversed trajectory drives backwards onto its endpoint',
    f.outcome === 'arrived' && r.endErr < 1.5 && r.peak < 1.5,
    `mean ${r.mean.toFixed(3)} peak ${r.peak.toFixed(2)} end ${r.endErr.toFixed(2)} in, ${r.s.toFixed(2)}s`);
}

// holdAtEnd promises to keep correcting until the tolerances are met. It used
// not to: every term of the RAMSETE law is multiplied by the reference
// velocity, and the reference is parked at v = 0 once the clock runs out, so
// k = 2 zeta sqrt(w^2 + b v^2) = 0 and both outputs were identically zero --
// the hold only delayed the exit and downgraded the outcome. The follower now
// hands over to the settle PIDs once the reference is spent.
{
  const opts = { driveConstraints: true, maxAccel: 60 };
  const tight = { ffKv: 6.1287, ffKs: 0.702, ffKp: 13, targetTol: 0.25, headingTol: 4 };
  const out = {};
  for (const hold of [1, 0]) {
    const g = gen(PATHS['clear diagonal'], { ...opts, follow: { ...tight, holdAtEnd: hold } });
    prep(g.states);
    api.trajRun(8000);
    const r = drive(g.states, () => api.trajRunning() === 1);
    out[hold] = { ...r, outcome: JSON.parse(api.trajFollowJson()).outcome, plan: g.res.totalTime };
    api.trajCancel();
  }
  t.check('holdAtEnd does spend extra time trying',
    out[1].s > out[0].s + 0.2, `${out[1].s.toFixed(2)}s vs ${out[0].s.toFixed(2)}s`);
  t.check('holdAtEnd actually reduces the final error, and still arrives',
    out[1].endErr < out[0].endErr - 0.05 && out[1].outcome === 'arrived',
    `hold: ${out[1].endErr.toFixed(3)} in / ${out[1].outcome}; ` +
    `no-hold: ${out[0].endErr.toFixed(3)} in / ${out[0].outcome}`);
}

// ===========================================================================
t.section('what the hold can close, and what it cannot');
// ===========================================================================

// A 12 in straight in the same clear gap: too short for the follower to remove
// a start offset, so a residual survives to the hold.
const SHORT = [
  { x: -40, y: -40, headingDeg: 40 },
  { x: -34.4, y: -33.3, headingDeg: 40 },
];

// Start the robot offset from the trajectory's first pose, in that pose's frame.
// A teleport would not do: the estimator rejects the jump, the controller never
// sees the residual, and the hold has nothing to close.
function prepOffset(states, dAlong, dPerp, dHeadDeg) {
  api.trajCancel();
  const f = states[0];
  const c = Math.cos(f.theta), sn = Math.sin(f.theta);
  api.reset(f.x + dAlong * c - dPerp * sn, f.y + dAlong * sn + dPerp * c,
            90 - (f.theta * 180) / Math.PI + dHeadDeg);
  api.setParam('collideObstacles', 1);
  for (let i = 0; i < 40; i++) api.step(10);
}

/// Drive to completion and split the residual along the final pose's axes.
/// Only the feedforward is set: the tolerances under test are the SHIPPED ones,
/// and FOLLOW above loosens targetTol to 1.5 in, which is above the error the
/// follower leaves and so switches the hold off.
const FF_ONLY = { ffKv: 6.1287, ffKs: 0.702, ffKp: 13 };
function residual(waypoints, follow, offset, limitMs = 9000) {
  const g = gen(waypoints, { driveConstraints: true, maxAccel: 60,
                             follow: { ...FF_ONLY, ...follow } });
  if (g.code !== 0) return { err: g.res.error };
  const last = g.states[g.states.length - 1];
  prepOffset(g.states, ...offset);
  const planMs = g.res.totalTime * 1000;
  api.trajRun(8000);
  let ms = 0;
  while (api.trajRunning() === 1 && ms < limitMs) { api.step(10); ms += 10; }
  const q = st().true;
  const f = JSON.parse(api.trajFollowJson());
  api.trajCancel();
  const ex = q.x - last.x, ey = q.y - last.y;
  const c = Math.cos(last.theta), sn = Math.sin(last.theta);
  return {
    outcome: f.outcome, holdMs: Math.max(0, ms - planMs),
    along: ex * c + ey * sn, perp: -ex * sn + ey * c,
    err: Math.hypot(ex, ey), collide: api.collideTicks(),
  };
}

// The shipped tolerance has to be tight enough that holdAtEnd engages at all.
// It was 1.0 in, and the follower already lands inside 0.79, so the hold ran for
// 0 ms and the feature was inert at its own default.
{
  const r = residual(PATHS['clear diagonal'], {}, [0, 0, 0]);
  t.check('the default tolerance actually engages the hold',
    r.holdMs > 150 && r.err < 0.35 && r.outcome === 'arrived' && !r.collide,
    `held ${Math.round(r.holdMs)} ms, finished ${r.err.toFixed(2)} in out, ${r.outcome}`);
}

// A purely perpendicular residual is the case the along-axis arrival test exists
// for. straightDemand projects onto the final heading, so the hold has no
// authority over the perpendicular part at all -- correcting it needs a
// turn-drive-turn, which is not something to do behind the caller's back. What
// must hold: the along part closes, the outcome is an arrival rather than a
// Stalled, and the perpendicular part is still THERE and not quietly dropped.
for (const side of [+4, -4]) {
  const r = residual(SHORT, {}, [0, side, 0]);
  t.check(`a ${side > 0 ? '+' : '-'}${Math.abs(side)} in perpendicular residual: ` +
          'along closes, perpendicular is reported not lost',
    r.outcome === 'arrived' && Math.abs(r.along) <= 0.35 &&
    Math.abs(r.perp) > 1.0 && Math.sign(r.perp) === Math.sign(side) && !r.collide,
    `${r.outcome}: along ${r.along.toFixed(2)} in, perp ${r.perp.toFixed(2)} in, ` +
    `held ${Math.round(r.holdMs)} ms`);
}

// The floor under targetTolerance. Below it the drivetrain drops through
// stallSpeed before the tolerance is met and every arrival becomes a Stalled --
// at the SAME final error, so it is the report that changes, not the outcome.
{
  const ok = residual(PATHS['clear diagonal'], { targetTol: 0.1 }, [0, 0, 0]);
  const tooTight = residual(PATHS['clear diagonal'], { targetTol: 0.05 }, [0, 0, 0]);
  t.check('0.1 in is inside what the hold can reach',
    ok.outcome === 'arrived' && ok.err < 0.15,
    `${ok.outcome} at ${ok.err.toFixed(3)} in after ${Math.round(ok.holdMs)} ms`);
  t.check('0.05 in is not, and says so rather than pretending',
    tooTight.outcome === 'stalled' && Math.abs(tooTight.err - ok.err) < 0.1,
    `${tooTight.outcome} at ${tooTight.err.toFixed(3)} in after ` +
    `${Math.round(tooTight.holdMs)} ms, against ${ok.err.toFixed(3)} in at 0.1`);
}

// maxSpeed clips the FOLLOWER, not the reference. The trajectory was generated
// against the whole wheel budget, so a ceiling under it saturates every tick and
// the path is not tracked -- and the residual is perpendicular, which the hold
// cannot close, so a run that ends 8 in off still reports arrived. timeScale is
// the knob for driving the same geometry slower.
{
  const clipped = residual(PATHS['clear diagonal'], { maxSpeed: 40 }, [0, 0, 0]);
  const scaled = residual(PATHS['clear diagonal'], { timeScale: 2 }, [0, 0, 0], 12000);
  t.check('clipping maxSpeed does not slow the run, it stops tracking it',
    Math.abs(clipped.perp) > 2.0,
    `maxSpeed 40 finished ${clipped.err.toFixed(2)} in out ` +
    `(${clipped.perp.toFixed(2)} in perpendicular), outcome ${clipped.outcome}`);
  t.check('timeScale is the knob that does slow it down',
    scaled.err < 0.5 && scaled.outcome === 'arrived' && !scaled.collide,
    `timeScale 2 finished ${scaled.err.toFixed(2)} in out, ${scaled.outcome}`);
}

// ===========================================================================
t.section('against the other followers, same geometry');
// ===========================================================================

for (const name of ['clear diagonal']) {
  const g = gen(PATHS[name], { driveConstraints: true, maxAccel: 60, follow: FOLLOW });
  const poses = T.posesForPathBuffer(g.states);

  // RAMSETE, matched as closely as its parameters allow: same output stage,
  // same acceleration ceiling, and its lateral limit raised from the omni
  // default of 60 to this base's actual 200.
  api.pathClear();
  for (const p of poses) api.pathPush(p.x, p.y, p.headingDeg);
  for (const [k, v] of Object.entries({
    ramseteVelCtrl: 1, ramseteFfKv: 6.1287, ramseteFfKs: 0.702, ramseteFfKp: 13,
    ramseteMaxAccel: 60, ramseteLatAccel: model.gripLimit,
    ramseteTolerance: 1.5, ramseteHeadingTol: 6, ramseteB: 48,
  })) api.setParam(k, v);
  prep(g.states);
  api.routeClear();
  api.routePush(13, 0, poses.length, 0, 8000);   // OP_FOLLOW_PATH
  api.routeRun();
  const ram = drive(g.states, () => api.routeRunning() === 1);

  // Speed-scheduled LQR on the same pose list.
  prep(g.states);
  api.lqrFollow(0, poses.length, 8000);
  const lqr = drive(g.states, () => api.lqrRunning() === 1);

  // LTV, fed THIS trajectory's own 10 ms samples, so the comparison is between
  // controllers and not between profiles.
  const n = api.trajResample(0.01);
  const rs = T.readStates(M, api.trajSamplesPtr(), n, api.trajStride());
  api.ltvSetPath(T.ltvBlob(rs));
  const blobLen = api.ltvPathLength();
  prep(g.states);
  api.ltvRun(0, 0);
  const ltv = drive(g.states, () => api.ltvRunning() === 1);
  api.ltvCancel();

  const traj = runs[name];
  console.log(`  ${name}: plan ${traj.plan.toFixed(2)}s over ${T.pathArcLength(g.states).toFixed(1)} in`);
  for (const [lbl, r] of [['followTrajectory', traj], ['RAMSETE', ram], ['LQR', lqr], ['LTV', ltv]]) {
    console.log(`    ${lbl.padEnd(16)} ${r.s.toFixed(2)}s  mean ${r.mean.toFixed(3)}  ` +
      `peak ${r.peak.toFixed(2)}  end ${r.endErr.toFixed(2)} in  ${r.endHeadDeg.toFixed(1)} deg`);
  }

  // The LTV blob is the only place a unit error would be silent: getPathLength
  // parses metres back out, so it has to agree with the inches that went in.
  t.check(`${name}: LTV blob round-trips the path length`,
    Math.abs(blobLen - T.pathArcLength(g.states)) < 0.5,
    `${blobLen.toFixed(2)} vs ${T.pathArcLength(g.states).toFixed(2)} in`);

  // Structural, not a race: followTrajectory and LTV both index their reference
  // by the clock, so both run for the planned duration. RAMSETE and the
  // scheduled LQR advance on the robot's progress, so theirs is emergent -- the
  // table above is the only place it is visible.
  t.check(`${name}: both clock-driven followers run for the planned duration`,
    Math.abs(traj.s - traj.plan) < 0.15 && Math.abs(ltv.s - traj.plan) < 0.15,
    `plan ${traj.plan.toFixed(2)}s, traj ${traj.s.toFixed(2)}s, LTV ${ltv.s.toFixed(2)}s, ` +
    `RAMSETE ${ram.s.toFixed(2)}s, LQR ${lqr.s.toFixed(2)}s`);

  // A regression net over the whole comparison surface: whatever the ranking,
  // all four have to stay on the path they were handed.
  t.check(`${name}: all four followers stay on the path`,
    [traj, ram, lqr, ltv].every((r) => r.mean < 0.5 && r.peak < 1.5) &&
      api.collideTicks() === 0,
    [traj, ram, lqr, ltv].map((r) => r.mean.toFixed(3)).join(' / ') + ' mean in');
}

// A leaked follower fiber would keep driving the motors into the next run.
{
  const before = st().tasks;
  const g = gen(PATHS.straight, { driveConstraints: true, maxAccel: 60, follow: FOLLOW });
  for (let i = 0; i < 3; i++) {
    prep(g.states);
    api.trajRun(4000);
    drive(g.states, () => api.trajRunning() === 1, 5000);
    api.trajCancel();
  }
  for (let i = 0; i < 20; i++) api.step(10);
  t.check('repeated runs leak no fibers', st().tasks <= before,
    `${before} tasks before, ${st().tasks} after three runs`);
}

process.exit(t.finish());
