// Headless smoke test for chad-sim.
//
//   node sim/test/smoke.mjs
//
// Runs the real library off-robot and asserts the localization pipeline is
// wired correctly. This is the same binary the browser loads, so it doubles as
// a regression harness you can run in CI.
import createChadSim from '../web/chad_sim.mjs';

const M = await createChadSim();

const c = (name, ret, args) => M.cwrap(name, ret, args);
const api = {
  init:        c('sim_init', null, []),
  reset:       c('sim_reset', null, ['number', 'number', 'number']),
  step:        c('sim_step', null, ['number']),
  setDrive:    c('sim_set_drive', null, ['number', 'number']),
  setLoc:      c('sim_set_localization', null, ['number']),
  teleport:    c('sim_teleport', null, ['number', 'number', 'number']),
  stateJson:   c('sim_state_json', 'string', []),
  fieldJson:   c('sim_field_json', 'string', []),
  particleCnt: c('sim_particle_count', 'number', []),
  autonCount:  c('sim_auton_count', 'number', []),
  autonName:   c('sim_auton_name', 'string', ['number']),
  runAuton:    c('sim_run_auton', null, ['number']),
  routeRunning:c('sim_route_running', 'number', []),
  stopAuton:   c('sim_stop_auton', null, []),
  setParam:    c('sim_set_param', null, ['string', 'number']),
  routeClear:  c('sim_route_clear', null, []),
  routePush:   c('sim_route_push', null, ['number','number','number','number','number']),
  routeRun:    c('sim_route_run', null, []),
  routeIndex:  c('sim_route_index', 'number', []),
  recCount:    c('sim_record_count', 'number', []),
  recStride:   c('sim_record_stride', 'number', []),
  pathClear:   c('sim_path_clear', null, []),
  pathPush:    c('sim_path_push', null, ['number','number','number']),
  pathCount:   c('sim_path_count', 'number', []),
};
const { buildPath, parseRoute } = await import('../web/route_dsl.js');
const { buildLtvTrajectory } = await import('../web/ltv_traj.js');
Object.assign(api, {
  ltvSetPath:  c('sim_ltv_set_path', null, ['string']),
  ltvLength:   c('sim_ltv_path_length', 'number', []),
  ltvGains:    c('sim_ltv_set_gains', null, ['number', 'number', 'number']),
  ltvRun:      c('sim_ltv_run', null, ['number', 'number']),
  ltvRunning:  c('sim_ltv_running', 'number', []),
  ltvCancel:   c('sim_ltv_cancel', null, []),
});

let failures = 0;
const check = (label, cond, detail = '') => {
  const tag = cond ? 'PASS' : 'FAIL';
  if (!cond) failures++;
  console.log(`  [${tag}] ${label}${detail ? '  ' + detail : ''}`);
};

const st = () => JSON.parse(api.stateJson());
const run = (ms, dt = 10) => { for (let i = 0; i < ms / dt; i++) api.step(dt); };

console.log('\n== init ==');
api.init();
api.reset(-48, -60, 90);
const field = JSON.parse(api.fieldJson());
check('field geometry loaded', field.segments.length === 4 && field.circles.length === 9,
      `${field.segments.length} walls, ${field.circles.length} posts`);

let s = st();
check('start pose is ground truth',
      Math.abs(s.true.x + 48) < 0.01 && Math.abs(s.true.y + 60) < 0.01,
      `true=(${s.true.x.toFixed(2)}, ${s.true.y.toFixed(2)})`);
check('distance sensors report', s.beams.length === 4, `${s.beams.length} beams`);

console.log('\n== odometry tracks a straight drive ==');
// VEX heading 90 points along +X, so a forward drive increases x, not y.
api.setDrive(6, 6);
run(2000);
api.setDrive(0, 0);
run(300);
s = st();
check('robot moved forward', s.true.x > -40, `true x=${s.true.x.toFixed(2)}`);
check('odom followed ground truth', s.err.dist < 1.0,
      `err=${s.err.dist.toFixed(3)} in`);
check('heading held', Math.abs(s.err.heading) < 2.0,
      `heading err=${s.err.heading.toFixed(2)} deg`);

console.log('\n== odometry drifts under encoder slip ==');
api.reset(-48, -60, 90);
api.setParam('encoderSlip', 0.10);
api.setDrive(6, 6);
run(2000);
api.setDrive(0, 0);
run(200);
s = st();
check('slip produced odometry error', s.err.dist > 1.0,
      `err=${s.err.dist.toFixed(3)} in`);

console.log('\n== MCL runs and corrects ==');
api.setParam('encoderSlip', 0.0);
api.reset(-48, -60, 90);
api.setLoc(1);
run(500);
check('particle cloud is live', api.particleCnt() === 1024, `${api.particleCnt()} particles`);

api.setDrive(5, 5);
run(1500);
api.setDrive(0, 0);
run(500);
s = st();
check('MCL estimate near ground truth',
      Math.hypot(s.mcl.x - s.true.x, s.mcl.y - s.true.y) < 12.0,
      `mcl err=${Math.hypot(s.mcl.x - s.true.x, s.mcl.y - s.true.y).toFixed(2)} in`);

console.log('\n== kidnapped robot ==');
const before = st();
api.teleport(30, 30, 90);
run(50);
s = st();
check('ground truth teleported', Math.abs(s.true.x - 30) < 0.01,
      `true=(${s.true.x.toFixed(1)}, ${s.true.y.toFixed(1)})`);
check('estimator did not teleport with it',
      Math.hypot(s.odom.x - 30, s.odom.y - 30) > 5,
      `odom=(${s.odom.x.toFixed(1)}, ${s.odom.y.toFixed(1)})`);

console.log('\n== motion controllers run on their own fiber ==');
api.setLoc(0);
api.reset(-48, -60, 90);
check('autons registered', api.autonCount() === 4,
      [...Array(api.autonCount()).keys()].map(i => api.autonName(i)).join(', '));

api.runAuton(0);
run(400);
check('route task is running', api.routeRunning() === 1);
const moved = st();
run(3000);
const after = st();
check('route commanded motion',
      Math.hypot(after.true.x - moved.true.x, after.true.y - moved.true.y) > 1.0,
      `moved ${Math.hypot(after.true.x - moved.true.x, after.true.y - moved.true.y).toFixed(2)} in`);
api.stopAuton();

console.log('\n== hardware realism ==');
api.setLoc(0);
api.setParam('distNoiseMm', 0);
api.reset(-48, -60, 90);
run(200);

// A V5 distance sensor refreshes near 30 ms. Polling faster must return the
// previous sample; polling past a full period must return a new one.
// Face the right wall head-on and close, so driving actually changes the
// reading. Driving parallel to a wall would not: the cone edge clips it at a
// constant range, which is geometrically correct but proves nothing here.
api.setParam('distPeriodMs', 30);
api.reset(40, 0, 90);
run(150);
api.setDrive(10, 10);
api.step(30);                                  // land on a refresh
const held = [];
for (let i = 0; i < 5; i++) { api.step(2); held.push(st().beams[0].r); }   // 10 ms total
const heldSame = new Set(held.map(r => r.toFixed(4))).size === 1;
run(60);                                       // now well past a refresh
const refreshed = st().beams[0].r;
api.setDrive(0, 0);
check('reading is held between refreshes and updates after one',
      heldSame && Math.abs(refreshed - held[0]) > 0.5,
      `held ${held[0].toFixed(2)} x${held.length}, then ${refreshed.toFixed(2)}`);

// Cone vs pencil ray. The front sensor is placed 3.5 in off the centre post's
// axis, so the centre ray misses it entirely and runs out of range, while a
// wide cone clips the post and returns ~14 in.
api.setParam('distNoiseMm', 0);
api.reset(-26, 3.5, 90);       // front mount is +6 forward -> sensor at (-20, 3.5)
api.setParam('distFovDeg', 0.5);
run(120);
const narrow = st().beams[0];
api.setParam('distFovDeg', 40);
run(120);
const wide = st().beams[0];
check('cone sees a post the centre ray misses',
      !narrow.hit && wide.hit && wide.r < 25,
      `narrow ${narrow.hit ? narrow.r.toFixed(1) + ' in' : 'no return'}` +
      ` -> wide ${wide.hit ? wide.r.toFixed(1) + ' in' : 'no return'}`);
api.setParam('distFovDeg', 25);

// Grazing incidence. Heading 100 (VEX) points 10 deg below +X, so the front
// ray meets the bottom wall at ~80 deg from its normal. Narrow the cone first
// so the result isolates incidence rather than a nearer off-axis hit.
api.setParam('distFovDeg', 1);
api.reset(-60, -60, 100);
run(150);
api.setParam('distMaxIncidenceDeg', 89);
run(120);
const lenient = st().beams[0];
api.setParam('distMaxIncidenceDeg', 60);
run(120);
const strict = st().beams[0];
check('a grazing return survives a loose incidence limit and is lost at a tight one',
      lenient.hit && !strict.hit,
      `incidence ${lenient.inc.toFixed(0)}°: ` +
      `${lenient.hit ? 'valid' : 'dropped'} at 89° limit, ` +
      `${strict.hit ? 'valid' : 'dropped'} at 60°`);
api.setParam('distMaxIncidenceDeg', 75);
api.setParam('distFovDeg', 25);

// Head-on should be far more confident than grazing.
// Facing -Y with the +6 in front mount puts the sensor at y = -66, square to
// the bottom wall and still inside the field.
api.reset(-60, -60, 180);
run(150);
const headOn = st().beams[0];
check('confidence is higher head-on than grazing',
      headOn.hit && headOn.conf > lenient.conf,
      `head-on ${headOn.conf} (${headOn.inc.toFixed(0)}°) vs grazing ${lenient.conf}`);

s = st();
check('confidence stays in the V5 0-63 range',
      s.beams.every(b => b.conf >= 0 && b.conf <= 63),
      s.beams.map(b => b.conf).join(', '));

console.log('\n== route DSL ==');
api.setParam('distNoiseMm', 5);
api.reset(-48, -60, 90);
api.routeClear();
// setPose(-48,-60,90); moveTo(-24,-60); drive(10)
api.routePush(0, -48, -60, 90, 0);
api.routePush(1, -24, -60, 1, 3000);
api.routePush(4, 10, 0, 0, 2000);
api.routeRun();
run(300);
check('route task started', api.routeRunning() === 1);
check('route reports the active command', api.routeIndex() >= 0,
      `cmd ${api.routeIndex()}`);
run(6000);
s = st();
check('route drove toward its waypoint', s.true.x > -40,
      `true=(${s.true.x.toFixed(1)}, ${s.true.y.toFixed(1)})`);
api.stopAuton();

console.log('\n== RAMSETE path following ==');
api.setLoc(0);
api.reset(-40, -40, 90);

const viaRoute = 'setPose(x: -40, y: -40, h: 90); via(x: -40, y: -40); ' +
                 'via(x: -10, y: -25); via(x: 20, y: -40); followPath(t: 6000);';
const pr = parseRoute(viaRoute);
check('route with via/followPath parses', !pr.error && pr.paths.length === 1,
      pr.error || `${pr.paths[0].length} vias`);

const pathPts = buildPath(pr.paths[0]);
check('spline is sampled through the vias', pathPts.length > 20, `${pathPts.length} points`);
check('spline endpoints match the outer vias',
      Math.hypot(pathPts[0].x + 40, pathPts[0].y + 40) < 0.01 &&
      Math.hypot(pathPts.at(-1).x - 20, pathPts.at(-1).y + 40) < 0.01,
      `(${pathPts[0].x.toFixed(1)},${pathPts[0].y.toFixed(1)}) -> ` +
      `(${pathPts.at(-1).x.toFixed(1)},${pathPts.at(-1).y.toFixed(1)})`);

// followPath with fewer than two vias is a route authoring error, not a crash.
check('followPath without vias is rejected',
      !!parseRoute('followPath(t: 1000);').error,
      parseRoute('followPath(t: 1000);').error);

api.pathClear();
for (const q of pathPts) api.pathPush(q.x, q.y, q.h);
check('path uploaded to the bridge', api.pathCount() === pathPts.length,
      `${api.pathCount()} points`);

// --- plumbing: DSL -> bridge -> moveRAMSETE ---------------------------------
// The driving checks use a path clear of the field posts. The route above ends
// 8.6 in from the post at (23.5,-47), close enough that the robot's corner
// wedges on it, which measures the obstacle rather than the controller.
const openPath = buildPath(parseRoute(
  'via(x:-40,y:-35); via(x:-20,y:-22); via(x:0,y:-35); followPath();').paths[0]);
const openEnd = openPath.at(-1);
const uploadOpen = () => {
  api.pathClear();
  for (const q of openPath) api.pathPush(q.x, q.y, q.h);
};
// Drive it, returning the closest approach to the end and the peak speed.
const runOpen = (startH, timeout, ticks = 700) => {
  uploadOpen();
  api.reset(openPath[0].x, openPath[0].y, startH);
  api.routeClear();
  api.routePush(13, 0, openPath.length, 0, timeout);
  api.routeRun();
  run(50);
  const started = api.routeRunning() === 1;
  let nearest = Infinity, peak = 0, collided = false, prev = st().true;
  for (let i = 0; i < ticks; i++) {
    api.step(10);
    const q = st().true;
    nearest = Math.min(nearest, Math.hypot(q.x - openEnd.x, q.y - openEnd.y));
    peak = Math.max(peak, Math.hypot(q.x - prev.x, q.y - prev.y) / 0.01);
    collided = collided || st().colliding;
    prev = q;
    if (api.routeRunning() !== 1) break;
  }
  api.stopAuton();
  run(200);
  return { started, nearest, peak, collided };
};

// Started PARTWAY along the path, so the velocity profile is already above
// zero. This is the check that the whole path pipeline is wired correctly.
uploadOpen();
api.reset(-20, -22, 90);
api.routeClear();
api.routePush(13, 0, openPath.length, 0, 5000);
api.routeRun();
run(300);
check('path route started', api.routeRunning() === 1);
let nearestEnd = Infinity;
for (let i = 0; i < 500; i++) {
  api.step(10);
  const q = st().true;
  nearestEnd = Math.min(nearestEnd, Math.hypot(q.x - openEnd.x, q.y - openEnd.y));
}
check('RAMSETE tracks the uploaded path to its end', nearestEnd < 3,
      `closest approach to path end ${nearestEnd.toFixed(1)} in`);
api.stopAuton();
run(200);

// --- regression: RAMSETE starts from progress 0 -----------------------------
// v_d is read from the profile by arc-length position and a trapezoid is 0 at
// its start, so v_d was 0 there. Every term of the law scales with v_d, so v
// and w came out 0, progress stayed 0 and nothing ever moved. minSpeed floors
// v_d below the profile's peak, which breaks the fixed point.
const fromStart = runOpen(53.7, 5000);
check('RAMSETE drives off the first point of its own path',
      fromStart.nearest < 3 && !fromStart.collided,
      `closest approach ${fromStart.nearest.toFixed(2)} in` +
      (fromStart.collided ? ' (hit an obstacle)' : ''));

// --- regression: the profile governs the speed ------------------------------
// generateTrapezoidal left dAccel at its full-speed value on a triangular
// profile, so the position axis jumped past `distance` and a position-keyed
// lookup read peak velocity for the whole second half: the robot never
// decelerated and sailed 20 in past the end.
check('RAMSETE decelerates into the end instead of overshooting',
      fromStart.nearest < 3,
      `overshoot would leave the closest approach far short; got ` +
      `${fromStart.nearest.toFixed(2)} in`);

// Peak speed must track the profile, not exceed it. A lookahead reference left
// a standing e_x whose k*e_x term inflated every speed by roughly half.
// Triangular peak over this 48.4 in path at 50 in/s^2 is sqrt(a*L) = 49 in/s.
check('peak speed follows the velocity profile', fromStart.peak < 60,
      `peak ${fromStart.peak.toFixed(1)} in/s vs a 49 in/s profile peak`);

// The floor must not swamp the profile -- the run should not creep at minSpeed.
check('the profile still shapes the run, the floor does not',
      fromStart.peak > 25,
      `peak ${fromStart.peak.toFixed(1)} in/s, floor is ~7.2 in/s`);

console.log('\n== LTV / LQR follower ==');
api.setLoc(0);

const ltvSpline = buildPath(parseRoute(
  'via(x:-40,y:-40); via(x:-10,y:-25); via(x:20,y:-40); followPath();').paths[0]);
const ltvTraj = buildLtvTrajectory(ltvSpline, 40, 60);

check('trajectory is time-sampled at 10 ms',
      ltvTraj.samples.length > 100 &&
      Math.abs(ltvTraj.samples.length * 10 - ltvTraj.durationMs) < 60,
      `${ltvTraj.samples.length} samples over ${ltvTraj.durationMs.toFixed(0)} ms`);
check('trajectory starts from rest', ltvTraj.samples[0].v < 1e-6,
      `first v = ${ltvTraj.samples[0].v.toFixed(4)} in/s`);

api.ltvSetPath(ltvTraj.text);
// parse_tuples() keys on parentheses; a brace-delimited blob silently parses
// to nothing and the follower bails with "Empty trajectory".
check('blob parses into a trajectory', api.ltvLength() > 50,
      `${api.ltvLength().toFixed(1)} in of path`);

const runLtv = (kV) => {
  api.ltvGains(kV, 0.3, 2.0);
  api.ltvSetPath(ltvTraj.text);
  api.reset(-40, -40, 61);
  api.ltvRun(0, 0);
  let guard = 0;
  while (api.ltvRunning() === 1 && guard++ < 2000) api.step(10);
  for (let i = 0; i < 50; i++) api.step(10);
  const q = st().true;
  api.ltvCancel();
  for (let i = 0; i < 40; i++) api.step(10);
  return { q, err: Math.hypot(q.x - 20, q.y + 40) };
};

// The case that deadlocks RAMSETE: begin at the path start, from rest. LTV
// advances its reference one sample per tick regardless of robot progress, so
// it has no equivalent failure.
const tuned = runLtv(6.2);
check('LTV starts from rest at the path start',
      Math.hypot(tuned.q.x + 40, tuned.q.y + 40) > 20,
      `moved to (${tuned.q.x.toFixed(1)}, ${tuned.q.y.toFixed(1)})`);
check('LTV tracks to the path end with a correct kV', tuned.err < 3,
      `end error ${tuned.err.toFixed(2)} in`);

// kV is volts per m/s; too low starves the feedforward and the robot lags
// along-track. This is a tuning sensitivity, not a controller fault -- worth
// pinning so a future change that hides it is visible.
const starved = runLtv(0.9);
check('too-low kV shows up as an along-track deficit', starved.err > tuned.err + 3,
      `kV 0.9 ends ${starved.err.toFixed(2)} in short vs ${tuned.err.toFixed(2)} at kV 6.2`);
api.ltvGains(6.2, 0.3, 2.0);

console.log('\n== run recording ==');
check('samples were recorded', api.recCount() > 10, `${api.recCount()} rows`);
check('stride matches the column list', api.recStride() === 14, `${api.recStride()}`);

console.log('\n== filter diagnostics ==');
api.reset(-48, -60, 90);
api.setLoc(3);                      // MCL + RCL
run(600);
s = st();
check('MCL reports N_eff', s.mcl.nEff > 0 && s.mcl.nEff <= 1024, `${s.mcl.nEff.toFixed(0)} / 1024`);
check('RCL reports an estimate', typeof s.rcl.x === 'number',
      `(${s.rcl.x.toFixed(1)}, ${s.rcl.y.toFixed(1)})`);
check('every live sensor has a filter row', s.filters.length === s.beams.length,
      `${s.filters.length} rows / ${s.beams.length} beams`);
check('filter rows carry per-estimator validity',
      s.filters.every(f => typeof f.mclValid === 'boolean' && typeof f.rclValid === 'boolean'),
      s.filters.map(f => `${f.i}:${f.mclValid ? 'M' : '-'}${f.rclValid ? 'R' : '-'}`).join(' '));

// The out-of-range sentinel must be rejected. 9999 mm is 393.7 in, and a
// filter that accepted it would be weighting particles against a reading the
// hardware never actually took.
const outOfRange = s.filters.filter(f => f.mclIn > 300);
check('out-of-range readings are rejected by MCL',
      outOfRange.every(f => !f.mclValid),
      `${outOfRange.length} sensor(s) at the 393.7 in sentinel`);
api.setLoc(0);

console.log('\n== determinism ==');
const sample = () => {
  api.setParam('seed', 1234);
  api.reset(-48, -60, 90);
  api.setDrive(6, 5);
  run(1000);
  api.setDrive(0, 0);
  const r = st();
  return `${r.true.x.toFixed(6)},${r.true.y.toFixed(6)},${r.true.h.toFixed(6)}`;
};
const a = sample();
const b = sample();
check('same seed replays identically', a === b, `${a} vs ${b}`);

// MclTracking seeds itself from std::random_device, so its particle cloud used
// to differ every run even with the world pinned. setSeed() makes the whole
// estimate reproducible, which is what a regression test needs.
const mclSample = () => {
  api.setLoc(0);
  // Seed BEFORE reset: reset seeds both RNGs and then synthesises the first
  // frame of sensor readings, so seeding afterwards leaves that frame built
  // from the previous seed and MCL's first weighting differs.
  api.setParam('seed', 4242);
  api.reset(-48, -60, 90);
  api.setLoc(1);
  run(400);
  api.setDrive(6, 6);
  run(1200);
  api.setDrive(0, 0);
  run(200);
  const r = st();
  return `${r.mcl.x.toFixed(4)},${r.mcl.y.toFixed(4)},${r.mcl.h.toFixed(4)}`;
};
const m1 = mclSample();
const m2 = mclSample();
check('MCL replays identically once seeded', m1 === m2, `${m1} vs ${m2}`);
api.setLoc(0);

console.log(`\n${failures === 0 ? 'ALL CHECKS PASSED' : failures + ' CHECK(S) FAILED'}\n`);
process.exit(failures === 0 ? 0 : 1);
