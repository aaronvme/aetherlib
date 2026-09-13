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
  autonSymbol: c('sim_auton_symbol', 'string', ['number']),
  runAuton:    c('sim_run_auton', null, ['number']),
  routeRunning:c('sim_route_running', 'number', []),
  stopAuton:   c('sim_stop_auton', null, []),
  setParam:    c('sim_set_param', null, ['string', 'number']),
  getParam:    c('sim_get_param', 'number', ['string']),
  fieldClear:  c('sim_field_clear', null, []),
  fieldDefault:c('sim_field_default', null, []),
  addSeg:      c('sim_field_add_segment', null,
                 ['number','number','number','number','number']),
  routeClear:  c('sim_route_clear', null, []),
  routePush:   c('sim_route_push', null, ['number','number','number','number','number']),
  routeRun:    c('sim_route_run', null, []),
  routeIndex:  c('sim_route_index', 'number', []),
  routeOpt:    c('sim_route_opt', null, ['string', 'number']),
  routeAsync:  c('sim_route_async', null, []),
  routeReport: c('sim_route_report_json', 'string', []),
  pathPreview: c('sim_path_preview_json', 'string',
                 ['number', 'number', 'number']),
  recCount:    c('sim_record_count', 'number', []),
  recStride:   c('sim_record_stride', 'number', []),
  pathClear:   c('sim_path_clear', null, []),
  pathPush:    c('sim_path_push', null, ['number','number','number']),
  pathCount:   c('sim_path_count', 'number', []),
};
const { buildPath, parseRoute } = await import('../web/route_dsl.js');
const { parseAuton, OP } = await import('../web/auton_cpp.js');

let failures = 0;
const check = (label, cond, detail = '') => {
  const tag = cond ? 'PASS' : 'FAIL';
  if (!cond) failures++;
  console.log(`  [${tag}] ${label}${detail ? '  ' + detail : ''}`);
};

const st = () => JSON.parse(api.stateJson());
const run = (ms, dt = 10) => { for (let i = 0; i < ms / dt; i++) api.step(dt); };
/// Signed difference between two headings in degrees, wrapped to +/-180.
const angErr = (a, b) => ((((a - b) % 360) + 540) % 360) - 180;

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
const autonIdx = [...Array(api.autonCount()).keys()];
check('autons registered',
      api.autonCount() >= 4 &&
      autonIdx.every(i => api.autonName(i) && api.autonSymbol(i)),
      autonIdx.map(i => `${api.autonName(i)}=${api.autonSymbol(i)}()`).join(', '));

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

// Cone vs pencil ray. The front sensor has to sit a little off the centre
// post's axis: far enough out that the on-axis ray misses a 2 in post and runs
// past max range to the far wall, close enough that a wide cone still clips it.
//
// Where that puts the ROBOT depends on the front mount, which is measured
// hardware and changes. This used to hardcode `reset(-26, 3.5)` with a comment
// claiming the mount was +6 forward; once config.cpp shipped (1.0, -3.75) the
// sensor landed at (-25.0, -0.25) instead, staring straight down the post's
// axis, and both cone widths returned the same 23.0 in. So place the robot,
// read where the sensor actually ended up -- beams carry their own origin --
// and correct. Heading 90 (VEX) points +X, so the sensor's y tracks the
// robot's one for one.
const kPostOffsetIn = 3.5;     // > 2 in post radius, << the 20 deg cone's reach
api.setParam('distNoiseMm', 0);
api.setParam('distFovDeg', 0.5);
api.reset(-26, 0, 90);
run(60);
const probeY = st().beams[0].oy;
api.reset(-26, kPostOffsetIn - probeY, 90);
run(120);
const narrow = st().beams[0];
api.setParam('distFovDeg', 40);
run(120);
const wide = st().beams[0];
check('cone sees a post the centre ray misses',
      Math.abs(narrow.oy - kPostOffsetIn) < 0.05 &&
      !narrow.hit && wide.hit && wide.obs && wide.r < 25,
      `sensor y ${narrow.oy.toFixed(2)}  ` +
      `narrow ${narrow.hit ? narrow.r.toFixed(1) + ' in' : 'no return'}` +
      ` -> wide ${wide.hit ? wide.r.toFixed(1) + ' in' : 'no return'}` +
      `${wide.hit && !wide.obs ? ' (a wall, not the post)' : ''}`);
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

console.log('\n== setPose places the robot only at the start ==');

// The opening setPose is where the auton says it begins, so it moves ground
// truth. A later one is a re-localization: moving truth to it would erase the
// error the sim exists to show.
api.reset(0, 0, 0);
api.routeClear();
api.routePush(OP.SET_POSE, -48, -60, 90, 0);
api.routePush(OP.DRIVE, 12, 0, 0, 2500);
api.routePush(OP.SET_POSE, 24, 24, 45, 0);
api.routeRun();
// Sampled early: the drive that follows is already under way at 200 ms.
run(50);
s = st();
check('the opening setPose places the robot',
      Math.hypot(s.true.x + 48, s.true.y + 60) < 2,
      `true=(${s.true.x.toFixed(1)}, ${s.true.y.toFixed(1)})`);

run(5000);
s = st();
check('a later setPose leaves ground truth alone',
      Math.hypot(s.true.x - 24, s.true.y - 24) > 20,
      `true=(${s.true.x.toFixed(1)}, ${s.true.y.toFixed(1)})`);
check('and still moves the estimate',
      Math.hypot(s.odom.x - 24, s.odom.y - 24) < 1,
      `odom=(${s.odom.x.toFixed(1)}, ${s.odom.y.toFixed(1)})`);
api.stopAuton();

console.log('\n== auton source runs the real API ==');

// Parse real C++, push it, run it to completion, and hand back the report.
// This is the whole routing path end to end: if an option in the source does
// not reach the library, the sim is lying about code it was handed.
function runSource(src, budgetMs = 20000, onTick = null) {
  const r = parseAuton(src);
  if (r.error) return { error: r.error };

  // Control points go up as written. moveRAMSETE splines them on the way in,
  // so the sim uploads exactly what a compiled auton would pass.
  api.pathClear();
  const slices = [];
  let cursor = 0;
  for (const controls of r.paths) {
    for (const q of controls) api.pathPush(q.x, q.y, q.h);
    slices.push([cursor, controls.length]);
    cursor += controls.length;
  }

  api.routeClear();
  r.cmds.forEach((cmd, i) => {
    let [op, a, b, cc, d] = cmd;
    if (op === OP.FOLLOW_PATH) {
      const slice = slices[a];
      if (!slice) return;
      [a, b] = slice;
    }
    api.routePush(op, a, b, cc, d);
    for (const [k, v] of Object.entries(r.opts[i] ?? {})) {
      if (k === '_async') api.routeAsync();
      else api.routeOpt(k, v);
    }
  });

  api.routeRun();
  for (let t = 0; t < budgetMs && api.routeRunning(); t += 10) {
    api.step(10);
    if (onTick) onTick();
  }
  api.stopAuton();
  return { report: JSON.parse(api.routeReport()), state: st(), parsed: r };
}

api.setLoc(0);
api.setParam('distNoiseMm', 0);

// --- .forwards has to reach the library -----------------------------------
//
// (-20, 35) facing +X, because the field centre is a goal post and a robot
// started there is stuck inside it. The first draft of these used (0, 0) and
// measured nothing but the collision.
const LANE = 'Pose::fromVex(-20.0f, 35.0f, 90.0f)';

let fwd = runSource(`
  chassis.setPose(${LANE});
  chassis.moveDistance(18.0f, 3000);
`);
check('a forward moveDistance increases x', !fwd.error && fwd.state.true.x > -6,
      fwd.error ?? `x=${fwd.state.true.x.toFixed(1)} (from -20)`);

let back = runSource(`
  chassis.setPose(${LANE});
  chassis.moveDistance(-18.0f, 3000);
`);
check('a negative distance drives backward, as Chassis::moveDistance does',
      !back.error && back.state.true.x < -34,
      back.error ?? `x=${back.state.true.x.toFixed(1)} (from -20)`);

let flagged = runSource(`
  chassis.setPose(${LANE});
  chassis.moveDistance(18.0f, 3000, {.forwards = false});
`);
check('.forwards = false reaches the params struct',
      !flagged.error && flagged.state.true.x < -34,
      flagged.error ?? `x=${flagged.state.true.x.toFixed(1)} (from -20)`);

// --- .maxSpeed has to reach it too, and it is visible in the timing --------
const timed = speed => runSource(`
  chassis.setPose(${LANE});
  chassis.moveToPoint(10.0f, 35.0f, 6000, {.maxSpeed = ${speed}.0f});
`).report[1].ms;
const slow = timed(40), quick = timed(120);
// An absolute margin, not a ratio: the accel ramp and the settle debounce are
// a fixed cost either way, so tripling the speed cap over 30 in buys 0.6 s
// rather than the 3x a ratio test would demand.
check('a lower .maxSpeed takes measurably longer', slow > quick + 400,
      `40 -> ${slow} ms, 120 -> ${quick} ms`);

// --- the report itself ----------------------------------------------------
const cut = runSource(`
  chassis.setPose(${LANE});
  chassis.moveToPoint(25.0f, 35.0f, 400);
  chassis.moveToPoint(-20.0f, 35.0f, 8000);
`);
check('one report row per command', !cut.error && cut.report.length === 3,
      cut.error ?? `${cut.report?.length} rows`);
check('a motion cut off by its timeout is flagged',
      cut.report?.[1]?.cutOff === true && cut.report[1].ms <= 450,
      `${cut.report?.[1]?.ms} ms of 400, cutOff=${cut.report?.[1]?.cutOff}`);
check('a motion that settled is not flagged', cut.report?.[2]?.cutOff === false,
      `${cut.report?.[2]?.ms} ms of 8000, cutOff=${cut.report?.[2]?.cutOff}`);
check('the report says how far it ended from the target',
      cut.report?.[2]?.targetErr >= 0 && cut.report[2].targetErr < 4,
      `${cut.report?.[2]?.targetErr?.toFixed(2)} in`);
check('setPose carries no target to miss', cut.report?.[0]?.targetErr === -1,
      String(cut.report?.[0]?.targetErr));

// Clearance is measured against the footprint, so the numbers have to be
// usable rather than always negative: 15 in of robot at the field centre is
// far from everything, and the same robot against the wall is not.
const middle = runSource(`
  chassis.setPose(Pose::fromVex(-30.0f, 30.0f, 90.0f));
  chassis.moveDistance(10.0f, 3000);
`);
check('open field reports metres of room, not a negative number',
      middle.report?.[1]?.clear > 5,
      `${middle.report?.[1]?.clear?.toFixed(1)} in`);

const wall = runSource(`
  chassis.setPose(Pose::fromVex(-30.0f, -60.0f, 180.0f));
  chassis.moveDistance(8.0f, 3000);
`);
check('driving at a wall reports the room running out',
      wall.report?.[1]?.clear < middle.report?.[1]?.clear &&
      wall.report?.[1]?.clear < 3,
      `${wall.report?.[1]?.clear?.toFixed(1)} in vs ` +
      `${middle.report?.[1]?.clear?.toFixed(1)} in in the open`);

// A post is a smaller target than a wall; the cone test elsewhere proves the
// sensor sees it, this proves the route report does.
const post = runSource(`
  chassis.setPose(Pose::fromVex(-20.0f, 0.0f, 90.0f));
  chassis.moveToPoint(-11.0f, 0.0f, 3000);
`);
check('a post shortens the clearance too', post.report?.[1]?.clear < 4,
      `${post.report?.[1]?.clear?.toFixed(1)} in from the centre post`);

// --- an option the bridge cannot apply is counted, not swallowed ----------
api.reset(-20, 35, 90);
api.routeClear();
api.routePush(OP.MOVE_TO, -10, 35, 1, 3000);
api.routeOpt('maxSpeed', 90);
api.routeOpt('nosuchfield', 1);
api.routeRun();
for (let t = 0; t < 5000 && api.routeRunning(); t += 10) api.step(10);
api.stopAuton();
const ignored = JSON.parse(api.routeReport());
check('an option the bridge cannot apply is reported',
      ignored[0]?.ignored === 1, `ignored=${ignored[0]?.ignored}`);

// --- async runs on while the route continues ------------------------------
const asyncRun = runSource(`
  chassis.setPose(${LANE});
  chassis.moveToPoint(10.0f, 35.0f, 6000, {.maxSpeed = 60.0f}, true);
  chassis.waitUntilSettled();
`);
check('an async motion returns immediately and waitUntilSettled holds',
      asyncRun.report?.[1]?.ms < 100 && asyncRun.report?.[2]?.ms > 500,
      `move ${asyncRun.report?.[1]?.ms} ms, wait ${asyncRun.report?.[2]?.ms} ms`);

// --- a whole function, in the shape autons are actually written ------------
//
// The braced setPose form on purpose: Pose.theta is RADIANS, so 0.0f faces +X
// (VEX 90), and getting that backwards would put every route 90 deg out.
const shipped = runSource(`
  void autonBlueLeft() {
      chassis.setPose({-20.0f, 35.0f, 0.0f});
      chassis.moveDistance(12.0f, 2000);
      chassis.turnToHeading(270.0f, 1500);
      chassis.moveDistance(18.0f, 2000);
  }
`);
check('a whole auton function runs as written',
      !shipped.error && shipped.report?.length === 4,
      shipped.error ?? `${shipped.report?.length} commands, ` +
        `ended (${shipped.state.true.x.toFixed(1)}, ${shipped.state.true.y.toFixed(1)})`);
check('theta 0 rad faced +X, so the first leg landed 12 in along x',
      shipped.report?.[1]?.targetErr >= 0 && shipped.report[1].targetErr < 3,
      `first leg missed by ${shipped.report?.[1]?.targetErr?.toFixed(2)} in`);
check('the turn then reversed it back past where it started',
      shipped.state?.true.x < -20 && Math.abs(shipped.state.true.y - 35) < 8,
      `ended (${shipped.state?.true.x.toFixed(1)}, ${shipped.state?.true.y.toFixed(1)})`);

// --- a pose list is control points, splined before it is driven -----------
console.log('\n== control points drive a smooth curve ==');

// The posts sit on a lattice at (+-47, +-23.5), (+-23.5, +-47) and the origin,
// so a curve has to be placed in a hole in it. The first draft of this arced
// through (-20, 45), which is 4 in from the post at (-23.5, 47): the robot hit
// it and the test read as a path-following failure. Every control below clears
// the nearest post by more than 11 in.
const curveSrc = `
  chassis.setPose(Pose::fromVex(-40.0f, 12.0f, 90.0f));
  std::vector<Pose> path = {
      {-40.0f, 12.0f, 0.0f},
      {-25.0f, 22.0f, 0.0f},
      {-10.0f, 12.0f, 0.0f},
  };
  chassis.moveRAMSETE(path, 8000);
`;

const parsedCurve = parseAuton(curveSrc);
check('the parser hands back three control points',
      parsedCurve.paths?.[0]?.length === 3,
      parsedCurve.error ?? `${parsedCurve.paths?.[0]?.length} controls`);

// The preview runs the library's own splinePath over the uploaded controls,
// so this is the curve moveRAMSETE will build from the same three points.
api.pathClear();
for (const q of parsedCurve.paths[0]) api.pathPush(q.x, q.y, q.h);
check('only the control points are uploaded', api.pathCount() === 3,
      `${api.pathCount()} poses`);

const built = JSON.parse(api.pathPreview(0, 3, 2.0));
check('the library turns them into a dense curve', built.length > 15,
      `3 controls -> ${built.length} poses`);
check('the curve starts and ends on the outer controls',
      Math.hypot(built[0].x + 40, built[0].y - 12) < 0.01 &&
      Math.hypot(built.at(-1).x + 10, built.at(-1).y - 12) < 0.01,
      `(${built[0].x}, ${built[0].y}) -> (${built.at(-1).x}, ${built.at(-1).y})`);
check('it bulges toward the middle control rather than cutting the corner',
      Math.max(...built.map(q => q.y)) > 21.5,
      `peak y ${Math.max(...built.map(q => q.y)).toFixed(1)} (control is at 22)`);
check('tangent headings were filled in, not left at zero',
      built.some(q => Math.abs(q.h - 90) > 5),
      `bearings ${built[0].h.toFixed(0)} .. ${built.at(-1).h.toFixed(0)}`);

// The one that matters: hand moveRAMSETE THREE POSES and watch where the
// robot goes. If the library were not splining internally it would drive the
// polyline through them, whose apex is the control itself -- so a trajectory
// that arcs past the straight lines is the proof.
const polyApexY = 22.0;      // the middle control
let peakY = -1e9;
const sp = runSource(curveSrc, 20000, () => {
  peakY = Math.max(peakY, st().true.y);
});
check('the route stayed clear of the posts', sp.report?.[1]?.hit === false,
      `clearance ${sp.report?.[1]?.clear?.toFixed(1)} in, hit=${sp.report?.[1]?.hit}`);
check('the route drove the curve to its end',
      sp.report?.[1]?.targetErr >= 0 && sp.report[1].targetErr < 6,
      `ended ${sp.report?.[1]?.targetErr?.toFixed(1)} in from the last control, ` +
      `${sp.report?.[1]?.ms} ms`);
check('the robot drove a curve, not the polyline through the controls',
      peakY > 16 && peakY < polyApexY + 4,
      `peak y ${peakY.toFixed(1)}; the straight-line pair peaks at ` +
      `${polyApexY} only at the corner itself`);

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

// ============================================================================
// Body collision
// ============================================================================
//
// The routing battery drives through open field, so none of it ever parks
// against a wall -- which is how a change that wedged the robot on contact got
// through a green suite. These press on the contact case directly.
//
// The body is an oriented rectangle, so its reach toward an axis-aligned wall
// is the bounding-box half-extent, not one radius: 7.5 in square-on and up to
// 10.61 in at 45 degrees on a 15 in square.
console.log('\n== body collision ==');

const HALF_WALL = 70.2;
const extent = (hDeg, L, W) => {
  const t = (90 - hDeg) * Math.PI / 180;
  return 0.5 * (L * Math.abs(Math.cos(t)) + W * Math.abs(Math.sin(t)));
};

// Posts off, so only the perimeter is in play and the geometry is unambiguous.
api.setParam('collideObstacles', 0);
const L = api.getParam('robotLengthIn');
const W = api.getParam('robotWidthIn');

/// Drive east out of (-30, 0) at `hDeg` until it stops, then report the gap
/// between the body's leading extent and the wall it came to rest on.
function wallGap(hDeg) {
  api.reset(-30, 0, hDeg);
  api.setDrive(9, 9);
  run(20000);
  api.setDrive(0, 0);
  run(400);
  const s = st();
  const e = extent(s.true.h, L, W);
  return Math.min(HALF_WALL - (Math.abs(s.true.x) + e),
                  HALF_WALL - (Math.abs(s.true.y) + e));
}

for (const h of [90, 60, 45]) {
  const gap = wallGap(h);
  check(`the body stops ON the wall at ${h} deg, not through or short of it`,
        gap > -0.02 && gap < 0.25, `gap ${gap.toFixed(3)} in`);
}

// Turning while parked used to be free, because a circle's footprint does not
// change when it rotates. A rectangle's does, so an unchecked turn buried a
// corner in the wall and then blocked translation in every direction.
api.reset(-30, 0, 90);
api.setDrive(9, 9);
run(20000);
api.setDrive(0, 0);
run(300);
const parkedX = st().true.x;
api.setDrive(6, -6);          // spin in place against the wall
run(1200);
api.setDrive(0, 0);
run(200);
const spun = st();
check('turning into a wall cannot bury the body in it',
      HALF_WALL - (Math.abs(spun.true.x) + extent(spun.true.h, L, W)) > -0.02,
      `h ${spun.true.h.toFixed(1)} deg, gap `
      + `${(HALF_WALL - (Math.abs(spun.true.x) + extent(spun.true.h, L, W))).toFixed(3)} in`);

api.setDrive(-9, -9);         // and it must still be able to leave
run(1500);
api.setDrive(0, 0);
run(200);
check('and the robot can still reverse away from it',
      st().true.x < parkedX - 5,
      `${parkedX.toFixed(2)} -> ${st().true.x.toFixed(2)}`);

// The perimeter is geometry, not a hardcoded box: clearing it has to free the
// robot, or the raycaster and the physics are describing different fields.
api.fieldClear();
api.reset(-30, 0, 90);
api.setDrive(9, 9);
run(20000);
api.setDrive(0, 0);
const escaped = st();
check('with the walls cleared the robot is no longer boxed in',
      escaped.true.x > HALF_WALL, `reached x ${escaped.true.x.toFixed(1)}`);
check('and the sensors agree there is nothing there',
      escaped.beams.every(b => !b.hit),
      `${escaped.beams.filter(b => b.hit).length} of ${escaped.beams.length} still returning`);

// Every wall in the default field is axis-aligned, so the general segment case
// never runs unless a test asks for it. A diagonal wall met at a diagonal body
// angle is the one that would expose a bad frame transform or a bad clip.
api.fieldClear();
api.addSeg(20, -40, -40, 20, 0);        // 45 deg, kind 0 so it is solid
api.reset(-60, -60, 45);
api.setDrive(9, 9);
run(20000);
api.setDrive(0, 0);
run(400);
{
  const s = st();
  // Exact separation between the body and the finite segment: sample the
  // segment, take the point-to-box exterior distance in the body frame.
  const t = (90 - s.true.h) * Math.PI / 180;
  const ct = Math.cos(t), stt = Math.sin(t);
  let sep = Infinity;
  for (let i = 0; i <= 4000; i++) {
    const u = i / 4000;
    const dx = (20 + (-40 - 20) * u) - s.true.x;
    const dy = (-40 + (20 - -40) * u) - s.true.y;
    const lx =  dx * ct + dy * stt;
    const ly = -dx * stt + dy * ct;
    const ox = Math.max(Math.abs(lx) - L / 2, 0);
    const oy = Math.max(Math.abs(ly) - W / 2, 0);
    sep = Math.min(sep, Math.hypot(ox, oy));
  }
  check('a diagonal wall stops the body exactly, not through it',
        s.colliding && sep < 0.05, `colliding=${s.colliding}, separation ${sep.toFixed(4)} in`);
}

api.fieldDefault();
api.setParam('collideObstacles', 1);
api.reset(-48, -60, 90);

console.log(`\n${failures === 0 ? 'ALL CHECKS PASSED' : failures + ' CHECK(S) FAILED'}\n`);
process.exit(failures === 0 ? 0 : 1);
