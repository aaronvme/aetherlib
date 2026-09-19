// Does the robot end up where the route asked, and does it get there cleanly?
//
// Routing in the browser is only worth doing if a waypoint you place is a
// waypoint the robot reaches. This measures that against GROUND TRUTH, for
// every motion the route editor can emit -- not against the estimate, because
// the estimate agreeing with itself is what a localization bug looks like.
//
// It measures the JOURNEY as well, and that part exists because leaving it out
// was a real mistake. A first version scored only the final pose. It reported
// 0.00 in on a 24 in drive that overshot the target by 3.5 in, reversed, and
// crept back -- a motion that is visibly badly tuned and perfect by that
// measure. Overshoot and reversals are why the `over` and `revs` columns
// exist.
//
//   node sim/test/accuracy.mjs             the battery, current tuning
//   node sim/test/accuracy.mjs --gate      same, but exit 1 if a bound is missed
//   node sim/test/accuracy.mjs --json      machine-readable, for the sweeps
//   node sim/test/accuracy.mjs --loc=5     with MCL+EKF up instead of dead
//                                          reckoning, which costs accuracy here
//   node sim/test/accuracy.mjs --tau=140   against a slower motor response
//   node sim/test/accuracy.mjs --stock    with kSimTuning reverted, i.e. the
//                                          robot's own numbers on this plant
//
// It is also the module the sweeps import, so a tuning run and the gate can
// never disagree about what "accurate" means.
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, '..', '..');

export async function boot() {
  const createChadSim =
    (await import('file://' + join(root, 'sim', 'web', 'chad_sim.mjs'))).default;
  const M = await createChadSim();
  const c = (n, r, a) => M.cwrap(n, r, a);
  const api = {
    init:        c('sim_init', null, []),
    reset:       c('sim_reset', null, ['number', 'number', 'number']),
    step:        c('sim_step', null, ['number']),
    setLoc:      c('sim_set_localization', null, ['number']),
    stateJson:   c('sim_state_json', 'string', []),
    setParam:    c('sim_set_param', null, ['string', 'number']),
    getParam:    c('sim_get_param', 'number', ['string']),
    routeClear:  c('sim_route_clear', null, []),
    routePush:   c('sim_route_push', null, ['number', 'number', 'number', 'number', 'number']),
    routeOpt:    c('sim_route_opt', null, ['string', 'number']),
    routeRun:    c('sim_route_run', null, []),
    routeRunning: c('sim_route_running', 'number', []),
    reportJson:  c('sim_route_report_json', 'string', []),
    pathClear:   c('sim_path_clear', null, []),
    pathPush:    c('sim_path_push', null, ['number', 'number', 'number']),
    pathPreview: c('sim_path_preview_json', 'string', ['number', 'number', 'number']),
    tuningJson:  c('sim_tuning_json', 'string', []),
    tuningStock: c('sim_tuning_stock', null, []),
  };
  api.init();
  return api;
}

// Op codes, mirroring the enum in sim/bridge/bridge.cpp.
export const OP = {
  SET_POSE: 0, MOVE_TO: 1, TURN_TO_HEADING: 2, TURN_TO_POINT: 3, DRIVE: 4,
  BOOMERANG: 5, ARC: 6, SWING: 7, DELAY: 8, WAIT_SETTLED: 9,
  LATERAL_GAINS: 10, ANGULAR_GAINS: 11, MAX_SPEED: 12, FOLLOW_PATH: 13,
};

// ============================================================================
// The battery
// ============================================================================
//
// One motion per case, because a case that chains motions cannot say which one
// missed. Start poses avoid the post lattice at +-23.5 and +-47 on both axes;
// `node sim/test/accuracy.mjs` fails loudly on a collision rather than
// reporting it as a controller error.
//
// `want` is what the command asked for, in the same units the route editor
// shows: point in inches, heading as a VEX bearing.
export const BATTERY = [
  // --- turns ---------------------------------------------------------------
  { name: 'turn 90',        start: [-36, -36, 0],
    cmd: [OP.TURN_TO_HEADING, 90, 0, 0, 2000],  want: { heading: 90 } },
  { name: 'turn 180',       start: [-36, -36, 0],
    cmd: [OP.TURN_TO_HEADING, 180, 0, 0, 2500], want: { heading: 180 } },
  { name: 'turn -135',      start: [-36, -36, 0],
    cmd: [OP.TURN_TO_HEADING, 225, 0, 0, 2500], want: { heading: 225 } },
  { name: 'turn 15',        start: [-36, -36, 0],
    cmd: [OP.TURN_TO_HEADING, 15, 0, 0, 1500],  want: { heading: 15 } },
  { name: 'turn 3',         start: [-36, -36, 0],
    cmd: [OP.TURN_TO_HEADING, 3, 0, 0, 1500],   want: { heading: 3 } },
  { name: 'turnToPoint',    start: [-36, -36, 0],
    cmd: [OP.TURN_TO_POINT, -12, -12, 0, 2000], want: { face: [-12, -12] } },

  // --- straight lines ------------------------------------------------------
  { name: 'drive 24',       start: [-36, -36, 90],
    cmd: [OP.DRIVE, 24, 0, 0, 2500],  want: { point: [-12, -36], heading: 90 } },
  { name: 'drive 48',       start: [-60, -36, 90],
    cmd: [OP.DRIVE, 48, 0, 0, 3500],  want: { point: [-12, -36], heading: 90 } },
  { name: 'drive 6',        start: [-36, -36, 90],
    cmd: [OP.DRIVE, 6, 0, 0, 1500],   want: { point: [-30, -36], heading: 90 } },
  { name: 'drive -18',      start: [-36, -36, 90],
    cmd: [OP.DRIVE, -18, 0, 0, 2500], want: { point: [-54, -36], heading: 90 } },

  // --- point to point ------------------------------------------------------
  { name: 'moveToPoint ahead',    start: [-36, -36, 90],
    cmd: [OP.MOVE_TO, -12, -36, 1, 3000], want: { point: [-12, -36] } },
  { name: 'moveToPoint diagonal', start: [-36, -36, 90],
    cmd: [OP.MOVE_TO, -12, -12, 1, 3500], want: { point: [-12, -12] } },
  { name: 'moveToPoint sideways', start: [-36, -36, 0],
    cmd: [OP.MOVE_TO, -36, -12, 1, 3500], want: { point: [-36, -12] } },
  { name: 'moveToPoint short',    start: [-36, -36, 90],
    cmd: [OP.MOVE_TO, -28, -36, 1, 2500], want: { point: [-28, -36] } },
  { name: 'moveToPoint reverse',  start: [-36, -36, 90],
    cmd: [OP.MOVE_TO, -56, -36, -1, 3000], want: { point: [-56, -36] } },

  // --- pose-ending motions -------------------------------------------------
  // 5 s, not 4: boomerang has to satisfy a point AND a final heading, and
  // its speed scale throttles the approach inside 6 in. A timeout tighter
  // than the motion can physically meet measures the timeout.
  { name: 'boomerang',      start: [-36, -36, 90],
    cmd: [OP.BOOMERANG, -12, -12, 0, 5000], want: { point: [-12, -12], heading: 0 } },
  // A swing pivots about one wheel set, so the body centre necessarily
  // translates -- 8.4 in for 90 degrees on an 11.9 in track. Bounding drift
  // here would be bounding the geometry.
  //
  // Started at y = 0 rather than the shared (-36, -36): a 180 degree swing
  // walks the centre ~7 in, and from (-36, -36) that puts the body onto the
  // post at (-47, -23.5). It used to pass only because the collision footprint
  // was half the robot, so the corner went through the post instead of
  // stopping on it -- and a blocked swing is not the swing being measured.
  // At (-36, 0) the nearest post is 25.9 in away, which leaves 8.9 in of
  // clearance at the closest approach. Heading error is 0.17 deg either way.
  { name: 'swing left',     start: [-36, 0, 90], noDrift: true,
    cmd: [OP.SWING, 180, 0, 0, 2500], want: { heading: 180 } },
  { name: 'swing right',    start: [-36, 0, 90], noDrift: true,
    cmd: [OP.SWING, 0, 1, 0, 2500],   want: { heading: 0 } },
  // 26 in is the radius of the only circle through both poses that is
  // tangent to the start heading: 24 in across and 16 in up solves
  // 24^2 + (16-R)^2 = R^2. A radius that does not fit the geometry makes the
  // arc unreachable and reads as a controller failure.
  { name: 'arc',            start: [-36, -36, 90],
    cmd: [OP.ARC, -12, -20, 26, 4000], want: { point: [-12, -20] } },

  // --- path following ------------------------------------------------------
  // Three control points, the way the editor writes them. The curve is
  // whatever chad::splinePath makes of them, so cross-track is measured
  // against the previewed curve rather than the polyline.
  { name: 'path gentle',    start: [-40, 12, 45],
    path: [[-40, 12, 0], [-25, 22, 0], [-10, 12, 0]], timeout: 5000 },
  // Started at (-57, -57) and not the corner's (-60, -60): at 35 degrees a
  // 15 in square reaches 10.45 in toward the corner walls and there is only
  // 10.2 in of it, so the body began 0.25 in inside them. It read as a clean
  // pass only while the collision footprint was a circle, which cannot see a
  // corner. 2.75 in of slack now, in line with the other path cases.
  { name: 'path S',         start: [-57, -57, 35],
    path: [[-57, -57, 0], [-45, -39, 0], [-27, -33, 0], [-12, -12, 0]],
    timeout: 6000 },
  { name: 'path L',         start: [-60, -36, 90],
    path: [[-60, -36, 0], [-36, -36, 0], [-36, -12, 0]], timeout: 5000 },
  { name: 'path long',      start: [-60, -56, 78],
    path: [[-60, -56, 0], [-40, -50, 0], [-24, -60, 0], [-8, -54, 0]],
    timeout: 6000 },
  { name: 'path straight',  start: [-60, -36, 90],
    path: [[-60, -36, 0], [-36, -36, 0], [-12, -36, 0]], timeout: 4000 },
];

// ============================================================================
// Running one case
// ============================================================================

const hyp = (ax, ay, bx, by) => Math.hypot(ax - bx, ay - by);

/// Signed smallest angle between two VEX bearings, degrees.
function bearingDiff(a, b) {
  let d = (a - b) % 360;
  if (d > 180) d -= 360;
  if (d < -180) d += 360;
  return d;
}

/// Perpendicular distance from a point to a polyline, not to its vertices --
/// on a 2 in sample spacing the difference is up to 1 in of invented error.
function distToCurve(pts, x, y) {
  let best = Infinity;
  for (let i = 1; i < pts.length; i++) {
    const ex = pts[i].x - pts[i - 1].x, ey = pts[i].y - pts[i - 1].y;
    const len2 = ex * ex + ey * ey;
    let t = len2 > 1e-9
      ? ((x - pts[i - 1].x) * ex + (y - pts[i - 1].y) * ey) / len2 : 0;
    t = Math.max(0, Math.min(1, t));
    best = Math.min(best, Math.hypot(x - (pts[i - 1].x + t * ex),
                                     y - (pts[i - 1].y + t * ey)));
  }
  return best;
}

export function runCase(api, cs, { loc = 0, settleMs = 300 } = {}) {
  const [sx, sy, sh] = cs.start;
  api.reset(sx, sy, sh);
  api.setLoc(loc);
  // Let odometry and the filters take a first sample before anything moves.
  for (let i = 0; i < 20; i++) api.step(10);

  api.routeClear();
  api.pathClear();
  let curve = null;

  if (cs.path) {
    for (const [x, y, h] of cs.path) api.pathPush(x, y, h);
    const spacing = api.getParam('ramseteSmooth');
    curve = JSON.parse(api.pathPreview(
      0, cs.path.length, Number.isNaN(spacing) ? 2.0 : spacing) || '[]');
    api.routePush(OP.FOLLOW_PATH, 0, cs.path.length, 0, cs.timeout);
  } else {
    api.routePush(...cs.cmd);
  }

  const budget = (cs.path ? cs.timeout : cs.cmd[4]) + 1500;
  api.routeRun();
  for (let i = 0; i < 3; i++) api.step(10);

  // The axis progress is measured along, so overshoot can be stated as
  // inches PAST the target rather than as distance from it -- distance from
  // the target cannot tell 3 in short from 3 in beyond.
  const want0 = cs.want || {};
  let axis = null, axisLen = 0;
  if (cs.path && curve && curve.length > 1) {
    const a = curve[curve.length - 2], b = curve[curve.length - 1];
    const len = Math.hypot(b.x - a.x, b.y - a.y) || 1;
    axis = [(b.x - a.x) / len, (b.y - a.y) / len];
    axisLen = (b.x - sx) * axis[0] + (b.y - sy) * axis[1];
  } else if (want0.point) {
    const dx = want0.point[0] - sx, dy = want0.point[1] - sy;
    axisLen = Math.hypot(dx, dy) || 1;
    axis = [dx / axisLen, dy / axisLen];
  }

  let elapsed = 0, xt = 0, hit = false;
  let peakProgress = -1e9, revs = 0, peakV = 0, peakW = 0, prevV = 0;
  let peakHeadErr = 0;
  const track = s => {
    if (s.colliding) hit = true;
    if (curve && curve.length > 1) {
      xt = Math.max(xt, distToCurve(curve, s.true.x, s.true.y));
    }
    if (axis) {
      peakProgress = Math.max(peakProgress,
        (s.true.x - sx) * axis[0] + (s.true.y - sy) * axis[1]);
    }
    if (want0.heading !== undefined) {
      // Signed the way the turn is going, so only going PAST the target
      // counts -- being short of it is not overshoot.
      const d = bearingDiff(s.true.h, want0.heading);
      const dir = Math.sign(bearingDiff(want0.heading, sh)) || 1;
      peakHeadErr = Math.max(peakHeadErr, d * dir);
    }
    peakV = Math.max(peakV, Math.abs(s.vel.lin));
    peakW = Math.max(peakW, Math.abs(s.vel.ang));
    // Only count a reversal that carries real speed, or encoder-level
    // dither around zero reads as hunting.
    if (Math.abs(s.vel.lin) > 2 && Math.abs(prevV) > 2 &&
        Math.sign(s.vel.lin) !== Math.sign(prevV)) revs++;
    if (Math.abs(s.vel.lin) > 2) prevV = s.vel.lin;
  };

  while (api.routeRunning() && elapsed < budget) {
    api.step(10);
    elapsed += 10;
    track(JSON.parse(api.stateJson()));
  }
  // Settling is part of the answer: a motion that exits still moving leaves the
  // robot somewhere the route never asked for.
  const atExit = JSON.parse(api.stateJson());
  for (let i = 0; i < settleMs / 10; i++) {
    api.step(10);
    track(JSON.parse(api.stateJson()));
  }
  const s = JSON.parse(api.stateJson());
  const rep = JSON.parse(api.reportJson());
  const row = rep[rep.length - 1] || {};

  const out = {
    name: cs.name,
    ms: row.ms ?? elapsed,
    cut: !!row.cutOff,
    stalled: elapsed >= budget,
    hit: hit || !!row.hit,
    estIn: hyp(s.true.x, s.true.y, s.odom.x, s.odom.y),
    estDeg: Math.abs(bearingDiff(s.true.h, s.odom.h)),
    // How far it kept going after the motion returned.
    coastIn: hyp(atExit.true.x, atExit.true.y, s.true.x, s.true.y),
    endIn: null, endDeg: null, driftIn: null, xtIn: curve ? xt : null,
    // Inches driven past the target along the approach axis. A motion that
    // sails through its waypoint and comes back is the single most visible
    // sign of a badly tuned loop, and the final pose cannot show it.
    overIn: axis ? Math.max(0, peakProgress - axisLen) : null,
    overDeg: want0.heading !== undefined ? Math.max(0, peakHeadErr) : null,
    // Along-track direction changes above 2 in/s: 0 for a motion that
    // decelerates into its target, 1 for one that overshoots and backs up,
    // more than 1 for one that hunts.
    revs,
    peakV, peakW,
  };

  const want = cs.want || {};
  if (cs.path) {
    const last = cs.path[cs.path.length - 1];
    out.endIn = hyp(s.true.x, s.true.y, last[0], last[1]);
  } else if (want.point) {
    out.endIn = hyp(s.true.x, s.true.y, want.point[0], want.point[1]);
  }
  if (want.heading !== undefined) {
    out.endDeg = Math.abs(bearingDiff(s.true.h, want.heading));
  } else if (want.face) {
    // turnToPoint aims from wherever the body ended up, which a turn that
    // translates makes different from where it started.
    const aim = 90 - Math.atan2(want.face[1] - s.true.y,
                                want.face[0] - s.true.x) * 180 / Math.PI;
    out.endDeg = Math.abs(bearingDiff(s.true.h, aim));
  }
  if (!want.point && !cs.path && !cs.noDrift) {
    out.driftIn = hyp(s.true.x, s.true.y, sx, sy);
  }
  return out;
}

export function runBattery(api, opts = {}) {
  return BATTERY.map(cs => runCase(api, cs, opts));
}

// ============================================================================
// Scoring
// ============================================================================
//
// Bounds are what "accurate enough to route on" means, and they are the sim's
// bounds, not the robot's -- see kSimTuning in sim/bridge/bridge.cpp. A miss is
// a failure of the TUNING, so the gate is worth having in CI.
export const BOUNDS = {
  endIn: 0.5,       // inches from the commanded point
  endDeg: 1.5,      // degrees from the commanded heading
  driftIn: 1.0,     // how far a turn may translate the body
  xtIn: 2.0,        // peak cross-track on a followed path
  // Inches past the waypoint. Not zero: the plant needs v*tau to stop, so
  // some is unavoidable at speed. 0.75 in is about a twentieth of the body,
  // small enough that a route drawn clear of a post stays clear of it.
  overIn: 0.75,
  overDeg: 2.0,     // degrees past the commanded heading
  // A motion that decelerates into its target reverses zero times. One is an
  // overshoot it had to drive out of; more than one is hunting.
  revs: 0.5,
  // Travel after the motion returns. RAMSETE ends its profile rather than
  // braking, so some is expected, and endIn is measured after it settles --
  // this only has to catch a motion that leaves the target once it arrives.
  coastIn: 1.5,
};

export function score(rows) {
  let worst = 0, fails = [];
  const agg = { endIn: 0, endDeg: 0, driftIn: 0, xtIn: 0, coastIn: 0, ms: 0 };
  for (const r of rows) {
    if (r.hit) fails.push(`${r.name}: collided`);
    if (r.stalled) fails.push(`${r.name}: never returned`);
    if (r.cut) fails.push(`${r.name}: cut off by its timeout`);
    for (const k of Object.keys(BOUNDS)) {
      if (r[k] === null || r[k] === undefined) continue;
      agg[k] = Math.max(agg[k], r[k]);
      const over = r[k] / BOUNDS[k];
      if (over > 1) fails.push(`${r.name}: ${k} ${r[k].toFixed(2)} > ${BOUNDS[k]}`);
      worst = Math.max(worst, over);
    }
    agg.ms += r.ms;
  }
  // One number for the sweeps: how close every case is to its own bound, plus
  // a small time term so two tunings that both pass are separated by the
  // faster one rather than by noise.
  const cost = rows.reduce((sum, r) => {
    let s = 0;
    for (const k of Object.keys(BOUNDS)) {
      if (r[k] === null || r[k] === undefined) continue;
      s += (r[k] / BOUNDS[k]) ** 2;
    }
    if (r.cut || r.stalled) s += 25;
    if (r.hit) s += 100;
    return sum + s;
  }, 0) + agg.ms / 100000;
  return { cost, worst, fails, agg };
}

// ============================================================================
// CLI
// ============================================================================

const n = (v, w, d = 2) =>
  (v === null || v === undefined ? '–' : v.toFixed(d)).padStart(w);

if (import.meta.url === 'file://' + process.argv[1].replace(/\\/g, '/') ||
    process.argv[1].endsWith('accuracy.mjs')) {
  const args = process.argv.slice(2);
  const api = await boot();

  const locArg = args.find(a => a.startsWith('--loc='));
  const loc = locArg ? parseInt(locArg.slice(6), 10) : 0;
  // responseTauMs is the only thing in the sim standing in for motor dynamics,
  // so it is the one plant parameter worth re-running the battery against: a
  // tuning that only holds at the default 70 ms is fitted to the model.
  const tauArg = args.find(a => a.startsWith('--tau='));
  if (tauArg) api.setParam('responseTauMs', parseFloat(tauArg.slice(6)));
  // Put the robot's own numbers back, to see what the sim's overrides buy. Not
  // a prediction about the robot: this is its tuning against the sim's plant.
  if (args.includes('--stock')) {
    api.tuningStock();
    console.log('kSimTuning reverted -- these are the values config.cpp ships:');
    for (const t of JSON.parse(api.tuningJson())) {
      console.log(`   ${t.key.padEnd(18)} ${t.robot}  (sim uses ${t.sim})`);
    }
    console.log('');
  }
  const rows = runBattery(api, { loc });
  const s = score(rows);

  if (args.includes('--json')) {
    console.log(JSON.stringify({ rows, ...s }, null, 1));
  } else {
    const locName = ['dead reckoning', 'MCL', 'RCL', 'MCL+RCL', 'EKF',
                     'MCL+EKF', 'RCL+EKF', 'MCL+RCL+EKF'][loc & 7];
    console.log(`== routing accuracy (${locName}) ==`);
    console.log('   end: how far ground truth finished from what the command asked for');
    console.log('');
    console.log('  motion                 end in  end deg  over in  over deg'
                + '  revs  x-track   peak v   peak w      ms');
    for (const r of rows) {
      const flag = r.hit ? ' HIT' : r.stalled ? ' STALL' : r.cut ? ' CUTOFF' : '';
      console.log(`  ${r.name.padEnd(20)} ${n(r.endIn, 7)} ${n(r.endDeg, 8)}`
        + ` ${n(r.overIn, 8)} ${n(r.overDeg, 9)} ${n(r.revs, 5, 0)}`
        + ` ${n(r.xtIn, 8)} ${n(r.peakV, 8, 1)} ${n(r.peakW, 8, 0)}`
        + ` ${n(r.ms, 7, 0)}${flag}`);
    }
    console.log('');
    console.log(`  worst, as a fraction of its bound: ${s.worst.toFixed(2)}`
                + `   total ${(s.agg.ms / 1000).toFixed(1)}s   cost ${s.cost.toFixed(2)}`);
    for (const f of s.fails) console.log(`  [FAIL] ${f}`);
    if (!s.fails.length) console.log('  [PASS] every motion is inside its bound');
  }
  if (args.includes('--gate') && s.fails.length) process.exitCode = 1;
}
