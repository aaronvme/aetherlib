// moveToPoint's last few inches, on a plant that can circle.
//
// Reported on the robot: approaches "start off strong but start to jitter as
// it gets closer to the point and even start circling around", and some time
// out short of the point. The stock plant never circles: it turns exactly
// about the tracking centre and has no static friction. These runs add both --
// the body pivots about a point ahead of the tracked one (pivotOffsetIn), and
// a side commanded under driveDeadband does not move -- and drive approaches
// that stress the end: short and long, pointed well off the bearing, arriving
// beside the point, forwards and reversed, on odometry and on RCL+EKF.
//
// Gate, per plant x localization x direction:
//   no circling (inside 6 in, > 270 deg more rotation than facing it needed),
//   no timeouts, mean end error under the plant's bound (0.8 in; 1.0 on harsh),
//   and no angular chatter: no run with >= 4 angular-output sign flips inside
//   10 in of the target, and under 1 flip per run on average.
//
//   node sim/test/near_target.mjs                 the gate
//   node sim/test/near_target.mjs --rows          print every run
//   node sim/test/near_target.mjs --plants=pivF2  one plant
//   node sim/test/near_target.mjs --set=bearingHold:3.5,targetTolIn:0.5
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, '..', '..');

async function boot() {
  const createChadSim =
    (await import('file://' + join(root, 'sim', 'web', 'chad_sim.mjs'))).default;
  const M = await createChadSim();
  const c = (n, r, a) => M.cwrap(n, r, a);
  const api = {
    init: c('sim_init', null, []),
    reset: c('sim_reset', null, ['number', 'number', 'number']),
    step: c('sim_step', null, ['number']),
    setLoc: c('sim_set_localization', null, ['number']),
    stateJson: c('sim_state_json', 'string', []),
    setParam: c('sim_set_param', null, ['string', 'number']),
    getParam: c('sim_get_param', 'number', ['string']),
    routeClear: c('sim_route_clear', null, []),
    routePush: c('sim_route_push', null, ['number', 'number', 'number', 'number', 'number']),
    routeRun: c('sim_route_run', null, []),
    routeRunning: c('sim_route_running', 'number', []),
    reportJson: c('sim_route_report_json', 'string', []),
    tuningStock: c('sim_tuning_stock', null, []),
    probeEnable: c('sim_probe_enable', null, []),
    probeDrain: c('sim_probe_drain', 'string', []),
  };
  api.init();
  // The robot's own numbers, not the sim's routing overrides.
  api.tuningStock();
  api.setParam('collideObstacles', 0);
  api.probeEnable();
  return api;
}

const OP_MOVE_TO = 1;
const MT_POINT = 2;
const DEG = Math.PI / 180;
const LOC_ODOM = 0;
const LOC_RCL_EKF = 6;

/// Signed smallest difference a-b of two bearings, degrees.
const bdiff = (a, b) => { let d = (a - b) % 360; if (d > 180) d -= 360; if (d < -180) d += 360; return d; };

// Plants. stock: the sim's default, which must not get worse. pivF2 / pivR2:
// turning about a point 2 in ahead of / behind the tracking centre, with a
// typical V5 static-friction deadband and a slower drive. harsh: more of both,
// plus IMU noise, encoder slip and a weak left side. Its looser end bound is
// geometry, not tuning: a held bearing cannot remove the cross-track that a
// 3 in pivot adds while the robot finishes turning onto it.
const PLANTS = {
  stock: { params: {}, endIn: 0.8 },
  pivF2: { params: { responseTauMs: 120, driveDeadband: 8, pivotOffsetIn: 2 }, endIn: 0.8 },
  pivR2: { params: { responseTauMs: 120, driveDeadband: 8, pivotOffsetIn: -2 }, endIn: 0.8 },
  harsh: { params: { responseTauMs: 150, driveDeadband: 8, pivotOffsetIn: 3,
                     imuNoiseDeg: 0.05, encoderSlip: 0.03, leftScale: 0.97 },
           endIn: 1.0 },
};

// VEX bearings: 0 = +y, 90 = +x. Away from walls and posts.
const TARGET = [-20, -20];
const APPROACH = [45, 90];
const DISTS = [3, 6, 12, 24];
const HDG_OFFS = [0, 45, 90];
const LAT_OFFS = [0, 4];

function cases(locs) {
  const out = [];
  for (const loc of locs) for (const dir of [1, -1]) for (const a of APPROACH)
    for (const d of DISTS) for (const ho of HDG_OFFS) for (const lat of LAT_OFFS) {
      const sx = TARGET[0] - d * Math.sin(a * DEG) + lat * Math.cos(a * DEG);
      const sy = TARGET[1] - d * Math.cos(a * DEG) - lat * Math.sin(a * DEG);
      const h = ((a + ho + (dir < 0 ? 180 : 0)) % 360 + 360) % 360;
      // Room for the turn as well as the drive: a robot pivoting about a
      // point ahead of its tracking centre swings away from a target beside
      // it while it turns, and has further to come back.
      out.push({ loc, dir, a, d, ho, lat, start: [sx, sy, h], timeout: 1200 + 60 * d + 5 * ho });
    }
  return out;
}

function runOne(api, cs) {
  const [sx, sy, sh] = cs.start;
  const [tx, ty] = TARGET;
  api.reset(sx, sy, sh);
  api.setLoc(cs.loc);
  for (let i = 0; i < 20; i++) api.step(10);
  api.probeDrain();

  api.routeClear();
  api.routePush(OP_MOVE_TO, tx, ty, cs.dir, cs.timeout);
  api.routeRun();

  const truth = [];
  let elapsed = 0;
  const sample = () => {
    const s = JSON.parse(api.stateJson());
    truth.push({ x: s.true.x, y: s.true.y, h: s.true.h });
  };
  while (api.routeRunning() && elapsed < cs.timeout + 1500) { api.step(10); elapsed += 10; sample(); }
  for (let i = 0; i < 30; i++) { api.step(10); sample(); }

  const raw = JSON.parse(api.probeDrain());
  const tel = [];
  for (let i = 0; i + 8 <= raw.length; i += 8) {
    if (raw[i + 1] !== MT_POINT) continue;
    tel.push({ x: raw[i + 2], y: raw[i + 3], ang: raw[i + 6] });
  }
  const rep = JSON.parse(api.reportJson());
  const row = rep[rep.length - 1] || {};

  const dist = p => Math.hypot(tx - p.x, ty - p.y);
  // Angular-output sign changes, ignoring outputs too small to move anything.
  // Counted from where the controller first saw itself within 10 in.
  const i10 = tel.findIndex(p => dist(p) < 10);
  let flips = 0, prev = 0;
  for (const p of i10 >= 0 ? tel.slice(i10) : []) {
    if (Math.abs(p.ang) < 5) continue;
    const s = Math.sign(p.ang);
    if (prev && s !== prev) flips++;
    prev = s;
  }

  // Circling, on ground truth: inside 6 in, the body turns more than 3/4 of a
  // turn beyond what facing the target needed when it first got there. A body
  // pivoting off its tracking centre legitimately turns past the naive bearing
  // error -- by up to 172 deg here, swinging wide and turning back -- where
  // the orbits this catches run 360 and up.
  let rot = 0, need = 0;
  const k6 = truth.findIndex(p => dist(p) < 6);
  if (k6 >= 0) {
    const p = truth[k6];
    const face = Math.atan2(tx - p.x, ty - p.y) / DEG + (cs.dir < 0 ? 180 : 0);
    need = Math.abs(bdiff(p.h, face));
    for (let i = k6 + 1; i < truth.length; i++) {
      if (dist(truth[i]) < 6) rot += Math.abs(bdiff(truth[i].h, truth[i - 1].h));
    }
  }
  const excessRot = rot - need;
  return { ...cs, ms: row.ms ?? elapsed, cut: !!row.cutOff,
           endErr: dist(truth[truth.length - 1]), flips, excessRot, circling: excessRot > 270,
           // Where the controller believed it finished: endErr minus this is
           // localization, not the motion.
           ctlEnd: tel.length ? dist(tel[tel.length - 1]) : NaN };
}

function summarize(rows) {
  const n = rows.length;
  const mean = k => rows.reduce((s, r) => s + r[k], 0) / n;
  const max = k => rows.reduce((m, r) => Math.max(m, r[k]), 0);
  return {
    n, circling: rows.filter(r => r.circling).length,
    cut: rows.filter(r => r.cut).length,
    jittery: rows.filter(r => r.flips >= 4).length,
    flips: mean('flips'), endErr: mean('endErr'), endErrMax: max('endErr'),
    ctlEnd: mean('ctlEnd'), ctlEndMax: max('ctlEnd'), ms: mean('ms'),
  };
}

const fmt = (s) =>
  `n=${s.n} circling=${s.circling} timeouts=${s.cut} chatter=${s.jittery}`
  + ` flips=${s.flips.toFixed(2)} ctl end=${s.ctlEnd.toFixed(2)}/max ${s.ctlEndMax.toFixed(2)} in`
  + ` true end=${s.endErr.toFixed(2)}/max ${s.endErrMax.toFixed(2)} in`
  + ` ms=${s.ms.toFixed(0)}`;

const args = process.argv.slice(2);
const opt = (k) => { const a = args.find(x => x.startsWith(`--${k}=`)); return a ? a.slice(k.length + 3) : null; };
const api = await boot();
const t0 = Date.now();
let failures = 0;
const allRows = [];
const check = (ok, label, detail) => {
  console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${label}  ${detail}`);
  if (!ok) failures++;
};

for (const [name, { params: plant, endIn }] of Object.entries(PLANTS)) {
  if (opt('plants') && !opt('plants').split(',').includes(name)) continue;
  api.tuningStock();
  const saved = Object.keys(plant).map(k => [k, api.getParam(k)]);
  for (const [k, v] of Object.entries(plant)) api.setParam(k, v);
  if (opt('set')) for (const kv of opt('set').split(',')) {
    const [k, v] = kv.split(':');
    api.setParam(k, parseFloat(v));
  }
  const rows = cases([LOC_ODOM, LOC_RCL_EKF]).map(cs => runOne(api, cs));
  allRows.push(...rows);
  for (const [k, v] of saved) api.setParam(k, v);
  for (const loc of [LOC_ODOM, LOC_RCL_EKF]) for (const dir of [1, -1]) {
    const g = rows.filter(r => r.loc === loc && r.dir === dir);
    const s = summarize(g);
    const label = `${name} ${loc === LOC_ODOM ? 'odom' : 'rcl+ekf'} ${dir > 0 ? 'fwd' : 'rev'}`;
    // End error as the controller saw it: on RCL+EKF the true error adds
    // the estimate's own, which no motion change can remove.
    check(s.circling === 0 && s.cut === 0 && s.jittery === 0 && s.flips < 1 &&
          s.ctlEnd < endIn, label, fmt(s));
  }
  if (args.includes('--rows')) {
    for (const r of rows) {
      console.log(`    loc${r.loc} ${r.dir > 0 ? 'F' : 'R'} a${r.a} d${r.d} ho${r.ho} lat${r.lat}:`
        + ` ms=${r.ms} cut=${r.cut ? 1 : 0} end=${r.endErr.toFixed(2)} flips=${r.flips}`
        + ` ctl=${r.ctlEnd.toFixed(2)} xrot=${r.excessRot.toFixed(0)}${r.circling ? ' CIRCLE' : ''}`);
    }
  }
}
if (args.includes('--json')) console.log(JSON.stringify(summarize(allRows)));
console.log(`(${((Date.now() - t0) / 1000).toFixed(1)} s)`);
console.log(failures ? `${failures} CHECK(S) FAILED` : 'ALL CHECKS PASSED');
process.exit(failures ? 1 : 0);
