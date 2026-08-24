// Searches MCL/EKF noise parameters instead of printing a table for you to read.
// Coordinate descent: each parameter is tried above and below its current value,
// the best feasible move is kept, repeat.
//
//   node sim/test/autotune.mjs
//   node sim/test/autotune.mjs --passes 3 --seeds 4
//
// Kidnap recovery is a HARD CONSTRAINT, not a term in the cost. Every parameter
// that tightens the estimate also narrows the cloud's search, so a plain
// weighted sum walks straight to a configuration that tracks beautifully and can
// never relocate. A candidate that recovers less often than the baseline is
// rejected outright no matter how well it tracks.
import { loadSim } from './harness.mjs';

const M = await loadSim();
const c = (n, r, a) => M.cwrap(n, r, a);
const api = {
  init: c('sim_init', null, []), reset: c('sim_reset', null, ['number','number','number']),
  step: c('sim_step', null, ['number']), setLoc: c('sim_set_localization', null, ['number']),
  stateJson: c('sim_state_json', 'string', []), setParam: c('sim_set_param', null, ['string','number']),
  routeClear: c('sim_route_clear', null, []), routeRun: c('sim_route_run', null, []),
  routePush: c('sim_route_push', null, ['number','number','number','number','number']),
  routeRunning: c('sim_route_running', 'number', []),
  pathClear: c('sim_path_clear', null, []), pathPush: c('sim_path_push', null, ['number','number','number']),
  teleport: c('sim_teleport', null, ['number','number','number']),
};
api.init();
const st = () => JSON.parse(api.stateJson());
const mean = a => a.reduce((x, y) => x + y, 0) / (a.length || 1);

const arg = (name, def) => {
  const i = process.argv.indexOf('--' + name);
  return i >= 0 ? Number(process.argv[i + 1]) : def;
};
const PASSES = arg('passes', 2);
const N = arg('seeds', 3);
// Search on one set of seeds, validate on a disjoint one. Scoring the winner on
// the seeds it was chosen against measures how well the search fit the noise.
const SEEDS = Array.from({ length: N }, (_, i) => i + 1);
const HOLDOUT = Array.from({ length: N }, (_, i) => i + 101);
let seeds = SEEDS;

const POSTS = [[-47,-23.5],[-23.5,-47],[23.5,47],[47,23.5],[0,0],[-47,23.5],[23.5,-47],[-23.5,47],[47,-23.5]];
const PATHS = {
  'tight S': u => [-50 + 44*u, -42 + 17*Math.sin(u*Math.PI*1.5)],
  'hairpin': u => [-52 + 22*Math.sin(u*Math.PI*0.9), -52 + 46*u],
};
function build(name) {
  api.pathClear();
  const pts = [];
  for (let i = 0; i <= 70; i++) pts.push(PATHS[name](i / 70));
  let wall = 99, post = 99;
  for (const [x, y] of pts) {
    wall = Math.min(wall, 70.2 - Math.max(Math.abs(x), Math.abs(y)));
    for (const [px, py] of POSTS) post = Math.min(post, Math.hypot(x-px, y-py) - 2);
  }
  if (wall < 2 || post < 2) throw new Error(`path "${name}" clearance wall ${wall.toFixed(1)} post ${post.toFixed(1)}`);
  for (let i = 0; i < pts.length; i++) {
    const [x, y] = pts[i];
    const [nx, ny] = pts[Math.min(i+1, pts.length-1)];
    const [px, py] = pts[Math.max(i-1, 0)];
    api.pathPush(x, y, 90 - Math.atan2(ny-py, nx-px) * 180/Math.PI);
  }
  return pts;
}

// Fixed context. Realistic odometry error, or dead reckoning is ground truth and
// none of this means anything.
const WORLD = {
  collideObstacles: 0,
  encoderSlip: 0.015, imuDriftDegPerSec: 0.015, imuNoiseDeg: 0.15,
  ramseteB: 48, ramseteZeta: 0.85, ramseteTolerance: 3, ramseteHeadingTol: 4,
  ramseteStall: 1.5, ramseteMaxAccel: 50, ramseteLatAccel: 60, ramseteMaxSpeed: 127,
  ramseteVelCtrl: 0, mtpBearingHold: 1.5, mtpTolerance: 1,
};

// name: [start, min, max]. Everything the bridge exposes for the two filters
// that plausibly trades accuracy against robustness.
const SPACE = {
  mclResampleVar:      [1.5,   0.3,  4.0],
  mclThetaResampleVar: [0.01,  0.002, 0.05],
  mclSyncProp:         [0.06,  0.01, 0.30],
  mclImeVariance:      [0.12,  0.02, 0.60],
  mclHorizProp:        [0.03,  0.005, 0.15],
  mclHorizConst:       [0.03,  0.005, 0.15],
  ekfSyncProp:         [0.12,  0.02, 1.00],
  ekfThetaSyncProp:    [0.20,  0.03, 1.00],
  ekfMclSigmaScale:    [1.6,   0.8,  4.0],
  ekfMclSigmaFloor:    [1.2,   0.3,  4.0],
};

function applyAll(cfg, seed) {
  for (const [k, v] of Object.entries(WORLD)) api.setParam(k, v);
  for (const [k, [start]] of Object.entries(SPACE)) api.setParam(k, start);
  for (const [k, v] of Object.entries(cfg)) api.setParam(k, v);
  api.setParam('seed', seed);
}

function track(cfg) {
  const E = [], J = [];
  let unsettled = 0;
  for (const name of ['tight S', 'hairpin']) {
    for (const sd of seeds) {
      applyAll(cfg, sd);
      const pts = build(name);
      const h0 = 90 - Math.atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0]) * 180/Math.PI;
      api.reset(pts[0][0], pts[0][1], h0);
      api.setLoc(5);
      applyAll(cfg, sd);
      for (let i = 0; i < 60; i++) api.step(10);
      api.routeClear(); api.routePush(13, 0, pts.length, 0, 9000); api.routeRun();
      for (let i = 0; i < 5; i++) api.step(10);
      let n = 0, errSum = 0, jSum = 0, jN = 0, prev = null;
      while (api.routeRunning() && n < 900) {
        api.step(10); n++;
        const s = st();
        errSum += Math.hypot(s.odom.x - s.true.x, s.odom.y - s.true.y);
        const e = { x: s.odom.x, y: s.odom.y };
        if (prev) {
          const dx = e.x - prev.x, dy = e.y - prev.y;
          if (prev.dx !== undefined) { jSum += Math.hypot(dx-prev.dx, dy-prev.dy)**2; jN++; }
          prev = { ...e, dx, dy };
        } else prev = { ...e };
      }
      E.push(errSum/(n||1));
      J.push(Math.sqrt(jSum/(jN||1)));
      if (n >= 890) unsettled++;
    }
  }
  return { err: mean(E), jerk: mean(J), unsettled };
}

function kidnap(cfg) {
  let hit = 0;
  const times = [];
  for (const sd of seeds) {
    applyAll(cfg, sd);
    const pts = build('hairpin');
    api.reset(pts[0][0], pts[0][1], 90);
    api.setLoc(5);
    applyAll(cfg, sd);
    for (let i = 0; i < 80; i++) api.step(10);
    const s0 = st();
    api.teleport(s0.true.x + 18, s0.true.y + 16, s0.true.h);
    let n = 0;
    while (n < 900) {
      api.step(10); n++;
      const s = st();
      if (Math.hypot(s.odom.x - s.true.x, s.odom.y - s.true.y) < 3.0) { hit++; times.push(n*10); break; }
    }
  }
  return { hit, of: seeds.length, ms: times.length ? mean(times) : Infinity };
}

// err and jerk live on different scales, so both are normalised against the
// baseline and weighted. Jerk is what the motion controllers actually feel.
let base = null;
function score(cfg) {
  const t = track(cfg);
  const k = kidnap(cfg);
  const m = { ...t, ...k };
  if (!base) return m;
  m.cost = 0.5 * (t.err / base.err) + 0.5 * (t.jerk / base.jerk)
           + 0.25 * t.unsettled;
  m.feasible = k.hit >= base.hit && k.ms <= base.ms * 1.5;
  return m;
}

const line = (label, m) =>
  `${label.padEnd(34)} err ${m.err.toFixed(3)}  jerk ${m.jerk.toFixed(4)}` +
  `  kidnap ${m.hit}/${m.of} ${m.ms === Infinity ? 'never' : m.ms.toFixed(0)+'ms'}` +
  (m.cost !== undefined ? `  cost ${m.cost.toFixed(4)}` : '') +
  (m.unsettled ? `  ${m.unsettled} UNSETTLED` : '') +
  (m.feasible === false ? '  INFEASIBLE' : '');

console.log(`autotuning MCL/EKF: ${PASSES} passes, ${SEEDS.length} search seeds, ` +
            `${HOLDOUT.length} holdout seeds`);
console.log('kidnap recovery is a hard constraint; candidates that lose it are rejected.\n');

const a = track({}), b = track({});
if (Math.abs(a.err - b.err) > 1e-9 || Math.abs(a.jerk - b.jerk) > 1e-9) {
  console.error('[FAIL] repeated identical config gave different numbers.');
  console.error('State is leaking between trials; the search would be noise.');
  process.exit(1);
}
console.log('[PASS] trials are independent');

base = score({});
base.cost = 1.0 + 0.25 * base.unsettled;
base.feasible = true;
console.log(line('baseline', base) + '\n');

let current = {};
let currentCost = base.cost;

for (let pass = 1; pass <= PASSES; pass++) {
  console.log(`-- pass ${pass} --`);
  let improved = false;
  for (const [key, [start, lo, hi]] of Object.entries(SPACE)) {
    const now = current[key] ?? start;
    let bestVal = null, bestM = null;
    for (const mult of [0.6, 1.6]) {
      const val = Math.min(hi, Math.max(lo, now * mult));
      if (Math.abs(val - now) < 1e-6) continue;
      const m = score({ ...current, [key]: val });
      const tag = `  ${key}=${val.toFixed(4)}`;
      if (!m.feasible) { console.log(line(tag, m)); continue; }
      console.log(line(tag, m));
      if (m.cost < currentCost - 1e-4 && (!bestM || m.cost < bestM.cost)) {
        bestVal = val; bestM = m;
      }
    }
    if (bestM) {
      current = { ...current, [key]: bestVal };
      currentCost = bestM.cost;
      improved = true;
      console.log(`  -> keep ${key}=${bestVal.toFixed(4)}  cost ${currentCost.toFixed(4)}`);
    }
  }
  if (!improved) { console.log('  no feasible improvement; stopping\n'); break; }
  console.log('');
}

const finalM = score(current);
console.log('');
console.log('== result, on the seeds the search used ==');
console.log(line('baseline', base));
console.log(line('tuned', finalM));

if (Object.keys(current).length === 0) {
  console.log('');
  console.log('No change beat the defaults under the recovery constraint.');
  console.log('That is a real answer: the shipped values are already at the knee.');
} else {
  // Re-score both on seeds the search never saw. A win that does not survive
  // this was the search fitting noise, which it will happily do.
  seeds = HOLDOUT;
  const savedBase = base;
  const baseOut = score({});
  baseOut.cost = 1.0 + 0.25 * baseOut.unsettled;
  delete baseOut.feasible;
  base = baseOut;
  const tunedOut = score(current);
  base = savedBase;

  const jGain = (baseOut.jerk - tunedOut.jerk) / baseOut.jerk * 100;
  const eGain = (baseOut.err - tunedOut.err) / baseOut.err * 100;
  console.log('');
  console.log('== holdout, seeds the search never saw ==');
  console.log(line('baseline', baseOut));
  console.log(line('tuned', tunedOut));
  console.log('   jerk ' + (jGain >= 0 ? '-' : '+') + Math.abs(jGain).toFixed(1) + '%' +
              '  err ' + (eGain >= 0 ? '-' : '+') + Math.abs(eGain).toFixed(1) + '%' +
              '  kidnap ' + tunedOut.hit + '/' + tunedOut.of +
              ' vs ' + baseOut.hit + '/' + baseOut.of);

  const held = jGain > 5 && tunedOut.hit >= baseOut.hit;
  console.log('');
  if (held) {
    console.log('[HOLDS] the gain survives unseen seeds.');
  } else {
    console.log('[DOES NOT HOLD] the gain vanished on unseen seeds. That is');
    console.log('        noise-fitting, not tuning. Keep the defaults.');
  }

  console.log('');
  console.log('Changed:');
  for (const [k, v] of Object.entries(current)) {
    console.log('  ' + k + ': ' + SPACE[k][0] + ' -> ' + v.toFixed(4));
  }
  if (held) {
    const MAP = {
      mclResampleVar: 'mcl.distResampleVariance', mclThetaResampleVar: 'mcl.thetaResampleVariance',
      mclSyncProp: 'mcl.distSyncProp', mclImeVariance: 'mcl.imeVariance',
      mclHorizProp: 'mcl.horizDependentVarianceProp', mclHorizConst: 'mcl.horizConstantNoise',
      ekfSyncProp: 'ekf.syncProp', ekfThetaSyncProp: 'ekf.thetaSyncProp',
      ekfMclSigmaScale: 'ekf.mclSigmaScale', ekfMclSigmaFloor: 'ekf.mclSigmaFloor',
    };
    console.log('');
    console.log('Apply:');
    for (const [k, v] of Object.entries(current)) {
      console.log('  config.' + MAP[k] + ' = ' + v.toFixed(4) + 'f;');
    }
    console.log('');
    console.log('Simulator-fitted. No traction, and the odometry error is sensor');
    console.log('staleness rather than slip -- re-check on hardware.');
  }
}
