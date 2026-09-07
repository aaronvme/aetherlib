// Sweeps the gains that can only be scored against a known true pose, which the
// robot does not have and the simulator does. Run it after characterize.cpp has
// given you kV/kS/trackWidth from the real drivetrain.
//
//   node sim/test/tune.mjs              all sections
//   node sim/test/tune.mjs ramsete      one section: ramsete | mcl | tolerance
//
// Every trial rewrites every key it might have changed, because sim_reset does
// NOT restore filter or motion config -- without that, one trial leaks into the
// next and the sweep reports the previous row's answer. The control section
// exists to prove that is not happening.
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
const here = dirname(fileURLToPath(import.meta.url));
const createChadSim = (await import('file://' + join(here, '..', 'web', 'chad_sim.mjs'))).default;

const M = await createChadSim();
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
const worst = a => Math.max(...a);

// Paths are validated for wall and post clearance; an out-of-bounds waypoint
// scores as a controller failure and quietly ruins a whole sweep.
const POSTS = [[-47,-23.5],[-23.5,-47],[23.5,47],[47,23.5],[0,0],[-47,23.5],[23.5,-47],[-23.5,47],[47,-23.5]];
const PATHS = {
  'gentle S': u => [-58 + 52*u, -52 + 13*Math.sin(u*Math.PI)],
  'tight S':  u => [-50 + 44*u, -42 + 17*Math.sin(u*Math.PI*1.5)],
  'hairpin':  u => [-52 + 22*Math.sin(u*Math.PI*0.9), -52 + 46*u],
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
  if (wall < 2 || post < 2) throw new Error(`path "${name}" has no clearance: wall ${wall.toFixed(1)} post ${post.toFixed(1)}`);
  for (let i = 0; i < pts.length; i++) {
    const [x, y] = pts[i];
    const [nx, ny] = pts[Math.min(i+1, pts.length-1)];
    const [px, py] = pts[Math.max(i-1, 0)];
    api.pathPush(x, y, 90 - Math.atan2(ny-py, nx-px) * 180/Math.PI);
  }
  return pts;
}

// Realistic V5 odometry error. Without this, dead reckoning is ground truth and
// every localization result is meaningless.
const DEFAULTS = {
  collideObstacles: 0,
  encoderSlip: 0.015, imuDriftDegPerSec: 0.015, imuNoiseDeg: 0.15,
  ramseteB: 48, ramseteZeta: 0.85, ramseteTolerance: 3, ramseteHeadingTol: 4,
  ramseteStall: 1.5, ramseteMaxAccel: 50, ramseteLatAccel: 60, ramseteMaxSpeed: 127,
  ramseteVelCtrl: 0,
  mtpBearingHold: 1.5, mtpTolerance: 1,
  ekfSyncProp: 0.12, ekfThetaSyncProp: 0.20,
  mclResampleVar: 1.5, mclThetaResampleVar: 0.01, mclSyncProp: 0.06,
  mclMaxRandomFrac: 0.0, mclRandWeight: 0.0008, mclImeVariance: 0.12,
};
function applyAll(cfg, seed) {
  for (const [k, v] of Object.entries(DEFAULTS)) api.setParam(k, v);
  for (const [k, v] of Object.entries(cfg)) api.setParam(k, v);
  api.setParam('seed', seed);
}

function followPath(name, mode, cfg, seeds) {
  const E = [], X = [], T = [], H = [], J = [];
  let unsettled = 0;
  for (const sd of seeds) {
    applyAll(cfg, sd);
    const pts = build(name);
    const h0 = 90 - Math.atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0]) * 180/Math.PI;
    api.reset(pts[0][0], pts[0][1], h0);
    api.setLoc(mode);
    applyAll(cfg, sd);            // setLoc can re-seed filter state; reassert
    for (let i = 0; i < 60; i++) api.step(10);
    api.routeClear(); api.routePush(13, 0, pts.length, 0, 9000); api.routeRun();
    for (let i = 0; i < 5; i++) api.step(10);
    let n = 0, sum = 0, peak = 0, jerkSum = 0, jerkN = 0, prev = null;
    while (api.routeRunning() && n < 900) {
      api.step(10); n++;
      const s = st();
      let d = 1e9;
      for (const [px, py] of pts) d = Math.min(d, Math.hypot(s.true.x-px, s.true.y-py));
      sum += d; peak = Math.max(peak, d);
      // Second difference of the ESTIMATE: a correction written as a step shows
      // up here and nowhere in the tracking columns. Without it a sweep happily
      // recommends the jerkiest setting because it tracks a hair better.
      const e = { x: s.odom.x, y: s.odom.y };
      if (prev) {
        const dx = e.x - prev.x, dy = e.y - prev.y;
        if (prev.dx !== undefined) {
          jerkSum += Math.hypot(dx - prev.dx, dy - prev.dy) ** 2; jerkN++;
        }
        prev = { ...e, dx, dy };
      } else prev = { ...e };
    }
    for (let i = 0; i < 20; i++) api.step(10);
    const s = st(), last = pts[pts.length-1];
    const end = Math.hypot(s.true.x-last[0], s.true.y-last[1]);
    E.push(sum/(n||1)); X.push(peak); T.push(n/100); H.push(end);
    J.push(Math.sqrt(jerkSum/(jerkN||1)));
    if (n >= 890) unsettled++;
  }
  return { xt: mean(E), peak: mean(X), t: mean(T), end: mean(H),
           jerk: mean(J), endWorst: worst(H), unsettled };
}

function kidnapRecovery(mode, cfg, seeds) {
  const R = [];
  for (const sd of seeds) {
    applyAll(cfg, sd);
    const pts = build('hairpin');
    api.reset(pts[0][0], pts[0][1], 90);
    api.setLoc(mode);
    applyAll(cfg, sd);
    for (let i = 0; i < 80; i++) api.step(10);
    const s0 = st();
    api.teleport(s0.true.x + 18, s0.true.y + 16, s0.true.h);
    let n = 0, ok = -1;
    while (n < 900) {
      api.step(10); n++;
      const s = st();
      if (Math.hypot(s.odom.x-s.true.x, s.odom.y-s.true.y) < 3.0) { ok = n; break; }
    }
    R.push(ok);
  }
  const good = R.filter(x => x > 0);
  return { ms: good.length ? mean(good) * 10 : -1, hit: good.length, of: R.length };
}

const SEEDS = [1,2,3,4];
const fmt = r => `xt ${r.xt.toFixed(2)}in peak ${r.peak.toFixed(2)}in jerk ${r.jerk.toFixed(4)}` +
                 `  ${r.t.toFixed(2)}s  end ${r.end.toFixed(2)}in` +
                 `${r.unsettled ? `  ${r.unsettled}/${SEEDS.length} UNSETTLED` : ''}`;

function control() {
  console.log('== control: the same config twice must give the same numbers ==');
  const a = followPath('hairpin', 5, {}, SEEDS);
  const b = followPath('hairpin', 5, {}, SEEDS);
  const same = Math.abs(a.xt - b.xt) < 1e-9 && Math.abs(a.t - b.t) < 1e-9;
  console.log(`   run A  ${fmt(a)}`);
  console.log(`   run B  ${fmt(b)}`);
  console.log(same ? '   [PASS] trials are independent\n'
                   : '   [FAIL] state is leaking between trials -- results below are junk\n');
  return same;
}

function tuneRamsete() {
  console.log('== RAMSETE b / zeta (default b=48 zeta=0.85) ==');
  console.log('   b sets how hard it pulls back to the path; zeta damps it.');
  for (const name of ['tight S', 'hairpin']) {
    console.log(`  ${name}`);
    for (const b of [24, 48, 72, 96]) {
      const r = followPath(name, 5, { ramseteB: b }, SEEDS);
      console.log(`    b=${String(b).padStart(3)}          ${fmt(r)}`);
    }
    for (const z of [0.7, 0.85, 1.0]) {
      const r = followPath(name, 5, { ramseteZeta: z }, SEEDS);
      console.log(`    zeta=${z.toFixed(2)}      ${fmt(r)}`);
    }
  }
  console.log('');
}

function tuneMcl() {
  console.log('== MCL / EKF: smoothness against kidnap recovery ==');
  console.log('   Both matter. A row that tracks better and never recovers is a loss.');
  const rows = [
    ['stock',                {}],
    ['mclSyncProp 0.03',     { mclSyncProp: 0.03 }],
    ['mclSyncProp 0.10',     { mclSyncProp: 0.10 }],
    ['mclResampleVar 1.0',   { mclResampleVar: 1.0 }],
    ['mclResampleVar 2.0',   { mclResampleVar: 2.0 }],
    ['ekfSyncProp 0.06',     { ekfSyncProp: 0.06 }],
    ['ekfSyncProp 1.00',     { ekfSyncProp: 1.00 }],
  ];
  for (const [label, cfg] of rows) {
    const t = followPath('hairpin', 5, cfg, SEEDS);
    const k = kidnapRecovery(5, cfg, SEEDS);
    console.log(`  ${label.padEnd(22)} ${fmt(t)}  kidnap ` +
                (k.ms < 0 ? 'NEVER' : `${k.ms.toFixed(0)}ms`) + ` ${k.hit}/${k.of}`);
  }
  console.log('   mclSyncProp is inert here: with the EKF up it owns the writeback,');
  console.log('   so those rows are meant to match stock exactly.');
  console.log('   Lowering mclResampleVar looks good on every tracking number and');
  console.log('   costs the ability to recover at all. Check the kidnap column.');
  console.log('   Raising ekfSyncProp tracks a hair better and is 5x jerkier,');
  console.log('   which is what the jerk column exists to stop you doing.');
  console.log('');
}

function tuneTolerance() {
  console.log('== settle tolerance against localization error ==');
  console.log('   Below the estimate error the settle test cannot fire and the');
  console.log('   motion burns its timeout. Pick the smallest that never fails.');
  for (const [nm, mode] of [['dead reckoning', 0], ['MCL only', 1], ['MCL+EKF', 5]]) {
    for (const tol of [1, 2, 3, 5]) {
      const r = followPath('hairpin', mode, { ramseteTolerance: tol }, SEEDS);
      console.log(`  ${nm.padEnd(15)} tol=${tol}  ${fmt(r)}`);
    }
  }
  console.log('');
}

const which = process.argv[2];
if (!control() && !which) process.exitCode = 1;
if (!which || which === 'ramsete')   tuneRamsete();
if (!which || which === 'mcl')       tuneMcl();
if (!which || which === 'tolerance') tuneTolerance();
