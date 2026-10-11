// RCL's pose-jump guard: automatic corrections are rate limited.
//
//   node sim/test/rcl_guard.mjs
//
// Every automatic RCL correction passes a token bucket, so the distance it
// moves the pose over any window of T seconds is at most
//   maxCorrectionBurst + maxCorrectionRate * T.
// A recovery used to step up to maxSyncDist (5 in) per reading -- ~150 in/s,
// which reads as a teleport -- and a recovery can be entered on bad readings,
// e.g. a robot parked in front of a sensor. These cases check the bound
// against the pose odometry actually reports, and that a real displacement is
// still found quickly.
import createChadSim from '../web/chad_sim.mjs';

const M = await createChadSim();
const c = (n, r, a) => M.cwrap(n, r, a);
const api = {
  init: c('sim_init', null, []),
  reset: c('sim_reset', null, ['number', 'number', 'number']),
  step: c('sim_step', null, ['number']),
  setDrive: c('sim_set_drive', null, ['number', 'number']),
  setLoc: c('sim_set_localization', null, ['number']),
  setPods: c('sim_set_odom_pods', null, ['number']),
  stateJson: c('sim_state_json', 'string', []),
  actorAdd: c('sim_actor_add', 'number', ['number', 'number', 'number', 'number']),
  actorClear: c('sim_actor_clear', null, []),
  setFar: c('sim_set_far_side_gate', null, ['number', 'number']),
  distFault: c('sim_distance_fault', null, ['number', 'number']),
  pieceClear: c('sim_piece_clear', null, []),
};

// RclTracking::Config defaults; config.cpp does not override them.
const RATE = 30.0;     // maxCorrectionRate, in/s
const BURST = 2.0;     // maxCorrectionBurst, in
// Odometry quantisation and the stationary robot's settle.
const TOL = 0.25;
const STEP_MS = 5;

const st = () => JSON.parse(api.stateJson());
const run = (ms) => { for (let i = 0; i < ms; i += STEP_MS) api.step(STEP_MS); };

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};

/// RCL only, no pieces, robot stationary. pods 2 = drive encoders only.
const begin = (x, y, h, pods = 0) => {
  for (let i = 0; i < 8; i++) api.distFault(i, 0);
  api.actorClear();
  api.setFar(0, 0);
  api.setPods(pods);
  api.init();
  api.reset(x, y, h);
  api.pieceClear();
  api.setLoc(2);
  run(600);
};

/// Samples the reported pose every STEP_MS for ms, calling during(tMs) first.
const record = (ms, during = () => {}) => {
  const out = [];
  for (let t = 0; t <= ms; t += STEP_MS) {
    during(t);
    api.step(STEP_MS);
    const s = st();
    out.push({ t: s.t * 1000, x: s.odom.x, y: s.odom.y, err: s.err.dist,
               displaced: s.rcl2.displaced });
  }
  return out;
};

/// Worst excess, over every window of the trace, of the distance the pose
/// moved against what the guard allows in that window. <= 0 means bounded.
const worstWindowExcess = (trace) => {
  let worst = -Infinity, at = '';
  for (let a = 0; a < trace.length; a++) {
    for (let b = a + 1; b < trace.length; b++) {
      const moved = Math.hypot(trace[b].x - trace[a].x, trace[b].y - trace[a].y);
      const T = (trace[b].t - trace[a].t) / 1000;
      const excess = moved - (BURST + RATE * T + TOL);
      if (excess > worst) {
        worst = excess;
        at = `${moved.toFixed(2)} in in ${(T * 1000).toFixed(0)} ms`;
      }
    }
  }
  return { worst, at };
};

const worstStep = (trace) => {
  let worst = 0;
  for (let i = 1; i < trace.length; i++) {
    worst = Math.max(worst, Math.hypot(trace[i].x - trace[i - 1].x,
                                       trace[i].y - trace[i - 1].y));
  }
  return worst;
};

/// First time after which err stays under bound for the rest of the trace.
const settledAfter = (trace, bound) => {
  for (let i = trace.length - 1; i >= 0; i--) {
    if (trace[i].err >= bound) {
      return i + 1 < trace.length ? trace[i + 1].t - trace[0].t : Infinity;
    }
  }
  return 0;
};

/// Worst excess of the summed applied corrections (rcl2.sync, one RCL cycle
/// per 10 ms sample) over every window, against burst + rate * T.
const worstSyncExcess = (syncs, periodMs) => {
  let worst = -Infinity, at = '';
  for (let a = 0; a < syncs.length; a++) {
    let sum = 0;
    for (let b = a; b < syncs.length; b++) {
      sum += syncs[b];
      const T = (b - a) * periodMs / 1000;
      const excess = sum - (BURST + RATE * T + 0.05);
      if (excess > worst) {
        worst = excess;
        at = `${sum.toFixed(2)} in over ${b - a + 1} cycles`;
      }
    }
  }
  return { worst, at };
};

// ---------------------------------------------------------------------------
console.log('\n== a recovery is glided, not teleported ==');
// Backed into the west wall on drive encoders only: the body stops, odometry
// runs on through the wall, the back beam reads long and RCL declares a
// displacement and recovers it. (A sim_teleport shove does not get here with
// RCL alone: the beam that disagrees cannot vote until the filter is already
// displaced.)
begin(-62, -48, 90, 2);
{
  const RCL_MS = 10;
  const syncs = [];
  let displaced = false;
  api.setDrive(-127, -127);
  for (let t = 0; t < 600; t += RCL_MS) {
    api.step(RCL_MS);
    const s = st();
    syncs.push(s.rcl2.sync);
    displaced = displaced || s.rcl2.displaced;
  }
  api.setDrive(0, 0);
  const errAtStop = st().err.dist;
  // The wheels take a moment to stop; past that only corrections move it.
  const coast = [];
  for (let t = 0; t < 300; t += RCL_MS) {
    api.step(RCL_MS);
    const s = st();
    syncs.push(s.rcl2.sync);
    coast.push(s.err.dist);
  }
  const trace = record(1700);
  for (const p of trace) displaced = displaced || p.displaced;

  const sync = worstSyncExcess(syncs, RCL_MS);
  const largest = Math.max(...syncs);
  const win = worstWindowExcess(trace);
  const settle = 300 + settledAfter(trace, 2.0);
  check('the runaway is declared a displacement', displaced);
  check('no correction exceeds the burst',
        largest <= BURST + RATE * RCL_MS / 1000 + 0.05,
        `largest ${largest.toFixed(2)} in in one ${RCL_MS} ms cycle`);
  check('corrections over any window stay under burst + rate * T',
        sync.worst <= 0, `worst ${sync.at}, excess ${sync.worst.toFixed(2)} in`);
  check('stationary, the pose never moves faster than the guard allows',
        win.worst <= 0, `worst ${win.at}, excess ${win.worst.toFixed(2)} in`);
  check('and it is still found within 1 s of stopping', settle <= 1000,
        `${errAtStop.toFixed(1)} in at the stop, within 2 in after ${settle.toFixed(0)} ms`);
  check('and stays found', trace[trace.length - 1].err < 1.0,
        `${trace[trace.length - 1].err.toFixed(2)} in at the end`);
}
api.setPods(0);

// ---------------------------------------------------------------------------
console.log('\n== a robot parked behind the only x beam, for 400 ms ==');
// At (-24,-48) facing +x the back beam is the only one ranging the x axis
// (the front one is out of range). A robot pulled up 5 in behind it reads
// ~40 in short: grossly enough to count as displacement evidence on its own,
// so RCL enters recovery and that one wrong beam is all the x axis has. The
// guard is what bounds how far it can drag the pose before it leaves.
begin(-24, -48, 90);
{
  const err0 = st().err.dist;
  let id = -1;
  const trace = record(2000, (t) => {
    if (t === 0) id = api.actorAdd(-38, -48, 90, 0);
    if (t === 400 && id >= 0) { api.actorClear(); id = -1; }
  });
  const during = trace.filter(p => p.t - trace[0].t <= 600);
  const worstErr = Math.max(...during.map(p => p.err));
  const step = worstStep(trace);
  const win = worstWindowExcess(trace);
  check('no single cycle moves the pose more than the burst',
        step <= BURST + RATE * STEP_MS / 1000 + TOL,
        `worst ${step.toFixed(2)} in per ${STEP_MS} ms`);
  check('no window moves it more than burst + rate * T',
        win.worst <= 0, `worst ${win.at}, excess ${win.worst.toFixed(2)} in`);
  check('the transient drags the pose no further than the guard allows in 400 ms',
        worstErr <= err0 + BURST + RATE * 0.6 + TOL,
        `${err0.toFixed(2)} in -> worst ${worstErr.toFixed(2)} in`);
  check('and it comes back once the robot leaves',
        trace[trace.length - 1].err < 2.0,
        `${trace[trace.length - 1].err.toFixed(2)} in at the end`);
}

console.log(failures === 0 ? '\nALL CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
