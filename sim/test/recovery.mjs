// Kidnap recovery: can the localization stack find the robot again after
// something picks it up and puts it down somewhere else?
//
//   CHAD_SIM_MJS=... node sim/test/recovery.mjs
//   node sim/test/recovery.mjs --sweep            # config comparison table
//   node sim/test/recovery.mjs --sweep --mag 36 --budget 4000
//
// Three things this suite exists to stop:
//
//  1. A stationary trial. A parked robot gets no new information, so it cannot
//     relocate and the rate it reports is not the recovery rate.
//     sim/test/autotune.mjs's kidnap trial does not drive, which is why it says
//     0/4. Here the robot is driven on a fixed arc with sim_set_drive, so the
//     controller cannot react to the corrupted pose and change the experiment.
//  2. A one-seed pass. Recovery is stochastic; the assertions below are rates
//     over a swept seed set, sized so a single seed flipping is not a pass.
//  3. A recovery-only tuning. Every parameter that widens the cloud's search
//     also loosens its tracking, so steady-state error is asserted in the same
//     run against the same configuration. A candidate that buys recovery by
//     giving up tracking fails here rather than looking like an improvement.
//
// sim_reset does NOT restore the filter configs, so every trial writes the whole
// baseline first. Without that, trial N runs with trial N-1's leftovers and the
// table is nonsense -- which is how the first version of this file lied.
import { loadSim, wrap, tally } from './harness.mjs';

const M = await loadSim([
  'sim_init', 'sim_teleport', 'sim_set_drive', 'sim_set_param', 'sim_get_param',
]);
const api = wrap(M, {
  init: [null, []],
  reset: [null, ['number', 'number', 'number']],
  step: [null, ['number']],
  setLoc: ['sim_set_localization', null, ['number']],
  stateJson: ['sim_state_json', 'string', []],
  setParam: ['sim_set_param', null, ['string', 'number']],
  getParam: ['sim_get_param', 'number', ['string']],
  teleport: [null, ['number', 'number', 'number']],
  setDrive: [null, ['number', 'number']],
});
api.init();

const t = tally('recovery.mjs');
const st = () => JSON.parse(api.stateJson());
const arg = (name, def) => {
  const i = process.argv.indexOf('--' + name);
  return i >= 0 ? Number(process.argv[i + 1]) : def;
};
const MAG = arg('mag', 24);
const BUDGET = arg('budget', 2500);

const LOC_MCL_EKF = 5;

/// Odometry that is wrong enough to need correcting. With every noise knob at
/// zero dead reckoning is ground truth and none of this measures anything.
/// Every world knob a check below moves has to be listed here at its default,
/// for the same reason BASE exists: sim_reset does not restore the world
/// parameters either, so the blinded-beam check silently blinded every trial
/// after it and three assertions went vacuous before this line was added.
const WORLD = {
  collideObstacles: 0,
  encoderSlip: 0.015, imuDriftDegPerSec: 0.015, imuNoiseDeg: 0.15,
  distNoiseMm: 5, distMaxRangeMm: 2000,
};

/// Every filter knob this file or a sweep touches, at its shipped value. Read
/// back from the binary once and checked below, so a config.cpp change shows up
/// as a failure here instead of silently rebasing the numbers.
///
/// A knob a sweep sets but this list omits is worse than useless: setup() only
/// rewrites WORLD and BASE, so the omitted one carries the PREVIOUS cell's value
/// into the next. That is not hypothetical -- mclLostFitSigmas and
/// mclSearchMismatchFloor were missing, and one --panel row inherited
/// lostFitSigmas 4 from the row above it and read 322/324 where the same
/// configuration measured alone reads 311/324.
const BASE = {
  mclResampleVar: 1.5, mclThetaResampleVar: 0.01,
  mclSearchVar: 12.0, mclThetaSearchVar: 0.01,
  mclSearchMismatchFull: 0.9, mclSearchMismatchFloor: 0.5,
  mclLostFitSigmas: 8, mclInjectResampleFloor: 0.03,
  mclMaxRandomFrac: 0.0, mclRandWeight: 0.0008,
  mclResampleThresh: 512, mclMaxThetaDev: 0.08, mclImuVariance: 0.005,
  mclImeVariance: 0.004, mclHorizProp: 0.004, mclHorizConst: 0.03,
  mclSyncProp: 0.1,
  ekfMclSigmaScale: 1.6, ekfMclSigmaFloor: 1.2, ekfMclGate: 11.8,
  ekfResetCycles: 10, ekfSyncProp: 0.12,
};

/// Four start poses spread over the field, so a result does not hinge on one
/// corner's wall geometry, and eight kidnap directions from each.
const STARTS = [[-30, 0, 0], [30, 20, 90], [0, -40, 180], [-20, 45, 270]];
const DIRS = [0, 45, 90, 135, 180, 225, 270, 315];

/// The wide set: eight starts x sixteen directions. 4x8 keeps at most 19 usable
/// trials once the through-the-wall teleports are dropped, and 19 trials cannot
/// separate 14/19 from 18/19 -- that split is McNemar one-sided p = 0.0625, so
/// the headline improvement this file was written to defend was never actually
/// established. The wide set costs about four times as long and still runs in
/// seconds, which is the only reason the narrow one was ever worth keeping.
/// The four extra starts sit inside +/-36 in so that all sixteen directions
/// stay on the field from them. A start near a wall silently drops half its
/// directions to the through-the-wall guard, which spends the run time without
/// buying a trial.
const STARTS_WIDE = [...STARTS, [34, -26, 45], [-34, -34, 135],
                     [8, 34, 225], [30, 34, 315]];
const DIRS_WIDE = Array.from({ length: 16 }, (_, i) => i * 22.5);

/// Independent repeats of a whole trial set. Every seed base re-rolls the
/// sensor noise and the resampling draws for all of the starts, so trials from
/// two bases are independent in a way two directions from one base are not.
const SEED_BASES = [1, 1000, 2000, 3000, 4000, 5000, 6000, 7000];

function setup(cfg, seed, start, drive = true) {
  for (const [k, v] of Object.entries(WORLD)) api.setParam(k, v);
  for (const [k, v] of Object.entries(BASE)) api.setParam(k, v);
  api.setParam('seed', seed);
  api.reset(start[0], start[1], start[2]);
  api.setLoc(LOC_MCL_EKF);
  // After setLoc, which re-seeds the cloud from the chassis pose.
  for (const [k, v] of Object.entries(BASE)) api.setParam(k, v);
  for (const [k, v] of Object.entries(cfg)) api.setParam(k, v);
  api.setParam('seed', seed);
  // A fixed differential arc: ground truth keeps moving on the same curve
  // whatever the estimate does, so the kidnap is the only variable.
  api.setDrive(drive ? 6 : 0, drive ? 4.5 : 0);
  for (let i = 0; i < 100; i++) api.step(10);
}

/// One kidnap per (start, direction). Returns rates and the mean latency of the
/// trials that did recover -- a mean over the successes only, since a failure
/// has no latency and averaging in the budget would flatter a config that
/// fails often.
function kidnap(cfg, budgetMs = BUDGET, drive = true, opts = {}) {
  const starts = opts.starts ?? STARTS;
  const dirs = opts.dirs ?? DIRS;
  const seedBase = opts.seedBase ?? 1;
  let mHit = 0, fHit = 0, trials = 0, relocs = 0;
  const mt = [], ft = [];
  const finalM = [], finalF = [];
  // Per-trial outcomes, keyed by (seed base, start, direction), so two
  // configurations can be compared PAIRED. Comparing the two rates as
  // independent samples throws away the fact that both arms ran the same
  // kidnaps with the same noise, which is most of the power available here.
  const keys = [], mOut = [], fOut = [];
  for (let si = 0; si < starts.length; si++) {
    for (let di = 0; di < dirs.length; di++) {
      setup(cfg, seedBase + si * dirs.length + di, starts[si], drive);
      const s0 = st();
      const a = (dirs[di] * Math.PI) / 180;
      const tx = s0.true.x + MAG * Math.cos(a);
      const ty = s0.true.y + MAG * Math.sin(a);
      // A teleport through the wall is not a kidnap; the world would just
      // collide the robot back out and the displacement would not be MAG.
      if (Math.abs(tx) > 60 || Math.abs(ty) > 60) continue;
      const reloc0 = s0.ekf.reloc;
      api.teleport(tx, ty, s0.true.h);
      trials++;
      let m = -1, f = -1, eM = 99, eF = 99, s = s0;
      for (let n = 1; n <= budgetMs / 10; n++) {
        api.step(10);
        s = st();
        eM = Math.hypot(s.mcl.x - s.true.x, s.mcl.y - s.true.y);
        eF = Math.hypot(s.odom.x - s.true.x, s.odom.y - s.true.y);
        if (m < 0 && eM < 3) m = n * 10;
        if (f < 0 && eF < 3) f = n * 10;
      }
      if (m >= 0) { mHit++; mt.push(m); }
      if (f >= 0) { fHit++; ft.push(f); }
      relocs += s.ekf.reloc - reloc0;
      finalM.push(eM); finalF.push(eF);
      keys.push(`${seedBase}:${si}:${di}`);
      mOut.push(m >= 0 ? 1 : 0);
      fOut.push(f >= 0 ? 1 : 0);
    }
  }
  const avg = (a) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : Infinity);
  // MEDIAN final error. The rate above already counts the trials that never
  // recovered; a mean would count them a second time, at 40 in each, and say
  // more about the rate than about where the successes landed.
  const med = (a) => [...a].sort((x, y) => x - y)[Math.floor(a.length / 2)];
  return { mHit, fHit, trials, relocs, mMs: avg(mt), fMs: avg(ft),
           endM: med(finalM), endF: med(finalF), keys, mOut, fOut };
}

/// Every seed base in SEED_BASES over the wide set, concatenated into one
/// result. The rates are pooled and the per-trial outcome vectors survive, so a
/// paired test can be run across the whole thing.
function kidnapAll(cfg, bases = SEED_BASES, opts = {}) {
  const wide = { starts: STARTS_WIDE, dirs: DIRS_WIDE, ...opts };
  let acc = null;
  for (const seedBase of bases) {
    const r = kidnap(cfg, BUDGET, true, { ...wide, seedBase });
    if (acc === null) { acc = r; continue; }
    acc = {
      mHit: acc.mHit + r.mHit, fHit: acc.fHit + r.fHit,
      trials: acc.trials + r.trials, relocs: acc.relocs + r.relocs,
      mMs: NaN, fMs: NaN,
      endM: Math.max(acc.endM, r.endM), endF: Math.max(acc.endF, r.endF),
      keys: [...acc.keys, ...r.keys],
      mOut: [...acc.mOut, ...r.mOut], fOut: [...acc.fOut, ...r.fOut],
    };
  }
  return acc;
}

/// Exact one-sided McNemar. `b` and `c` are the discordant counts -- trials one
/// arm won and the other lost. The concordant trials carry no information about
/// which arm is better, which is exactly why the test discards them; a
/// two-proportion z on the pooled rates does not, and overstates n.
function mcnemarP(b, c) {
  const n = b + c;
  if (n === 0) return 1.0;
  const k = Math.max(b, c);
  let p = 0.0;
  for (let i = k; i <= n; i++) {
    let ch = 1.0;
    for (let j = 0; j < i; j++) ch = (ch * (n - j)) / (j + 1);
    p += ch * Math.pow(0.5, n);
  }
  return Math.min(p, 1.0);
}

/// Pair two kidnap results on their trial keys. `field` picks mOut (raw MCL) or
/// fOut (the fused pose the robot actually drives on).
function paired(a, b, field = 'mOut') {
  const bi = new Map(b.keys.map((k, i) => [k, b[field][i]]));
  let bWins = 0, aWins = 0, both = 0, neither = 0, n = 0;
  a.keys.forEach((k, i) => {
    if (!bi.has(k)) return;
    const x = a[field][i], y = bi.get(k);
    n++;
    if (x === 1 && y === 1) both++;
    else if (x === 0 && y === 0) neither++;
    else if (y === 1) bWins++;
    else aWins++;
  });
  return { n, both, neither, aWins, bWins, p: mcnemarP(aWins, bWins) };
}

const pairLine = (label, a, b, field) => {
  const r = paired(a, b, field);
  return `${label.padEnd(34)} n ${String(r.n).padStart(3)}  ` +
         `both ${String(r.both).padStart(3)}  neither ${String(r.neither).padStart(2)}  ` +
         `only-A ${String(r.aWins).padStart(2)}  only-B ${String(r.bWins).padStart(2)}  ` +
         `McNemar p=${r.p.toFixed(4)}`;
};

/// Steady-state tracking, no kidnap: the other half of every trade-off here.
/// Also reports what the mismatch signal reads while the filter is healthy,
/// which is what the search ramp is driven by.
function track(cfg) {
  let eM = 0, eF = 0, n = 0, relocs = 0;
  const mis = [], inj = [];
  // WORST as well as mean, on both estimates. A mean over 3600 ticks hides a
  // one-second excursion completely, and the excursion is the failure mode the
  // search width is capable of causing.
  let worstM = 0, worstF = 0;
  for (let si = 0; si < STARTS.length; si++) {
    for (const seed of [1, 2, 3]) {
      setup(cfg, seed + si * 100, STARTS[si]);
      const r0 = st().ekf.reloc;
      let s = null;
      for (let i = 0; i < 300; i++) {
        api.step(10);
        s = st();
        const dM = Math.hypot(s.mcl.x - s.true.x, s.mcl.y - s.true.y);
        eM += dM;
        worstM = Math.max(worstM, dM);
        const dF = Math.hypot(s.odom.x - s.true.x, s.odom.y - s.true.y);
        eF += dF;
        worstF = Math.max(worstF, dF);
        mis.push(s.mcl.mismatch);
        inj.push(s.mclInject);
        n++;
      }
      relocs += s.ekf.reloc - r0;
    }
  }
  const pct = (a, p) => {
    const v = [...a].sort((x, y) => x - y);
    return v[Math.min(v.length - 1, Math.floor(p * v.length))];
  };
  return { mcl: eM / n, fused: eF / n, worstM, worstF, relocs,
           mis50: pct(mis, 0.5), mis90: pct(mis, 0.9), inj90: pct(inj, 0.9) };
}

const line = (label, k, r) =>
  `${label.padEnd(26)} mcl ${String(k.mHit).padStart(2)}/${k.trials} ` +
  `${(k.mMs === Infinity ? '-' : k.mMs.toFixed(0)).padStart(4)}ms  ` +
  `fused ${String(k.fHit).padStart(2)}/${k.trials} ` +
  `${(k.fMs === Infinity ? '-' : k.fMs.toFixed(0)).padStart(4)}ms  ` +
  `reloc ${String(k.relocs).padStart(2)}  ` +
  `steady mcl ${r.mcl.toFixed(2)} (worst ${r.worstM.toFixed(1)}) ` +
  `fused ${r.fused.toFixed(3)} (worst ${r.worstF.toFixed(2)})  ` +
  `mis ${r.mis50.toFixed(2)}/${r.mis90.toFixed(2)}`;

// ---------------------------------------------------------------------------
// The sweep is a measurement tool, not a test. It shares the harness so that
// what it prints is the same quantity the assertions below are made of.
// ---------------------------------------------------------------------------
if (process.argv.includes('--one')) {
  const spec = process.argv[process.argv.indexOf('--one') + 1] ?? '';
  const cfg = {};
  for (const kv of spec.split(',').filter(Boolean)) {
    const [k, v] = kv.split('=');
    cfg[k] = Number(v);
  }
  console.log(line(spec || 'shipped', kidnap(cfg), track(cfg)));
  process.exit(0);
}

// A cross product of named sim parameters, so a two-way trade can be read off
// one table instead of assembled from remembered single runs.
//
//   node sim/test/recovery.mjs --grid mclResampleVar=0.15,0.5,1.5 mclSearchVar=6,12
//   node sim/test/recovery.mjs --grid --wide mclLostFitSigmas=6,12,24
//
// --wide runs the eight-by-sixteen set, which is what the rates in the report
// have to come off: a 4x8 cell resolves nothing finer than four trials.
if (process.argv.includes('--grid')) {
  const specs = process.argv.slice(process.argv.indexOf('--grid') + 1)
    .filter((a) => a.includes('='))
    .map((a) => {
      const [k, v] = a.split('=');
      return [k, v.split(',').map(Number)];
    });
  const opts = process.argv.includes('--wide')
    ? { starts: STARTS_WIDE, dirs: DIRS_WIDE } : {};
  const cells = specs.reduce(
    (acc, [k, vs]) => acc.flatMap((c) => vs.map((v) => ({ ...c, [k]: v }))), [{}]);
  const starts = opts.starts ?? STARTS, dirs = opts.dirs ?? DIRS;
  console.log(`kidnap ${MAG} in, ${starts.length} starts x ${dirs.length} directions
`);
  for (const cfg of cells) {
    const name = Object.entries(cfg).map(([k, v]) => `${k}=${v}`).join(' ') || 'shipped';
    console.log(line(name, kidnap(cfg, BUDGET, true, opts), track(cfg)));
  }
  process.exit(0);
}

// Several candidate configurations over every seed base, each paired against
// the first. This is the only form in which a rate difference here can be
// called established: 4x8 keeps 19 trials, and 19 trials cannot separate 14/19
// from 18/19.
//
//   node sim/test/recovery.mjs --panel "" "mclSearchVar=12" "mclResampleVar=0.6,mclSearchVar=12"
if (process.argv.includes('--panel')) {
  const specs = process.argv.slice(process.argv.indexOf('--panel') + 1)
    .filter((a) => !a.startsWith('--'));
  const parse = (spec) => {
    const cfg = {};
    for (const kv of (spec ?? '').split(',').filter(Boolean)) {
      const [k, v] = kv.split('=');
      cfg[k] = Number(v);
    }
    return cfg;
  };
  console.log(`seed bases ${SEED_BASES.join(',')}, ` +
              `${STARTS_WIDE.length} starts x ${DIRS_WIDE.length} directions, ` +
              `kidnap ${MAG} in\n`);
  const runs = [];
  for (const spec of specs) {
    const cfg = parse(spec);
    const k = kidnapAll(cfg);
    const r = track(cfg);
    runs.push({ spec, k, r });
    console.log(`${(spec || 'shipped').padEnd(46)} ` +
                `mcl ${String(k.mHit).padStart(3)}/${k.trials} ` +
                `fused ${String(k.fHit).padStart(3)}/${k.trials}  ` +
                `steady mcl ${r.mcl.toFixed(2)} (worst ${r.worstM.toFixed(1)}) ` +
                `fused ${r.fused.toFixed(3)} (worst ${r.worstF.toFixed(2)})  ` +
                `reloc ${k.relocs}`);
  }
  console.log();
  for (let i = 1; i < runs.length; i++) {
    console.log(pairLine(`${runs[i].spec} raw MCL`, runs[0].k, runs[i].k, 'mOut'));
    console.log(pairLine(`${runs[i].spec} fused`, runs[0].k, runs[i].k, 'fOut'));
  }
  process.exit(0);
}

if (process.argv.includes('--sweep')) {
  const CASES = {
    'shipped': {},
    'roughening 0.15': { mclResampleVar: 0.15, mclThetaResampleVar: 0.001 },
    'search 6.0': { mclSearchVar: 6.0 },
    'search 18.0': { mclSearchVar: 18.0 },
    'rough 0.15, search 12': { mclResampleVar: 0.15, mclThetaResampleVar: 0.001, mclSearchVar: 12.0 },
    'lostFitSigmas 4': { mclLostFitSigmas: 4 },
    'lostFitSigmas 12': { mclLostFitSigmas: 12 },
    'inject 0.05': { mclMaxRandomFrac: 0.05 },
    'inject 0.10': { mclMaxRandomFrac: 0.10 },
    'inject 0.10 + search 6': { mclMaxRandomFrac: 0.10, mclSearchVar: 6.0, mclResampleVar: 0.15 },
    'thetaDev 0.30': { mclMaxThetaDev: 0.30 },
    'randWeight 0.01': { mclRandWeight: 0.01 },
    'mismatchFull 0.15': { mclSearchMismatchFull: 0.15 },
  };
  console.log(`kidnap ${MAG} in, ${STARTS.length} starts x ${DIRS.length} directions\n`);
  for (const [name, cfg] of Object.entries(CASES)) {
    console.log(line(name, kidnap(cfg), track(cfg)));
  }
  process.exit(0);
}

// ---------------------------------------------------------------------------
// Assertions
// ---------------------------------------------------------------------------

t.section('the fixture measures what it claims');

// Shipped values, straight out of the binary. If config.cpp moves, BASE is
// stale and every number below is against a configuration nobody ships.
{
  const wrong = Object.entries(BASE)
    .filter(([k, v]) => Math.abs(api.getParam(k) - v) > Math.max(1e-4, 1e-3 * Math.abs(v)))
    .map(([k, v]) => `${k} is ${api.getParam(k)}, expected ${v}`);
  t.check('BASE matches the shipped config', wrong.length === 0,
          wrong.join('; ') || `all ${Object.keys(BASE).length}`);
}

// The cold-run divergence this used to discard is fixed at source: sim_reset
// now drains the tasks it cancels before rewinding the clock, so trial 1 is
// bit-identical to trial 2. Kept as a check rather than a workaround.
{
  const a = kidnap({}, 300);
  const b = kidnap({}, 300);
  t.check('the first sweep after sim_init matches the second exactly',
    a.mHit === b.mHit && a.fHit === b.fHit && a.trials === b.trials &&
    Math.abs(a.endM - b.endM) < 1e-6 && Math.abs(a.endF - b.endF) < 1e-6,
    `mcl ${a.mHit}/${a.trials} vs ${b.mHit}/${b.trials}, ` +
    `endM ${a.endM.toFixed(6)} vs ${b.endM.toFixed(6)} in`);
}

// After that, two identical trials must give identical numbers to the bit, or
// the sweep is noise and a config leak reads as an effect. sim_reset leaves the
// filter configs alone, which is why setup() rewrites the whole baseline.
{
  const a = kidnap({}), b = kidnap({});
  t.check('trials are independent',
          a.mHit === b.mHit && a.fHit === b.fHit &&
          Math.abs(a.endM - b.endM) < 1e-9 && Math.abs(a.endF - b.endF) < 1e-9,
          `${a.mHit}/${a.fHit} vs ${b.mHit}/${b.fHit}, ` +
          `final ${a.endM.toFixed(6)} vs ${b.endM.toFixed(6)} in`);
}

t.section('recovery, shipped configuration');

const base = kidnap({});
const baseTrack = track({});
console.log('  ' + line('shipped', base, baseTrack));

// Rates over the swept set, with a margin of two trials, so one seed moving
// does not decide the verdict and a real regression does. The wide set below is
// what the rates in the report come off; this narrow one is the fast gate.
t.check('MCL relocates on nearly every kidnap',
        base.mHit >= base.trials - 2,
        `${base.mHit}/${base.trials} at ${base.mMs.toFixed(0)} ms`);
t.check('the fused pose follows it',
        base.fHit >= base.trials - 5,
        `${base.fHit}/${base.trials} at ${base.fMs.toFixed(0)} ms`);
// Inside 3 in at some instant is not recovery. This is the error at the END of
// the budget, so a config that passes through and carries on elsewhere fails.
t.check('and lands close, not merely inside 3 in',
        base.endM < 1.5 && base.endF < 1.5,
        `median final mcl ${base.endM.toFixed(2)} in, fused ${base.endF.toFixed(2)} in`);

t.section('the trade-off the other way');

t.check('steady-state tracking is not given up for it',
        baseTrack.mcl < 1.9 && baseTrack.fused < 0.55,
        `mcl ${baseTrack.mcl.toFixed(3)} fused ${baseTrack.fused.toFixed(3)} in`);
// The tail is the one that catches a search ramp driven by the cloud's own mean
// likelihood. That driver rises when the cloud merely widens, so it widens
// because it widened: with the same 6 in search width it measured 17.7 in of
// worst-case excursion from a filter that was never lost, against 7.5 here.
t.check('and has no far-side-of-the-field tail',
        baseTrack.worstM < 10.0, `worst mcl ${baseTrack.worstM.toFixed(1)} in`);
// Which is also why the ramp is not driven by it. The mean-based gap flags a
// filter tracking to under two inches as this mismatched, so no threshold on it
// separates healthy from lost.
t.check('the mean-based mismatch flags healthy tracking',
        baseTrack.mis90 > 0.05 && baseTrack.mcl < 2.0,
        `p90 ${baseTrack.mis90.toFixed(2)} while tracking to ${baseTrack.mcl.toFixed(2)} in`);

t.section('the EKF relocation gate is live, and only when it should be');

// Both halves matter. A gate that never fires is the bug this replaced; a gate
// that fires during healthy tracking throws away a correct filter.
t.check('it fires on a kidnap', base.relocs > 0, `${base.relocs} over ${base.trials} trials`);
t.check('it does not fire while tracking', baseTrack.relocs === 0,
        `${baseTrack.relocs} relocations over ${STARTS.length * 3 * 3} s of driving`);

// The relocation must be an inference from the range readings. Blind the
// distance sensors -- the readings the adjudication is made of -- and it must
// stop, even though MCL keeps insisting just as hard.
{
  const blind = kidnap({ distMaxRangeMm: 1 });
  t.check('with the beams blinded it does not fire',
          blind.relocs === 0, `${blind.relocs} relocations, mcl ${blind.mHit}/${blind.trials}`);
}

t.section('augmented-MCL injection stays off');

// The round-two decision, re-measured rather than re-argued. Injection buys the
// last kidnap or two and costs a factor of four in tracking, with excursions to
// the far side of the field: an injected particle whose beam points at a mapped
// post gets that beam deadzoned rather than scored, so a hypothesis 60 in from
// the truth survives on a technicality. The search width buys the same recovery
// without it.
{
  const inj = kidnap({ mclMaxRandomFrac: 0.10 });
  const injTrack = track({ mclMaxRandomFrac: 0.10 });
  console.log('  ' + line('maxRandomFrac 0.10', inj, injTrack));
  // The cost is what is asserted, because it is the part 19 trials can carry:
  // recovery is at the ceiling in both arms and a one-trial difference either
  // way is noise. 40 in is more than half the field, so the estimate is not a
  // pose the robot could be in.
  t.check('it would cost far more tracking than it buys recovery',
          injTrack.mcl > 1.8 * baseTrack.mcl && injTrack.worstM > 40.0,
          `recovery ${inj.mHit}/${inj.trials} vs ${base.mHit}/${base.trials}, ` +
          `steady ${injTrack.mcl.toFixed(2)} in vs ${baseTrack.mcl.toFixed(2)}, ` +
          `worst ${injTrack.worstM.toFixed(0)} in vs ${baseTrack.worstM.toFixed(1)}`);
  t.check('and it is off', baseTrack.inj90 === 0, `p90 inject ${baseTrack.inj90}`);
}

t.section('why the robot has to be driving');

// A parked robot is NOT information-free: four absolute ranges pin a position
// given the heading, and it relocates about half the time. What it lacks is the
// parallax that separates a real fix from a wall-parallel alias. So it is a
// rate, not an impossibility -- and a stationary trial measures a different and
// much harder problem than a match ever poses. That is what autotune.mjs's
// kidnap trial does, which is why its figure should not be read as the recovery
// rate. Compared as rates, since a parked robot stays inside the field and so
// gets more of the swept directions.
// On the WIDE set: the narrow one leaves 19 driving trials against 28 parked,
// and the difference here is 20 percentage points, which 19 trials cannot hold.
{
  const bases = SEED_BASES.slice(0, 2);
  const wide = { starts: STARTS_WIDE, dirs: DIRS_WIDE };
  let pHit = 0, pN = 0, dHit = 0, dN = 0;
  for (const seedBase of bases) {
    const parked = kidnap({}, BUDGET, false, { ...wide, seedBase });
    const driven = kidnap({}, BUDGET, true, { ...wide, seedBase });
    pHit += parked.mHit; pN += parked.trials;
    dHit += driven.mHit; dN += driven.trials;
  }
  const pr = pHit / pN, dr = dHit / dN;
  t.check('driving recovers a much larger share than parked',
          dr - pr > 0.10,
          `parked ${pHit}/${pN} = ${(100 * pr).toFixed(0)}%, ` +
          `driving ${dHit}/${dN} = ${(100 * dr).toFixed(0)}%`);
}

t.section('the search width is the mechanism, and it is reachable');

// The ramp from mclResampleVar to mclSearchVar is what recovery runs on. Pin
// that it is not a no-op: a cloud that cannot widen cannot cross 24 in.
{
  const pinned = kidnap({ mclSearchVar: 0.05, mclThetaSearchVar: 0.001,
                          mclResampleVar: 0.05, mclThetaResampleVar: 0.001 });
  t.check('a cloud that cannot widen cannot recover',
          pinned.mHit < base.mHit - 2,
          `${pinned.mHit}/${pinned.trials} vs ${base.mHit}/${base.trials}`);
}

t.section('and the improvement is established, not merely observed');

// 4x8 keeps 19 usable trials once the through-the-wall teleports are dropped,
// and 19 cannot separate 14/19 from 18/19 -- that split is McNemar one-sided
// p = 0.0625, so the rate this file was written to defend was never actually
// established. Two seed bases of the wide set is 162 paired trials, and it runs
// in the same order of time. Paired, because both arms drive the same kidnaps
// under the same noise and discarding that costs most of the available power.
//
// The shipped 12 in search width against the 6 in it replaced is p = 0.0009 raw
// and p = 0.0041 fused over all four bases; that comparison is not gated here
// because 6 is no longer a configuration anything ships.
{
  const bases = SEED_BASES.slice(0, 2);
  const narrow = kidnapAll({ mclSearchVar: 1.5 }, bases);
  const wide = kidnapAll({}, bases);
  const raw = paired(narrow, wide, 'mOut');
  const fused = paired(narrow, wide, 'fOut');
  console.log('  ' + pairLine('search 1.5 -> shipped, raw MCL', narrow, wide, 'mOut'));
  console.log('  ' + pairLine('search 1.5 -> shipped, fused', narrow, wide, 'fOut'));
  t.check('enough paired trials to conclude anything at all',
          raw.n >= 150, `n = ${raw.n} over seed bases ${bases.join(',')}`);
  t.check('the search width carries raw MCL, at p < 0.01', raw.p < 0.01,
          `${narrow.mHit}/${narrow.trials} -> ${wide.mHit}/${wide.trials}, ` +
          `p = ${raw.p.toExponential(2)}`);
  t.check('and the fused pose with it, at p < 0.01', fused.p < 0.01,
          `${narrow.fHit}/${narrow.trials} -> ${wide.fHit}/${wide.trials}, ` +
          `p = ${fused.p.toExponential(2)}`);
}

t.section('a harder kidnap still degrades gracefully');

// 40 in is most of a quadrant. The point is not a rate but that nothing hangs,
// diverges, or reports success while sitting on the far side of the field.
{
  const saveMag = MAG;
  const hard = (() => {
    const dirs = [0, 90, 180, 270];
    let hit = 0, trials = 0, worst = 0, onField = true;
    for (const start of STARTS) {
      for (const d of dirs) {
        setup({}, 900 + trials, start);
        const s0 = st();
        const a = (d * Math.PI) / 180;
        const tx = s0.true.x + 40 * Math.cos(a), ty = s0.true.y + 40 * Math.sin(a);
        if (Math.abs(tx) > 60 || Math.abs(ty) > 60) continue;
        api.teleport(tx, ty, s0.true.h);
        trials++;
        let e = 99;
        for (let i = 0; i < 400; i++) {
          api.step(10);
          const s = st();
          e = Math.hypot(s.mcl.x - s.true.x, s.mcl.y - s.true.y);
          if (!Number.isFinite(s.mcl.x) || Math.abs(s.mcl.x) > 71 ||
              Math.abs(s.mcl.y) > 71) onField = false;
        }
        if (e < 3) hit++;
        worst = Math.max(worst, e);
      }
    }
    return { hit, trials, worst, onField };
  })();
  // 40 in is most of a quadrant. It recovers more often than not, and the trials
  // that do not have to fail by staying on the field rather than by diverging:
  // the estimate must remain a pose a robot could be in.
  t.check('a 40 in kidnap mostly recovers, and never diverges',
          hard.hit >= 3 && hard.worst < 60.0 && hard.onField,
          `${hard.hit}/${hard.trials} recovered, worst residual ` +
          `${hard.worst.toFixed(1)} in, estimate ${hard.onField ? 'on' : 'OFF'} the field`);
  void saveMag;
}

process.exit(t.finish());
