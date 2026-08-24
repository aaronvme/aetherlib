// The browser's own logic, and the reset contract it relies on.
//
//   CHAD_SIM_MJS=... node sim/test/ui.mjs
//
// Two things this suite exists to stop.
//
// 1. Untested UI code. sim/web/app.js touches the DOM at import time and cannot
//    be loaded headlessly, so every decision that is not "put this string in
//    that element" lives in sim/web/{motion,traj,mpc}_view.js and is asserted
//    here. What is left in app.js is DOM plumbing, and the report says so.
// 2. A name list in JavaScript drifting from the enum it mirrors. Every list of
//    names below is checked against the C++ that defines it -- parsed out of the
//    source, the way sim/test/dsl.mjs checks RouteOp -- rather than against a
//    second copy of itself. A list checked against itself is the test that
//    cannot fail.
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { loadSim, wrap, tally } from './harness.mjs';
import {
  OUTCOMES, MOTION_TYPES, outcomeClass, seamOf, settleText, motionRow,
  motionSummary, motionTypeName,
} from '../web/motion_view.js';
import {
  TRAJ_ERRORS, generationVerdict, decimate, headingTicks, profileExtremes,
  auditRows,
} from '../web/traj_view.js';
import {
  MPC_MODES, mpcVerdict, constraintRows, rowBudgetRows, TRACE_COLUMNS,
  decimateExtremes,
} from '../web/mpc_view.js';
import { auditWheelVoltages } from '../web/traj.js';
import {
  locSummary, scrubState, seekFraction, fieldGeomText, parseFieldGeom,
} from '../web/view_panel.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const read = (p) => readFileSync(join(ROOT, p), 'utf8');

const M = await loadSim([
  'sim_init', 'sim_param_count', 'sim_param_name', 'sim_param_restored',
  'sim_param_baseline', 'sim_params_restore', 'sim_params_dirty_json',
  'sim_mpc_trace_stride',
]);
const api = wrap(M, {
  init: [null, []],
  reset: [null, ['number', 'number', 'number']],
  step: [null, ['number']],
  setLoc: ['sim_set_localization', null, ['number']],
  setDrive: [null, ['number', 'number']],
  setParam: ['sim_set_param', null, ['string', 'number']],
  getParam: ['sim_get_param', 'number', ['string']],
  paramBaseline: ['sim_param_baseline', 'number', ['string']],
  paramCount: ['sim_param_count', 'number', []],
  paramName: ['sim_param_name', 'string', ['number']],
  paramRestored: ['sim_param_restored', 'number', ['string']],
  paramsRestore: ['sim_params_restore', null, []],
  dirtyJson: ['sim_params_dirty_json', 'string', []],
  stateJson: ['sim_state_json', 'string', []],
  motionClear: [null, []],
  motionPush: [null, ['number', 'number', 'number', 'number']],
  motionSet: [null, ['string', 'number']],
  motionRun: [null, []],
  motionRunning: ['sim_motion_running', 'number', []],
  motionJson: ['sim_motion_json', 'string', []],
  trajClear: ['sim_traj_clear', null, []],
  trajWaypoint: ['sim_traj_waypoint', null, ['number', 'number', 'number', 'number']],
  trajDefaults: ['sim_traj_defaults', null, []],
  trajSet: ['sim_traj_set', null, ['string', 'number']],
  trajGenerate: ['sim_traj_generate', 'number', []],
  trajResultJson: ['sim_traj_result_json', 'string', []],
  mpcTraceStride: ['sim_mpc_trace_stride', 'number', []],
  recStride: ['sim_record_stride', 'number', []],
});
api.init();

const t = tally('ui.mjs');
const st = () => JSON.parse(api.stateJson());
const dirty = () => JSON.parse(api.dirtyJson());

// ===========================================================================
// The sim_reset / parameter-baseline contract
// ===========================================================================
t.section('the parameter set is enumerable, and the enumeration is the real one');

// The three lists that have to agree: what sim_set_param writes, what
// sim_get_param reads, and what the binary enumerates. Parsed from the source,
// so adding a knob and forgetting the table fails HERE rather than showing up as
// a slider that silently does nothing.
const bridgeSrc = read('sim/bridge/bridge.cpp');
function keysIn(startMarker, endMarker) {
  const a = bridgeSrc.indexOf(startMarker);
  const b = bridgeSrc.indexOf(endMarker, a);
  if (a < 0 || b < 0) throw new Error(`cannot find ${startMarker} .. ${endMarker}`);
  const body = bridgeSrc.slice(a, b);
  return [...new Set([...body.matchAll(/k == "([A-Za-z0-9_]+)"/g)].map((m) => m[1]))];
}
const setKeys = keysIn('void sim_set_param(', 'Reading parameters back');
const getKeys = keysIn('float readParam(', 'ParamSet gBaseline;');
const enumerated = Array.from({ length: api.paramCount() }, (_, i) => api.paramName(i));

const missingFromEnum = setKeys.filter((k) => !enumerated.includes(k));
const extraInEnum = enumerated.filter((k) => !setKeys.includes(k));
t.check('every sim_set_param key is enumerated, and nothing else is',
        missingFromEnum.length === 0 && extraInEnum.length === 0,
        `${setKeys.length} in source, ${enumerated.length} enumerated`
        + (missingFromEnum.length ? `; missing ${missingFromEnum.join(',')}` : '')
        + (extraInEnum.length ? `; extra ${extraInEnum.join(',')}` : ''));

const writeOnly = setKeys.filter((k) => !getKeys.includes(k));
t.check('no key is write-only', writeOnly.length === 0,
        writeOnly.length ? writeOnly.join(',') : `${getKeys.length} readable`);

// Reading is not the same as reading SOMETHING. A key that falls off the end of
// readParam returns NaN, which would make it permanently "dirty".
{
  const nan = enumerated.filter((k) => !Number.isFinite(api.getParam(k)));
  t.check('every enumerated key reads back finite', nan.length === 0,
          nan.length ? nan.join(',') : `${enumerated.length} keys`);
}
t.check('an unknown key is NaN and reports -1, not 0',
        Number.isNaN(api.getParam('nosuchknob')) && api.paramRestored('nosuchknob') === -1);
t.check('sim_param_name is bounds-checked',
        api.paramName(-1) === '' && api.paramName(api.paramCount()) === '');

t.section('sim_reset does not restore parameters, and that is the contract');

// The decision, pinned both ways. Restoring inside sim_reset would remove the
// ability to configure the world a reset establishes -- every suite here writes
// its knobs and THEN resets, and World::resetTo() consumes them (it seeds the
// IMU from imuUnderReport and synthesises the beams from the distance knobs).
// Measured, restoring fails 18 of 59 checks in world.mjs.
// imuUnderReport is the sharpest case: World::resetTo() SEEDS the IMU with it,
// so a reset that restored it first would silently discard the calibration
// error the caller asked for. Measured as the reported-to-true heading ratio
// over a real turn, which is the only place the bias shows -- setPose() writes
// the pose exactly, so the heading right after a reset says nothing.
{
  api.paramsRestore();
  const base = api.getParam('imuUnderReport');
  const scale = (k) => {
    api.paramsRestore();
    api.setParam('imuUnderReport', k);
    api.setLoc(0);
    api.reset(0, 0, 90);
    // One tick, so odometry has read the freshly seeded IMU: setPose() writes
    // the pose exactly, and a mis-seeded IMU shows up as a constant offset on
    // the NEXT tick. Measuring from there isolates the scale from the offset.
    api.step(10);
    const s0 = st();
    api.setDrive(6, -6);
    for (let i = 0; i < 60; i++) api.step(10);
    api.setDrive(0, 0);
    const s1 = st();
    return (s1.odom.h - s0.odom.h) / (s1.true.h - s0.true.h);
  };
  const nominal = scale(base);
  const skewed = scale(0.95);
  t.check('a knob written before a reset is in force for the run it starts',
          Math.abs(nominal - 1) < 5e-3 && Math.abs(skewed - 0.95 / base) < 5e-3,
          `heading scale ${nominal.toFixed(4)} at the default, `
          + `${skewed.toFixed(4)} at 0.95 (predicted ${(0.95 / base).toFixed(4)})`);
  api.paramsRestore();
  t.check('and sim_params_restore puts it back',
          Math.abs(api.getParam('imuUnderReport') - base) < 1e-9,
          `${api.getParam('imuUnderReport')} vs ${base}`);
}

// The whole point of the dirty scan: it covers every key, so the leak that
// blinded every trial after one -- distMaxRangeMm left at 1 -- is a one-line
// assertion instead of a 26-entry hand-written table.
{
  api.paramsRestore();
  const clean = dirty();
  api.setParam('distMaxRangeMm', 1);
  api.setParam('mclSearchVar', 9.5);
  const d = dirty();
  const names = d.params.map((p) => p.k).sort();
  t.check('a leak of any key shows up in sim_params_dirty_json',
          clean.count === 0 && d.count === 2
          && names.join(',') === 'distMaxRangeMm,mclSearchVar',
          `clean ${clean.count}, dirty ${d.count} [${names.join(',')}] of ${d.of}`);
  t.check('and it reports both values, so the leak is identifiable',
          d.params.every((p) => Number.isFinite(p.is) && Number.isFinite(p.was))
          && d.params.find((p) => p.k === 'distMaxRangeMm').was === 2000,
          JSON.stringify(d.params));
  // Reachable without learning the new call at all.
  t.check('sim_state_json carries the same count in diag.params',
          st().diag.params.dirty === d.count && st().diag.params.of === d.of,
          `diag ${JSON.stringify(st().diag.params)}`);
  api.paramsRestore();
  t.check('one call cleans all of it', dirty().count === 0 && st().diag.params.dirty === 0);
}

// Restoring everything would break a swept seed, so seed is carved out. Pinned
// so the carve-out cannot quietly grow.
{
  const notRestored = enumerated.filter((k) => api.paramRestored(k) === 0);
  t.check('exactly one key is exempt from the restore, and it is the seed',
          notRestored.length === 1 && notRestored[0] === 'seed',
          notRestored.join(',') || 'none');
  api.setParam('seed', 4242);
  api.paramsRestore();
  t.check('a pinned seed survives a restore',
          api.getParam('seed') === 4242 && dirty().count === 0,
          `seed ${api.getParam('seed')}`);
}

// A restore has to reach every family, not just the world struct. One key from
// each of the six that applyParams writes.
{
  const probes = { encoderSlip: 0.2, mclSearchVar: 11, ekfRangeGate: 3.5,
                   rclSyncProp: 0.42, lateralKp: 3, ramseteB: 7, mtpTolerance: 5 };
  const baseline = Object.fromEntries(
    Object.keys(probes).map((k) => [k, api.paramBaseline(k)]));
  for (const [k, v] of Object.entries(probes)) api.setParam(k, v);
  const moved = Object.keys(probes).filter((k) => api.getParam(k) !== baseline[k]);
  api.paramsRestore();
  const back = Object.keys(probes).filter(
    (k) => Math.abs(api.getParam(k) - baseline[k]) < 1e-6);
  t.check('the restore reaches the world, MCL, EKF, RCL, chassis, RAMSETE and moveToPoint',
          moved.length === Object.keys(probes).length
          && back.length === Object.keys(probes).length,
          `${moved.length} moved, ${back.length} restored of ${Object.keys(probes).length}`);
}

// ===========================================================================
// Task 2: the first run is comparable with every later one
// ===========================================================================
t.section('a run is reproducible from the first one, not the second');

{
  const trial = (loc) => {
    api.paramsRestore();
    api.setParam('collideObstacles', 0);
    api.setParam('seed', 1);
    api.reset(-30, 0, 0);
    api.setLoc(loc);
    api.setDrive(6, 4.5);
    for (let i = 0; i < 100; i++) api.step(10);
    const s = st();
    return { x: s.true.x, y: s.true.y, h: s.true.h };
  };
  // Cold FIRST, before anything else in this file has enabled a filter: the
  // whole defect was that sim_reset spent virtual time in stopTracking()'s
  // pros::delay loop only when a filter had been up, so the tasks cancelled by
  // the reset parked before the first sim_step in a warm run and after it in a
  // cold one -- and the chassis task's post-cancel zero write then clobbered the
  // first physics slice of the run.
  const a = trial(1), b = trial(1), c = trial(5);
  const gap = Math.hypot(a.x - b.x, a.y - b.y);
  t.check('the first trial lands exactly where the second does',
          gap === 0 && a.h === b.h,
          `gap ${gap.toFixed(6)} in, heading ${a.h} vs ${b.h}`);
  // The observable the defect was measured on, asserted directly: a command
  // issued before a step has to be in force during that step.
  //
  // setLoc(0) FIRST, and it is load-bearing. With a filter still up, sim_reset's
  // own stopTracking() spends virtual time in a pros::delay loop and parks the
  // tasks as a side effect -- which is exactly the accident this check exists to
  // stop relying on. Only from the no-filter path does it have teeth.
  api.paramsRestore();
  api.setLoc(0);
  api.reset(-58, -60, 90);
  api.setDrive(9, 9);
  let lost = 0;
  while (lost < 6 && st().vel.left === 0) { api.step(1); lost++; }
  api.setDrive(0, 0);
  t.check('a drive command issued after a reset reaches the wheels on the next tick',
          lost === 1, `${lost - 1} physics tick(s) lost`);
  void c;
}

// ===========================================================================
// motion_view.js
// ===========================================================================
t.section('motion outcomes');

// The name list against the enum that defines it, not against itself.
{
  const src = read('include/chad/movement_control/settle_spec.hpp');
  const cppNames = [...src.matchAll(/case MotionOutcome::\w+:\s*return "(\w+)";/g)]
    .map((m) => m[1]);
  t.check('OUTCOMES matches outcomeName() in settle_spec.hpp',
          cppNames.length === OUTCOMES.length
          && cppNames.every((n, i) => n === OUTCOMES[i]),
          `cpp [${cppNames.join(',')}] vs js [${OUTCOMES.join(',')}]`);
  const typeBlock = src.slice(src.indexOf('enum class MotionType'),
                              src.indexOf('enum class MotionOutcome'));
  const cppTypes = typeBlock.slice(typeBlock.indexOf('{') + 1, typeBlock.indexOf('}'))
    .split(',').map((s) => s.trim()).filter(Boolean)
    .map((s) => s[0].toLowerCase() + s.slice(1));
  // MotionType::None is shown as "idle" because the HUD reads it as "nothing is
  // running", so only the tail is compared by name.
  t.check('MOTION_TYPES has one entry per MotionType, in order',
          cppTypes.length === MOTION_TYPES.length
          && cppTypes.slice(1).join(',').toLowerCase()
             === MOTION_TYPES.slice(1).join(',').toLowerCase(),
          `cpp ${cppTypes.length} [${cppTypes.join(',')}] vs js [${MOTION_TYPES.join(',')}]`);
}

// The bridge sends the name AND the int. If outcomeClass keyed off the wrong
// index, the panel would colour a timeout green -- so the two are cross-checked
// through a real run rather than assumed.
{
  api.paramsRestore();
  api.setLoc(0);
  api.reset(-48, -60, 90);
  api.motionClear();
  api.motionPush(2, -24, -60, 0);        // MotionType::Point
  api.motionSet('timeout', 3000);
  api.motionPush(3, 180, 0, 0);         // MotionType::Heading, unreachable in 200 ms
  api.motionSet('timeout', 200);
  api.motionRun();
  for (let i = 0; i < 600 && api.motionRunning(); i++) api.step(10);
  const recs = JSON.parse(api.motionJson());
  t.check('a motion program produces one record per motion', recs.length === 2,
          `${recs.length} records`);
  const mismatched = recs.filter((r) => OUTCOMES[r.outcome] !== r.name);
  t.check('the JS outcome table agrees with the name the bridge sent',
          mismatched.length === 0,
          recs.map((r) => `${r.outcome}=${r.name}`).join(' '));
  const rows = recs.map(motionRow);
  t.check('the reached motion is arrived/ok and the 200 ms turn is not',
          rows[0].outcome === 'arrived' && rows[0].outcomeClass === 'ok'
          && rows[1].outcomeClass !== 'ok',
          `${rows[0].outcome}/${rows[0].outcomeClass}, ${rows[1].outcome}/${rows[1].outcomeClass}`);
  t.check('a settled motion reports a settle time and a hold',
          rows[0].settle !== '—' && /\+\d+ hold/.test(rows[0].settle),
          rows[0].settle);
  t.check('both motions stopped the drive',
          rows.every((r) => r.seam === 'stopped'),
          rows.map((r) => `${r.seam} (${r.cmdSeam})`).join(' · '));
}

// Every branch of the two decision functions, including the ones a run above
// cannot produce.
t.check('outcomeClass separates arrived, chained, gave-up and nothing-ran',
        outcomeClass(1) === 'ok' && outcomeClass(2) === 'warn'
        && outcomeClass(3) === 'bad' && outcomeClass(4) === 'bad'
        && outcomeClass(5) === 'warn' && outcomeClass(6) === 'bad'
        && outcomeClass(0) === 'off' && outcomeClass(99) === 'off');
t.check('the chaining seam names all four combinations',
        seamOf({ outcome: 2, cmdOut: 6 }).kind === 'handover'
        && seamOf({ outcome: 2, cmdOut: 0 }).kind === 'chainDropped'
        && seamOf({ outcome: 1, cmdOut: 6 }).kind === 'runaway'
        && seamOf({ outcome: 1, cmdOut: 0 }).kind === 'stopped');
t.check('a motion left driving without asking is flagged bad, not neutral',
        seamOf({ outcome: 1, cmdOut: 6 }).cls === 'bad'
        && seamOf({ outcome: 2, cmdOut: 6 }).cls === 'ok');
t.check('a 0.02 V residual command is not read as a handover',
        seamOf({ outcome: 1, cmdOut: 0.02 }).kind === 'stopped'
        && seamOf({ outcome: 1, cmdOut: 0.2 }).kind === 'runaway');
// A boomerang reaches Arrived without ever passing through tolerance, so the
// bridge sends -1 and this must not render as a number.
t.check('an unavailable settle time renders as unavailable',
        settleText(-1, 900) === '—' && settleText(0, 900) === '0 ms (+900 hold)'
        && settleText(400, 900) === '400 ms (+500 hold)');
t.check('the live summary says who owns the motors before how it ended',
        motionSummary({ t: 1, routeRunning: false, motion: { trajRunning: true, ticks: 7 } })
          .startsWith('trajectory')
        && motionSummary({ t: 1, routeRunning: false, motion: { mpcRunning: true, ticks: 7 } })
          .startsWith('MPC')
        && motionSummary({ t: 1, routeRunning: true, motion: { type: 2, ticks: 3, toleranceAtMs: -1 } })
          .startsWith('point running')
        && motionSummary({ t: 1, routeRunning: false,
                           motion: { name: 'arrived', arrived: true, cmdL: 0, cmdR: 0 } })
          === 'arrived');
t.check('a finished motion still holding the drive says so',
        motionSummary({ t: 1, routeRunning: false,
                        motion: { name: 'arrived', arrived: true, cmdL: 6, cmdR: 6 } })
          .includes('drive still on'));
t.check('motionTypeName covers the whole enum and degrades on an unknown',
        motionTypeName(0) === 'idle' && motionTypeName(8) === 'trajectory'
        && motionTypeName(42) === '?42');

// ===========================================================================
// traj_view.js
// ===========================================================================
t.section('trajectory generation, including the failures');

{
  const src = read('sim/bridge/bridge_traj.cpp');
  const cppNames = [...src.matchAll(/case chad::TrajectoryError::\w+:\s*return "(\w+)";/g)]
    .map((m) => m[1]);
  t.check('TRAJ_ERRORS matches errorName() in bridge_traj.cpp',
          cppNames.length === TRAJ_ERRORS.length
          && cppNames.every((n, i) => n === TRAJ_ERRORS[i]),
          `cpp ${cppNames.length} [${cppNames.join(',')}]`);
  t.check('unusableConstraint and cuspedPath are both present',
          TRAJ_ERRORS.includes('unusableConstraint') && TRAJ_ERRORS.includes('cuspedPath'));
}

// The one a human will hit from the panel: config.cpp ships kaStraight = 0, so
// ticking the voltage constraint refuses. Driven through the real bridge, so if
// the library ever starts accepting kA = 0 again this check notices.
{
  const wps = [[-48, -48, 90], [0, -48, 90], [24, -24, 0]];
  const gen = (opts) => {
    api.trajDefaults();
    api.trajClear();
    for (const [x, y, h] of wps) api.trajWaypoint(x, y, h, 0);
    for (const [k, v] of Object.entries(opts)) api.trajSet(k, v);
    api.trajGenerate();
    return JSON.parse(api.trajResultJson());
  };
  const okRes = gen({});
  const good = generationVerdict(okRes);
  t.check('a plain generate reads as a success with a state count and a duration',
          good.ok && good.cls === 'ok' && /\d+ states/.test(good.headline),
          `${good.headline} — ${good.detail}`);

  const badRes = gen({ voltageConstraint: 1 });   // kA defaults to 0
  const bad = generationVerdict(badRes);
  t.check('kA = 0 with the voltage constraint refuses, and the panel says why',
          !bad.ok && badRes.error === 'unusableConstraint'
          && bad.cls === 'err' && /kaStraight = 0/.test(bad.detail),
          `${bad.headline} — ${bad.detail}`);
  t.check('a refusal reports zero states rather than a stale trajectory',
          badRes.count === 0, `count ${badRes.count}`);

  const fixedRes = gen({ voltageConstraint: 1, kA: 0.4 });
  t.check('and giving it a kA makes the same request generate',
          generationVerdict(fixedRes).ok && fixedRes.count > 0,
          `${fixedRes.error}, ${fixedRes.count} states`);
  t.check('an unset result is handled rather than throwing',
          generationVerdict(null).cls === 'err');
}

// Drawing helpers. Striding alone silently shortens every path it thins.
{
  const pts = Array.from({ length: 1000 }, (_, i) => ({ x: i, y: 2 * i }));
  const d = decimate(pts, 100);
  t.check('decimate keeps both endpoints and respects the budget',
          d.length <= 100 && d[0].x === 0 && d[d.length - 1].x === 999,
          `${d.length} points, first ${d[0].x}, last ${d[d.length - 1].x}`);
  // 7 into 4 is the case a bare stride gets wrong: stride 2 gives 0,2,4,6 by
  // luck, stride from ceil((n-1)/(k-1)) has to land the last point every time.
  for (const n of [3, 5, 7, 11, 101]) {
    for (const k of [2, 3, 4, 10]) {
      const p = Array.from({ length: n }, (_, i) => ({ x: i }));
      const r = decimate(p, k);
      if (r[r.length - 1].x !== n - 1 || r.length > k) {
        t.check(`decimate(${n},${k}) keeps the endpoint`, false,
                `got ${r.length} points ending at ${r[r.length - 1].x}`);
      }
    }
  }
  t.check('decimate is exhaustively endpoint-safe over n x k', true, '5 x 4 cases');
  t.check('a short path is passed through unchanged and copied',
          decimate(pts.slice(0, 10), 100).length === 10
          && decimate(pts.slice(0, 10), 100) !== pts);
}
{
  // theta is standard math radians here, not a VEX bearing: a state facing +y
  // has theta = pi/2, so the tick must point at (0, +1).
  const states = [{ x: 0, y: 0, theta: Math.PI / 2 }, { x: 0, y: 1, theta: Math.PI / 2 }];
  const ticks = headingTicks(states, 1);
  t.check('heading ticks use standard radians, not the VEX bearing',
          ticks.length === 2 && Math.abs(ticks[0].dx) < 1e-9
          && Math.abs(ticks[0].dy - 1) < 1e-9,
          `dx ${ticks[0].dx.toFixed(6)} dy ${ticks[0].dy.toFixed(6)}`);
  t.check('headingTicks cannot loop forever on every = 0',
          headingTicks(states, 0).length === 2);
}
{
  const states = [{ v: 10, a: 5, k: 0.01 }, { v: 40, a: -2, k: 0.25 }];
  const e = profileExtremes(states);
  t.check('profileExtremes reports the tightest radius, not the mean curvature',
          e.peakV === 40 && e.peakA === 5 && Math.abs(e.minRadiusIn - 4) < 1e-9
          && e.atK === 1,
          `peakV ${e.peakV} peakK ${e.peakK} R ${e.minRadiusIn}`);
  t.check('a straight path reports an infinite radius rather than dividing by zero',
          profileExtremes([{ v: 1, a: 0, k: 0 }]).minRadiusIn === Infinity);
}
{
  // Over budget must be visible as `bad`, and the rigid figure must NOT be:
  // it drops the v^2 (b/2) dk/ds term and can read either side of the truth.
  const over = auditRows({ worstTrue: 12.4, overBudgetTrue: 3, worstRigid: 14.1,
                           overBudgetRigid: 40, peakOmitted: 176, worstWheelSpeed: 60,
                           rows: new Array(754) }, null);
  const under = auditRows({ worstTrue: 11.9, overBudgetTrue: 0, worstRigid: 14.1,
                            overBudgetRigid: 40, peakOmitted: 176, worstWheelSpeed: 60,
                            rows: new Array(754) }, null);
  t.check('the audit panel flags a true over-budget and stays quiet about the rigid one',
          over[0].cls === 'bad' && under[0].cls === 'ok'
          && over[1].cls === 'off' && under[1].cls === 'off',
          `${over[0].v} / ${under[0].v}`);
}

// The audit itself. `aTrue` is the interval AVERAGE acceleration, so it has to
// be paired with both of the interval's wheel speeds: the pair (v at the end of
// interval i, acceleration OF interval i) is a real instantaneous demand and is
// the one a leading-speed-only pairing never evaluates.
//
// Hand-worked. kV = kA = 1 V per metre unit = 0.0254 V per inch unit, kS = 0,
// straight (k = 0, so both wheels are the chassis). v = 0 -> 100 -> 0 in/s on a
// 1 s grid, so a = +100 then -100 in/s^2.
//   leading only: (0,+100) = 2.54 V, (100,-100) = 0 V, (0,0) = 0 V -> 2.54 V
//   both ends:    interval 0 also gives (100,+100) = 2.54 + 2.54 = 5.08 V
// 5.08 V is the demand an instant before the profile's peak, and it is real.
{
  const states = [{ t: 0, v: 0, a: 100, k: 0 },
                  { t: 1, v: 100, a: -100, k: 0 },
                  { t: 2, v: 0, a: 0, k: 0 }];
  const a = auditWheelVoltages(states, { trackWidth: 10, kS: 0, kV: 1, kA: 1,
                                         maxVoltage: 12 });
  t.check('auditWheelVoltages takes the worse end of each interval',
          Math.abs(a.worstTrue - 5.08) < 1e-9,
          `${a.worstTrue.toFixed(6)} V, expected 5.080000 `
          + `(a leading-speed-only pairing gives 2.540000)`);
  // And the rigid figure must NOT move: it is the constraint's own per-state
  // formula, so the two stay independent readings of the same trajectory.
  t.check('and leaves the rigid figure alone',
          Math.abs(a.worstRigid - 2.54) < 1e-9, `${a.worstRigid.toFixed(6)} V`);
}

// ===========================================================================
// mpc_view.js
// ===========================================================================
t.section('MPC diagnostics');

{
  const src = read('include/chad/movement_control/mpc_follower.hpp');
  const block = src.slice(src.indexOf('enum class Mode'));
  const names = block.slice(block.indexOf('{') + 1, block.indexOf('}'))
    .replace(/\/\/[^\n]*/g, '')
    .split(',').map((s) => s.split('=')[0].trim()).filter(Boolean);
  t.check('MPC_MODES has one entry per MpcPathFollower::Mode, in order',
          names.length === MPC_MODES.length
          && names.every((n, i) => n.toLowerCase() === MPC_MODES[i].toLowerCase()),
          `cpp [${names.join(',')}] vs js [${MPC_MODES.join(',')}]`);
  t.check('the trace column list matches the bridge stride',
          TRACE_COLUMNS.length === api.mpcTraceStride(),
          `${TRACE_COLUMNS.length} columns vs stride ${api.mpcTraceStride()}`);
}

// The verdict has to fire on worstPrimalResidual with NO boolean diagnostic set,
// because that is exactly the case docs/sim-audit.md records: an infeasible
// corridor plus a rate limit destabilised the loop and nothing else fired.
{
  const base = {
    ticks: 400, solverIterations: 400, maxIters: 400, meanIters: 1, rebuilds: 0,
    convergedTicks: 400, worstTickUs: 40, admmTicks: 400, constraintRows: 30,
    factorizationFailures: 0, relaxedRowTicks: 0, infeasibleTicks: 0,
    droppedRowTicks: 0, constraintsRejected: false, worstPrimalResidual: 1e-9,
    worstDualResidual: 1e-7, satTicks: 0, activeBoundTicks: 12,
    peakWheelDemand: 1.2,
  };
  t.check('a healthy run is ok', mpcVerdict(base).cls === 'ok',
          mpcVerdict(base).headline);
  const silent = { ...base, worstPrimalResidual: 0.4, meanIters: 400 };
  const v = mpcVerdict(silent);
  t.check('an infeasible corridor is caught by the residual alone',
          v.cls === 'bad' && /infeasible/.test(v.headline) && v.residualBad && v.atCap,
          v.headline);
  const rti = { ...base, convergedTicks: 0, meanIters: 1, worstPrimalResidual: 1e-5 };
  t.check('an RTI budget that stops short but tracks is a warning, not a failure',
          mpcVerdict(rti).cls === 'warn' && !mpcVerdict(rti).residualBad,
          mpcVerdict(rti).headline);
  const fact = { ...base, factorizationFailures: 2, worstPrimalResidual: 1e-9 };
  t.check('a factorization failure outranks everything else',
          mpcVerdict(fact).cls === 'bad' && /factorization/.test(mpcVerdict(fact).headline));
  t.check('a run that never happened is off, not ok',
          mpcVerdict({ ticks: 0 }).cls === 'off' && mpcVerdict(null).cls === 'off');
  t.check('the budget fraction is against the control period',
          Math.abs(mpcVerdict(base, { tickBudgetUs: 10000 }).budgetFrac - 0.004) < 1e-9,
          `${mpcVerdict(base).detail}`);

  const rows = constraintRows(base);
  const byKey = Object.fromEntries(rows.map((r) => [r.k, r]));
  t.check('all nine constraint diagnostics are surfaced',
          rows.length >= 9 && byKey['rows dropped'] && byKey['rows relaxed']
          && byKey['reported infeasible'] && byKey['factorization failed']
          && byKey['worst primal residual'],
          `${rows.length} rows`);
  t.check('a dropped row is bad and an active bound is neutral',
          constraintRows({ ...base, droppedRowTicks: 1 })
            .find((r) => r.k === 'rows dropped').cls === 'bad'
          && byKey['input box active'].cls === 'off');
  t.check('a rejected constraint set is bad even with everything else clean',
          constraintRows({ ...base, constraintsRejected: true })
            .find((r) => r.k === 'rejected at setup').cls === 'bad');
  t.check('constraintRows on an unrun solver is empty rather than NaN-filled',
          constraintRows({ ticks: 0 }).length === 0 && constraintRows(null).length === 0);
}
{
  const budget = { horizon: 20, maxQpRows: 160, maxStateRows: 80, maxSpecs: 4,
                   specsAttempted: 3, specsAccepted: 3, plannedRows: 160,
                   constraintRows: 160, droppedRows: 0, sizeofMpc: 64512,
                   sizeofFollower: 65946 };
  t.check('the row budget reads ok when everything asked for was installed',
          rowBudgetRows(budget).every((r) => r.cls !== 'bad'),
          rowBudgetRows(budget).map((r) => `${r.k}=${r.v}`).join(' · '));
  t.check('an accepted spec whose rows vanished is bad',
          rowBudgetRows({ ...budget, constraintRows: 120, droppedRows: 40 })
            .filter((r) => r.cls === 'bad').length === 2);
  t.check('rowBudgetRows(null) is empty', rowBudgetRows(null).length === 0);
}

// Decimation that hides a three-tick spike out of four thousand hides the whole
// point of the trace.
{
  const n = 4000;
  const data = new Array(n).fill(1e-9);
  data[1234] = 0.9;
  const d = decimateExtremes((i) => data[i], n, 240);
  const peak = Math.max(...d.map((b) => b.max));
  t.check('a single-tick residual spike survives decimation',
          d.length <= 240 && peak === 0.9, `${d.length} buckets, peak ${peak}`);
  t.check('and the bucket points back at the tick',
          d.find((b) => b.max === 0.9).i <= 1234
          && d.find((b) => b.max === 0.9).i + Math.ceil(n / 240) > 1234);
  t.check('an empty trace decimates to nothing',
          decimateExtremes(() => 0, 0, 240).length === 0
          && decimateExtremes(() => 0, 10, 0).length === 0);
  t.check('fewer ticks than buckets is one bucket per tick',
          decimateExtremes((i) => i, 5, 240).length === 5);
}

// ===========================================================================
// app.js against index.html
// ===========================================================================
//
// app.js is a module: one `document.getElementById('typo').addEventListener` and
// the WHOLE page is blank, with the failure in the console and nothing on
// screen. That is the worst bug available here and the one a headless suite can
// still catch, because both sides are static text.
t.section('every element app.js reaches for exists in the markup');

{
  const app = read('sim/web/app.js');
  const html = read('sim/web/index.html');
  const declared = new Set(
    [...html.matchAll(/\bid="([^"]+)"/g)].map((m) => m[1]));

  const wanted = new Set(
    [...app.matchAll(/getElementById\('([^']+)'\)/g)].map((m) => m[1]));
  // The three panels reach for their elements through a prefixing helper, so the
  // prefix has to be reassembled or two thirds of the new markup goes unchecked.
  for (const [, helper, suffix] of
       app.matchAll(/\b(mpEl|tjEl|mpcEl)\('([^']+)'\)/g)) {
    wanted.add({ mpEl: 'mp-', tjEl: 'tj-', mpcEl: 'mpc-' }[helper] + suffix);
  }
  // ...and the HUD map is a list of suffixes on one prefix.
  const hudBlock = app.slice(app.indexOf('const hud = Object.fromEntries'));
  const hudKeys = hudBlock.slice(hudBlock.indexOf('['), hudBlock.indexOf(']'))
    .match(/'([^']+)'/g) ?? [];
  for (const k of hudKeys) wanted.add('hud-' + k.slice(1, -1));
  // buildParams hosts are passed as bare strings, not through getElementById.
  for (const [, host] of app.matchAll(/buildParams\('([^']+)'/g)) wanted.add(host);

  const missing = [...wanted].filter((id) => !declared.has(id)).sort();
  t.check('no element id in app.js is absent from index.html',
          missing.length === 0,
          missing.length ? missing.join(', ') : `${wanted.size} ids checked`);
  t.check('the scan found the new panels, so it is not vacuous',
          [...wanted].filter((id) => /^(mp|tj|mpc)-/.test(id)).length >= 25,
          `${[...wanted].filter((id) => /^(mp|tj|mpc)-/.test(id)).length} panel ids`);

  // A tab with no pane is a button that blanks the panel.
  const tabs = [...html.matchAll(/data-tab="([^"]+)"/g)].map((m) => m[1]);
  const panes = [...html.matchAll(/data-pane="([^"]+)"/g)].map((m) => m[1]);
  t.check('every tab has a pane and every pane has a tab',
          tabs.length === panes.length && tabs.every((x) => panes.includes(x)),
          `tabs [${tabs.join(',')}] panes [${panes.join(',')}]`);

  // Every cwrap has to name a symbol the build actually exports, or the panel
  // throws on load. cwrap itself does not check until the call.
  const exported = [...app.matchAll(/w\('(sim_[a-z0-9_]+)'/g)].map((m) => m[1]);
  const absent = [...new Set(exported)].filter((n) => typeof M['_' + n] !== 'function');
  t.check('every symbol app.js cwraps is exported by the build',
          absent.length === 0,
          absent.length ? absent.join(', ') : `${new Set(exported).size} symbols`);
}

// ===========================================================================
// The markup, statically
// ===========================================================================
//
// Nobody here can open a browser, so everything that IS decidable from the two
// files is decided here: the id graph in both directions, the element KIND
// behind every read, the classes the panels emit against the ones the stylesheet
// defines, and the palette's contrast. What is left needs eyes, and the report
// says which parts those are.
t.section('the id graph, in both directions');

const WEB_JS = ['app.js', 'view_panel.js', 'motion_view.js', 'traj_view.js',
                'mpc_view.js', 'health.js', 'route_dsl.js', 'traj.js',
                'ltv_traj.js'];
const htmlSrc = read('sim/web/index.html');
const appSrc = read('sim/web/app.js');
const webJs = WEB_JS.map((f) => read('sim/web/' + f)).join('\n');
const cssSrc = read('sim/web/style.css');
const PREFIX = { mpEl: 'mp-', tjEl: 'tj-', mpcEl: 'mpc-' };
// A one-liner that wraps a prefix helper (mpNum) reaches elements too, and the
// ids it touches are invisible to this scan in BOTH directions unless it is
// discovered rather than listed. It also records WHICH property it reads, so the
// element-kind checks below see through it.
const WRAP = {};
for (const m of read('sim/web/app.js').matchAll(
       /const (\w+) = \((\w+)\) =>[^\n]*?\b(mpEl|tjEl|mpcEl)\((\w+)\)\.(\w+)/g)) {
  if (m[2] === m[4]) WRAP[m[1]] = { prefix: PREFIX[m[3]], prop: m[5] };
}
const helperNames = [...Object.keys(PREFIX), ...Object.keys(WRAP)].join('|');
const elGet = () => new RegExp("\\b(" + helperNames + ")\\('([^']+)'\\)", 'g');
const prefixOf = (h) => PREFIX[h] ?? WRAP[h].prefix;

// id -> the element behind it. Tags first, ids second, rather than one regex
// that has to guess attribute order and silently drops what it did not expect.
const elems = new Map();
const dupes = [];
for (const m of htmlSrc.matchAll(/<([a-zA-Z][\w-]*)\b([^>]*)>/g)) {
  const idm = /\bid="([^"]+)"/.exec(m[2]);
  if (!idm) continue;
  if (elems.has(idm[1])) dupes.push(idm[1]);
  elems.set(idm[1], { tag: m[1].toLowerCase(), attrs: m[2] });
}
const declared = new Set(elems.keys());

t.check('no id is declared twice in the markup', dupes.length === 0,
        dupes.length ? dupes.join(', ') : `${declared.size} unique ids`);

// --- every id the JavaScript names -----------------------------------------
// Literal getElementById, the three panel prefix helpers, the HUD suffix map,
// the buildParams hosts, and the two lists that reach their elements through a
// VARIABLE id -- which a getElementById scan alone cannot see at all.
const wanted = new Set(
  [...webJs.matchAll(/getElementById\('([^']+)'\)/g)].map((m) => m[1]));
for (const [, helper, suffix] of webJs.matchAll(elGet())) {
  wanted.add(prefixOf(helper) + suffix);
}
const hudBlock = appSrc.slice(appSrc.indexOf('const hud = Object.fromEntries'));
const hudKeys = hudBlock.slice(hudBlock.indexOf('['), hudBlock.indexOf(']'))
  .match(/'([^']+)'/g) ?? [];
for (const k of hudKeys) wanted.add('hud-' + k.slice(1, -1));
for (const [, host] of webJs.matchAll(/buildParams\('([^']+)'/g)) wanted.add(host);

const viaVar = [];
const locObj = /LOC_IDS\s*=\s*\{([^}]*)\}/.exec(appSrc);
if (locObj) viaVar.push(...(locObj[1].match(/'([^']+)'/g) ?? []));
for (const m of appSrc.matchAll(
       /\[([^\]]*)\]\s*\.map\(\s*\w+\s*=>\s*document\.getElementById\(/g)) {
  viaVar.push(...(m[1].match(/'([^']+)'/g) ?? []));
}
for (const q of viaVar) wanted.add(q.slice(1, -1));

const absentIds = [...wanted].filter((id) => !declared.has(id)).sort();
t.check('every id the browser code names exists in the markup',
        absentIds.length === 0,
        absentIds.length ? absentIds.join(', ')
                         : `${wanted.size} ids over ${WEB_JS.length} files`);
// The variable-id paths are the ones a naive scan misses, so prove they were
// picked up instead of trusting the count.
t.check('the scan sees the ids reached through a variable, so it is not vacuous',
        ['start-x', 'start-y', 'start-h', 'loc-mcl', 'loc-rcl', 'loc-ekf']
          .every((id) => wanted.has(id)) && viaVar.length >= 6,
        `${viaVar.length} variable-id references recovered`);
t.check('the scan sees the panel prefixes and their wrappers, so it is not vacuous',
        [...wanted].filter((id) => /^(mp|tj|mpc|hud)-/.test(id)).length >= 50 &&
        Object.keys(WRAP).length >= 1 && wanted.has('mp-a'),
        `${[...wanted].filter((id) => /^(mp|tj|mpc|hud)-/.test(id)).length} prefixed ids, ` +
        `wrappers [${Object.keys(WRAP).join(', ')}]`);

// --- and the other direction ----------------------------------------------
// An id in the markup that nothing mentions -- no JavaScript, no #id rule in the
// stylesheet -- is a dead hook or a rename that only half landed.
{
  const orphans = [...declared].filter((id) => {
    if (wanted.has(id)) return false;
    if (webJs.includes(`'${id}'`)) return false;
    return !new RegExp('#' + id + '\\b').test(cssSrc);
  }).sort();
  t.check('every id in the markup is used by the code or the stylesheet',
          orphans.length === 0,
          orphans.length ? orphans.join(', ') : `${declared.size} ids accounted for`);
}

// --- handlers and element kinds -------------------------------------------
{
  // A listener bound to a missing node throws at import time and the WHOLE page
  // is blank, so this is called out separately from the id graph above.
  const varOf = new Map();
  for (const m of appSrc.matchAll(
         /(?:const|let)\s+([A-Za-z_$][\w$]*)\s*=\s*document\.getElementById\('([^']+)'\)/g)) {
    varOf.set(m[1], m[2]);
  }
  for (const m of appSrc.matchAll(
         /(?:const|let)\s+([A-Za-z_$][\w$]*)\s*=\s*(mpEl|tjEl|mpcEl)\('([^']+)'\)/g)) {
    varOf.set(m[1], PREFIX[m[2]] + m[3]);
  }
  /// Every id that appears with `.prop`, however it was reached.
  const useOf = (prop) => {
    const ids = new Set();
    for (const m of appSrc.matchAll(
           new RegExp("getElementById\\('([^']+)'\\)\\." + prop + "\\b", 'g'))) {
      ids.add(m[1]);
    }
    for (const m of appSrc.matchAll(
           new RegExp("\\b(mpEl|tjEl|mpcEl)\\('([^']+)'\\)\\." + prop + "\\b", 'g'))) {
      ids.add(PREFIX[m[1]] + m[2]);
    }
    for (const [helper, w] of Object.entries(WRAP)) {
      if (w.prop !== prop) continue;
      for (const m of appSrc.matchAll(new RegExp("\\b" + helper + "\\('([^']+)'\\)", 'g'))) {
        ids.add(w.prefix + m[1]);
      }
    }
    for (const [v, id] of varOf) {
      if (new RegExp('\\b' + v + '\\.' + prop + '\\b').test(appSrc)) ids.add(id);
    }
    return [...ids];
  };

  const bound = useOf('addEventListener');
  const dead = bound.filter((id) => !declared.has(id)).sort();
  t.check('no handler is bound to a node that does not exist',
          dead.length === 0 && bound.length >= 20,
          dead.length ? dead.join(', ') : `${bound.length} listeners`);

  // Class selectors are the other way to miss: querySelectorAll('.tab') against
  // markup that renamed the class is an empty list and a dead tab bar.
  const classSel = new Set(
    [...appSrc.matchAll(/document\.querySelectorAll?\('\.([\w-]+)'\)/g)].map((m) => m[1]));
  const htmlClasses = new Set(
    [...htmlSrc.matchAll(/class="([^"]+)"/g)].flatMap((m) => m[1].trim().split(/\s+/)));
  const absentSel = [...classSel].filter((c) => !htmlClasses.has(c));
  t.check('every class selector the code queries exists in the markup',
          absentSel.length === 0 && classSel.size >= 2,
          absentSel.length ? absentSel.join(', ') : `[${[...classSel].join(', ')}]`);

  // Reading `.checked` off a div is undefined and writing `.innerHTML` into an
  // <input> is a silent no-op. Both look exactly like markup nobody wired up.
  const known = (ids) => ids.filter((id) => declared.has(id));
  const isCheckbox = (id) => elems.get(id).tag === 'input' &&
                             /type="checkbox"/.test(elems.get(id).attrs);
  const valueTags = new Set(['input', 'select', 'textarea']);

  const checkedIds = known(useOf('checked'));
  const badChecked = checkedIds.filter((id) => !isCheckbox(id));
  t.check('every element read as .checked is a checkbox',
          badChecked.length === 0 && checkedIds.length >= 3,
          badChecked.length
            ? badChecked.map((id) => `${id} is <${elems.get(id).tag}>`).join(', ')
            : `${checkedIds.length} checkboxes`);

  const valueIds = known(useOf('value'));
  const badValue = valueIds.filter((id) => !valueTags.has(elems.get(id).tag));
  t.check('every element read as .value takes a value',
          badValue.length === 0 && valueIds.length >= 15,
          badValue.length
            ? badValue.map((id) => `${id} is <${elems.get(id).tag}>`).join(', ')
            : `${valueIds.length} value-bearing elements`);

  const written = new Set(known([...useOf('innerHTML'), ...useOf('textContent'),
                                 ...useOf('appendChild')]));
  const badWrite = [...written].filter((id) => elems.get(id).tag === 'input');
  t.check('nothing writes children into a void <input>',
          badWrite.length === 0 && written.size >= 8,
          badWrite.length ? badWrite.join(', ') : `${written.size} write targets`);
}

// ===========================================================================
// The stylesheet against the code that leans on it
// ===========================================================================
t.section('classes and colours');

// A class the code toggles but the stylesheet never mentions is a state change
// that lands in the DOM and nowhere on screen.
{
  const toggled = new Set(
    [...webJs.matchAll(/classList\.(?:add|remove|toggle)\('([\w-]+)'/g)].map((m) => m[1]));
  const unstyled = [...toggled].filter((c) => !new RegExp('\\.' + c + '\\b').test(cssSrc));
  t.check('every class the code toggles is styled',
          unstyled.length === 0 && toggled.size >= 3,
          unstyled.length ? unstyled.join(', ') : `[${[...toggled].join(', ')}]`);

  // The three view modules decide a colour by returning a class name. With no
  // rule for one, the verdict is computed, tested, and then rendered in the
  // ordinary text colour -- the panel says "bad" in the same grey as "ok".
  //
  // Per CONTAINER, not "styled somewhere": every one of these names goes into a
  // `.rows` grid, and `.diag .warn` existing says nothing about whether the row
  // grids can show a warning.
  const viewSrc = ['motion_view.js', 'traj_view.js', 'mpc_view.js']
    .map((f) => read('sim/web/' + f)).join('\n');
  const ROW_FNS = ['outcomeClass', 'motionRow', 'motionSummary', 'auditRows',
                   'constraintRows', 'rowBudgetRows'];
  const bodies = ROW_FNS.map((fn) => {
    const a = viewSrc.indexOf('export function ' + fn + '(');
    if (a < 0) return null;
    const b = viewSrc.indexOf('\nexport ', a + 1);
    return viewSrc.slice(a, b < 0 ? undefined : b);
  });
  t.check('every row builder was found, so the class scan is not vacuous',
          bodies.every((b) => b !== null),
          `${bodies.filter((b) => b).length} of ${ROW_FNS.length} functions`);
  const emitted = new Set(bodies.filter((b) => b).join('\n')
    .match(/'(ok|warn|bad|off|err)'/g)?.map((q) => q.slice(1, -1)) ?? []);
  const unpainted = [...emitted].filter(
    (c) => !new RegExp('\\.rows\\s+\\.' + c + '\\b').test(cssSrc));
  t.check('every verdict class a row builder emits has a colour in .rows',
          unpainted.length === 0 && emitted.size >= 4,
          unpainted.length ? unpainted.join(', ') : `[${[...emitted].sort().join(', ')}]`);
  // The HUD's diagnostics grid takes the same names, so the two have to agree.
  const diagShort = [...emitted].filter(
    (c) => !new RegExp('\\.diag\\s+\\.' + c + '\\b').test(cssSrc));
  t.check('the HUD diagnostics grid defines the same class set',
          diagShort.length === 0,
          diagShort.length ? diagShort.join(', ') : `${emitted.size} classes`);
}

// Contrast. The palette is one block of custom properties, so the ratios ARE
// checkable without a browser: WCAG relative luminance is a closed form.
{
  const vars = {};
  for (const m of cssSrc.matchAll(/--([a-z-]+):\s*(#[0-9a-fA-F]{3,6})\s*;/g)) vars[m[1]] = m[2];
  const chan = (v) => (v / 255 <= 0.03928 ? v / 255 / 12.92
                                          : Math.pow((v / 255 + 0.055) / 1.055, 2.4));
  const lum = (hex) => {
    let h = hex.slice(1);
    if (h.length === 3) h = h.split('').map((c) => c + c).join('');
    return 0.2126 * chan(parseInt(h.slice(0, 2), 16))
         + 0.7152 * chan(parseInt(h.slice(2, 4), 16))
         + 0.0722 * chan(parseInt(h.slice(4, 6), 16));
  };
  const ratio = (a, b) => {
    const [hi, lo] = [lum(a), lum(b)].sort((x, y) => y - x);
    return (hi + 0.05) / (lo + 0.05);
  };
  const fg = ['text', 'dim', 'accent', 'true', 'odom', 'mcl', 'beam'];
  const bg = ['bg', 'panel', 'card'];
  t.check('the palette is read out of the stylesheet, not restated here',
          fg.concat(bg).every((k) => /^#[0-9a-fA-F]{6}$/.test(vars[k] ?? '')),
          `${Object.keys(vars).length} colour tokens parsed`);
  let worst = { r: Infinity, pair: '' };
  for (const f of fg) {
    for (const b of bg) {
      const r = ratio(vars[f], vars[b]);
      if (r < worst.r) worst = { r, pair: `--${f} on --${b}` };
    }
  }
  // 4.5:1 is WCAG AA for body text and every one of these tokens is used as
  // text somewhere. The WORST pair, not the mean: a mean passes while one
  // unreadable label sits in the corner.
  t.check('every foreground token clears 4.5:1 on every surface',
          worst.r >= 4.5,
          `worst is ${worst.pair} at ${worst.r.toFixed(2)}:1 over ` +
          `${fg.length * bg.length} pairs`);
}

// ===========================================================================
// The recorder contract the plots and the CSV both assume
// ===========================================================================
t.section('plot and export columns against the recorder');
{
  const stride = api.recStride();
  const sig = [...appSrc.matchAll(/^\s*\['(\w+)',\s*'[^']*',\s*(\d+),/gm)]
    .map((m) => ({ key: m[1], col: Number(m[2]) }));
  t.check('the signal list was found and every column is inside the record',
          sig.length >= 8 && sig.every((s) => s.col >= 0 && s.col < stride),
          `${sig.length} signals, highest column ${Math.max(...sig.map((s) => s.col))} ` +
          `of stride ${stride}`);

  const cols = /const cols = \[([^\]]*)\]/.exec(appSrc);
  const names = cols ? (cols[1].match(/'[^']+'/g) ?? []) : [];
  t.check('the CSV header has exactly one name per recorded column',
          names.length === stride,
          `${names.length} header names, stride ${stride}`);
}

// ===========================================================================
// view_panel.js: the branches the transport bar and the View tab take
// ===========================================================================
t.section('transport and geometry-editor logic');
{
  // All eight switch combinations against the bits sim_set_localization takes.
  const combos = [];
  for (let i = 0; i < 8; i++) {
    combos.push({ i, s: locSummary({ mcl: !!(i & 1), rcl: !!(i & 2), ekf: !!(i & 4) }) });
  }
  t.check('the localisation bitmask is one bit per switch',
          combos.every((c) => c.s.mode === c.i),
          combos.map((c) => `${c.i}->${c.s.mode}`).join(' '));
  t.check('nothing ticked reads as dead reckoning, not as an empty label',
          combos[0].s.label === 'dead reckoning' &&
          combos[7].s.label === 'MCL + RCL + EKF',
          `0 -> "${combos[0].s.label}", 7 -> "${combos[7].s.label}"`);
  t.check('the label says who owns the pose once the EKF is up',
          combos.filter((c) => /EKF owns/.test(c.s.title)).map((c) => c.i).join(',')
            === '4,5,6,7',
          `EKF-owned modes: ${combos.filter((c) => /EKF owns/.test(c.s.title))
            .map((c) => c.i).join(',')}`);

  // Scrubbing. "not scrubbing" and "nothing recorded" are different states and
  // the slider is enabled in one and not the other.
  const idle = scrubState(false, 500, 100);
  const empty = scrubState(true, 500, 0);
  t.check('an idle scrubber is disabled and says live',
          idle.index === null && idle.text === 'live' && idle.disabled === true,
          `index ${idle.index}, "${idle.text}", disabled ${idle.disabled}`);
  t.check('an empty recording says so instead of scrubbing nothing',
          empty.index === null && empty.text === 'no data' && empty.disabled === false,
          `index ${empty.index}, "${empty.text}", disabled ${empty.disabled}`);
  t.check('the scrub maps 0..1000 onto the whole recording',
          scrubState(true, 0, 101).index === 0 &&
          scrubState(true, 500, 101).index === 50 &&
          scrubState(true, 1000, 101).index === 100,
          `0/500/1000 -> ${[0, 500, 1000].map((v) => scrubState(true, v, 101).index)
            .join('/')} of 101 rows`);
  t.check('an out-of-range or NaN slider cannot index off the recording',
          scrubState(true, 4000, 101).index === 100 &&
          scrubState(true, -50, 101).index === 0 &&
          scrubState(true, NaN, 101).index === 0,
          `4000 -> ${scrubState(true, 4000, 101).index}, ` +
          `-50 -> ${scrubState(true, -50, 101).index}, ` +
          `NaN -> ${scrubState(true, NaN, 101).index}`);
  t.check('a click on either plot gutter seeks to an end, not off the buffer',
          seekFraction(0, 640, 38, 8) === 0 && seekFraction(640, 640, 38, 8) === 1 &&
          Math.abs(seekFraction(38 + (640 - 46) / 2, 640, 38, 8) - 0.5) < 1e-9,
          `x=0 -> 0, x=640 -> 1, midpoint -> ` +
          `${seekFraction(38 + (640 - 46) / 2, 640, 38, 8).toFixed(4)}`);

  // The geometry editor round trip. The kind tag is what makes a segment the
  // collision perimeter, so losing it in the textarea turned Apply with no edits
  // into a field with no walls.
  const geom = {
    segments: [[-70.2, -70.2, 70.2, -70.2, 0], [-20, -68, -20, 68, 2]],
    circles: [[0, 0, 2, 1]],
  };
  const round = parseFieldGeom(fieldGeomText(geom));
  t.check('the geometry editor round trip keeps the perimeter tag',
          !round.error && round.segments.length === 2 &&
          round.segments[0][4] === 0 && round.segments[1][4] === 2 &&
          round.circles.length === 1,
          `kinds [${round.segments?.map((s) => s[4]).join(', ')}] out of ` +
          `[${geom.segments.map((s) => s[4]).join(', ')}]`);
  t.check('a hand-typed segment defaults to a structure, not to perimeter',
          parseFieldGeom('{"segments":[[0,0,1,1]]}').segments[0][4] === 2,
          `kind ${parseFieldGeom('{"segments":[[0,0,1,1]]}').segments[0][4]}`);
  t.check('an omitted key is empty geometry, not a crash',
          (() => {
            const g = parseFieldGeom('{}');
            return !g.error && g.segments.length === 0 && g.circles.length === 0;
          })());
  for (const [text, why] of [['{', 'truncated JSON'],
                             ['[1,2]', 'a bare array'],
                             ['"x"', 'a bare string'],
                             ['{"segments":[[0,0,1]]}', 'a three-number segment'],
                             ['{"circles":[[0,0]]}', 'a two-number circle'],
                             ['{"segments":"no"}', 'a non-array segments']]) {
    const g = parseFieldGeom(text);
    t.check(`the editor reports ${why} instead of clearing the field`,
            typeof g.error === 'string' && g.error.length > 0 &&
            g.segments === undefined,
            g.error);
  }
}

process.exit(t.finish());
