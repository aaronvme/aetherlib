// The MPC / RTI study: solver correctness, the general-constraint surface, and
// what a constrained solve costs. Run with: node sim/test/mpc.mjs
//
// The three things this suite is built around, because each one hid a bug that a
// passing suite did not catch:
//
//  * Mode numbers are NAMED here and pinned by a run whose fingerprint only the
//    right mode can produce. Reading them off as bare integers is how the whole
//    closed-loop section came to be exercising the LQR baselines instead of the
//    MPC for as long as it did.
//  * sim_mpc_set returns whether it recognised the key, and every setter call
//    goes through cfg() which throws on a typo. A silently ignored knob turns a
//    constrained experiment into a rerun of the defaults.
//  * A corridor can be primal INFEASIBLE, and then no amount of solving will
//    honour it. Feasibility is decided by an exact reachability bound derived
//    below from the plant, not by asking the solver whether it is happy.
import { loadSim, wrap, tally } from './harness.mjs';

const M = await loadSim([
  'sim_mpc_run', 'sim_mpc_rows_json', 'sim_mpc_trace_ptr', 'sim_mpc_stats_json',
]);
const api = wrap(M, {
  init: ['sim_init', null, []],
  reset: ['sim_reset', null, ['number', 'number', 'number']],
  step: ['sim_step', null, ['number']],
  setLoc: ['sim_set_localization', null, ['number']],
  stateJson: ['sim_state_json', 'string', []],
  setParam: ['sim_set_param', null, ['string', 'number']],
  collideTicks: ['sim_collide_ticks', 'number', []],
  trajClear: ['sim_mpc_traj_clear', null, []],
  trajPush: ['sim_mpc_traj_push', null, ['number', 'number', 'number', 'number', 'number']],
  defaults: ['sim_mpc_defaults', null, []],
  set: ['sim_mpc_set', 'number', ['string', 'number']],
  run: ['sim_mpc_run', null, []],
  running: ['sim_mpc_running', 'number', []],
  statsJson: ['sim_mpc_stats_json', 'string', []],
  bench: ['sim_mpc_bench_json', 'string', ['number', 'number', 'number', 'number', 'number']],
  check: ['sim_mpc_check_json', 'string', ['number', 'number']],
  rows: ['sim_mpc_rows_json', 'string', ['number', 'number', 'number', 'number']],
  tracePtr: ['sim_mpc_trace_ptr', 'number', []],
  traceCount: ['sim_mpc_trace_count', 'number', []],
  traceStride: ['sim_mpc_trace_stride', 'number', []],
});
api.init();

const t = tally('mpc.mjs');
const M2IN = 39.3701;

/// MpcPathFollower::Mode, in declaration order. Named because the integers are
/// a wire format and getting them wrong is silent -- an LQR run reports ticks
/// and cross-track just like an MPC run does.
const MODE = { scheduledLqr: 0, riccatiPerTick: 1, mpcConverged: 2, mpcRti: 3 };
/// MpcSolver, likewise.
const SOLVER = { fastGradient: 0, gaussSeidel: 1, admm: 2 };

/// Every knob goes through here. sim_mpc_set returns 0 for a key it does not
/// know, and a typo that silently leaves the defaults in place would make the
/// experiment below measure nothing.
function cfg(obj) {
  api.defaults();
  for (const [k, v] of Object.entries(obj)) {
    if (!api.set(k, v)) throw new Error(`sim_mpc_set rejected key "${k}"`);
  }
}

/// Per-tick solver diagnostics, read straight out of the WASM heap. HEAPF32 is
/// re-created when memory grows, so it is fetched per call.
function trace() {
  const p = api.tracePtr() >> 2, n = api.traceCount(), s = api.traceStride();
  const f = M.HEAPF32, out = [];
  for (let i = 0; i < n; i++) {
    const b = p + i * s;
    out.push({
      iters: f[b], primal: f[b + 1], dual: f[b + 2], converged: f[b + 3] > 0.5,
      activeRows: f[b + 4], dropped: f[b + 5], relaxed: f[b + 6],
      infeasible: f[b + 7] > 0.5, ey: f[b + 8], theta: f[b + 9],
      u: f[b + 10], cmd: f[b + 11], wRef: f[b + 12], plan: f[b + 13],
      activeBounds: f[b + 14],
    });
  }
  return out;
}

// ---------------------------------------------------------------------------
// Exact feasibility oracle for a cross-track corridor
// ---------------------------------------------------------------------------
//
// The error plant decouples: along-track is driven only by u_v, and cross-track
// and heading only by u_w. Unrolling
//
//     theta_{k+1} = theta_k + dt u_k
//     e_{k+1}     = e_k + v dt theta_k + 0.5 v dt^2 u_k
//
// gives
//
//     e(k) = e0 + k v dt theta0 + v dt^2 SUM_{j<k} (k - 0.5 - j) u_j
//
// Every coefficient (k - 0.5 - j) is strictly positive, so e(k) is minimised by
// taking each u_j as small as its constraints allow. The componentwise-smallest
// admissible sequence -- drop by exactly duMax each step until the box floor --
// is itself admissible, so that value IS the minimum of e(k) over the feasible
// set. The corridor is therefore primal infeasible at stage k exactly when this
// bound exceeds it. No solver output is consulted.
function minCrossAtStage(k, e0, theta0, v, dt, duMax, anchor, uFloor) {
  let sum = 0, u = anchor;
  for (let j = 0; j < k; j++) {
    u = duMax > 0 ? Math.max(uFloor, u - duMax) : uFloor;
    sum += (k - 0.5 - j) * u;
  }
  return e0 + k * v * dt * theta0 + v * dt * dt * sum;
}

/// Smallest corridor half-width that is reachable at every stage of the horizon.
function reachableCross(e0, theta0, N, plant, duMax = 0, anchor = 0) {
  let worst = 0;
  for (let k = 1; k <= N; k++) {
    const b = minCrossAtStage(k, e0, theta0, plant.v, plant.dt, duMax, anchor, -plant.wBudget);
    if (b > worst) worst = b;
  }
  return worst;
}

// The plant the bench linearises about, taken from the bench itself rather than
// restated here -- a hardcoded vRef would silently detach from the config.
const PLANT = (() => {
  cfg({});
  const r = JSON.parse(api.bench(10, SOLVER.fastGradient, 1, 1, 4));
  return { v: r.vRef, dt: 0.01, wBudget: r.wBudget };
})();

// ===========================================================================
t.section('setter contract: an unrecognised key must be reported');
// ===========================================================================
t.check('a known key is applied', api.set('corridor', 0.02) === 1);
t.check('an unknown key is refused', api.set('notAKnob', 1) === 0);
t.check('a mistyped key is refused', api.set('crossTrackCorridor', 1) === 0,
        'the C name is not the wire name');

// ===========================================================================
t.section('first move equals the LQR move, on every solver path');
// ===========================================================================
// With the DARE cost-to-go as terminal cost and nothing active, the condensed
// QP's first move IS -Kx at any horizon. Forcing ADMM on a box-only problem is
// what makes the general path comparable to the fast path.
for (const [name, s] of Object.entries(SOLVER)) {
  for (const h of [10, 20]) {
    cfg({});
    const r = JSON.parse(api.check(h, s));
    t.check(`${name} h${h}`, r.worstRelErr < 1e-3,
            `rel err ${r.worstRelErr.toExponential(2)} in ${r.maxIters} iters`);
  }
}
{
  // Negative control. rho is fixed for the life of a factorization, so an
  // absurd one cannot be recovered from inside the iteration; if this still
  // passed, the check above would not be measuring the ADMM path.
  cfg({ admmScale: 1e6 });
  const r = JSON.parse(api.check(10, SOLVER.admm));
  t.check('ADMM at rhoScale 1e6 fails the same check', r.worstRelErr >= 1e-3,
          `rel err ${r.worstRelErr.toExponential(2)}`);
}

// ===========================================================================
t.section('row budget is exactly funded, and acceptance means installation');
// ===========================================================================
{
  const worst = JSON.parse(api.rows(20, 2, 4, 0));
  t.check('box + rate + 4 full-span specs exactly fills the budget',
          worst.plannedRows === worst.maxQpRows,
          `${worst.plannedRows} of ${worst.maxQpRows}`);
  t.check('budget arithmetic: 2*2*N + MAX_STATE_ROWS == MAX_QP_ROWS',
          2 * 2 * 20 + worst.maxStateRows === worst.maxQpRows,
          `${2 * 2 * 20} + ${worst.maxStateRows} == ${worst.maxQpRows}`);

  // Embedded footprint. The solver preallocates for the worst case, so this is
  // a fixed cost paid whether or not a constraint is ever configured, and it is
  // far too large to sit on a PROS task stack -- MpcPathFollower has to be a
  // member or a static, never a local.
  t.check('the solver footprint is a fixed ~64 KB, all preallocated',
          worst.sizeofMpc > 32 * 1024 && worst.sizeofMpc < 128 * 1024 &&
            worst.sizeofFollower < 160 * 1024,
          `CondensedMpc ${(worst.sizeofMpc / 1024).toFixed(1)} KiB, ` +
          `MpcPathFollower ${(worst.sizeofFollower / 1024).toFixed(1)} KiB`);

  const over = JSON.parse(api.rows(20, 2, 5, 0));
  t.check('a fifth spec is refused, not dropped',
          over.specsAccepted === over.maxSpecs && over.droppedRows === 0,
          `accepted ${over.specsAccepted}/5, dropped ${over.droppedRows}`);

  for (const [h, ra, sp, span] of [[20, 2, 4, 0], [20, 0, 4, 0], [10, 2, 4, 0],
                                   [20, 1, 3, 5], [20, 2, 2, 0]]) {
    const r = JSON.parse(api.rows(h, ra, sp, span));
    t.check(`h${h} rate${ra} specs${sp} span${span}: every planned row reached C`,
            r.constraintRows === r.plannedRows && r.droppedRows === 0 &&
              r.specsAccepted === sp && r.factorized,
            `planned ${r.plannedRows}, in C ${r.constraintRows}, dropped ${r.droppedRows}`);
  }
}

// ===========================================================================
t.section('input-rate limits bound the COMMAND, not the correction');
// ===========================================================================
// The distinction only exists when the reference itself moves, so the bench
// drives a triangle-wave reference omega whose consecutive samples differ by
// exactly `bSlew`. Every assertion is per-tick, off the trace, not an aggregate.
const DU = 0.05, SLEW = 0.02;
function rateRun(extra, iters = 1) {
  cfg({ rateW: DU, bSlew: SLEW, ...extra });
  const r = JSON.parse(api.bench(10, SOLVER.admm, iters, 1, 600));
  const tr = trace();
  let cmd = 0, corr = 0;
  for (let i = 1; i < tr.length; i++) {
    cmd = Math.max(cmd, Math.abs(tr[i].cmd - tr[i - 1].cmd));
    corr = Math.max(corr, Math.abs(tr[i].u - tr[i - 1].u));
  }
  return { r, cmd, corr, ticks: tr.length };
}
{
  const total = rateRun({ rateTotal: 1 });
  t.check('rateTotal: the command rate is inside the limit every tick',
          total.cmd <= DU + 1e-6,
          `max |d cmd| ${total.cmd.toFixed(6)} <= ${DU} over ${total.ticks} ticks`);
  t.check('rateTotal: the CORRECTION is free to exceed it', total.corr > DU + 1e-6,
          `max |d u| ${total.corr.toFixed(6)} = limit + slew`);

  const corrOnly = rateRun({ rateTotal: 0 });
  t.check('correction mode: the correction rate is inside the limit every tick',
          corrOnly.corr <= DU + 1e-6, `max |d u| ${corrOnly.corr.toFixed(6)}`);
  t.check('correction mode: the COMMAND is then free to exceed it',
          corrOnly.cmd > DU + 1e-6, `max |d cmd| ${corrOnly.cmd.toFixed(6)}`);
  t.check('the two modes are genuinely different problems',
          Math.abs(total.cmd - corrOnly.cmd) > 1e-3,
          `${total.cmd.toFixed(6)} vs ${corrOnly.cmd.toFixed(6)}`);

  // The reference deltas are the only thing that separates them: with a
  // stationary reference the two constraints coincide, which is what proves the
  // difference above comes from setReferenceDeltas and not from anything else.
  const still = rateRun({ rateTotal: 1, bSlew: 0 });
  t.check('a stationary reference collapses the two modes',
          Math.abs(still.cmd - still.corr) < 1e-6 && still.cmd <= DU + 1e-6,
          `cmd ${still.cmd.toFixed(6)}, corr ${still.corr.toFixed(6)}`);

  // setPreviousInput fixes the committed move's rate on its own, so dropping
  // the per-stage reference deltas is invisible in the numbers above -- it moves
  // only the INTERIOR of the plan. Measured on a CONVERGED solve, where the rate
  // rows are actually satisfied rather than merely clamped at the head.
  //
  // capTicks === 0 is asserted as a PREMISE, not decoration. This limit is tight
  // -- 0.05 rad/s against a slewing reference -- and needs 344 iterations on its
  // worst tick, so a maxIters default below that turns this into a test of a
  // truncated solve while still reading like a test of a converged one. That
  // already happened once: at a cap of 100 the bound below is missed at 1.13x,
  // and the number underneath is honest, not a bug.
  const planTotal = rateRun({ rateTotal: 1 }, 0);
  const planCorr = rateRun({ rateTotal: 0 }, 0);
  t.check('rateTotal: the whole PLAN respects the command rate',
          planTotal.r.capTicks === 0 &&
            planTotal.r.maxPlanCmdRate <= DU * 1.05 &&
            planTotal.r.maxPlanCorrRate > DU * 1.2,
          `plan cmd ${planTotal.r.maxPlanCmdRate.toFixed(6)} ` +
          `(${(planTotal.r.maxPlanCmdRate / DU).toFixed(3)}x), ` +
          `plan corr ${planTotal.r.maxPlanCorrRate.toFixed(6)}, ` +
          `${planTotal.r.capTicks} ticks at the cap, worst ${planTotal.r.maxIters} it`);
  t.check('correction mode: the whole PLAN respects the correction rate',
          planCorr.r.capTicks === 0 &&
            planCorr.r.maxPlanCorrRate <= DU * 1.05 &&
            planCorr.r.maxPlanCmdRate > DU * 1.2,
          `plan cmd ${planCorr.r.maxPlanCmdRate.toFixed(6)}, ` +
          `plan corr ${planCorr.r.maxPlanCorrRate.toFixed(6)} ` +
          `(${(planCorr.r.maxPlanCorrRate / DU).toFixed(3)}x), ` +
          `${planCorr.r.capTicks} ticks at the cap, worst ${planCorr.r.maxIters} it`);

  // And the relationship the premise protects, recorded rather than left to be
  // rediscovered: truncating the converged solve loosens the INTERIOR rate rows
  // monotonically, while the committed move stays exactly inside the limit
  // because solve() clamps its head. This is the number that says how much
  // maxIters is worth on a tight rate limit.
  {
    const caps = [25, 50, 100, 400];
    const got = caps.map((m) => rateRun({ rateTotal: 1, maxIters: m }, 0));
    const ratios = got.map((g) => g.r.maxPlanCmdRate / DU);
    let monotone = true;
    for (let i = 1; i < ratios.length; i++) if (ratios[i] > ratios[i - 1]) monotone = false;
    t.check('a truncated converged solve loosens the plan interior, not the head',
            monotone && ratios[0] > 3 && ratios[ratios.length - 1] < 1.05 &&
              got.every((g) => g.cmd <= DU + 1e-6),
            caps.map((m, i) => `${m}:${ratios[i].toFixed(2)}x`).join(' ') +
            `  head max |d cmd| ${Math.max(...got.map((g) => g.cmd)).toFixed(6)}`);
  }

  // The sharp version of the head-clamp story: at one iteration the plan's own
  // rate profile is 70x the limit, and the committed move is still exactly
  // inside it, because solve() clamps the head against the anchor on the way
  // out. A corridor gets no such treatment -- that is the whole difference
  // between a constraint on the committed input and one on a predicted state.
  const one = rateRun({ rateTotal: 1 }, 1);
  t.check('at one iteration the head is clamped but the plan is not',
          one.cmd <= DU + 1e-6 && one.r.maxPlanCmdRate > 20 * DU,
          `committed ${one.cmd.toFixed(6)}, plan ${one.r.maxPlanCmdRate.toFixed(4)}`);

  // Same loop, same one-iteration budget, no rate rows: the command swings an
  // order of magnitude harder. Without this the assertions above could all pass
  // on a loop where nothing was moving in the first place.
  cfg({ bSlew: SLEW });
  const free = JSON.parse(api.bench(10, SOLVER.fastGradient, 1, 1, 600));
  t.check('unconstrained, the same loop blows through that rate',
          free.maxCmdRate > 10 * DU,
          `max |d cmd| ${free.maxCmdRate.toFixed(4)} vs limit ${DU}`);
}

// ===========================================================================
t.section('cross-track corridor: enforced when feasible');
// ===========================================================================
// theta0 = 0.12 with no rate limit. The corridor is active -- the unconstrained
// plan reaches 0.0164 m -- and the oracle puts the reachable floor at 0.0065 m,
// so 0.008 .. 0.016 both bind and are feasible.
const TH0 = 0.12;
const FLOOR = reachableCross(0, TH0, 10, PLANT);
t.check('oracle: the reachable floor is below the corridors under test',
        FLOOR > 0.006 && FLOOR < 0.007, `${FLOOR.toFixed(5)} m`);
{
  cfg({ bTheta0: TH0, bEy0: 0, bDisturb: 0 });
  const off = JSON.parse(api.bench(10, SOLVER.fastGradient, 0, 1, 300));
  t.check('unconstrained, the plan leaves every corridor under test',
          off.maxPlanCrossM > 0.016, `plan reaches ${off.maxPlanCrossM.toFixed(5)} m`);
}
for (const corr of [0.016, 0.012, 0.010, 0.008]) {
  t.check(`oracle says corridor ${corr} is feasible`, FLOOR <= corr);
  cfg({ bTheta0: TH0, bEy0: 0, bDisturb: 0, corridor: corr });
  const r = JSON.parse(api.bench(10, SOLVER.admm, 0, 1, 300));
  const tr = trace();
  const outside = tr.filter((q) => q.plan > corr + 1e-6).length;
  t.check(`corridor ${corr}, converged: the plan stays inside`,
          r.maxPlanViolationM < 1e-5 && outside === 0 && r.convergedFrac > 0.99 &&
            r.infeasTicks === 0,
          `worst excursion ${r.maxPlanViolationM.toExponential(2)} m, ` +
          `${outside}/${tr.length} ticks outside, ${r.meanIters.toFixed(1)} iters`);
}
{
  // The fixed-budget caveat the header documents, and the bound it offers in
  // exchange. A corridor is a row on a state the controller reaches later, so
  // unlike the box it cannot be clamped on the way out.
  cfg({ bTheta0: TH0, bEy0: 0, bDisturb: 0, corridor: 0.010 });
  const one = JSON.parse(api.bench(10, SOLVER.admm, 1, 1, 300));
  const tr1 = trace();
  t.check('at one iteration the corridor is a penalty, not a constraint',
          one.maxPlanViolationM > 1e-5 && one.maxPlanViolationM < 2e-3,
          `worst excursion ${(one.maxPlanViolationM * 1e3).toFixed(3)} mm`);

  cfg({ bTheta0: TH0, bEy0: 0, bDisturb: 0, corridor: 0.010 });
  const twenty = JSON.parse(api.bench(10, SOLVER.admm, 20, 1, 300));
  t.check('more iterations shrink the excursion',
          twenty.maxPlanViolationM < 0.2 * one.maxPlanViolationM,
          `${one.maxPlanViolationM.toExponential(2)} -> ` +
          `${twenty.maxPlanViolationM.toExponential(2)} m`);

  // primalResidual is claimed to bound how far outside, in the row's own units.
  // Checked per tick, because a max-against-max comparison would pass even if
  // the bound broke on every individual tick.
  let broken = 0, worstGap = -Infinity;
  for (const q of tr1) {
    if (q.primal < 0) continue;
    const gap = (q.plan - 0.010) - q.primal;
    if (gap > 1e-4) broken++;
    if (gap > worstGap) worstGap = gap;
  }
  t.check('primalResidual bounds the excursion on every tick', broken === 0,
          `worst (excursion - residual) ${worstGap.toExponential(2)} m over ${tr1.length} ticks`);
}

// ===========================================================================
t.section('cross-track corridor: infeasible is infeasible, not a bug');
// ===========================================================================
{
  const corr = 0.002;
  t.check(`oracle says corridor ${corr} is infeasible`, FLOOR > corr,
          `floor ${FLOOR.toFixed(5)} > ${corr}`);
  cfg({ bTheta0: TH0, bEy0: 0, bDisturb: 0, corridor: corr });
  const r = JSON.parse(api.bench(10, SOLVER.admm, 0, 1, 300));
  t.check('the solver does not claim to have honoured it',
          r.maxPlanViolationM > 1e-3 && r.infeasTicks > 0,
          `excursion ${r.maxPlanViolationM.toExponential(2)} m, ` +
          `${r.infeasTicks}/300 ticks flagged infeasible`);
  // The interesting part: it still delivers the best input physically available.
  t.check('the closed loop settles at the oracle floor, not worse',
          Math.abs(r.peakCrossM - FLOOR) / FLOOR < 0.02,
          `peak ${r.peakCrossM.toFixed(5)} m vs floor ${FLOOR.toFixed(5)} m`);
}
{
  // The case in the brief, verbatim: at a 0.10 rad/s command-rate limit a
  // 0.35 rad heading error cannot be killed before cross-track leaves a 0.04 m
  // corridor. Derived, then confirmed against the solver.
  const bound = reachableCross(0, 0.35, 10, PLANT, 0.10, 0);
  t.check('oracle: rate 0.10 + theta0 0.35 cannot hold a 0.04 m corridor',
          bound > 0.04, `floor ${bound.toFixed(5)} m > 0.04 m`);
  cfg({ bTheta0: 0.35, bEy0: 0, bDisturb: 0, rateW: 0.10, corridor: 0.04 });
  const r = JSON.parse(api.bench(10, SOLVER.admm, 0, 1, 300));
  const trInf = trace();
  // Most ticks cannot be certified and infeasibility is detected. NOT "none
  // certifies": as heading decays the corridor does come back into reach on a
  // few ticks, and those are legitimately solvable. Tuning rho took that from 0
  // ticks to 6, which is the solver improving, not the claim weakening -- so
  // what is asserted is the MEANING of converged rather than a count of it.
  const certified = trInf.filter((q) => q.converged);
  const certifiedButOutside = certified.filter((q) => q.plan > 0.04 + 1e-5);
  t.check('the solver agrees, and certifies only ticks it really solved',
          r.infeasTicks > 150 && certified.length < 0.1 * trInf.length &&
            certifiedButOutside.length === 0,
          `${r.infeasTicks}/300 flagged, ${certified.length} certified, ` +
          `${certifiedButOutside.length} of those outside the corridor`);
  t.check('the rate limit still holds even so', r.maxCmdRate <= 0.10 + 1e-6,
          `max |d cmd| ${r.maxCmdRate.toFixed(6)}`);
  // An infeasible corridor is not merely unhonoured -- combined with a rate
  // limit it wrecks the loop, because the committed move comes off a diverging
  // iterate. Pinned so that stops being a surprise.
  t.check('an infeasible corridor plus a rate limit destabilises the loop',
          r.peakCrossM > 10 * bound,
          `peak ${r.peakCrossM.toFixed(4)} m against a ${bound.toFixed(4)} m floor`);
}

// ===========================================================================
t.section('solver cost against a 10 ms V5 tick');
// ===========================================================================
// usPerSolve is x86-under-WASM and is only good for catching a regression.
// flopsPerTick is the hardware-free number: a V5 brain is a 667 MHz Cortex-A9
// with scalar VFPv3, so ~1 flop/cycle counting a multiply-add as two flops is a
// generous ceiling, i.e. about 6.7 Mflop in a 10 ms tick.
const A9_FLOPS_PER_TICK = 6.67e6;
const COSTS = [
  ['box FISTA h10 it1', { }, 10, SOLVER.fastGradient, 1],
  ['box GS h20 it50', { }, 20, SOLVER.gaussSeidel, 50],
  ['box FISTA h20 converged', { }, 20, SOLVER.fastGradient, 0],
  ['box ADMM h20 converged', { }, 20, SOLVER.admm, 0],
  ['rate h10 it1', { rateW: 0.05, rateV: 0.4 }, 10, SOLVER.admm, 1],
  ['rate h20 it4', { rateW: 0.05, rateV: 0.4 }, 20, SOLVER.admm, 4],
  ['corridor h10 it1', { corridor: 0.010, bTheta0: TH0, bEy0: 0, bDisturb: 0 },
   10, SOLVER.admm, 1],
  ['corridor h10 converged', { corridor: 0.010, bTheta0: TH0, bEy0: 0, bDisturb: 0 },
   10, SOLVER.admm, 0],
  ['corr+rate h20 it4',
   { corridor: 0.030, rateW: 0.05, rateV: 0.4, bTheta0: TH0, bEy0: 0, bDisturb: 0 },
   20, SOLVER.admm, 4],
  ['corr+rate h20 converged',
   { corridor: 0.030, rateW: 0.05, rateV: 0.4, bTheta0: TH0, bEy0: 0, bDisturb: 0 },
   20, SOLVER.admm, 0],
];
for (const [label, knobs, h, s, it] of COSTS) {
  cfg(knobs);
  const r = JSON.parse(api.bench(h, s, it, 1, 1200));
  const a9ms = (r.flopsPerTick / 6.67e8) * 1000;
  const worstMs = (r.flopsWorstTick / 6.67e8) * 1000;
  // A loop misses its deadline on the WORST tick. Asserting the mean was close
  // to vacuous: the worst value any of these configurations can reach is 2.43e6
  // flop against a 6.67e6 ceiling, so nothing the bench can be asked to do
  // trips it, while the worst TICK of the box-only converged case is 41x its own
  // mean. Both are reported; the budget test is on the worst.
  t.check(label,
          r.flopsWorstTick < A9_FLOPS_PER_TICK && r.usPerSolve < 2000,
          `rows ${r.rows}  ${r.meanIters.toFixed(1)}/${r.maxIters} it  ` +
          `${r.usPerSolve.toFixed(2)}us wasm  mean ~${a9ms.toFixed(2)}ms A9  ` +
          `WORST ~${worstMs.toFixed(2)}ms (${(worstMs / 10 * 100).toFixed(0)}% of tick)  ` +
          `setup ${r.setupUs.toFixed(1)}us  conv ${r.convergedFrac.toFixed(3)}`);
}

// The worst tick on the converged path is the iteration CAP whenever any tick
// fails to certify, because those ticks spend all of it. So maxIters is the
// deadline knob, and the two configurations above reach 27% and 42% of a tick
// only because the cap is 400.
//
// Recorded, not acted on: 400 is the default precisely because a tight rate
// limit needs 344 iterations (see the rate section above), and these two
// configurations tolerating a cap of 100 is what made 100 look safe in general.
// It is not. What this pins is that the cap sets the worst tick and not the
// mean, in the same hardware-free flop units as the table above.
for (const [label, knobs, h] of [
  ['box ADMM h20', {}, 20],
  ['corr+rate h20',
   { corridor: 0.030, rateW: 0.05, rateV: 0.4, bTheta0: TH0, bEy0: 0, bDisturb: 0 },
   20],
]) {
  cfg({ ...knobs, maxIters: 400 });
  const wide = JSON.parse(api.bench(h, SOLVER.admm, 0, 1, 1200));
  cfg({ ...knobs, maxIters: 100 });
  const tight = JSON.parse(api.bench(h, SOLVER.admm, 0, 1, 1200));
  const wideMs = (wide.flopsWorstTick / 6.67e8) * 1000;
  const tightMs = (tight.flopsWorstTick / 6.67e8) * 1000;
  // The cap has to move the WORST tick by most of its ratio, and must not cost
  // certification on a problem that is solvable at all -- the mean iteration
  // count is far below either cap, so capping lower cannot change the answer on
  // the ticks that were converging.
  t.check(`${label}: maxIters is the worst-tick knob, not the mean`,
          tightMs < 0.45 * wideMs && tight.convergedFrac >= wide.convergedFrac - 0.02 &&
            tight.meanIters < 100,
          `cap 400 -> WORST ~${wideMs.toFixed(2)}ms ` +
          `(${(wideMs / 10 * 100).toFixed(0)}%), cap 100 -> ` +
          `~${tightMs.toFixed(2)}ms (${(tightMs / 10 * 100).toFixed(0)}%); ` +
          `mean ${wide.meanIters.toFixed(1)} -> ${tight.meanIters.toFixed(1)} it, ` +
          `conv ${wide.convergedFrac.toFixed(3)} -> ${tight.convergedFrac.toFixed(3)}`);
}

// ===========================================================================
// World runs
// ===========================================================================
const PATHS = {
  // Same shapes as before, moved to the middle of the field. The posts sit on a
  // ring at radius ~52.5, so the centre is the one large region genuinely clear
  // of them; the old routes ran 1.5 in and 4.1 in INSIDE a post's collision
  // envelope, which only passed because prep() switched the obstacles off.
  // Margins here are 18 in and 15 in, including follow()'s lateral start offset.
  'gentle S': (u) => [-26 + 52 * u, 13 * Math.sin(u * Math.PI)],
  hairpin: (u) => [-15 + 30 * Math.sin(u * Math.PI * 0.95), -20 + 40 * u],
};

// The follower indexes its reference by time, so the trajectory has to be
// sampled at dt. Feeding a geometric path straight in races the robot.
function timeTrajectory(pts, dt = 0.01, cruise = 40, accel = 50, latAccel = 60) {
  const n = pts.length, cum = [0];
  for (let i = 1; i < n; i++)
    cum.push(cum[i - 1] + Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]));
  const total = cum[n - 1], curv = new Array(n).fill(0);
  for (let i = 1; i < n - 1; i++) {
    const [ax, ay] = pts[i - 1], [bx, by] = pts[i], [cx, cy] = pts[i + 1];
    const a = Math.hypot(bx - ax, by - ay), b = Math.hypot(cx - bx, cy - by);
    const d = Math.hypot(cx - ax, cy - ay);
    const cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    curv[i] = (a * b * d > 1e-9) ? 2 * cross / (a * b * d) : 0;
  }
  curv[0] = curv[1]; curv[n - 1] = curv[n - 2];
  const v = curv.map((k) =>
    Math.min(cruise, Math.abs(k) > 1e-6 ? Math.sqrt(latAccel / Math.abs(k)) : cruise));
  v[0] = 0; v[n - 1] = 0;
  for (let i = 1; i < n; i++)
    v[i] = Math.min(v[i], Math.sqrt(v[i - 1] ** 2 + 2 * accel * (cum[i] - cum[i - 1])));
  for (let i = n - 2; i >= 0; i--)
    v[i] = Math.min(v[i], Math.sqrt(v[i + 1] ** 2 + 2 * accel * (cum[i + 1] - cum[i])));
  const out = []; let s = 0;
  for (let step = 0; step < 2000; step++) {
    let lo = 0, hi = n - 1;
    if (s >= total) { lo = n - 2; }
    else while (hi - lo > 1) { const m = (lo + hi) >> 1; if (cum[m] <= s) lo = m; else hi = m; }
    const seg = cum[lo + 1] - cum[lo], f = s >= total ? 1 : (seg > 1e-9 ? (s - cum[lo]) / seg : 0);
    const j = Math.min(lo + 1, n - 1), k = Math.max(lo - 1, 0);
    const x = pts[lo][0] + f * (pts[lo + 1][0] - pts[lo][0]);
    const y = pts[lo][1] + f * (pts[lo + 1][1] - pts[lo][1]);
    const th = Math.atan2(pts[j][1] - pts[k][1], pts[j][0] - pts[k][0]);
    const sp = Math.max(v[lo] + f * (v[j] - v[lo]), step === 0 ? 0 : 4);
    out.push([x, y, th, sp, sp * (curv[lo] + f * (curv[j] - curv[lo]))]);
    if (s >= total) break;
    s += sp * dt;
  }
  return out;
}

/// Run a path and report the follower's OWN cross-track, which is the error
/// against the reference sample the controller was tracking at that tick. The
/// nearest-vertex distance is kept only as an independent sanity bound: with 71
/// vertices it floors out around half the vertex spacing, so it cannot tell a
/// good controller from a much better one.
function follow(name, knobs, lateralIn = 0, loc = 5) {
  const fn = PATHS[name], pts = [];
  for (let i = 0; i <= 70; i++) pts.push(fn(i / 70));
  api.trajClear();
  for (const [x, y, th, v, w] of timeTrajectory(pts)) api.trajPush(x, y, th, v, w);
  cfg(knobs);
  const [x0, y0] = pts[0], [x1, y1] = pts[1];
  const hdg = Math.atan2(y1 - y0, x1 - x0);
  api.reset(x0 - lateralIn * Math.sin(hdg), y0 + lateralIn * Math.cos(hdg),
            90 - hdg * 180 / Math.PI);
  api.setLoc(loc);
  // Obstacles stay ON: with them off, the r.collide === 0 conjuncts below assert
  // nothing, since Field::collides returns after the perimeter test.
  api.setParam('collideObstacles', 1);
  for (let i = 0; i < 60; i++) api.step(10);
  const collide0 = api.collideTicks();
  api.run();
  let ms = 0, vertPeak = 0, vertSum = 0, n = 0;
  for (let i = 0; i < 5; i++) { api.step(10); ms += 10; }
  while (api.running() && ms < 9000) {
    api.step(10); ms += 10;
    const s = JSON.parse(api.stateJson());
    let best = 1e9;
    for (const [px, py] of pts) best = Math.min(best, Math.hypot(s.true.x - px, s.true.y - py));
    vertPeak = Math.max(vertPeak, best); vertSum += best; n++;
  }
  for (let i = 0; i < 30; i++) api.step(10);
  const st = JSON.parse(api.statsJson());
  return {
    secs: ms / 1000, vertPeak, vertMean: n ? vertSum / n : 0,
    collide: api.collideTicks() - collide0, st,
    meanIn: st.meanCrossM * M2IN, peakIn: st.peakCrossM * M2IN, endIn: st.endErrorM * M2IN,
  };
}
const shape = (r) =>
  `${r.secs.toFixed(2)}s  cross mean ${r.meanIn.toFixed(2)}in peak ${r.peakIn.toFixed(2)}in ` +
  `end ${r.endIn.toFixed(2)}in  admm ${r.st.admmTicks}/${r.st.ticks} rows ${r.st.constraintRows}`;

// ===========================================================================
t.section('mode numbering: the general path is reached only from an MPC mode');
// ===========================================================================
// The fingerprint no other mode can fake. This is the check that catches a mode
// constant drifting -- without it the whole section below can pass while
// exercising the LQR baselines.
{
  const rti = follow('gentle S', { mode: MODE.mpcRti, horizon: 10, rtiIters: 1,
                                   corridor: 0.10 });
  t.check('MpcRti with a corridor takes the general path every tick',
          rti.st.admmTicks === rti.st.ticks && rti.st.constraintRows === 30 &&
            rti.st.worstPrimalResidual >= 0,
          shape(rti));
  const lqr = follow('gentle S', { mode: MODE.riccatiPerTick, horizon: 10,
                                   corridor: 0.10 });
  t.check('RiccatiPerTick ignores it, as it must',
          lqr.st.admmTicks === 0 && lqr.st.constraintRows === 0, shape(lqr));
}

// ===========================================================================
t.section('follower tracks closed loop');
// ===========================================================================
const BASE = {};
for (const name of ['gentle S', 'hairpin']) {
  for (const [label, knobs] of [
    ['RTI h10', { mode: MODE.mpcRti, horizon: 10, rtiIters: 1 }],
    ['converged h15', { mode: MODE.mpcConverged, horizon: 15, maxIters: 50 }],
  ]) {
    const r = follow(name, knobs);
    if (label === 'RTI h10') BASE[name] = r;
    t.check(`${name} ${label}`,
            r.secs < 8.9 && r.vertMean < 1.5 && r.meanIn < 1.5 && r.collide === 0,
            `${shape(r)}  [vertex ${r.vertMean.toFixed(2)}/${r.vertPeak.toFixed(2)}in]`);
  }
}

// ===========================================================================
t.section('constrained follower: rows survive, and the cost is visible');
// ===========================================================================
for (const name of ['gentle S', 'hairpin']) {
  // A corridor tighter than the error the run already makes acts as a heavy
  // cross-track penalty. Measured on ODOMETRY, not the MCL+EKF pose the rest of
  // this section uses: the corridor is a property of the controller, and a
  // filtered pose puts estimator noise on both sides of the comparison. With the
  // filters on, the effect does not even keep its sign when the path moves to a
  // different part of the field -- so the earlier "peak 1.56 -> 0.84 in" was
  // reading the estimator as much as the constraint.
  const noCorr = follow(name, { mode: MODE.mpcRti, horizon: 10, rtiIters: 1 }, 0, 0);
  const tight = follow(name, { mode: MODE.mpcRti, horizon: 10, rtiIters: 1,
                               corridor: 0.02 }, 0, 0);
  // Both peak AND mean must fall: a single-number threshold on the peak alone
  // was tuned to a fixture that has since changed twice (paths moved off the
  // posts, sim_reset made authoritative), and the effect size moves with it.
  // Requiring both to improve tests the mechanism instead of the magnitude.
  t.check(`${name}: a 0.02 m corridor cuts peak cross-track`,
          tight.peakIn < 0.90 * noCorr.peakIn &&
          tight.meanIn < noCorr.meanIn && tight.collide === 0,
          `peak ${noCorr.peakIn.toFixed(2)} -> ${tight.peakIn.toFixed(2)}in, ` +
          `mean ${noCorr.meanIn.toFixed(2)} -> ${tight.meanIn.toFixed(2)}in`);
  t.check(`${name}: no requested row was lost`,
          tight.st.droppedRowTicks === 0 && tight.st.factorizationFailures === 0 &&
            tight.st.constraintsRejected === false && tight.st.constraintRows === 30,
          `dropped ${tight.st.droppedRowTicks}, factFail ` +
          `${tight.st.factorizationFailures}, rows ${tight.st.constraintRows}`);

  // A corridor far looser than the error still costs, because it moves the
  // solve off the box fast path onto a one-iteration ADMM. The diagnostic that
  // says so is the dual residual: the committed move is not the optimum.
  const loose = follow(name, { mode: MODE.mpcRti, horizon: 10, rtiIters: 1,
                               corridor: 0.20 });
  t.check(`${name}: a geometrically inactive corridor is reported uncertified`,
          loose.st.worstDualResidual > 1e-3 && loose.st.convergedTicks === 0,
          `worst dual ${loose.st.worstDualResidual.toExponential(2)}, ` +
          `mean ${BASE[name].meanIn.toFixed(2)} -> ${loose.meanIn.toFixed(2)}in`);
}
{
  // A rate limit tight enough to stop the robot following the path is a
  // field-loss configuration, and none of the boolean diagnostics fire on it.
  // worstPrimalResidual does. Stated as an implication so a later fix does not
  // leave a test pinning the bad behaviour.
  const r = follow('hairpin', { mode: MODE.mpcRti, horizon: 10, rtiIters: 1,
                                rateW: 0.10 });
  t.check('a rate limit too tight for the path is either harmless or flagged',
          r.peakIn < 3.0 || r.st.worstPrimalResidual > 1.0,
          `peak ${r.peakIn.toFixed(2)}in, end ${r.endIn.toFixed(2)}in, ` +
          `worst primal ${r.st.worstPrimalResidual.toExponential(2)}, ` +
          `infeasTicks ${r.st.infeasibleTicks}`);
  const ok = follow('hairpin', { mode: MODE.mpcRti, horizon: 10, rtiIters: 4,
                                 rateW: 0.30, rateV: 2.0, corridor: 0.10 });
  t.check('a rate limit sized for the path keeps the run',
          ok.meanIn < 1.5 && ok.collide === 0 && ok.st.constraintRows === 50 &&
            ok.st.droppedRowTicks === 0,
          shape(ok));
}

process.exit(t.finish());
