// The constrained MPC, reduced to the one question that decides whether it can
// run on a V5: is the solve converging inside its budget?
//
// Pure: no DOM, no wasm. docs/sim-audit.md's timing result is that every
// FEASIBLE configuration sits under 2.4% of a 10 ms tick and the only expensive
// one is infeasible -- so cost is decided by feasibility, and the panel has to
// lead with feasibility rather than with microseconds.

/// MpcPathFollower::Mode, in enum order.
export const MPC_MODES = ['scheduledLqr', 'riccatiPerTick', 'mpcConverged', 'mpcRti'];
export const modeName = (i) => MPC_MODES[i] ?? `?${i}`;

/**
 * The verdict.
 *
 * `worstPrimalResidual` is the load-bearing number and the reason this is not a
 * boolean read: docs/sim-audit.md records an infeasible corridor combined with a
 * rate limit destabilising the loop to a 0.514 m peak cross-track while NO
 * boolean diagnostic fired. So a residual that never comes down is reported as a
 * problem even when infeasibleTicks is 0.
 *
 * @param s parsed sim_mpc_stats_json()
 * @param opts.residualTol  primal residual treated as converged
 * @param opts.tickBudgetUs one control period, microseconds
 */
export function mpcVerdict(s, opts = {}) {
  const residualTol = opts.residualTol ?? 1e-3;
  const tickBudgetUs = opts.tickBudgetUs ?? 10000;
  if (!s || !s.ticks) {
    return { cls: 'off', headline: 'not run', detail: '', bars: [] };
  }
  const convergedFrac = s.convergedTicks / s.ticks;
  const meanIters = s.meanIters;
  const atCap = s.maxIters > 0 && meanIters >= 0.95 * s.maxIters;
  const residualBad = s.worstPrimalResidual > residualTol;

  let cls = 'ok';
  let headline;
  if (s.factorizationFailures > 0) {
    cls = 'bad';
    headline = `${s.factorizationFailures} factorization failure(s)`;
  } else if (residualBad && atCap) {
    cls = 'bad';
    headline = `infeasible: residual ${s.worstPrimalResidual.toExponential(2)} at the iteration cap`;
  } else if (residualBad) {
    cls = 'warn';
    headline = `residual ${s.worstPrimalResidual.toExponential(2)} above ${residualTol.toExponential(0)}`;
  } else if (convergedFrac < 1 && s.admmTicks > 0) {
    // An RTI budget is SUPPOSED to stop short of convergence; the residual
    // above is what says whether stopping short cost anything.
    cls = 'warn';
    headline = `${s.convergedTicks}/${s.ticks} ticks converged, residual ok`;
  } else {
    headline = `${s.convergedTicks}/${s.ticks} ticks converged`;
  }

  // Worst tick against the control period, which is what a V5 actually has.
  // The wasm clock is not an A9, so this is labelled as a share of the budget
  // measured HERE, not as a prediction for the brain.
  const budgetFrac = s.worstTickUs / tickBudgetUs;
  return {
    cls,
    headline,
    detail: `mean ${meanIters.toFixed(1)}/${s.maxIters} iters · worst tick `
          + `${s.worstTickUs.toFixed(1)} us = ${(100 * budgetFrac).toFixed(1)}% of `
          + `${(tickBudgetUs / 1000).toFixed(0)} ms (wasm, not A9)`,
    convergedFrac,
    budgetFrac,
    residualBad,
    atCap,
  };
}

/**
 * The nine constraint diagnostics, as rows, with only the ones that fired called
 * out. `constraintsRejected` and `droppedRowTicks` mean a constraint the caller
 * ASKED for is not in the problem being solved, which is worse than one that is
 * merely active -- so those are `bad` while `activeBoundTicks` is neutral.
 */
export function constraintRows(s) {
  if (!s || !s.ticks) return [];
  const bad = (n) => (n > 0 ? 'bad' : 'off');
  const warn = (n) => (n > 0 ? 'warn' : 'off');
  return [
    { k: 'rows in the QP', v: String(s.constraintRows), cls: s.constraintRows > 0 ? 'ok' : 'off' },
    { k: 'ADMM ticks', v: `${s.admmTicks}/${s.ticks}`, cls: s.admmTicks > 0 ? 'ok' : 'off' },
    { k: 'rejected at setup', v: s.constraintsRejected ? 'YES' : 'no',
      cls: s.constraintsRejected ? 'bad' : 'off' },
    { k: 'rows dropped', v: String(s.droppedRowTicks), cls: bad(s.droppedRowTicks) },
    { k: 'rows relaxed', v: String(s.relaxedRowTicks), cls: warn(s.relaxedRowTicks) },
    { k: 'reported infeasible', v: String(s.infeasibleTicks), cls: warn(s.infeasibleTicks) },
    { k: 'factorization failed', v: String(s.factorizationFailures),
      cls: bad(s.factorizationFailures) },
    { k: 'input box active', v: `${s.activeBoundTicks}/${s.ticks}`, cls: 'off' },
    { k: 'output saturated', v: `${s.satTicks}/${s.ticks}`, cls: warn(s.satTicks) },
    { k: 'peak wheel demand', v: `${s.peakWheelDemand.toFixed(3)} m/s`, cls: 'off' },
    { k: 'worst primal residual', v: s.worstPrimalResidual.toExponential(2),
      cls: s.worstPrimalResidual > 1e-3 ? 'bad' : 'ok' },
    { k: 'worst dual residual', v: s.worstDualResidual.toExponential(2), cls: 'off' },
  ];
}

/// The row budget, from sim_mpc_rows_json. `accepted` but `planned > installed`
/// is the failure the budget check exists for.
export function rowBudgetRows(r) {
  if (!r) return [];
  return [
    { k: 'horizon', v: `${r.horizon} (max ${r.maxSpecs} specs)`, cls: 'off' },
    { k: 'specs', v: `${r.specsAccepted}/${r.specsAttempted} accepted`,
      cls: r.specsAccepted < r.specsAttempted ? 'warn' : 'ok' },
    { k: 'rows', v: `${r.constraintRows} of ${r.plannedRows} planned, `
                  + `cap ${r.maxQpRows}`,
      cls: r.constraintRows < r.plannedRows ? 'bad' : 'ok' },
    { k: 'dropped', v: String(r.droppedRows), cls: r.droppedRows > 0 ? 'bad' : 'off' },
    { k: 'sizeof', v: `${(r.sizeofMpc / 1024).toFixed(1)} KiB solver, `
                   + `${(r.sizeofFollower / 1024).toFixed(1)} KiB follower`, cls: 'off' },
  ];
}

/// Column layout of sim_mpc_trace_ptr(). Must match kTraceStride in
/// sim/bridge/bridge_mpc.cpp; ui.mjs takes the stride from the build.
export const TRACE_COLUMNS = ['iters', 'primal', 'dual', 'converged', 'activeRows',
                              'droppedRows', 'relaxedRows', 'infeasible', 'ey',
                              'etheta', 'u', 'cmd', 'wRef', 'planCross', 'activeBounds'];

/**
 * Thin a trace column to `buckets` points, keeping the min AND max of each
 * bucket. Plain striding hides exactly the thing the trace exists to show: a
 * residual that spikes for three ticks out of four thousand disappears under a
 * stride of 8, and that spike is the infeasibility.
 *
 * Returns one entry per bucket with the bucket's first index, so a click can be
 * mapped back to a tick.
 */
export function decimateExtremes(read, count, buckets = 240) {
  if (count <= 0 || buckets < 1) return [];
  const per = Math.ceil(count / buckets);
  const out = [];
  for (let i = 0; i < count; i += per) {
    const end = Math.min(count, i + per);
    let lo = Infinity, hi = -Infinity;
    for (let j = i; j < end; j++) {
      const v = read(j);
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    out.push({ i, min: lo, max: hi });
  }
  return out;
}
