// Trajectory generation, as the browser has to present it.
//
// Pure: no DOM, no wasm. The interesting part is that a generation FAILURE is a
// normal, expected answer here -- config.cpp ships `kaStraight = 0`, so ticking
// the wheel-voltage constraint returns UnusableConstraint and an empty
// trajectory. That has to read as "this constraint needs kA measured first",
// not as a crash or an empty canvas.

/// TrajectoryError names, in enum order, matching errorName() in
/// sim/bridge/bridge_traj.cpp. sim/test/ui.mjs parses that switch out of the
/// source and fails if the two drift apart.
export const TRAJ_ERRORS = [
  'none', 'noWaypoints', 'badConfig', 'tooManyWaypoints', 'degenerateWaypoints',
  'malformedSpline', 'pathTooDense', 'nonFiniteWaypoint', 'infeasibleConstraint',
  'stalledProfile', 'didNotConverge', 'unusableConstraint', 'cuspedPath',
];

/// What a human should do about each failure. Only the ones a person can hit
/// from the panel are spelled out; the rest fall back to the enum name, which is
/// better than a wrong guess.
const REMEDY = {
  unusableConstraint:
    'a constraint has no usable limit — config.cpp ships kaStraight = 0, so the '
    + 'wheel-voltage constraint refuses rather than silently protecting nothing. '
    + 'Measure kA, or set kA on this panel.',
  cuspedPath:
    'the auto-chosen tangents double back — the spline has a cusp. Give the '
    + 'waypoints explicit tangents, or move a heading.',
  didNotConverge:
    'the forward/backward profile pass hit its iteration cap. Try a coarser '
    + 'maxDx, or a lower maxAccel.',
  infeasibleConstraint: 'no speed satisfies every constraint at some point on the path.',
  degenerateWaypoints: 'two waypoints are at the same place.',
  noWaypoints: 'add at least two waypoints.',
  pathTooDense: 'the sampling tolerance produced more states than the buffer holds.',
};

/**
 * The headline for a generate call.
 *
 * @param res  parsed sim_traj_result_json()
 * @returns {{ ok, cls, headline, detail }} — `cls` is one of the stylesheet's
 *          existing status classes.
 */
export function generationVerdict(res) {
  if (!res) return { ok: false, cls: 'err', headline: 'no result', detail: '' };
  const name = res.error ?? TRAJ_ERRORS[res.errorCode] ?? 'unknown';
  if (name === 'none') {
    return {
      ok: true,
      cls: 'ok',
      headline: `${res.count} states · ${res.totalTime.toFixed(2)} s`
              + (res.reversed ? ' · reversed' : ''),
      detail: `maxVel ${res.maxVelocity.toFixed(1)} in/s, `
            + `maxAccel ${res.maxAcceleration.toFixed(0)} in/s²`,
    };
  }
  return {
    ok: false,
    cls: 'err',
    headline: `generation refused: ${name}`,
    detail: REMEDY[name] ?? 'see TrajectoryError in trajectory.hpp',
  };
}

/**
 * Thin a polyline for drawing without moving its endpoints. Striding alone drops
 * the last point whenever (n-1) is not a multiple of the stride, which shortens
 * every drawn path by up to one sample -- visible on a coarse trajectory as a
 * curve that stops before its endpoint.
 */
export function decimate(points, maxPoints = 400) {
  const n = points.length;
  if (n <= maxPoints || maxPoints < 2) return points.slice();
  const stride = Math.ceil((n - 1) / (maxPoints - 1));
  const out = [];
  for (let i = 0; i < n - 1; i += stride) out.push(points[i]);
  out.push(points[n - 1]);
  return out;
}

/**
 * Heading ticks along a trajectory, as (x, y) pairs plus a unit direction in
 * FIELD inches. Kept in field units so the caller's own inches-to-pixel mapping
 * is the only place the scale lives.
 *
 * `theta` is standard math radians, which is what the state buffer carries --
 * NOT the VEX bearing the route DSL uses.
 */
export function headingTicks(states, every = 8) {
  const out = [];
  for (let i = 0; i < states.length; i += Math.max(1, every)) {
    const s = states[i];
    out.push({ x: s.x, y: s.y, dx: Math.cos(s.theta), dy: Math.sin(s.theta) });
  }
  return out;
}

/// Where the profile is fastest and where it is tightest, for the panel.
export function profileExtremes(states) {
  let peakV = 0, peakK = 0, peakA = 0, atK = -1;
  for (let i = 0; i < states.length; i++) {
    peakV = Math.max(peakV, Math.abs(states[i].v));
    peakA = Math.max(peakA, Math.abs(states[i].a));
    if (Math.abs(states[i].k) > peakK) { peakK = Math.abs(states[i].k); atK = i; }
  }
  return {
    peakV, peakA, peakK, atK,
    minRadiusIn: peakK > 1e-9 ? 1 / peakK : Infinity,
  };
}

/**
 * Turn the physics audits in traj.js into rows a panel can show, with the
 * verdict attached. `overBudgetTrue > 0` is the one that matters: it means the
 * generator emitted a state the battery cannot deliver.
 */
export function auditRows(audit, ceilings) {
  const rows = [];
  const volts = (v) => `${v.toFixed(2)} V`;
  rows.push({
    k: 'wheel volts (true)',
    v: `${volts(audit.worstTrue)} — ${audit.overBudgetTrue} of ${audit.rows.length} over`,
    cls: audit.overBudgetTrue > 0 ? 'bad' : 'ok',
  });
  rows.push({
    k: 'wheel volts (rigid)',
    v: `${volts(audit.worstRigid)} — ${audit.overBudgetRigid} over`,
    // The rigid model drops the v^2 (b/2) dk/ds term, so it can read either
    // side of the truth. Over budget here alone is not a defect.
    cls: 'off',
  });
  rows.push({ k: 'dropped accel term', v: `${audit.peakOmitted.toFixed(0)} in/s²`, cls: 'off' });
  rows.push({
    k: 'peak wheel speed',
    v: `${audit.worstWheelSpeed.toFixed(1)} in/s`,
    cls: 'off',
  });
  if (ceilings) {
    rows.push({
      k: 'over its own ceiling',
      v: `${ceilings.worstExcess.toFixed(3)} in/s`,
      cls: ceilings.worstExcess > 0.05 ? 'bad' : 'ok',
    });
    rows.push({
      k: 'binding term',
      v: `grip ${ceilings.gripBound} · wheel ${ceilings.wheelBound} · config ${ceilings.configBound}`,
      cls: 'off',
    });
  }
  return rows;
}
