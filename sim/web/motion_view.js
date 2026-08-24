// Motion outcomes for the browser: how a motion ended, and whether the drive
// was left running afterwards.
//
// Pure, and its own module for the same reason health.js is: everything in
// app.js touches the DOM at import time and cannot be loaded headlessly.
// sim/test/ui.mjs asserts against every branch below.

/// MotionOutcome, in enum order. Must match settle_spec.hpp; the bridge sends
/// the name as well, so ui.mjs checks the two against each other rather than
/// trusting this list.
export const OUTCOMES = ['none', 'arrived', 'chained', 'timeout', 'stalled',
                         'cancelled', 'invalid'];

/// MotionType, in enum order (settle_spec.hpp).
export const MOTION_TYPES = ['idle', 'distance', 'point', 'heading', 'turnPoint',
                             'arc', 'boomerang', 'ramsete', 'trajectory', 'swing'];

export const outcomeName = (i) => OUTCOMES[i] ?? `?${i}`;
export const motionTypeName = (i) => MOTION_TYPES[i] ?? `?${i}`;

/**
 * Which CSS class the existing stylesheet should colour an outcome with.
 *
 * `chained` is deliberately not `ok`: it means the motion exited early with the
 * drive still running, which is correct when it was asked for and a runaway when
 * it was not. Warning is the honest reading of it on its own.
 */
export function outcomeClass(outcome) {
  switch (outcome) {
  case 1: return 'ok';        // arrived
  case 2: return 'warn';      // chained out -- drive left running
  case 3:                     // timed out
  case 4:                     // stalled
  case 6: return 'bad';       // invalid request
  case 5: return 'warn';      // cancelled, i.e. someone asked for it
  default: return 'off';      // none: nothing has run
  }
}

/**
 * Did the drive stop? Every outcome except ChainedOut has to end at zero
 * command, and a nonzero one is either a handover or a motion that walked away
 * with the motors on. Compared against a threshold rather than exact zero
 * because the command is reconstructed from per-motor millivolts.
 */
export function seamOf(record, epsVolts = 0.05) {
  const live = Math.abs(record.cmdOut ?? 0) > epsVolts;
  const chained = record.outcome === 2;
  if (chained && live) return { kind: 'handover', cls: 'ok', text: 'handover' };
  if (chained && !live) return { kind: 'chainDropped', cls: 'bad', text: 'chained but stopped' };
  if (!chained && live) return { kind: 'runaway', cls: 'bad', text: 'drive LEFT ON' };
  return { kind: 'stopped', cls: 'off', text: 'stopped' };
}

/**
 * "Time to settle" is only defined when the settle test actually passed. A
 * boomerang or a path motion can reach `arrived` without ever passing through
 * tolerance, and the bridge reports -1 for that -- which has to render as "not
 * available", not as a number.
 */
export function settleText(toleranceMs, totalMs) {
  if (!(toleranceMs >= 0)) return '—';
  const hold = totalMs - toleranceMs;
  return `${toleranceMs.toFixed(0)} ms` + (hold >= 0 ? ` (+${hold.toFixed(0)} hold)` : '');
}

/**
 * One row of the results table, as plain values. Kept separate from the DOM so
 * the number formatting and the class decisions are testable; app.js only turns
 * this into cells.
 */
export function motionRow(record) {
  const seam = seamOf(record);
  return {
    i: record.i,
    type: motionTypeName(record.type),
    outcome: record.name ?? outcomeName(record.outcome),
    outcomeClass: outcomeClass(record.outcome),
    ms: `${record.ms.toFixed(0)} ms`,
    settle: settleText(record.toleranceMs, record.ms),
    ticks: record.ticks,
    peak: `${record.peakLat.toFixed(0)}/${record.peakAng.toFixed(0)}`,
    vSeam: `${record.vIn.toFixed(1)} → ${record.vOut.toFixed(1)} in/s`,
    cmdSeam: `${record.cmdIn.toFixed(1)} → ${record.cmdOut.toFixed(1)} V`,
    seam: seam.text,
    seamClass: seam.cls,
    // Where the library thinks it is against where it actually is. An auton that
    // called setPose has its own frame, so this is only meaningful for a motion
    // program driven from the reset pose.
    driftIn: Math.hypot(record.x - record.trueX, record.y - record.trueY),
  };
}

/**
 * The live one-liner for the HUD, from sim_state_json's `motion` block. Says
 * what is driving before it says how the last thing ended, because "who owns
 * the motors right now" is the question a stuck run raises.
 */
export function motionSummary(s) {
  const m = s.motion;
  if (!m) return '—';
  if (m.trajRunning) return `trajectory · ${m.ticks} ticks`;
  if (m.mpcRunning) return `MPC · ${m.ticks} ticks`;
  if (s.routeRunning) {
    const settle = m.toleranceAtMs >= 0
      ? ` · in tol ${(s.t - m.toleranceAtMs / 1000).toFixed(2)} s`
      : '';
    return `${motionTypeName(m.type)} running · ${m.ticks} ticks${settle}`;
  }
  const seam = Math.abs(m.cmdL) > 0.05 || Math.abs(m.cmdR) > 0.05;
  return `${m.name}` + (m.arrived ? '' : ' (not arrived)')
       + (seam ? ' · drive still on' : '');
}
