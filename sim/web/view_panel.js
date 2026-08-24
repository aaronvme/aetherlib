// The transport bar and the View tab's decisions, extracted so they can be
// tested: everything left in app.js touches the DOM at import time and cannot be
// loaded headlessly. Same split as motion_view.js / traj_view.js / mpc_view.js.

/// Localisation is four independent switches over the bitmask the bridge takes:
/// 1 MCL, 2 RCL, 4 EKF. The label has to say who owns the pose, because with the
/// EKF up the other two only feed it.
export function locSummary(on) {
  const mode = (on.mcl ? 1 : 0) | (on.rcl ? 2 : 0) | (on.ekf ? 4 : 0);
  const names = [];
  if (mode & 1) names.push('MCL');
  if (mode & 2) names.push('RCL');
  if (mode & 4) names.push('EKF');
  return {
    mode,
    label: names.length ? names.join(' + ') : 'dead reckoning',
    title: (mode & 4)
      ? 'EKF owns the pose; MCL and RCL feed it'
      : 'nothing ticked = IMU + motor encoders only',
  };
}

/// Which recorded row a scrub position names. `index` is null for both "not
/// scrubbing" and "nothing recorded", which are different states on screen:
/// a slider that is live and one that has nothing to show.
export function scrubState(enabled, slider, count) {
  if (!enabled) return { index: null, text: 'live', disabled: true };
  if (!(count > 0)) return { index: null, text: 'no data', disabled: false };
  const frac = Number.isFinite(slider) ? Math.max(0, Math.min(1000, slider)) : 0;
  return { index: Math.round((frac / 1000) * (count - 1)), text: null, disabled: false };
}

/// Fraction of the recording a click at canvas x lands on. The pads are the plot
/// axis gutters, so a click on the axis label seeks to the end rather than off
/// the front of the buffer.
export function seekFraction(x, width, leftPad, rightPad) {
  const span = width - leftPad - rightPad;
  if (!(span > 0)) return 0;
  return Math.max(0, Math.min(1, (x - leftPad) / span));
}

/// Text for the geometry editor. The `kind` tag is part of the round trip and
/// not decoration: collision contains the robot inside the kind-0 box, so
/// dropping it turned Apply-with-no-edits into a field with no walls.
export function fieldGeomText(geom) {
  return JSON.stringify({
    segments: (geom.segments ?? []).map((s) => s.slice(0, 5)),
    circles: (geom.circles ?? []).map((c) => c.slice(0, 3)),
  }, null, 1);
}

/// Parse the editor back. Returns `{ error }` for anything unusable, so the
/// panel reports it instead of clearing the field and adding nothing.
export function parseFieldGeom(text) {
  let g;
  try {
    g = JSON.parse(text);
  } catch (e) {
    return { error: e.message };
  }
  if (g === null || typeof g !== 'object' || Array.isArray(g)) {
    return { error: 'expected an object with segments and circles' };
  }
  const segs = g.segments ?? [], circs = g.circles ?? [];
  if (!Array.isArray(segs) || !Array.isArray(circs)) {
    return { error: 'segments and circles must be arrays' };
  }
  const num = (v) => Number.isFinite(v);
  for (const s of segs) {
    if (!Array.isArray(s) || s.length < 4 || !s.slice(0, 4).every(num)) {
      return { error: `segment ${JSON.stringify(s)} needs four numbers` };
    }
  }
  for (const c of circs) {
    if (!Array.isArray(c) || c.length < 3 || !c.slice(0, 3).every(num)) {
      return { error: `circle ${JSON.stringify(c)} needs three numbers` };
    }
  }
  return {
    // Default 2 = user structure. A hand-typed segment is NOT perimeter: it
    // stops the robot where it is drawn rather than silently resizing the field.
    segments: segs.map((s) => [s[0], s[1], s[2], s[3], num(s[4]) ? s[4] : 2]),
    circles: circs.map((c) => [c[0], c[1], c[2]]),
  };
}
