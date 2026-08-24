// chad-sim — constrained trajectory request building and physics audits.
//
// Pure module: no DOM, no globals, nothing imported. The UI and
// sim/test/traj.mjs both build a generate request the same way, and both audit
// the result the same way.
//
// Every audit here is deliberately written from physics rather than from the
// library's own classes. `chad::WheelVoltageConstraint` decides a trajectory is
// feasible; auditWheelVoltages() independently reconstructs what each state
// demands of each wheel and compares it to the battery. If the two ever
// disagree, one of them is wrong -- which is the whole point. Re-calling the
// constraint would only prove the formula was typed twice.

export const IN_TO_M = 0.0254;

/// Column layout of the float buffers behind sim_traj_states_ptr() and
/// sim_traj_samples_ptr(). Must match kStateStride in sim/bridge/bridge_traj.cpp;
/// readStates() takes the stride from the build so a change shows up as a length
/// mismatch rather than as silently shifted columns.
export const STATE_COLUMNS = ['t', 'v', 'a', 'x', 'y', 'theta', 'k'];

// ---------------------------------------------------------------------------
// Request building
// ---------------------------------------------------------------------------

/**
 * A generate request: waypoints plus the flat key/value config
 * sim_traj_set() consumes. Separated from applying it so a UI can show the
 * request, diff two of them, or serialise one without a wasm module loaded.
 *
 * @param waypoints [{x, y, headingDeg, tangent}] inches / VEX degrees. `tangent`
 *                  is optional; <= 0 lets the spline choose.
 * @param opts      any of the sim_traj_set keys, plus `follow: {}` for the
 *                  sim_traj_follow_set keys.
 */
export function buildTrajectoryRequest(waypoints, opts = {}) {
  const { follow = {}, ...config } = opts;
  const wps = (waypoints ?? []).map((w) => ({
    x: +w.x,
    y: +w.y,
    headingDeg: +(w.headingDeg ?? w.h ?? 0),
    tangent: +(w.tangent ?? 0),
  }));

  const flat = {};
  for (const [k, v] of Object.entries(config)) {
    flat[k] = typeof v === 'boolean' ? (v ? 1 : 0) : +v;
  }
  const flatFollow = {};
  for (const [k, v] of Object.entries(follow)) {
    flatFollow[k] = typeof v === 'boolean' ? (v ? 1 : 0) : +v;
  }
  return { waypoints: wps, config: flat, follow: flatFollow };
}

/**
 * Push a request through the bridge and generate.
 *
 * @param api  { clear, waypoint, defaults, set, followSet, generate } cwraps
 * @returns the TrajectoryError int (0 == None)
 */
export function applyTrajectoryRequest(api, req) {
  api.defaults();
  api.clear();
  for (const w of req.waypoints) api.waypoint(w.x, w.y, w.headingDeg, w.tangent);
  for (const [k, v] of Object.entries(req.config)) api.set(k, v);
  for (const [k, v] of Object.entries(req.follow)) api.followSet(k, v);
  return api.generate();
}

/// Straight-line waypoints from `a` to `b`, both facing along the line.
export function straightWaypoints(ax, ay, bx, by) {
  const h = 90 - (Math.atan2(by - ay, bx - ax) * 180) / Math.PI;
  return [{ x: ax, y: ay, headingDeg: h }, { x: bx, y: by, headingDeg: h }];
}

// ---------------------------------------------------------------------------
// Reading the buffers
// ---------------------------------------------------------------------------

/**
 * Copy a state buffer out of the wasm heap into plain objects.
 *
 * @param M      the emscripten module (for HEAPF32)
 * @param ptr    byte pointer from sim_traj_states_ptr()
 * @param count  state count
 * @param stride sim_traj_stride()
 */
export function readStates(M, ptr, count, stride) {
  if (stride !== STATE_COLUMNS.length) {
    throw new Error(
      `bridge stride ${stride} != ${STATE_COLUMNS.length} columns; ` +
        'sim/web/traj.js and sim/bridge/bridge_traj.cpp have drifted apart');
  }
  const base = ptr >> 2;
  const out = [];
  for (let i = 0; i < count; i++) {
    const row = {};
    for (let c = 0; c < stride; c++) row[STATE_COLUMNS[c]] = M.HEAPF32[base + i * stride + c];
    out.push(row);
  }
  return out;
}

// ---------------------------------------------------------------------------
// Geometry, from scratch
// ---------------------------------------------------------------------------

const wrapPi = (a) => {
  let x = a;
  while (x > Math.PI) x -= 2 * Math.PI;
  while (x < -Math.PI) x += 2 * Math.PI;
  return x;
};

/**
 * Arc length of the constant-curvature arc joining two poses.
 *
 * chord = 2 R sin(dtheta/2) and arc = R dtheta, so arc = chord * (h / sin h)
 * with h = dtheta/2. Written this way on purpose: chad's arcLengthBetween()
 * gets the same number out of Pose::log's rotate-and-scale, so agreement
 * between the two is evidence and not tautology.
 */
export function arcLengthBetween(a, b) {
  const chord = Math.hypot(b.x - a.x, b.y - a.y);
  const h = 0.5 * wrapPi(b.theta - a.theta);
  if (Math.abs(h) < 1e-7) return chord;
  return (chord * h) / Math.sin(h);
}

export function pathArcLength(states) {
  let s = 0;
  for (let i = 1; i < states.length; i++) s += arcLengthBetween(states[i - 1], states[i]);
  return s;
}

/// Shortest distance from a point to the polyline through the states, so a
/// coarsely sampled path is not credited with error it does not have.
export function crossTrack(states, px, py) {
  let best = Infinity;
  for (let i = 1; i < states.length; i++) {
    const ax = states[i - 1].x, ay = states[i - 1].y;
    const bx = states[i].x, by = states[i].y;
    const dx = bx - ax, dy = by - ay;
    const len2 = dx * dx + dy * dy;
    let t = len2 > 1e-12 ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0;
    t = Math.max(0, Math.min(1, t));
    best = Math.min(best, Math.hypot(px - (ax + t * dx), py - (ay + t * dy)));
  }
  if (states.length === 1) best = Math.hypot(px - states[0].x, py - states[0].y);
  return best;
}

// ---------------------------------------------------------------------------
// The audits
// ---------------------------------------------------------------------------

const signOf = (v) => (v > 0 ? 1 : v < 0 ? -1 : 0);

/**
 * Reconstruct the per-wheel voltage every state demands and compare it to the
 * battery. THE check this surface exists for.
 *
 * Two accelerations are reported per wheel, because they are not the same
 * quantity and only one of them is what the drivetrain has to deliver:
 *
 *   rigid  a_wheel = a * c,  c = 1 +/- k*b/2, the constant-curvature ratio.
 *          This is the assumption WheelVoltageConstraint makes.
 *   true   a_wheel = d/dt (v * c), differenced over the real time step. On a
 *          path whose curvature changes it carries an extra v^2 (b/2) dk/ds
 *          that the rigid form drops.
 *
 * The differenced acceleration is the interval AVERAGE, so it has to be paired
 * with the worse of the interval's two wheel speeds, not with the leading one.
 * kV*vw grows across an accelerating interval, so pairing only the leading speed
 * understates the demand and makes the verdict a function of the resample step:
 * at kA = 2.0 the leading pairing reported 11.984 V and 0 states over budget
 * where the trailing pairing reported 12.106 V and 97 over. Taking the max of
 * the two endpoints is what makes "never over 12 V" a statement about the
 * trajectory rather than about the grid it was sampled on.
 *
 * @param model { trackWidth (in), kS (V), kV (V per m/s), kA (V per m/s^2),
 *                maxVoltage (V) }
 */
export function auditWheelVoltages(states, model) {
  const half = 0.5 * model.trackWidth;
  const kV = model.kV * IN_TO_M;      // V per in/s
  const kA = model.kA * IN_TO_M;      // V per in/s^2
  const kS = model.kS;
  const volts = (v, a) => kS * signOf(v) + kV * v + kA * a;

  const out = {
    worstRigid: 0, worstTrue: 0, worstWheelSpeed: 0,
    worstRigidAt: -1, worstTrueAt: -1,
    peakOmitted: 0,                 // largest |true - rigid| wheel accel, in/s^2
    overBudgetRigid: 0, overBudgetTrue: 0,
    maxVoltage: model.maxVoltage,
    rows: [],
  };

  for (let i = 0; i < states.length; i++) {
    const s = states[i];
    const next = states[i + 1];
    const dt = next ? next.t - s.t : 0;

    for (let w = 0; w < 2; w++) {
      const sgn = w === 0 ? -1 : 1;
      const vw = s.v * (1 + sgn * s.k * half);
      const vwNext = next ? next.v * (1 + sgn * next.k * half) : vw;
      const aRigid = s.a * (1 + sgn * s.k * half);
      const aTrue = dt > 1e-9 ? (vwNext - vw) / dt : aRigid;

      const vRigid = Math.abs(volts(vw, aRigid));
      const vTrue = Math.max(Math.abs(volts(vw, aTrue)), Math.abs(volts(vwNext, aTrue)));
      out.worstWheelSpeed = Math.max(out.worstWheelSpeed, Math.abs(vw));
      if (vRigid > out.worstRigid) { out.worstRigid = vRigid; out.worstRigidAt = i; }
      if (vTrue > out.worstTrue) { out.worstTrue = vTrue; out.worstTrueAt = i; }
      out.peakOmitted = Math.max(out.peakOmitted, Math.abs(aTrue - aRigid));
      if (vRigid > model.maxVoltage + 1e-3) out.overBudgetRigid++;
      if (vTrue > model.maxVoltage + 1e-3) out.overBudgetTrue++;
      out.rows.push({ i, wheel: w, vw, aRigid, aTrue, vRigid, vTrue });
    }
  }
  return out;
}

/**
 * omega = v * curvature is the identity followTrajectory's feedforward is built
 * on (`wRef = vRef * ref.curvature` in chassis.cpp). Differencing the emitted
 * headings against the emitted (velocity, curvature) pairs tests the spline's
 * curvature against its own poses, including the sign convention the reversed
 * path depends on.
 *
 * Returns the worst absolute and relative disagreement in rad/s.
 */
export function auditCurvature(states) {
  let worstAbs = 0, worstRel = 0, worstAt = -1, peakOmega = 0;
  for (let i = 1; i < states.length; i++) {
    const dt = states[i].t - states[i - 1].t;
    if (!(dt > 1e-6)) continue;
    const measured = wrapPi(states[i].theta - states[i - 1].theta) / dt;
    // Midpoint of the segment: v and k are both state quantities and the
    // heading difference is a segment quantity.
    const vMid = 0.5 * (states[i].v + states[i - 1].v);
    const kMid = 0.5 * (states[i].k + states[i - 1].k);
    const predicted = vMid * kMid;
    const abs = Math.abs(measured - predicted);
    peakOmega = Math.max(peakOmega, Math.abs(measured));
    if (abs > worstAbs) { worstAbs = abs; worstAt = i; }
    const rel = abs / Math.max(Math.abs(measured), Math.abs(predicted), 1e-3);
    worstRel = Math.max(worstRel, rel);
  }
  return { worstAbs, worstRel, worstAt, peakOmega };
}

/**
 * Does the time axis match the geometry? Integrating the emitted speed profile
 * over the emitted timestamps has to reproduce the arc length of the emitted
 * poses. Velocity is linear in t on a constant-acceleration segment, so the
 * trapezoid rule is exact and any residual is a real disagreement between the
 * profile and the path -- which is the class of bug that made moveRAMSETE hold
 * cruise speed to the endpoint.
 */
export function auditProfileDistance(states) {
  let integrated = 0;
  let geometric = 0;
  let worstSegment = 0;
  for (let i = 1; i < states.length; i++) {
    const dt = states[i].t - states[i - 1].t;
    const ds = 0.5 * (Math.abs(states[i].v) + Math.abs(states[i - 1].v)) * dt;
    const arc = arcLengthBetween(states[i - 1], states[i]);
    integrated += ds;
    geometric += arc;
    worstSegment = Math.max(worstSegment, Math.abs(ds - arc));
  }
  return {
    integrated,
    geometric,
    worstSegment,
    relError: geometric > 1e-6 ? Math.abs(integrated - geometric) / geometric : 0,
  };
}

/// Are the emitted (v, a, t) triples self-consistent? a[i] describes the
/// segment LEAVING state i, so (v[i+1] - v[i]) / dt must equal it.
export function auditAcceleration(states) {
  let worst = 0, worstAt = -1;
  for (let i = 0; i + 1 < states.length; i++) {
    const dt = states[i + 1].t - states[i].t;
    if (!(dt > 1e-6)) continue;
    const measured = (states[i + 1].v - states[i].v) / dt;
    const d = Math.abs(measured - states[i].a);
    if (d > worst) { worst = d; worstAt = i; }
  }
  return { worst, worstAt };
}

/**
 * Independent closed forms for the two speed ceilings a differential drive
 * faces in a corner. Both are textbook, neither reads the library.
 *
 *   grip        v <= sqrt(aLat / |k|)
 *   wheel speed the outer wheel runs at v(1 + |k| b/2), so v <= vmax / (1 + |k| b/2)
 */
export function speedCeilings(k, model) {
  const mag = Math.abs(k);
  const grip = mag < 1e-9 ? Infinity : Math.sqrt(model.gripLimit / mag);
  const wheel = model.maxWheelSpeed / (1 + 0.5 * mag * model.trackWidth);
  return { grip, wheel, binding: grip < wheel ? 'grip' : 'wheel', limit: Math.min(grip, wheel) };
}

/// Every state against both ceilings, plus which term actually bound.
export function auditSpeedCeilings(states, model, configMaxVel) {
  let worstExcess = -Infinity, worstAt = -1;
  let gripBound = 0, wheelBound = 0, configBound = 0;
  let peakSpeed = 0;
  for (let i = 0; i < states.length; i++) {
    const c = speedCeilings(states[i].k, model);
    const ceiling = Math.min(c.limit, configMaxVel);
    const v = Math.abs(states[i].v);
    peakSpeed = Math.max(peakSpeed, v);
    const excess = v - ceiling;
    if (excess > worstExcess) { worstExcess = excess; worstAt = i; }
    // "Bound" means the profile is sitting on that ceiling rather than being
    // held down by the acceleration ramp somewhere else.
    if (v > 0.98 * ceiling) {
      if (ceiling === configMaxVel) configBound++;
      else if (c.binding === 'grip') gripBound++;
      else wheelBound++;
    }
  }
  return { worstExcess, worstAt, gripBound, wheelBound, configBound, peakSpeed };
}

// ---------------------------------------------------------------------------
// Handing the same geometry to the other followers
// ---------------------------------------------------------------------------

/// Poses for sim_path_push (RAMSETE and the speed-scheduled LQR), which take a
/// VEX bearing in degrees.
export function posesForPathBuffer(states) {
  return states.map((s) => ({
    x: s.x,
    y: s.y,
    headingDeg: 90 - (s.theta * 180) / Math.PI,
  }));
}

/**
 * The LTV follower's trajectory blob, built from THIS trajectory's own speeds,
 * so the comparison is between two controllers and not between two profiles.
 *
 * LTVPathFollower::parse_tuples() keys on parentheses, and the follower consumes
 * exactly one sample per 10 ms tick, so the sample count sets the duration.
 * Positions in metres, velocities in m/s and rad/s.
 */
export function ltvBlob(sampled) {
  if (!sampled || sampled.length < 2) return '';
  const p = [], v = [];
  for (const s of sampled) {
    p.push(`(${(s.x * IN_TO_M).toFixed(6)}, ${(s.y * IN_TO_M).toFixed(6)}, ${s.theta.toFixed(6)})`);
    v.push(`(${(s.v * IN_TO_M).toFixed(6)}, ${(s.v * s.k).toFixed(6)})`);
  }
  return `P = {${p.join(', ')}}\nV = {${v.join(', ')}}\n`;
}
