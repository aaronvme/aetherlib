// riku-sim — LTV/LQR trajectory blob generation.
//
// LTVPathFollower::prepare_trajectory() parses a text blob rather than taking
// a pose vector:
//
//   P = {(x, y, theta), ...}     positions in METRES, heading in radians
//   V = {(v, omega), ...}        m/s and rad/s
//
// parse_tuples() keys on PARENTHESES for each tuple; the outer braces are
// incidental. Emitting {x, y, z} instead of (x, y, z) parses to an empty
// trajectory and the follower bails with "Empty trajectory".
//
// The follower consumes exactly one sample per 10 ms tick
// (`pros::Task::delay_until(&t, 10)`), so the trajectory is a TIME series, not
// a spatial one: sample count sets the duration. That is also why it cannot
// stall the way RAMSETE does — the reference advances on the clock, not on the
// robot's progress.

const IN_TO_M = 0.0254;

/**
 * Time-parameterise a spatial path into 10 ms samples with a trapezoidal speed
 * profile, then emit it in the follower's blob format.
 *
 * @param pts      [{x, y}] in inches (the spline from route_dsl.buildPath)
 * @param cruiseIn cruise speed, inches/sec
 * @param accelIn  acceleration, inches/sec^2
 */
export function buildLtvTrajectory(pts, cruiseIn = 40, accelIn = 60, dtMs = 10) {
  if (!pts || pts.length < 2) return { text: '', samples: [] };

  // Arc length at each input point.
  const cum = [0];
  for (let i = 1; i < pts.length; i++) {
    cum.push(cum[i - 1] + Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y));
  }
  const total = cum[cum.length - 1];
  if (total < 1e-6) return { text: '', samples: [] };

  // Trapezoid, collapsing to a triangle when there is no room to cruise.
  let vMax = cruiseIn;
  let dAccel = (vMax * vMax) / (2 * accelIn);
  if (2 * dAccel > total) {
    dAccel = total / 2;
    vMax = Math.sqrt(2 * accelIn * dAccel);
  }
  const dCruise = total - 2 * dAccel;
  const tAccel = vMax / accelIn;
  const tCruise = dCruise / vMax;
  const tTotal = 2 * tAccel + tCruise;

  const speedAt = t => {
    if (t < tAccel) return accelIn * t;
    if (t < tAccel + tCruise) return vMax;
    const td = t - tAccel - tCruise;
    return Math.max(0, vMax - accelIn * td);
  };
  const distAt = t => {
    if (t < tAccel) return 0.5 * accelIn * t * t;
    if (t < tAccel + tCruise) return dAccel + vMax * (t - tAccel);
    const td = Math.min(t - tAccel - tCruise, tAccel);
    return dAccel + dCruise + vMax * td - 0.5 * accelIn * td * td;
  };

  // Resample the geometry at the distance the profile reaches each tick.
  const at = d => {
    if (d <= 0) return { ...pts[0] };
    if (d >= total) return { ...pts[pts.length - 1] };
    let i = 1;
    while (i < cum.length && cum[i] < d) i++;
    const f = (d - cum[i - 1]) / Math.max(cum[i] - cum[i - 1], 1e-9);
    return {
      x: pts[i - 1].x + (pts[i].x - pts[i - 1].x) * f,
      y: pts[i - 1].y + (pts[i].y - pts[i - 1].y) * f,
    };
  };

  const dt = dtMs / 1000;
  const samples = [];
  for (let t = 0; t <= tTotal + dt; t += dt) {
    const d = distAt(t);
    const p = at(d);
    const ahead = at(Math.min(d + 0.5, total));
    const behind = at(Math.max(d - 0.5, 0));
    // Heading is the path tangent in standard math radians -- Pose::theta's
    // convention, which is what the follower compares against directly.
    const theta = Math.atan2(ahead.y - behind.y, ahead.x - behind.x);
    samples.push({ x: p.x, y: p.y, theta, v: speedAt(t) });
  }

  // Angular velocity from the heading derivative.
  for (let i = 0; i < samples.length; i++) {
    const a = samples[Math.max(0, i - 1)];
    const b = samples[Math.min(samples.length - 1, i + 1)];
    let dth = b.theta - a.theta;
    while (dth > Math.PI) dth -= 2 * Math.PI;
    while (dth < -Math.PI) dth += 2 * Math.PI;
    const span = (Math.min(samples.length - 1, i + 1) - Math.max(0, i - 1)) * dt;
    samples[i].w = span > 1e-9 ? dth / span : 0;
  }

  const P = samples
    .map(s => `(${(s.x * IN_TO_M).toFixed(5)}, ${(s.y * IN_TO_M).toFixed(5)}, ${s.theta.toFixed(5)})`)
    .join(', ');
  const V = samples
    .map(s => `(${(s.v * IN_TO_M).toFixed(5)}, ${s.w.toFixed(5)})`)
    .join(', ');

  return { text: `P = {${P}}\nV = {${V}}\n`, samples, durationMs: tTotal * 1000 };
}
