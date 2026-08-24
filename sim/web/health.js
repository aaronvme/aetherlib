// Reduces the estimator's diagnostic counters to one line for the HUD.
// Pure, and its own module so it can be tested: everything in app.js touches
// the DOM at import time and cannot be loaded headlessly.

export /// Only what is actually wrong. These are counters, so a row of zeros is noise
/// and the whole value of them is that a non-zero one is worth noticing. A
/// streak threshold of 3 keeps a single rejected reading, which is normal, off
/// the line.
function healthText(s) {
  const d = s.diag;
  if (!d) return '—';
  const bits = [];
  if (d.enc) {
    const total = d.enc.live + d.enc.faulted;
    if (!d.enc.valid) bits.push(`encoders DEAD (${total} faulted)`);
    else if (d.enc.faulted > 0) bits.push(`encoders ${d.enc.live}/${total}`);
  }
  if (d.odom) {
    if (d.odom.outages > 0) bits.push(`${d.odom.outages} outage`);
    if (d.odom.jumps > 0) bits.push(`${d.odom.jumps} jump rejected`);
    if (d.odom.imuFaults > 0) bits.push(`${d.odom.imuFaults} imu fault`);
  }
  if ((s.locMode & 4) && d.ekf) {
    if (d.ekf.streakR >= 3) bits.push(`range rejected x${d.ekf.streakR}`);
    if (d.ekf.streakBlind >= 3) bits.push(`blind x${d.ekf.streakBlind}`);
    if (d.ekf.streakM >= 3) bits.push(`mcl rejected x${d.ekf.streakM}`);
    if (d.ekf.repairs > 0) bits.push(`${d.ekf.repairs} cov repair`);
  }
  return bits.length ? bits.join(' · ') : 'ok';
}
