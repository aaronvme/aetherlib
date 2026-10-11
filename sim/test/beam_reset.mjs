// autons::resetFromBeamsNear(): a beam reset may only move the pose as far as
// the evidence supports.
//
//   node sim/test/beam_reset.mjs
//
// The mapped solve ranks poses by a centre-ray model; the sensor reports the
// nearest thing in a ~25 degree cone. Near a post the two disagree and a
// lattice alias 20-100 in away can explain more beams than the truth, so a
// jump past the gate needs an opposing pair that brackets the field span.
import createChadSim from '../web/chad_sim.mjs';

const M = await createChadSim();
const c = (n, r, a) => M.cwrap(n, r, a);
const api = {
  init: c('sim_init', null, []),
  reset: c('sim_reset', null, ['number', 'number', 'number']),
  step: c('sim_step', null, ['number']),
  setLoc: c('sim_set_localization', null, ['number']),
  teleport: c('sim_teleport', null, ['number', 'number', 'number']),
  stateJson: c('sim_state_json', 'string', []),
  beamResetNear: c('sim_beam_reset_near', 'number', ['number']),
};
const st = () => JSON.parse(api.stateJson());
const run = (ms) => { for (let i = 0; i < ms; i += 5) api.step(5); };

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};

const ODOM = 0, RCL = 2, RCL_EKF = 2 | 4;
const modeName = { [ODOM]: 'odom', [RCL]: 'RCL', [RCL_EKF]: 'RCL+EKF' };

// Robot placed at (x, y, h); ground truth then moved by (ox, oy) without the
// estimator being told, and the reset taken standing still.
function attempt(x, y, h, loc, ox = 0, oy = 0) {
  api.init();
  api.reset(x, y, h);
  api.setLoc(loc);
  run(150);
  if (ox || oy) { api.teleport(x + ox, y + oy, h); run(100); }
  const before = st();
  const ok = api.beamResetNear(4);
  run(10);
  const after = st();
  return {
    ok,
    moved: Math.hypot(after.odom.x - before.odom.x, after.odom.y - before.odom.y),
    errBefore: before.err.dist,
    errAfter: after.err.dist,
  };
}
const fmt = (r) => `ok=${r.ok} moved ${r.moved.toFixed(1)} in, error ` +
  `${r.errBefore.toFixed(1)} -> ${r.errAfter.toFixed(1)} in`;

// Red 3 Pin's second reset, taken at heading 66 after placing a pin: it moved
// the pose 56.7 in (RCL) and 79.6 in (RCL+EKF).
console.log('\n== off a field axis ==');
for (const [x, y, h] of [[-37.5, -19.3, 66], [-38, -18.5, 61]]) {
  for (const loc of [RCL, RCL_EKF]) {
    const r = attempt(x, y, h, loc);
    check(`heading ${h}, ${modeName[loc]}: refused, pose stays`,
          r.ok === 0 && r.moved < 0.5, fmt(r));
  }
}
{
  // 8 degrees off is still square enough to use.
  const r = attempt(-6, 0, 98, ODOM, 3, 0);
  check('heading 98 (8 off) still corrects a 3 in offset',
        r.ok === 1 && r.errAfter < 1.5, fmt(r));
}

console.log('\n== square on, an alias must not win ==');
{
  // Red 3 Pin's preload reset spot, 3 in out: the mapped solve picked
  // (-48.1, 30.8).
  const r = attempt(-52, -16.5, 270, ODOM, 3, 0);
  check('preload spot, 3 in out: never lands further off than it started',
        r.errAfter <= r.errBefore + 0.5, fmt(r));
}
{
  const r = attempt(-48, -54, 0, ODOM);
  check('(-48,-54) on the spot: stays put', r.errAfter < 1.0, fmt(r));
}
{
  const r = attempt(-58, 0, 270, ODOM);
  check('(-58,0) against the wall: stays put', r.errAfter < 1.0, fmt(r));
}

console.log('\n== square on, a real offset is corrected ==');
for (const [x, y, h, ox, oy, loc, why] of [
  [-12, -6, 90, 6, 0, ODOM, 'bracketed, 6 in, past the gate'],
  [-6, 0, 0, -5, 3, ODOM, 'bracketed, 5.8 in, past the gate'],
  [-12, -6, 90, 4, 3, RCL_EKF, 'bracketed, 5 in, shipping mode'],
  [-36, -10, 90, 3, 0, ODOM, 'mapped, 3 in, inside the gate'],
  [-54, -18, 0, 3, 0, ODOM, 'mapped near a goal, 3 in, inside the gate'],
]) {
  const r = attempt(x, y, h, loc, ox, oy);
  check(`(${x},${y}) h${h} ${modeName[loc]}: ${why}`,
        r.ok === 1 && r.errAfter < 1.0, fmt(r));
}

console.log(failures === 0 ? '\nALL CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
