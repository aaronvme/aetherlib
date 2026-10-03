// Localization under disturbances that make odometry lie.
//
//   node sim/test/disturbance.mjs
//
// Every case here is a real match event that collapses the particle weights.
// The filter has to come back from all of them without relocating onto a
// mirror of itself: the field is a symmetric square with nine posts, so a
// beam pattern near one wall matches several other places, and a global
// search will happily pick the wrong one.
import createChadSim from '../web/chad_sim.mjs';

const M = await createChadSim();
const c = (n, r, a) => M.cwrap(n, r, a);
const api = {
  init: c('sim_init', null, []),
  reset: c('sim_reset', null, ['number', 'number', 'number']),
  step: c('sim_step', null, ['number']),
  setDrive: c('sim_set_drive', null, ['number', 'number']),
  setLoc: c('sim_set_localization', null, ['number']),
  teleport: c('sim_teleport', null, ['number', 'number', 'number']),
  stateJson: c('sim_state_json', 'string', []),
};
const st = () => JSON.parse(api.stateJson());
const run = (ms) => { for (let i = 0; i < ms; i += 5) api.step(5); };
const mclErr = () => {
  const s = st();
  return Math.hypot(s.mcl.x - s.true.x, s.mcl.y - s.true.y);
};

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};

const begin = (x, y, h) => {
  api.init();
  api.reset(x, y, h);
  api.setLoc(1 | 4);           // MCL + EKF, as config.cpp ships
  run(600);
};
const report = (label, bound) => {
  const s = st();
  const mcl = mclErr();
  check(label, mcl < bound && s.err.dist < bound,
        `mcl ${mcl.toFixed(1)} in, pose ${s.err.dist.toFixed(1)} in, bound ${bound}`);
};

// red_3_pin opens by ramming the wall to square up, three times. The body
// stops; the wheels do not, so odometry gains ~90 in that never happened.
console.log('\n== squaring on a wall ==');
begin(-60, 0, 270);
api.setDrive(127, 127); run(1500);
api.setDrive(-100, -100); run(400);
api.setDrive(127, 127); run(1500);
api.setDrive(0, 0); run(1500);
report('two wall rams leave the estimate where the robot is', 4.0);

// Same thing against a post, where there is no wall to bound the travel.
console.log('\n== pushing into a post ==');
begin(-30, 0, 90);
api.setDrive(110, 110); run(2500);
// Loose on purpose. 2.5 s of stall piles up ~100 in of odometry that never
// happened, and the pose settles wherever the beams pulling back balance it
// piling on. What matters is that the beams are still winning: unbounded,
// this rides the phantom travel out the far side of the field. Where it
// settles depends on the particle draw: over seeds 1-20 it spans 3-26 in
// (median 10), so 15 only ever passed on a lucky draw.
report('the estimate holds while the post holds the robot', 30.0);
api.setDrive(-90, -90); run(600);
api.setDrive(80, -80); run(500);
api.setDrive(90, 90); run(1200);
api.setDrive(0, 0); run(1200);
report('and comes back once it drives on', 5.0);

// Defense: the robot is displaced without its wheels turning. This is the
// case injection exists for, so it has to survive bounding the search.
console.log('\n== shoved sideways ==');
begin(-48, -20, 90);
{
  const s = st();
  api.teleport(s.true.x, s.true.y + 18, 90);
}
api.setDrive(80, 80); run(2500);
api.setDrive(0, 0); run(1500);
report('an 18 in shove is found again', 5.0);

console.log(failures === 0 ? '\nALL CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
