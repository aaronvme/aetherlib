// Side toggles: a flip is one half turn forward on that side only, R2 held in
// driver spins both at half speed and release holds, a later flip does not
// catch up the hand-spun travel, Up homes both forward to the next half-turn
// index, Side::Both drives both, and maxSpeed caps a flip.
// Leaving driver mid-hold stops them.
//
//   node sim/test/sidetoggle.mjs
import createChadSim from '../web/chad_sim.mjs';

const M = await createChadSim();
const c = (name, ret, args) => M.cwrap(name, ret, args);
const api = {
  init:        c('sim_init', null, []),
  reset:       c('sim_reset', null, ['number', 'number', 'number']),
  step:        c('sim_step', null, ['number']),
  stopAuton:   c('sim_stop_auton', null, []),
  enterDriver: c('sim_enter_driver', null, []),
  setDigital:  c('sim_set_digital', null, ['number', 'number']),
  flipRaw:     c('sim_side_toggle_flip', null, ['number', 'number', 'number']),
  home:        c('sim_side_toggle_home', null, ['number']),
  busy:        c('sim_side_toggle_busy', 'number', ['number']),
  pos:         c('sim_side_toggle_position', 'number', ['number']),
};

api.flip = (side, flips, maxSpeed = 0) => api.flipRaw(side, flips, maxSpeed);

const LEFT = 0, RIGHT = 1, BOTH = 2;
// 5.5 W motor free speed, 200 rpm, in deg per 10 ms. Speed checks use this, not
// a flip's peak, so they do not move when the gains are retuned.
const FREE = 12;
const halfSpeed = peak => peak > 0.4 * FREE && peak < 0.6 * FREE;
const R2 = 9, UP = 10;
const TOL = 6;

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};
const both = () => `L ${api.pos(LEFT).toFixed(1)}  R ${api.pos(RIGHT).toFixed(1)}`;

// Steps ms, failing if either shaft ever moves backwards. peakStep is the
// largest 10 ms travel since it was last cleared.
let reversed = 0;
let peakStep = 0;
const run = ms => {
  for (let i = 0; i < ms / 10; i++) {
    const before = [api.pos(LEFT), api.pos(RIGHT)];
    api.step(10);
    const d = [api.pos(LEFT) - before[0], api.pos(RIGHT) - before[1]];
    if (d[0] < -0.5 || d[1] < -0.5) reversed++;
    peakStep = Math.max(peakStep, d[0], d[1]);
  }
};
const press = button => { api.setDigital(button, 1); run(30); api.setDigital(button, 0); };
// Signed distance from the nearest half-turn index.
const offHome = deg => deg - 180 * Math.round(deg / 180);
const atHome = deg => offHome(deg) > -3 && offHome(deg) < 11;

api.init();
api.reset(0, 0, 0);
run(100);
check('starts at rest', Math.abs(api.pos(LEFT)) < 0.5 && Math.abs(api.pos(RIGHT)) < 0.5, both());

console.log('flip');
peakStep = 0;
api.flip(LEFT, 1);
check('busy right after the request', api.busy(LEFT) === 1);
run(800);
check('left half turn forward', Math.abs(api.pos(LEFT) - 180) < TOL, both());
check('right untouched', Math.abs(api.pos(RIGHT)) < 0.5, both());
check('left settled', api.busy(LEFT) === 0);
const autonPeak = peakStep;

api.flip(RIGHT, 1);
run(50);
api.flip(RIGHT, 1);
run(1200);
check('two quick right flips add up', Math.abs(api.pos(RIGHT) - 360) < TOL, both());
check('left held', Math.abs(api.pos(LEFT) - 180) < TOL, both());

console.log('R2 in driver');
api.enterDriver();
run(50);
const start = [api.pos(LEFT), api.pos(RIGHT)];
api.setDigital(R2, 1);
peakStep = 0;
run(300);
const held = [api.pos(LEFT) - start[0], api.pos(RIGHT) - start[1]];
check('R2 spins both forward', held[0] > 90 && held[1] > 90,
      `+${held[0].toFixed(0)} / +${held[1].toFixed(0)} deg`);
check('R2 runs at about half speed', halfSpeed(peakStep),
      `${peakStep.toFixed(1)} vs free ${FREE} deg per 10 ms`);
api.setDigital(R2, 0);
run(60);
const released = [api.pos(LEFT), api.pos(RIGHT)];
run(500);
check('release holds where it stopped',
      Math.abs(api.pos(LEFT) - released[0]) < 1 && Math.abs(api.pos(RIGHT) - released[1]) < 1,
      both());
check('not busy after release', api.busy(LEFT) === 0 && api.busy(RIGHT) === 0);

// The next flip goes to the next index ahead, never back over the spun travel.
api.flip(LEFT, 1);
run(800);
const l = api.pos(LEFT);
const offIndex = Math.abs(l - 180 * Math.round(l / 180));
check('flip after manual lands on an index', offIndex < TOL, both());
check('flip after manual does not catch up', l > released[0] && l - released[0] <= 180 + TOL,
      `${released[0].toFixed(1)} -> ${l.toFixed(1)}`);

console.log('Up homes both in driver');
// Leave both between indexes so Up has somewhere to go.
api.setDigital(R2, 1);
run(100);
api.setDigital(R2, 0);
run(60);
const beforeUp = [api.pos(LEFT), api.pos(RIGHT)];
check('left left between indexes', !atHome(beforeUp[0]), both());
peakStep = 0;
press(UP);
run(1500);
check('both land on a half-turn index', atHome(api.pos(LEFT)) && atHome(api.pos(RIGHT)), both());
// A side already on an index (toleranceDeg 2 short to indexSnapDeg 10 past)
// stays; one between goes forward to the next.
const onIndex = deg => offHome(deg) >= -2 && offHome(deg) <= 10;
const upOk = side => {
  const before = beforeUp[side === LEFT ? 0 : 1];
  const moved = api.pos(side) - before;
  return onIndex(before) ? Math.abs(moved) < 0.5 : moved > 0 && moved < 180 + TOL;
};
check('Up takes the nearest index ahead, under a half turn', upOk(LEFT) && upOk(RIGHT), both());
check('driver home is half speed', peakStep < 0.6 * FREE,
      `${peakStep.toFixed(1)} vs free ${FREE} deg per 10 ms`);
check('not busy once home', api.busy(LEFT) === 0 && api.busy(RIGHT) === 0);
const homed = [api.pos(LEFT), api.pos(RIGHT)];
press(UP);
run(500);
check('pressing Up again does nothing',
      Math.abs(api.pos(LEFT) - homed[0]) < 0.5 && Math.abs(api.pos(RIGHT) - homed[1]) < 0.5,
      both());

api.setDigital(R2, 1);
run(100);
api.setDigital(R2, 0);
run(60);
press(UP);
run(30);
check('home under way', api.busy(LEFT) === 1);
api.setDigital(R2, 1);
run(100);
api.setDigital(R2, 0);
run(100);
const afterR2 = api.pos(LEFT);
run(500);
check('R2 cancels a home in progress',
      api.busy(LEFT) === 0 && Math.abs(api.pos(LEFT) - afterR2) < 0.5 && !atHome(afterR2),
      both());
press(UP);
run(1500);
check('home after manual works from where it was left', atHome(api.pos(LEFT)), both());

console.log('leaving driver mid-hold');
api.setDigital(R2, 1);
run(200);
api.stopAuton();   // kills the driver task, as a mode change does
run(60);
const stopped = [api.pos(LEFT), api.pos(RIGHT)];
run(400);
check('stops when driver goes away with R2 still held',
      Math.abs(api.pos(LEFT) - stopped[0]) < 1 && Math.abs(api.pos(RIGHT) - stopped[1]) < 1,
      both());
api.setDigital(R2, 0);

console.log('reset rebases');
api.flip(LEFT, 3);
run(100);
api.reset(0, 0, 0);
run(500);
check('reset drops queued flips and rebases at zero',
      Math.abs(api.pos(LEFT)) < 0.5 && api.busy(LEFT) === 0, both());
api.flip(LEFT, 1);
run(800);
check('flip after reset is one half turn', Math.abs(api.pos(LEFT) - 180) < TOL, both());

console.log('autonomous home');
api.home(RIGHT);
run(300);
check('auton home from 0 does nothing', Math.abs(api.pos(RIGHT)) < 0.5, both());
api.home(LEFT);
run(500);
check('auton home on the 180 index does nothing', Math.abs(api.pos(LEFT) - 180) < TOL, both());
api.enterDriver();
run(50);
api.setDigital(R2, 1);
run(100);
api.setDigital(R2, 0);
run(60);
api.stopAuton();
run(60);
const between = api.pos(LEFT);
check('left left between indexes for auton home', !atHome(between), both());
api.home(LEFT);
run(1000);
const homedL = api.pos(LEFT);
check('auton home goes forward to the next half turn',
      atHome(homedL) && homedL > between && homedL - between < 180 + TOL,
      `${between.toFixed(1)} -> ${homedL.toFixed(1)}`);

console.log('Side::Both and maxSpeed');
const halfTurn = (from, to) => atHome(to) && to - from > 160 && to - from < 180 + TOL;
api.home(BOTH);    // start both on an index
run(1500);
check('home Both puts both on an index', atHome(api.pos(LEFT)) && atHome(api.pos(RIGHT)), both());
let b0 = [api.pos(LEFT), api.pos(RIGHT)];
peakStep = 0;
api.flip(BOTH, 1, 0);
check('Both is busy right after the request', api.busy(BOTH) === 1);
run(1000);
check('Both flips each side one half turn',
      halfTurn(b0[0], api.pos(LEFT)) && halfTurn(b0[1], api.pos(RIGHT)), both());
check('Both settles', api.busy(BOTH) === 0);
check('an uncapped flip runs at full speed', peakStep > 0.8 * autonPeak,
      `${peakStep.toFixed(1)} vs ${autonPeak.toFixed(1)} deg per 10 ms`);
b0 = [api.pos(LEFT), api.pos(RIGHT)];
peakStep = 0;
api.flip(BOTH, 1, 63);
run(1500);
check('a capped flip still lands a half turn on',
      halfTurn(b0[0], api.pos(LEFT)) && halfTurn(b0[1], api.pos(RIGHT)), both());
check('maxSpeed 63 caps the flip at about half speed', halfSpeed(peakStep),
      `${peakStep.toFixed(1)} vs free ${FREE} deg per 10 ms`);

check('never spun backwards', reversed === 0, `${reversed} reversed steps`);

console.log(failures === 0 ? 'sidetoggle: all passed' : `sidetoggle: ${failures} failed`);
// exitCode rather than process.exit(): exiting with stdout still flushing trips
// a libuv assertion on Windows and turns a pass into a crash.
process.exitCode = failures === 0 ? 0 : 1;
