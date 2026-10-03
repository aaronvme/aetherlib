// Competition mode changes and motion-queue cancellation.
//
//   node sim/test/competition.mjs
//
// PROS deletes the autonomous task on a mode change without unwinding it, so an
// async motion it issued keeps running on the chassis motion task. main.cpp
// cancels it on entry to each mode, and the motion itself ends when the
// competition status changes. sim_enter_driver mirrors opcontrol()'s entry and
// then runs the real driverControl(); main.cpp itself is not compiled here.
import createChadSim from '../web/chad_sim.mjs';

const M = await createChadSim();
const c = (name, ret, args) => M.cwrap(name, ret, args);
const api = {
  init:         c('sim_init', null, []),
  reset:        c('sim_reset', null, ['number', 'number', 'number']),
  step:         c('sim_step', null, ['number']),
  stateJson:    c('sim_state_json', 'string', []),
  stopAuton:    c('sim_stop_auton', null, []),
  routeClear:   c('sim_route_clear', null, []),
  routePush:    c('sim_route_push', null, ['number', 'number', 'number', 'number', 'number']),
  routeOpt:     c('sim_route_opt', null, ['string', 'number']),
  routeAsync:   c('sim_route_async', null, []),
  routeRun:     c('sim_route_run', null, []),
  routeRunning: c('sim_route_running', 'number', []),
  setComp:      c('sim_set_competition', null, ['number']),
  setAnalog:    c('sim_set_analog', null, ['number', 'number']),
  enterDriver:  c('sim_enter_driver', null, []),
  abortMotion:  c('sim_abort_motion', null, []),
  stallMotion:  c('sim_stall_motion_task', null, ['number']),
};

const OP = { TURN_TO_HEADING: 2, DRIVE: 4, BOOMERANG: 5, DELAY: 8, TANK: 15 };
const COMP = { DISABLED: 1, AUTONOMOUS: 2, CONNECTED: 4 };
const STICK = { LEFT_Y: 1, RIGHT_X: 2 };

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};
const st = () => JSON.parse(api.stateJson());
const run = ms => { for (let i = 0; i < ms / 10; i++) api.step(10); };
const angErr = (a, b) => ((((a - b) % 360) + 540) % 360) - 180;
const wheels = v => `wheels ${v.left.toFixed(1)} / ${v.right.toFixed(1)} in/s`;

// An async half-turn the "autonomous" routine leaves behind when it ends.
const leaveTurnRunning = (comp = 0) => {
  api.reset(0, 0, 90);
  api.setComp(comp);
  api.routeClear();
  api.routePush(OP.TURN_TO_HEADING, 270, 0, 0, 4000);
  api.routeOpt('targetTolerance', 0);
  api.routeAsync();
  api.routePush(OP.DELAY, 30, 0, 0, 0);
  api.routeRun();
  run(150);
};

// A one-shot open-loop command through Chassis::tank(), which shares the
// driver loop's isSettled() gate.
const tankStraight = () => {
  api.routeClear();
  api.routePush(OP.TANK, 80, 80, 0, 0);
  api.routePush(OP.DELAY, 1000, 0, 0, 0);
  api.routeRun();
};

const driveCovers = (label, inches) => {
  const from = st().true;
  api.routeClear();
  api.routePush(OP.DRIVE, inches, 0, 0, 3000);
  api.routeRun();
  for (let i = 0; i < 400 && api.routeRunning() === 1; i++) api.step(10);
  const to = st().true;
  const d = Math.hypot(to.x - from.x, to.y - from.y);
  check(label, Math.abs(d - inches) < 2, `moved ${d.toFixed(2)} in`);
};

api.init();

console.log('\n== a leftover async motion owns the drive until cancelled ==');
leaveTurnRunning();
check('the routine returned while its turn kept going',
      api.routeRunning() === 0 && !st().settled);
tankStraight();
run(200);
let v = st().vel;
check('uncancelled, a tank command is dropped',
      Math.sign(v.left) !== Math.sign(v.right), wheels(v));
api.stopAuton();
run(50);

console.log('\n== entering driver control hands the driver the drive ==');
// The wheels carry the turn for a few ticks whatever is commanded, so the
// stick is judged against the same entry with the stick centred.
const enterDriverWith = stick => {
  leaveTurnRunning();
  api.setAnalog(STICK.LEFT_Y, stick);
  api.setAnalog(STICK.RIGHT_X, 0);
  api.enterDriver();
  const settled = st().settled;
  run(40);
  const early = st().vel;
  run(160);
  const later = st().vel;
  api.setAnalog(STICK.LEFT_Y, 0);
  api.stopAuton();
  run(50);
  return { settled, early, later };
};
const centred = enterDriverWith(0);
const pushed = enterDriverWith(80);
check('settled as soon as the mode is entered', pushed.settled);
check('stick input moves the drive from the first driver passes',
      pushed.early.right - centred.early.right > 10 &&
      pushed.later.left > 5 && pushed.later.right > 5,
      `at 40 ms ${wheels(pushed.early)} vs centred ${wheels(centred.early)}, ` +
      `then ${wheels(pushed.later)}`);

console.log('\n== a competition status change ends an async motion ==');
leaveTurnRunning(COMP.AUTONOMOUS | COMP.CONNECTED);
check('the turn outlives the routine that issued it', !st().settled);
api.setComp(COMP.CONNECTED);
run(30);
check('settled within a tick of the switch, with nothing cancelling it',
      st().settled);
run(300);
v = st().vel;
check('and the drive has stopped',
      Math.abs(v.left) < 2 && Math.abs(v.right) < 2, wheels(v));
api.setComp(0);
api.stopAuton();
run(50);

console.log('\n== a motion issued after a cancel still runs ==');
leaveTurnRunning();
api.stopAuton();
driveCovers('an 18 in drive after the cancel covers 18 in', 18);
api.stopAuton();
run(50);

console.log('\n== abortMotion while idle does not swallow the next motion ==');
api.reset(0, 0, 90);
api.abortMotion();
run(20);
driveCovers('an 18 in drive after it covers 18 in', 18);
api.stopAuton();
run(50);

console.log('\n== a cancel that times out still leaves the driver in control ==');
leaveTurnRunning();
api.stallMotion(300);
run(20);
api.stopAuton();
check('settled once cancelMotion gives up waiting', st().settled);
tankStraight();
run(600);
v = st().vel;
check('the stalled motion does not stop the drive when it wakes',
      v.left > 5 && v.right > 5, wheels(v));
api.stopAuton();
run(50);

console.log('\n== reverse boomerang backs in ==');
api.reset(0, 0, 90);
api.routeClear();
api.routePush(OP.BOOMERANG, -24, 12, 90, 5000);
api.routeOpt('forwards', 0);
api.routeRun();
let worstTurn = 0;
for (let i = 0; i < 600 && api.routeRunning() === 1; i++) {
  api.step(10);
  worstTurn = Math.max(worstTurn, Math.abs(angErr(st().true.h, 90)));
}
const end = st().true;
check('reaches the target', Math.hypot(end.x + 24, end.y - 12) < 2,
      `ended (${end.x.toFixed(1)}, ${end.y.toFixed(1)})`);
check('and ends on the asked heading', Math.abs(angErr(end.h, 90)) < 5,
      `h=${end.h.toFixed(1)}`);
// Turning round, driving forwards and turning back arrives too; the heading on
// the way is what shows it went back-first.
check('without turning round on the way', worstTurn < 60,
      `worst ${worstTurn.toFixed(1)} deg off the start heading`);
api.stopAuton();

console.log(failures === 0 ? '\nALL PASSED' : `\n${failures} FAILED`);
process.exit(failures === 0 ? 0 : 1);
