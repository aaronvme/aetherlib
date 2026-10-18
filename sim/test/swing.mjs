// swingToHeading drives only the side it names and holds the other where it
// stands, whatever brake mode the routine left the drive in.
//
//   node sim/test/swing.mjs
//
// coastDragFrac makes a 0 V side in coast roll with the driven one, as an
// unpowered side does on a real drivetrain; without it every 0 V side stops
// and a coasting "held" side would look held.
import createChadSim from '../web/chad_sim.mjs';

const M = await createChadSim();
const c = (name, ret, args) => M.cwrap(name, ret, args);
const api = {
  init:         c('sim_init', null, []),
  reset:        c('sim_reset', null, ['number', 'number', 'number']),
  step:         c('sim_step', null, ['number']),
  stateJson:    c('sim_state_json', 'string', []),
  setParam:     c('sim_set_param', null, ['string', 'number']),
  getParam:     c('sim_get_param', 'number', ['string']),
  stopAuton:    c('sim_stop_auton', null, []),
  routeClear:   c('sim_route_clear', null, []),
  routePush:    c('sim_route_push', null, ['number', 'number', 'number', 'number', 'number']),
  routeAsync:   c('sim_route_async', null, []),
  routeRun:     c('sim_route_run', null, []),
  routeRunning: c('sim_route_running', 'number', []),
  reportJson:   c('sim_route_report_json', 'string', []),
  stallMotion:  c('sim_stall_motion_task', null, ['number']),
  sideDeg:      c('sim_drive_side_deg', 'number', ['number']),
  brakeMode:    c('sim_drive_brake_mode', 'number', ['number']),
  setBrake:     c('sim_set_drive_brake_mode', null, ['number']),
};

const OP = { SWING: 7, DELAY: 8, TANK: 15 };
const SIDE = { LEFT: 0, RIGHT: 1 };
const MODE = { COAST: 0, BRAKE: 1, HOLD: 2 };
const modeName = m => ['coast', 'brake', 'hold'][m] ?? `mixed(${m})`;
const sideName = s => (s === SIDE.LEFT ? 'left' : 'right');
const START = [-36, 0, 90];   // open field, clear of every post for a 90 deg swing

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};
const st = () => JSON.parse(api.stateJson());
const run = ms => { for (let i = 0; i < ms / 10; i++) api.step(10); };
const angErr = (a, b) => ((((a - b) % 360) + 540) % 360) - 180;

api.init();
api.setParam('coastDragFrac', 0.5);

const track = api.getParam('trackWidthIn');
const wheelDiam = api.getParam('wheelDiameterIn');
// Blue cartridges under a 450 rpm wheel.
const ratio = api.getParam('maxWheelRpm') / 600;
const wheelDeg = motorDeg => motorDeg * ratio;
const wheelIn = motorDeg => wheelDeg(motorDeg) / 360 * Math.PI * wheelDiam;

/// Where one side's wheels touch the floor, from a VEX-frame pose.
const contact = (pose, side) => {
  const th = (90 - pose.h) * Math.PI / 180;
  const s = side === SIDE.LEFT ? 1 : -1;
  return { x: pose.x - s * Math.sin(th) * track / 2, y: pose.y + s * Math.cos(th) * track / 2 };
};

const swingCase = (side, heading, mode) => {
  const other = 1 - side;
  console.log(`\n== ${sideName(side)} swing to ${heading}, drive in ${modeName(mode)} ==`);
  api.reset(...START);
  api.setBrake(mode);
  run(50);
  const deg0 = [api.sideDeg(0), api.sideDeg(1)];
  const pose0 = st().true;
  const pivot0 = contact(pose0, other);

  api.routeClear();
  api.routePush(OP.SWING, heading, side, 0, 2500);
  api.routeRun();
  let mid = null;
  for (let i = 0; i < 400 && api.routeRunning() === 1; i++) {
    api.step(10);
    if (i === 20) mid = [api.brakeMode(0), api.brakeMode(1)];
  }
  const pose1 = st().true;
  const moved = [api.sideDeg(0) - deg0[0], api.sideDeg(1) - deg0[1]];
  const rep = JSON.parse(api.reportJson())[0];

  // A 90 deg pivot about the held side sweeps the driven side a quarter circle
  // of radius track width, forward for a turn away from the driven side.
  const turnDeg = angErr(heading, pose0.h);
  const towardOther = side === SIDE.LEFT ? turnDeg > 0 : turnDeg < 0;
  const want = (towardOther ? 1 : -1) * Math.abs(turnDeg) * Math.PI / 180 * track;
  const driven = wheelIn(moved[side]);
  check(`${sideName(side)} side drives`, Math.abs(driven - want) < 0.25 * Math.abs(want),
        `${driven.toFixed(1)} in, want ${want.toFixed(1)} in`);
  check(`${sideName(other)} side holds`, Math.abs(wheelDeg(moved[other])) < 1.5,
        `${wheelDeg(moved[other]).toFixed(2)} wheel deg`);
  check('reaches the heading', Math.abs(angErr(pose1.h, heading)) < 1,
        `h=${pose1.h.toFixed(2)}`);
  const pivot1 = contact(pose1, other);
  const drift = Math.hypot(pivot1.x - pivot0.x, pivot1.y - pivot0.y);
  check(`pivots about the ${sideName(other)} wheels`, drift < 0.3, `moved ${drift.toFixed(2)} in`);
  check('in open field', rep && !rep.hit && !rep.cutOff, JSON.stringify(rep));
  check(`${sideName(other)} side in hold during the swing, ${sideName(side)} untouched`,
        mid && mid[other] === MODE.HOLD && mid[side] === mode,
        mid ? `left ${modeName(mid[0])}, right ${modeName(mid[1])}` : 'never sampled');
  const after = [api.brakeMode(0), api.brakeMode(1)];
  check(`both sides back in ${modeName(mode)} after`, after[0] === mode && after[1] === mode,
        `left ${modeName(after[0])}, right ${modeName(after[1])}`);
};

for (const mode of [MODE.BRAKE, MODE.COAST]) {
  swingCase(SIDE.LEFT, 180, mode);    // left forward
  swingCase(SIDE.RIGHT, 0, mode);     // right forward
  swingCase(SIDE.LEFT, 0, mode);      // left backward
  swingCase(SIDE.RIGHT, 180, mode);   // right backward
}

// An async swing the routine leaves behind, cancelled part way.
const leaveSwingRunning = () => {
  api.reset(...START);
  api.setBrake(MODE.COAST);
  api.routeClear();
  api.routePush(OP.SWING, 180, SIDE.LEFT, 0, 4000);
  api.routeAsync();
  api.routePush(OP.DELAY, 30, 0, 0, 0);
  api.routeRun();
  run(200);
};
const tankStraight = () => {
  api.routeClear();
  api.routePush(OP.TANK, 80, 80, 0, 0);
  api.routePush(OP.DELAY, 1000, 0, 0, 0);
  api.routeRun();
};
const wheels = v => `wheels ${v.left.toFixed(1)} / ${v.right.toFixed(1)} in/s`;

console.log('\n== a cancel mid-swing hands the drive back ==');
leaveSwingRunning();
check('right side in hold mid-swing', api.brakeMode(SIDE.RIGHT) === MODE.HOLD,
      modeName(api.brakeMode(SIDE.RIGHT)));
api.stopAuton();
check('settled once cancelled', st().settled);
check('both sides back in coast',
      api.brakeMode(0) === MODE.COAST && api.brakeMode(1) === MODE.COAST,
      `left ${modeName(api.brakeMode(0))}, right ${modeName(api.brakeMode(1))}`);
tankStraight();
run(600);
let v = st().vel;
check('a tank command after it drives both sides', v.left > 5 && v.right > 5, wheels(v));
api.stopAuton();
run(50);

console.log('\n== a cancel that times out mid-swing still hands the drive back ==');
leaveSwingRunning();
api.stallMotion(300);
run(20);
api.stopAuton();
check('settled once cancelMotion gives up waiting', st().settled);
tankStraight();
run(600);
v = st().vel;
check('the stalled swing does not stop the drive when it wakes',
      v.left > 5 && v.right > 5, wheels(v));
check('and leaves both sides in coast',
      api.brakeMode(0) === MODE.COAST && api.brakeMode(1) === MODE.COAST,
      `left ${modeName(api.brakeMode(0))}, right ${modeName(api.brakeMode(1))}`);
api.stopAuton();

console.log(failures === 0 ? '\nswing: all passed' : `\nswing: ${failures} failed`);
process.exitCode = failures === 0 ? 0 : 1;
