// Brake modes reach every motor in a group. PROS 4.2.2's
// MotorGroup::set_brake_mode(mode) sets only index 0, so a whole-group call
// has to be set_brake_mode_all -- otherwise "HOLD" on a 3-motor lift holds one.
//
//   node sim/test/brake.mjs
import createChadSim from '../web/chad_sim.mjs';

const M = await createChadSim();
const c = (name, ret, args) => M.cwrap(name, ret, args);
const api = {
  init:          c('sim_init', null, []),
  reset:         c('sim_reset', null, ['number', 'number', 'number']),
  step:          c('sim_step', null, ['number']),
  enterDriver:   c('sim_enter_driver', null, []),
  setDriveBrake: c('sim_set_drive_brake_mode', null, ['number']),
  driveBrake:    c('sim_drive_brake_mode', 'number', ['number']),
  liftBrake:     c('sim_lift_brake_mode', 'number', []),
};

const COAST = 0, BRAKE = 1, HOLD = 2;
const NAMES = ['coast', 'brake', 'hold'];

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};

api.init();
api.reset(0, 0, 0);
api.step(50);

console.log('drive');
for (const mode of [HOLD, COAST, BRAKE]) {
  api.setDriveBrake(mode);
  const l = api.driveBrake(0), r = api.driveBrake(1);
  check(`chassis.setBrakeMode(${NAMES[mode]}) sets every drive motor`,
        l === mode && r === mode, `left ${l}  right ${r}  (-1 = motors disagree)`);
}

console.log('lift');
api.enterDriver();
api.step(100);
const lift = api.liftBrake();
check('driver control holds every lift motor', lift === HOLD, `lift ${lift}  (-1 = motors disagree)`);

console.log(failures === 0 ? 'brake: all passed' : `brake: ${failures} failed`);
process.exitCode = failures === 0 ? 0 : 1;
