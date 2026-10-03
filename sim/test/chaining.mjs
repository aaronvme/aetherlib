// Chained moveToPoint exits.
//
//   node sim/test/chaining.mjs
//
// A chained move (minSpeed > 0) that passes beside its target never gets
// inside earlyExitRange, so it has to exit on crossing the line through the
// target square to the approach, or it runs to its timeout.
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
  routeRun:     c('sim_route_run', null, []),
  routeRunning: c('sim_route_running', 'number', []),
};

const OP = { MOVE_TO: 1 };

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};
const st = () => JSON.parse(api.stateJson());

// Runs the route and returns how long it took, in ms.
const timeRoute = (limitMs) => {
  api.routeRun();
  let ms = 0;
  while (ms < limitMs && api.routeRunning() === 1) { api.step(10); ms += 10; }
  return ms;
};

api.init();

// Full speed straight ahead, then a second chained leg off to the side with its
// bearing held from the start: the robot is going too fast to make the turn,
// so it runs parallel to the line it was aimed along and passes about 4 in
// beside the target, never within earlyExitRange.
const TIMEOUT = 3000;
const chainedPass = () => {
  api.reset(0, 0, 0);
  api.routeClear();
  api.routePush(OP.MOVE_TO, 0, 30, 1, TIMEOUT);
  api.routeOpt('minSpeed', 127);
  api.routeOpt('earlyExitRange', 2);
  api.routePush(OP.MOVE_TO, 8, 34, 1, TIMEOUT);
  api.routeOpt('minSpeed', 127);
  api.routeOpt('earlyExitRange', 2);
  api.routeOpt('bearingHoldDist', 100);
  let closest = Infinity;
  api.routeRun();
  let ms = 0;
  while (ms < 2 * TIMEOUT + 500 && api.routeRunning() === 1) {
    api.step(10);
    ms += 10;
    const p = st().true;
    closest = Math.min(closest, Math.hypot(p.x - 8, p.y - 34));
  }
  api.stopAuton();
  return { ms, closest };
};

console.log('== chained moveToPoint passing beside its target ==');
const pass = chainedPass();
check('passes beside it, outside earlyExitRange', pass.closest > 3,
      `closest ${pass.closest.toFixed(1)} in`);
check('still exits well before the timeout', pass.ms < TIMEOUT,
      `route took ${pass.ms} ms`);

console.log('\n== non-chained moveToPoint is unchanged ==');
api.reset(0, 0, 0);
api.routeClear();
api.routePush(OP.MOVE_TO, 4, 36, 1, TIMEOUT);
const plainMs = timeRoute(TIMEOUT + 500);
const end = st().true;
api.stopAuton();
check('settles on the target', Math.hypot(end.x - 4, end.y - 36) < 1.5,
      `ended (${end.x.toFixed(1)}, ${end.y.toFixed(1)}) after ${plainMs} ms`);

console.log(failures === 0 ? '\nALL PASSED' : `\n${failures} FAILED`);
process.exit(failures === 0 ? 0 : 1);
