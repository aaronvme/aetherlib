// RCL, the filter that owns the pose.
//
//   node sim/test/rcl.mjs
//
// Every case here is something the sampling rules are supposed to do, checked
// against ground truth rather than against the filter's own opinion of itself.
// The ones that matter most are the two rejections: a beam ranging an alliance
// partner must not move the pose, and a beam ranging a post must not either.
// Both were measured relocating it by 14 to 16 in before the rules below
// existed, and neither announced itself -- the filter reported healthy.
import createChadSim from '../web/chad_sim.mjs';

const M = await createChadSim();
const c = (n, r, a) => M.cwrap(n, r, a);
const api = {
  init: c('sim_init', null, []),
  reset: c('sim_reset', null, ['number', 'number', 'number']),
  step: c('sim_step', null, ['number']),
  setDrive: c('sim_set_drive', null, ['number', 'number']),
  setLoc: c('sim_set_localization', null, ['number']),
  setPods: c('sim_set_odom_pods', null, ['number']),
  setParam: c('sim_set_param', null, ['string', 'number']),
  getParam: c('sim_get_param', 'number', ['string']),
  stateJson: c('sim_state_json', 'string', []),
  actorAdd: c('sim_actor_add', 'number', ['number', 'number', 'number', 'number']),
  actorClear: c('sim_actor_clear', null, []),
  actorPathAdd: c('sim_actor_path_push', null, ['number', 'number', 'number']),
  actorFlags: c('sim_actor_set_flags', null,
                ['number', 'number', 'number', 'number', 'number']),
  actorSpeed: c('sim_actor_set_speed', null, ['number', 'number']),
  actorsJson: c('sim_actors_json', 'string', []),
  setFar: c('sim_set_far_side_gate', null, ['number', 'number']),
  pieceClear: c('sim_piece_clear', null, []),
  pieceDefault: c('sim_piece_default', null, []),
  setPeriod: c('sim_set_period_mode', null, ['number']),
  periodLeft: c('sim_period_remaining_ms', 'number', []),
  autonCount: c('sim_auton_count', 'number', []),
  runAuton: c('sim_run_auton', null, ['number']),
  stopAuton: c('sim_stop_auton', null, []),
  routeRunning: c('sim_route_running', 'number', []),
  autonName: c('sim_auton_name', 'string', ['number']),
  distFault: c('sim_distance_fault', null, ['number', 'number']),
  routeClear: c('sim_route_clear', null, []),
  routePush: c('sim_route_push', null, ['number', 'number', 'number', 'number', 'number']),
  routeRun: c('sim_route_run', null, []),
};

// Route opcodes, from sim/bridge/bridge.cpp.
const OP_SET_POSE = 0, OP_DELAY = 8;

const st = () => JSON.parse(api.stateJson());
const run = (ms) => { for (let i = 0; i < ms; i += 5) api.step(5); };
const sensor = (s, i) => s.filters.find(f => f.i === i);

// Mirrors chad::RclTracking::ReadingClass. The bridge sends the raw enum, so
// this list failing to match it is itself the bug.
const CLS = {
  invalid: 0, wall: 1, stale: 2, conf: 3, axis: 4, turn: 5,
  post: 6, far: 7, short: 8, long: 9, silent: 10,
};

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};

/// RCL only, so nothing else can be credited with holding the pose.
///
/// The game pieces come off too. They are real obstacles and a cup sitting in
/// a beam's path is a fair thing to test, but not here: these cases are about
/// the sampling rules, and a check that fails because a cup moved is a check
/// that has stopped measuring what it names. sim/test/_pieceprobe covers the
/// pieces themselves.
const begin = (x, y, h, pods = 0) => {
  for (let i = 0; i < 8; i++) api.distFault(i, 0);
  api.actorClear();
  api.setFar(0, 0);
  api.setPods(pods);
  api.init();
  api.reset(x, y, h);
  // After the reset, which restores whatever layout was loaded.
  api.pieceClear();
  api.setLoc(2);
  run(600);
};

api.init();

// ---------------------------------------------------------------------------
console.log('\n== it holds the pose on its own ==');
begin(-48, -60, 90);
api.setDrive(70, 70); run(1500);
api.setDrive(0, 0); run(800);
check('RCL alone tracks a straight drive', st().err.dist < 1.5,
      `${st().err.dist.toFixed(2)} in`);

// ---------------------------------------------------------------------------
console.log('\n== one sample, one vote ==');
// A V5 distance sensor refreshes near 30 ms while this loop runs at 10, so at
// most about a third of cycles can carry a reading that has not been counted
// already. More than that and the held value is being folded in again, which
// triples the correction rate and makes the noise look three times more
// consistent than it is.
begin(-48, -48, 90);
let synced = 0, cycles = 0;
api.setDrive(50, 50);
for (let i = 0; i < 400; i++) {
  api.step(5);
  const r = st().rcl2;
  cycles++;
  if (r.sync > 0) synced++;
}
api.setDrive(0, 0); run(400);
const frac = synced / cycles;
check('held readings are not counted twice', frac > 0.10 && frac < 0.60,
      `${(frac * 100).toFixed(0)}% of cycles applied a correction`);

// ---------------------------------------------------------------------------
console.log('\n== the turn gate ==');
begin(0, 0, 0);
api.setDrive(110, -110); run(400);
{
  const s = st();
  check('a fast turn suppresses the cycle', s.rcl2.held && s.rcl2.votes === 0,
        `${s.rcl2.turn.toFixed(2)} rad/s, ${s.rcl2.votes} votes`);
  check('and says so per sensor', sensor(s, 0).cls === CLS.turn,
        `sensor 0 class ${sensor(s, 0).cls}`);
}
api.setDrive(0, 0); run(600);

// ---------------------------------------------------------------------------
console.log('\n== the auton read gate ==');
// At (30, -50) facing +x the front beam reaches the east wall at about 40 in,
// and that wall is across the diagonal -- the half where the other two robots
// are moving about.
begin(30, -50, 90);
check('the far beam is used with the gate off',
      sensor(st(), 0).cls === CLS.wall,
      `sensor 0 class ${sensor(st(), 0).cls}`);
api.setFar(1, 0); run(400);
{
  const s = st();
  check('and dropped with it on', sensor(s, 0).cls === CLS.far,
        `sensor 0 class ${sensor(s, 0).cls}`);
  check('the gate reports itself armed', s.rcl2.gate === true);
  // The class is latched and survives a cycle with no new sample; the vote is
  // not, by design -- one physical reading gets one. So "does it still count"
  // has to be asked across a device period rather than at one instant.
  let voted = false;
  for (let i = 0; i < 20 && !voted; i++) {
    api.step(5);
    voted = sensor(st(), 3).rclValid;
  }
  check('a beam on our own half still counts',
        sensor(st(), 3).cls === CLS.wall && voted,
        `sensor 3 class ${sensor(st(), 3).cls}, voted ${voted}`);
}
api.setFar(0, 0); run(300);
check('and it disarms', st().rcl2.gate === false);

// ---------------------------------------------------------------------------
console.log('\n== the gate, with a robot working the far half ==');
// The case the gate exists for. At (30,-50) facing +x the front beam reaches
// the east wall across the Autonomous Line, and an opponent moving over there
// keeps walking into it.
//
// The gate is geometric on WHERE THE RETURN LANDED, not on which way the beam
// was pointing. That distinction is the whole behaviour: a robot straddling
// the line and ranged on its near face gives a return on our own half, and the
// gate passes it to the ordinary innovation test like any other short reading.
// So the invariant worth asserting is the narrow one -- nothing that landed
// over there ever votes -- and then whether the pose survived regardless.
const gateRun = (gateOn, withOpp = true) => {
  begin(30, -50, 90);
  if (withOpp) {
    const opp = api.actorAdd(58, -50, 180, 1);
    api.actorPathAdd(opp, 58, -57);
    api.actorPathAdd(opp, 58, -41);
    api.actorFlags(opp, 1, 1, 0, 1);    // on, patrolling, open path, facing
    api.actorSpeed(opp, 30);
  }
  api.setFar(gateOn ? 1 : 0, 0);
  // Settle before counting. Arming is not instant: update() returns early when
  // no sensor has a new sample, so for up to one device period the per-sensor
  // panel still shows the verdict from before the gate went on. Nothing is
  // applied in those cycles -- there is no sync at all -- but they would read
  // as leaks, and the steady state is what the question is about.
  run(300);

  let worst = 0, voted = 0, cycles = 0, leaked = 0, farVoted = 0, oppVoted = 0;
  for (let i = 0; i < 1200; i++) {
    api.step(5);
    const s = st();
    if (s.err.dist > worst) worst = s.err.dist;
    const f = sensor(s, 0);
    const b = s.beams.find(v => v.i === 0);
    if (!f || !b) continue;
    cycles++;
    // Only a cycle that carried a NEW sample counts; a held frame is the same
    // reading again, not a second vote for it.
    if (!f.fresh) continue;
    if (f.rclValid) voted++;
    // Where the return actually landed, in field inches.
    const rad = b.dir * Math.PI / 180;
    const ex = b.ox + Math.cos(rad) * b.r, ey = b.oy + Math.sin(rad) * b.r;
    const over = ex + ey > 0;                 // beyond the NW-SE diagonal
    if (over && f.rclValid) leaked++;
    if (over) farVoted++;
    // Short of the east wall: the return came off the opponent.
    if (f.rclValid && ex < 69.0) oppVoted++;
  }
  api.actorClear();
  api.setFar(0, 0);
  return { worst, frac: voted / cycles, leaked, farFrac: farVoted / cycles, oppVoted };
};

// The opponent first. Reading trust drops a beam that keeps reading short of
// its wall, so with the gate off the opponent's own returns already never
// vote, and the wall glimpses between its passes do not earn the beam back
// (hold + rebuild outlasts them). The gate must not make that worse.
const gateOff = gateRun(false);
const gateOn = gateRun(true);
check('with the gate off, no return off the opponent is voted (reading trust)',
      gateOff.oppVoted === 0,
      `${gateOff.oppVoted} opponent returns voted, beam voted ${(gateOff.frac * 100).toFixed(0)}%`);
check('with it on, not one return from over the line is', gateOn.leaked === 0,
      `${gateOn.leaked} leaked of ${(gateOn.farFrac * 100).toFixed(0)}% cycles landing over there`);
check('and the gate never adds votes', gateOn.frac <= gateOff.frac,
      `voted ${(gateOff.frac * 100).toFixed(1)}% off -> ${(gateOn.frac * 100).toFixed(1)}% on`);

// What the gate adds over trust: a far half that looks clean right now. The
// east wall is across the line from here, so with nobody over there the beam
// is trusted and votes, and only the gate keeps a robot that drives in later
// from being ranged before trust has seen it disagree.
{
  const clearOff = gateRun(false, false);
  const clearOn = gateRun(true, false);
  check('on an empty far half the gate is what drops the returns from over there',
        clearOff.leaked > 0 && clearOn.leaked === 0 && clearOn.frac < clearOff.frac * 0.2,
        `voted ${(clearOff.frac * 100).toFixed(0)}% off -> ${(clearOn.frac * 100).toFixed(1)}% on, ` +
        `${clearOff.leaked} far-side votes off, ${clearOn.leaked} on`);
}
// What the gate is NOT: the thing keeping the pose. A robot ranged on its near
// face returns a point on our own half whatever the gate says, and it is the
// innovation test that throws that away.
check('and the pose holds either way -- the gate is the second line, not the first',
      gateOn.worst < 3.0 && gateOff.worst < 3.0,
      `worst error ${gateOff.worst.toFixed(2)} in off, ${gateOn.worst.toFixed(2)} in on`);

// ---------------------------------------------------------------------------
console.log('\n== the gate picks its side from the pose the routine declares ==');
// Driven through the route VM so setPose happens mid-run without sim_reset,
// which would clear the gate. The route's clock starts at 0 with its first
// setPose; the gate is armed by hand partway through.
const gateAcrossSetPose = (first, second, delayMs, armAtMs) => {
  begin(first[0], first[1], 0);
  api.routeClear();
  api.routePush(OP_SET_POSE, first[0], first[1], 0, 0);
  api.routePush(OP_DELAY, delayMs, 0, 0, 0);
  api.routePush(OP_SET_POSE, second[0], second[1], 0, 0);
  api.routePush(OP_DELAY, 3000, 0, 0, 0);
  api.routeRun();
  run(armAtMs);
  api.setFar(1, 0);
  const armed = st().rcl2;
  run(delayMs - armAtMs + 200);
  const after = st().rcl2;
  api.stopAuton();
  api.setFar(0, 0);
  const sgn = g => `(${Math.sign(g.gnx)},${Math.sign(g.gny)})`;
  return { armed: sgn(armed), after: sgn(after) };
};
{
  // Armed long after the last pose, then the start pose arrives: re-picked.
  const r = gateAcrossSetPose([40, 40], [-40, -40], 1000, 700);
  check('armed with no recent pose, the next setPose picks the side',
        r.armed === '(-1,-1)' && r.after === '(1,1)', `${r.armed} -> ${r.after}`);
}
{
  // Armed right after a declared pose: that pose decides, later resets do not.
  const r = gateAcrossSetPose([-40, -40], [40, 40], 1000, 100);
  check('armed after setPose, a later reset across the line does not flip it',
        r.armed === '(1,1)' && r.after === '(1,1)', `${r.armed} -> ${r.after}`);
}
{
  // Pending, but the next reset comes after the window: no flip.
  const r = gateAcrossSetPose([40, 40], [-40, -40], 1500, 700);
  check('a reset after the pick window does not flip it',
        r.armed === '(-1,-1)' && r.after === '(-1,-1)', `${r.armed} -> ${r.after}`);
}

// ---------------------------------------------------------------------------
console.log('\n== the gate expires on the clock, not on RCL cycles ==');
// With RCL stopped nothing counts a cycle down, so a 16 s gate armed in
// autonomous and timed in cycles would stay up well into driver control.
begin(30, -50, 90);
api.setLoc(0);
api.setFar(1, 500);
check('armed with RCL stopped', st().rcl2.gate === true);
run(700);
check('and gone once its lifetime has passed', st().rcl2.gate === false,
      `${st().rcl2.gms.toFixed(0)} ms reported left`);
api.setFar(0, 0);

// ---------------------------------------------------------------------------
console.log('\n== an unplugged sensor is not a measurement ==');
// A disconnected V5 distance sensor answers PROS_ERR (INT32_MAX). Read as a
// range it is past 2000 mm, which with a wall 20 in away is "no return where
// there must be one" -- displacement evidence every cycle, from a cable. Both
// near-wall beams at (-48,-48) facing east are pulled, so the old reading
// would put two Impossible votes up every cycle.
begin(-48, -48, 90);
{
  const before = st().err.dist;
  api.distFault(2, 1);
  api.distFault(3, 1);
  let displaced = false, fresh = false, worst = 0;
  for (let i = 0; i < 300; i++) {
    api.step(5);
    const s = st();
    displaced = displaced || s.rcl2.displaced;
    // The first frames can still show the last cycle before the pull.
    if (i >= 4) fresh = fresh || sensor(s, 2).fresh || sensor(s, 3).fresh;
    worst = Math.max(worst, s.err.dist);
  }
  const s = st();
  check('no displacement is declared', !displaced);
  check('the faulted slots are never fresh, even held constant', !fresh);
  check('and read as no device, not as silence',
        sensor(s, 2).cls === CLS.invalid && sensor(s, 3).cls === CLS.invalid,
        `classes ${sensor(s, 2).cls}, ${sensor(s, 3).cls}`);
  check('the pose does not move', worst < before + 1.0,
        `${before.toFixed(2)} in -> worst ${worst.toFixed(2)} in`);

  api.distFault(2, 0);
  api.distFault(3, 0);
  let back2 = false, back3 = false;
  for (let i = 0; i < 40 && !(back2 && back3); i++) {
    api.step(5);
    const t = st();
    back2 = back2 || sensor(t, 2).rclValid;
    back3 = back3 || sensor(t, 3).rclValid;
  }
  check('plugged back in, both vote again', back2 && back3);
}

// ---------------------------------------------------------------------------
console.log('\n== a robot parked where a beam was ranging ==');
// The failure this filter exists to survive. An alliance partner flat against
// the west wall shortens one beam from 19 in to 3.5; believing it relocates the
// robot by 15.7 in, and nothing downstream can tell that happened.
begin(-48, -48, 90);
const clearErr = st().err.dist;
const clearRange = st().beams.find(b => b.i === 2).r;
api.actorAdd(-62, -48, 90, 0);
run(1200);
{
  const s = st();
  const blockedRange = s.beams.find(b => b.i === 2).r;
  check('the beam really does stop on it', blockedRange < clearRange - 10,
        `${clearRange.toFixed(1)} in -> ${blockedRange.toFixed(1)} in`);
  check('the reading is rejected, not averaged in',
        sensor(s, 2).cls === CLS.short && !sensor(s, 2).rclValid,
        `class ${sensor(s, 2).cls}`);
  check('and the pose does not move', s.err.dist < clearErr + 1.0,
        `${clearErr.toFixed(2)} in -> ${s.err.dist.toFixed(2)} in`);
}
api.actorClear();

// ---------------------------------------------------------------------------
console.log('\n== wheels spinning against a post ==');
// Drive wheels turn, the body does not, and odometry gains travel that never
// happened. Unpowered tracking pods do not: they roll on the floor, so they
// measure what the robot actually did.
begin(-40, -23.5, 90, 1);
api.setDrive(-127, -127); run(2500);
const wedgedPods = st().err.dist;
api.setDrive(127, 127); run(1200); api.setDrive(0, 0); run(1200);
check('with odom pods the pose survives it', st().err.dist < 3.0,
      `${wedgedPods.toFixed(2)} in wedged, ${st().err.dist.toFixed(2)} in after`);

begin(-40, -23.5, 90, 2);
api.setDrive(-127, -127); run(2500);
{
  const s = st();
  check('without them odometry runs away', s.err.dist > 20,
        `${s.err.dist.toFixed(1)} in`);
  // The point of the flag: the routine can see that a beam reset is needed.
  check('and the filter says it is lost rather than reporting healthy',
        s.rcl2.displaced === true);
}
api.setDrive(0, 0); run(1500);
api.setPods(0);

// ---------------------------------------------------------------------------
console.log('\n== the match clock ==');
api.actorClear();
api.pieceClear();
api.setPeriod(1);
api.reset(-48, -60, 90);
check('a 15 s period starts full', Math.abs(api.periodLeft() - 15000) < 50,
      `${api.periodLeft().toFixed(0)} ms`);
run(3000);
check('and counts down', Math.abs(api.periodLeft() - 12000) < 200,
      `${api.periodLeft().toFixed(0)} ms`);
run(13000);
check('then expires', api.periodLeft() === 0);
check('and the drive is cut', Math.abs(st().vel.lin) < 1.0,
      `${st().vel.lin.toFixed(2)} in/s`);
api.setPeriod(0);

// ---------------------------------------------------------------------------
console.log('\n== other robots on the field ==');
api.actorClear();
api.reset(0, -40, 90);
const id = api.actorAdd(-30, -30, 0, 1);
api.actorPathAdd(id, -30, -30);
api.actorPathAdd(id, 30, -30);
api.actorFlags(id, 1, 1, 1, 1);
const startX = JSON.parse(api.actorsJson())[0].x;
run(1500);
const movedX = JSON.parse(api.actorsJson())[0].x;
check('a patrolling robot drives its path', movedX > startX + 5,
      `x ${startX.toFixed(1)} -> ${movedX.toFixed(1)}`);
api.actorClear();
run(200);
check('clearing them empties the field', JSON.parse(api.actorsJson()).length === 0);

// ---------------------------------------------------------------------------
console.log('\n== the gate keeps the half each routine starts in ==');
// registerAutonDisableLine() reads the kept half off the pose. Called before
// the routine's setPose, that pose is whatever came before -- the origin, which
// is on the diagonal, or the other alliance's half -- and a gate kept from it
// would lock out the robot's own half for the whole period.
//
// Red starts where x + y < 0, so its gate normal points to +x+y; blue's the
// other way. Last in the file, and the mirrored routines last of all: stopping
// a routine kills its fiber without unwinding it, so runMirrored() never puts
// the mirror back and every later setPose would be reflected.
{
  const order = ['Red 3 Pin', 'UNc', 'Red 3 Pin Far'];
  const index = {};
  for (let i = 0; i < api.autonCount(); i++) index[api.autonName(i)] = i;
  const starts = [[0, 0], [40, 40], [-40, -40]];
  for (const name of order) {
    if (!(name in index)) { check(`${name} is in the sim's auton list`, false); continue; }
    const sign = name.startsWith('Blue') ? -1 : 1;
    const bad = [];
    for (const [sx, sy] of starts) {
      begin(sx, sy, 0);
      api.runAuton(index[name]);
      run(60);
      const g = st().rcl2;
      if (!g.gate || g.gnx * sign < 0.5 || g.gny * sign < 0.5) {
        bad.push(`from (${sx},${sy}): gate ${g.gate}, n (${g.gnx.toFixed(2)}, ${g.gny.toFixed(2)})`);
      }
      api.stopAuton();
    }
    check(`${name}: armed, keeping its own half`, bad.length === 0, bad.join('; '));
  }
}

console.log(failures === 0 ? '\nALL CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
