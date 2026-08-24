// How a motion ends, and what the estimators say about themselves.
//
//   node sim/test/motion.mjs
//
// Covers the parts of the chassis and the estimators that had no way of being
// seen from the sim at all: MotionOutcome, the SettleMonitor's hold and its
// stall test, motion chaining across the queue seam, the EKF/MCL/odometry fault
// counters, MCL's seedability, and a total drive-encoder outage.
//
// Every check here is on observed behaviour -- an outcome, a duration, a speed
// carried across a seam, a pose that did or did not move -- rather than on the
// library's own arithmetic restated.
import { loadSim, wrap, tally } from './harness.mjs';
import { healthText } from '../web/health.js';

const M = await loadSim(['sim_motion_run', 'sim_kill_drive_encoders', 'sim_mcl_seed']);
const api = wrap(M, {
  init: [null, []],
  reset: [null, ['number', 'number', 'number']],
  step: [null, ['number']],
  setLoc: ['sim_set_localization', null, ['number']],
  stateJson: ['string', []],
  setParam: [null, ['string', 'number']],
  getParam: ['number', ['string']],
  setDrive: [null, ['number', 'number']],
  collideTicks: ['number', []],
  motionClear: [null, []],
  motionPush: [null, ['number', 'number', 'number', 'number']],
  motionSet: [null, ['string', 'number']],
  motionRun: [null, []],
  motionRunning: ['number', []],
  motionJson: ['string', []],
  motionCancel: [null, []],
  driveCmdLeft: ['number', []],
  driveCmdRight: ['number', []],
  killDriveEncoders: [null, ['number']],
  mclSeed: [null, ['number']],
});

const t = tally('motion.mjs');
const st = () => JSON.parse(api.stateJson());
/// Mean magnitude of what the two sides are being commanded, volts.
const cmd = () => 0.5 * (Math.abs(api.driveCmdLeft()) + Math.abs(api.driveCmdRight()));

// A clear lane: y = -60 from x = -48 to x = +60. The nearest posts are the ones
// at (+/-23.5, -47), 13 in off the line, well outside the collision radius.
const LANE = [-48, -60, 90];

const STEP_KEYS = new Set(['type', 'a', 'b', 'c']);

/// Run a motion program from a known pose and return one record per motion.
/// Localization is off unless asked for: an estimator writing the pose back
/// would mask exactly the odometry behaviour some of these tests are about.
function prog(steps, { loc = 0, pose = LANE, maxMs = 15000 } = {}) {
  api.setLoc(0);
  api.reset(pose[0], pose[1], pose[2]);
  if (loc) api.setLoc(loc);
  for (let i = 0; i < 20; i++) api.step(10);
  api.motionClear();
  for (const s of steps) {
    api.motionPush(s.type, s.a ?? 0, s.b ?? 0, s.c ?? 0);
    for (const [k, v] of Object.entries(s)) if (!STEP_KEYS.has(k)) api.motionSet(k, v);
  }
  api.motionRun();
  let elapsed = 0;
  while (api.motionRunning() && elapsed < maxMs) { api.step(10); elapsed += 10; }
  return JSON.parse(api.motionJson());
}

api.init();

// ============================================================================
t.section('a motion says how it ended');
// ============================================================================

// The int is the contract: ChassisSample::type goes into telemetry CSVs, and the
// `type` here comes back off the library's own telemetry callback rather than
// from what the test asked for. Inserting a case in MotionType renumbers these.
const TYPES = [
  ['moveDistance',   1, { type: 1, a: 24, timeout: 4000 }],
  ['moveToPoint',    2, { type: 2, a: -24, b: -60, timeout: 4000 }],
  ['turnToHeading',  3, { type: 3, a: 180, timeout: 3000 }],
  ['turnToPoint',    4, { type: 4, a: -24, b: -40, timeout: 3000 }],
  // Targets have to clear the posts at (+/-23.5, -47) by more than the collision
  // proxy radius, not just the lane the run starts in: (-30,-48) and (-24,-48)
  // both press the robot against the post at (-23.5,-47) and now stall there.
  // They used to read `arrived` only because the world fabricated wheel travel
  // while pinned. The boomerang also avoids a heading it cannot reach, since it
  // loses all authority as the profile drives its reference speed to zero.
  ['moveArc',        5, { type: 5, a: -36, b: -52, c: 24, timeout: 5000 }],
  ['moveBoomerang',  6, { type: 6, a: -30, b: -54, c: 45, timeout: 5000 }],
  ['swingToHeading', 9, { type: 9, a: 150, b: 1, timeout: 4000 }],
];
for (const [name, wantType, step] of TYPES) {
  const r = prog([step])[0];
  t.check(`${name} arrives and reports MotionType ${wantType}`,
    r.type === wantType && r.name === 'arrived' && r.arrived === true,
    `type=${r.type} outcome=${r.name} ${r.ms}ms`);
  t.check(`${name} leaves the drive commanded to zero`, r.cmdOut === 0,
    `cmd=${r.cmdOut.toFixed(2)} V`);
}

{
  // A tolerance no drivetrain can hold. The motion must run its window out and
  // must not claim it got there.
  const r = prog([{ type: 1, a: 30, timeout: 1200, tolerance: 0.001 }])[0];
  t.check('an unreachable tolerance times out at its timeout',
    r.name === 'timeout' && Math.abs(r.ms - 1200) <= 30, `${r.name} ${r.ms}ms`);
  t.check('a timed-out motion does not report arrival', r.arrived === false);
  t.check('a timed-out motion stops the drive', r.cmdOut === 0,
    `cmd=${r.cmdOut.toFixed(2)} V`);
}

{
  // The settle hold is the only thing that should differ between these two, so
  // the moment the settle test first passes must not move and the exit must.
  const held = {};
  for (const hold of [110, 400]) {
    api.setParam('settleTimeMs', hold);
    held[hold] = prog([{ type: 1, a: 24, timeout: 4000 }])[0];
  }
  api.setParam('settleTimeMs', 110);
  const a = held[110], b = held[400];
  const tailA = a.ms - a.toleranceMs, tailB = b.ms - b.toleranceMs;
  t.check('the settle test passes at the same point either way',
    a.toleranceMs > 0 && Math.abs(a.toleranceMs - b.toleranceMs) <= 20,
    `${a.toleranceMs}ms vs ${b.toleranceMs}ms`);
  t.check('the exit waits out settleTimeMs after that',
    Math.abs(tailA - 110) <= 20 && Math.abs(tailB - 400) <= 20,
    `hold 110 -> ${tailA}ms, hold 400 -> ${tailB}ms`);
}

{
  // A cancelled motion must report its own outcome, not the previous motion's.
  api.setLoc(0);
  api.reset(LANE[0], LANE[1], LANE[2]);
  for (let i = 0; i < 20; i++) api.step(10);
  api.motionClear();
  api.motionPush(1, 60, 0, 0);
  api.motionSet('timeout', 5000);
  api.motionRun();
  for (let i = 0; i < 30; i++) api.step(10);
  api.motionCancel();
  for (let i = 0; i < 30; i++) api.step(10);
  const r = JSON.parse(api.motionJson())[0];
  t.check('a cancelled motion reports cancelled, not arrived',
    r.name === 'cancelled' && r.arrived === false, `${r.name} at ${r.ms}ms`);
  t.check('a cancelled motion stops the drive', cmd() === 0,
    `cmd=${cmd().toFixed(2)} V`);
}

// ============================================================================
t.section('stall detection ends a motion that cannot arrive');
// ============================================================================
//
// The scenario is a drivetrain commanded to full output whose wheels do not
// turn: leftScale/rightScale 0. That is the signal a robot pinned against a wall
// gives a real brain -- the motors stall and the encoders stop -- and it is the
// only way to produce it here, because this world model integrates encoder
// travel from the COMMANDED wheel speed and so reports a pinned robot as driving
// at full speed forever. See the wall case at the end of this section.

{
  const grace = 300, hold = 500;   // ChassisConfig stallGraceMs / stallTimeMs
  api.setParam('leftScale', 0);
  api.setParam('rightScale', 0);
  const r = prog([{ type: 1, a: 24, timeout: 4000 }])[0];
  api.setParam('leftScale', 1);
  api.setParam('rightScale', 1);
  t.check('a blocked drivetrain reports stalled', r.name === 'stalled',
    `${r.name} at ${r.ms}ms`);
  t.check('it gives up after the grace and hold, not at the timeout',
    r.ms >= grace + hold && r.ms <= grace + hold + 40,
    `${r.ms}ms, timeout was 4000ms`);
  t.check('a stalled motion does not report arrival', r.arrived === false);
  t.check('a stalled motion stops the drive', r.cmdOut === 0,
    `cmd=${r.cmdOut.toFixed(2)} V`);
}

{
  // The complement: a motion crawling but still closing must not be called
  // stalled. maxSpeed 3 of 127 is about 1.5 in/s of wheel, three times the
  // 0.5 in/s rest threshold; 0.8 is below it.
  const slow = prog([{ type: 1, a: 6, timeout: 6000, maxSpeed: 3 }])[0];
  const crawl = prog([{ type: 1, a: 6, timeout: 6000, maxSpeed: 0.8 }])[0];
  t.check('slow but converging is not a stall',
    slow.name === 'arrived' && slow.trueX > LANE[0] + 4,
    `${slow.name} after ${slow.ms}ms, ${(slow.trueX - LANE[0]).toFixed(2)} in`);
  t.check('below the rest threshold it is', crawl.name === 'stalled',
    `${crawl.name} after ${crawl.ms}ms, ${(crawl.trueX - LANE[0]).toFixed(2)} in`);
}

{
  // Every encoder dead is not a rest measurement, and must not be read as one:
  // a stalled report here would be the filter inventing a stop out of a dead
  // sensor. It cannot arrive either, so the only honest end is the timeout.
  api.setLoc(0);
  api.reset(-60, -60, 90);
  for (let i = 0; i < 20; i++) api.step(10);
  api.motionClear();
  api.motionPush(1, 60, 0, 0);
  api.motionSet('timeout', 3000);
  api.motionRun();
  for (let i = 0; i < 40; i++) api.step(10);
  api.killDriveEncoders(1);
  let ms = 0;
  while (api.motionRunning() && ms < 8000) { api.step(10); ms += 10; }
  const r = JSON.parse(api.motionJson())[0];
  api.killDriveEncoders(0);
  t.check('a total encoder outage is neither a stall nor an arrival',
    r.name === 'timeout', `${r.name} at ${r.ms}ms`);
  t.check('and it still ends with the drive stopped', r.cmdOut === 0,
    `cmd=${r.cmdOut.toFixed(2)} V`);
}

{
  // Drive into the perimeter, sampling the wheels while the robot is actually
  // pinned -- after the motion ends the command is off and the speed has
  // decayed, which says nothing about what the rest test was given.
  api.setLoc(0);
  api.reset(40, -60, 90);
  for (let i = 0; i < 20; i++) api.step(10);
  api.motionClear();
  api.motionPush(1, 40, 0, 0);
  api.motionSet('timeout', 2500);
  api.motionRun();
  let ms = 0, pinnedWheelSpeed = 0, pinnedTicks = 0;
  while (api.motionRunning() && ms < 5000) {
    api.step(10);
    ms += 10;
    const s = st();
    if (!s.colliding) continue;
    pinnedTicks++;
    pinnedWheelSpeed = Math.max(pinnedWheelSpeed,
      Math.abs(s.vel.left), Math.abs(s.vel.right));
  }
  const r = JSON.parse(api.motionJson())[0];
  t.check('a motion that hits the wall still ends and stops the drive',
    r.ms <= 2500 && r.cmdOut === 0 && pinnedTicks > 0,
    `${r.name} at ${r.ms}ms, ${pinnedTicks} contact ticks`);
  // The stall report is what a real robot gives here. It needs a world that
  // stops the wheels when the robot stops; this one keeps clocking encoder
  // travel from the command, so the rest test can never fire. Probed rather
  // than assumed, so this becomes a live check the day that is fixed.
  if (pinnedWheelSpeed > 5.0) {
    console.log('  [BLOCKED] a robot pinned against the wall reports stalled  ' +
      `world reports ${pinnedWheelSpeed.toFixed(1)} in/s of wheel while pinned, ` +
      `and the motion ${r.name} with the pose ${r.x.toFixed(1)} in ` +
      '(outside the field): sim/world integrates encoders from the command, ' +
      'not from travel');
  } else {
    t.check('a robot pinned against the wall reports stalled',
      r.name === 'stalled' && r.ms < 2500, `${r.name} at ${r.ms}ms`);
  }
}

// ============================================================================
t.section('motion chaining carries speed across the queue seam');
// ============================================================================
//
// earlyExitRange exists to hand a running drivetrain to the next motion. The
// regression it guards: the motion task zeroing the drive the instant its queue
// drained, microseconds after releasing a blocking caller, which turned every
// chained motion on the default API into a hard stop. Both halves are asserted
// here -- the handover, and the drive going quiet when nothing follows.

const CHAIN = [{ type: 1, a: 30, timeout: 3000, earlyExit: 8 },
               { type: 1, a: 30, timeout: 3000 }];
const PLAIN = [{ type: 1, a: 30, timeout: 3000 },
               { type: 1, a: 30, timeout: 3000 }];
{
  const chained = prog(CHAIN);
  const plain = prog(PLAIN);
  t.check('the first motion of a chain reports chained, not arrived',
    chained[0].name === 'chained' && chained[0].arrived === true,
    `${chained[0].name} at ${chained[0].ms}ms`);
  t.check('the same motion without earlyExitRange reports arrived',
    plain[0].name === 'arrived', plain[0].name);
  // The command, not the speed, is what the regression destroyed. A robot
  // coasting on the world's own velocity lag still reads ~20 in/s a tick after
  // the drive is cut, so speed alone cannot tell a handover from a hard stop
  // -- the volts standing on the motors at the seam can.
  t.check('a chained seam hands the next motion a driven drivetrain',
    chained[1].cmdIn > 0.5 && chained[1].vIn > 15.0,
    `${chained[1].cmdIn.toFixed(2)} V and ${chained[1].vIn.toFixed(2)} in/s ` +
    'into the second motion');
  t.check('an un-chained seam hands over a stopped one',
    plain[1].cmdIn === 0 && plain[1].vIn < 5.0,
    `${plain[1].cmdIn.toFixed(2)} V, ${plain[1].vIn.toFixed(2)} in/s`);
  t.check('and chaining buys the time it is for',
    chained[0].ms + chained[1].ms < 0.9 * (plain[0].ms + plain[1].ms),
    `${chained[0].ms + chained[1].ms}ms vs ${plain[0].ms + plain[1].ms}ms`);
}

{
  // A chain whose follow-up never comes. The grace has to be short and it has to
  // end: voltage left on a drained queue is a robot driving itself off the field.
  api.setLoc(0);
  api.reset(LANE[0], LANE[1], LANE[2]);
  for (let i = 0; i < 20; i++) api.step(10);
  api.motionClear();
  api.motionPush(1, 30, 0, 0);
  api.motionSet('timeout', 3000);
  api.motionSet('earlyExit', 8);
  api.motionRun();
  let ms = 0;
  while (api.motionRunning() && ms < 6000) { api.step(10); ms += 10; }
  const trace = [];
  for (let i = 0; i < 60; i++) { trace.push(cmd()); api.step(10); }
  const held = trace.slice(0, 3).some((v) => v > 0.5);
  const quietFrom = trace.findIndex((v) => v === 0);
  t.check('a chained-out motion holds its voltage for the follow-up',
    held, `first 30ms: ${trace.slice(0, 3).map((v) => v.toFixed(2)).join(' ')} V`);
  t.check('and a drained queue then stops the drive',
    quietFrom >= 0 && quietFrom <= 15 && trace.slice(quietFrom).every((v) => v === 0),
    `zero from ${quietFrom * 10}ms and stays zero to ${trace.length * 10}ms`);
}

{
  // Past the grace there is nothing to hand over, so the next motion starts from
  // rest -- which is the same property as above, seen from the other side.
  // 60 ms is only just past the grace, and the robot is still coasting at
  // 19 in/s when the next motion starts -- the volts are what have gone.
  const justPast = prog([{ ...CHAIN[0], delayAfter: 60 }, CHAIN[1]]);
  const gapped = prog([{ ...CHAIN[0], delayAfter: 300 }, CHAIN[1]]);
  t.check('a gap just past the grace already ends the handover',
    justPast[0].name === 'chained' && justPast[1].cmdIn === 0,
    `${justPast[1].cmdIn.toFixed(2)} V, still coasting at ` +
    `${justPast[1].vIn.toFixed(2)} in/s`);
  t.check('a 300ms gap starts the next motion from rest',
    gapped[0].name === 'chained' && gapped[1].cmdIn === 0 && gapped[1].vIn < 5.0,
    `${gapped[1].cmdIn.toFixed(2)} V, ${gapped[1].vIn.toFixed(2)} in/s`);
}

// ============================================================================
t.section('localization diagnostics');
// ============================================================================

{
  const s0 = st();
  t.check('the diagnostics block reports every filter',
    s0.diag !== undefined && s0.diag.ekf !== undefined &&
    s0.diag.odom !== undefined && s0.diag.enc !== undefined,
    Object.keys(s0.diag ?? {}).join(','));
  // Two decimals in the JSON, full float32 from the getter, so compare as
  // numbers rather than for equality.
  t.check('the EKF reports the gate it is holding measurements to',
    Math.abs(s0.diag.ekf.gateR - api.getParam('ekfRangeGate')) < 0.01 &&
    Math.abs(s0.diag.ekf.gateM - api.getParam('ekfMclGate')) < 0.01,
    `range ${s0.diag.ekf.gateR} vs ${api.getParam('ekfRangeGate')}, ` +
    `mcl ${s0.diag.ekf.gateM} vs ${api.getParam('ekfMclGate')}`);
}

{
  // A filter that rejects or deadzones every beam looks identical to a healthy
  // one from the pose alone. Drive a leg with the EKF up and require that it is
  // never blind for long: the counters are what make that visible.
  api.setLoc(0);
  api.reset(-48, -60, 90);
  api.setLoc(1 | 4);
  for (let i = 0; i < 40; i++) api.step(10);
  api.setDrive(6, 6);
  let worstBlind = 0, worstReject = 0, everUsed = 0;
  const nEff = [];
  for (let i = 0; i < 150; i++) {
    api.step(10);
    const s = st();
    worstBlind = Math.max(worstBlind, s.diag.ekf.streakBlind);
    worstReject = Math.max(worstReject, s.diag.ekf.streakR);
    everUsed = Math.max(everUsed, s.ekf.beams);
    nEff.push(s.mcl.nEff);
  }
  api.setDrive(0, 0);
  for (let i = 0; i < 30; i++) api.step(10);
  const s = st();
  t.check('the EKF is folding beams in, not silently dropping them',
    everUsed > 0 && worstBlind < 40,
    `peak beams ${everUsed}, longest blind streak ${worstBlind}, ` +
    `reject streak ${worstReject}`);
  // N_eff has to be the evidence-weighted spread, measured before the resample
  // flattens every weight to 1. Published after it instead, it is a constant
  // equal to the particle count -- which is what a filter reports when it is
  // completely lost, so the one number that says "MCL is degenerating" said
  // "MCL has no idea" forever. Distinctness is the test that catches that.
  const distinct = new Set(nEff.map((v) => v.toFixed(1))).size;
  t.check('MCL publishes a live N_eff, not the particle count',
    distinct > 5 && Math.max(...nEff) < 512,
    `${distinct} distinct values, ${Math.min(...nEff).toFixed(1)}-` +
    `${Math.max(...nEff).toFixed(1)} of 1024`);
  t.check('MCL reports its resampling',
    s.mcl.resamples > 0 && s.mcl.gen > 0,
    `${s.mcl.resamples} resamples over ${s.mcl.gen} published ticks`);
  t.check('the EKF stays with the robot over the leg',
    Math.hypot(s.ekf.x - s.true.x, s.ekf.y - s.true.y) < 4.0,
    `${Math.hypot(s.ekf.x - s.true.x, s.ekf.y - s.true.y).toFixed(2)} in`);
}

// ============================================================================
t.section('MCL is reproducible, and reseedable on purpose');
// ============================================================================
//
// sim/README.md still says MclTracking seeds from std::random_device and cannot
// replay. MclCore::kDefaultSeed and setSeed() made that false; this is what says
// so. Independent runs have to be asked for, which is the point: a divergence
// you cannot replay is a divergence you cannot debug.

function mclRun(seed) {
  api.setLoc(0);
  api.reset(-48, -60, 90);
  api.setLoc(1);
  if (seed !== null) api.mclSeed(seed);
  for (let i = 0; i < 20; i++) api.step(10);
  api.setDrive(6, 6);
  const trace = [];
  for (let i = 0; i < 150; i++) {
    api.step(10);
    const s = st();
    trace.push(`${s.mcl.x.toFixed(4)},${s.mcl.y.toFixed(4)}`);
  }
  api.setDrive(0, 0);
  for (let i = 0; i < 30; i++) api.step(10);
  const s = st();
  return { trace: trace.join('|'), err: Math.hypot(s.mcl.x - s.true.x, s.mcl.y - s.true.y) };
}
{
  const a = mclRun(null), b = mclRun(null), c = mclRun(20260823), d = mclRun(20260823);
  t.check('two runs at the default seed are identical', a.trace === b.trace,
    `final error ${a.err.toFixed(3)} / ${b.err.toFixed(3)} in`);
  t.check('an explicit seed replays identically too', c.trace === d.trace,
    `final error ${c.err.toFixed(3)} in`);
  t.check('a different seed is a genuinely different run', a.trace !== c.trace);
  t.check('and it localizes just as well', c.err < 3.0 && a.err < 3.0,
    `${a.err.toFixed(3)} in vs ${c.err.toFixed(3)} in`);
}

// ============================================================================
t.section('a total drive-encoder outage holds the pose');
// ============================================================================
//
// With every drive encoder dead the scalar average reads 0.0, which is
// indistinguishable from standing at the origin: the first tick looks like the
// whole accumulated distance driven backwards, the jump guard rejects it and
// re-baselines at zero, and from then on the pose sits still while the robot
// drives -- and nothing says why. Holding the last good value instead means the
// baseline survives, so the outage costs nothing once the encoders come back.

{
  api.setLoc(0);
  api.reset(-60, -60, 90);
  for (let i = 0; i < 30; i++) api.step(10);
  api.setDrive(6, 6);
  for (let i = 0; i < 120; i++) api.step(10);
  const before = st();
  api.killDriveEncoders(1);
  for (let i = 0; i < 40; i++) api.step(10);
  const during = st();
  api.killDriveEncoders(0);
  for (let i = 0; i < 10; i++) api.step(10);
  const after = st();
  api.setDrive(0, 0);
  for (let i = 0; i < 30; i++) api.step(10);

  const odomHeld = during.odom.x - before.odom.x;
  const droveBlind = during.true.x - before.true.x;
  const odomTotal = after.odom.x - before.odom.x;
  const trueTotal = after.true.x - before.true.x;

  // The library's counters run from construction, not from sim_reset, so this
  // is the increase over the outage rather than an absolute.
  const outageTicks = during.diag.odom.outages - before.diag.odom.outages;
  t.check('the outage is counted rather than silent',
    outageTicks > 30 && during.diag.enc.live === 0 &&
    during.diag.enc.valid === false,
    `${outageTicks} outage ticks over 400ms, enc.live ${during.diag.enc.live}`);
  t.check('the pose holds instead of reading zero travel',
    Math.abs(odomHeld) < 1.0 && droveBlind > 10.0,
    `odom moved ${odomHeld.toFixed(2)} in while the robot covered ` +
    `${droveBlind.toFixed(2)} in`);
  t.check('holding keeps the jump guard out of it',
    during.diag.odom.jumps === before.diag.odom.jumps,
    `jumps ${before.diag.odom.jumps} -> ${during.diag.odom.jumps}`);
  t.check('so the travel is recovered when the encoders come back',
    trueTotal > 10.0 && Math.abs(odomTotal - trueTotal) < 0.15 * trueTotal,
    `odom ${odomTotal.toFixed(2)} in vs true ${trueTotal.toFixed(2)} in`);
  t.check('and the encoders read healthy again',
    after.diag.enc.live === 4 && after.diag.enc.valid === true,
    `enc.live ${after.diag.enc.live}`);
}


// ============================================================================
t.section('the HUD health line says only what is wrong');
// ============================================================================
// The line the browser shows is built from the same diagnostics this suite
// asserts on, so it is worth pinning: a wall of zeros would be noise, and a
// counter nobody reads is a counter that never warns anyone.
{
  const clean = {
    locMode: 5,
    diag: {
      enc: { live: 4, faulted: 0, valid: true },
      odom: { outages: 0, jumps: 0, imuFaults: 0 },
      ekf: { streakR: 0, streakBlind: 0, streakM: 0, repairs: 0 },
    },
  };
  const withDiag = (over) => ({
    locMode: 5,
    diag: {
      enc: { ...clean.diag.enc, ...(over.enc || {}) },
      odom: { ...clean.diag.odom, ...(over.odom || {}) },
      ekf: { ...clean.diag.ekf, ...(over.ekf || {}) },
    },
  });

  t.check('a healthy stack reads ok', healthText(clean) === 'ok', healthText(clean));
  t.check('no diagnostics at all is not "ok"', healthText({ locMode: 5 }) === '—');

  const dead = healthText(withDiag({ enc: { live: 0, faulted: 4, valid: false } }));
  t.check('a total encoder outage is stated, not counted', /DEAD/.test(dead), dead);

  const partial = healthText(withDiag({ enc: { live: 3, faulted: 1 } }));
  t.check('a partial encoder fault shows the survivors', /3\/4/.test(partial), partial);

  // One rejected reading is normal and must not cry wolf; a streak is the signal.
  const one = healthText(withDiag({ ekf: { streakR: 1 } }));
  const many = healthText(withDiag({ ekf: { streakR: 4 } }));
  t.check('a single rejected range is not reported', one === 'ok', one);
  t.check('a streak of them is', /range rejected x4/.test(many), many);

  // EKF counters are meaningless when the EKF is not running.
  const ekfOff = healthText({ locMode: 1, diag: withDiag({ ekf: { streakR: 9 } }).diag });
  t.check('EKF streaks are suppressed when the EKF is off', ekfOff === 'ok', ekfOff);

  const both = healthText(withDiag({ odom: { outages: 2, jumps: 1 } }));
  t.check('several faults are joined, not collapsed to the first',
    /2 outage/.test(both) && /1 jump/.test(both), both);
}

process.exit(t.finish());
