// The Override field objects, the claw sensor, the lift, the rear aligner and
// the toggles -- everything on the field the robot touches rather than ranges.
//
//   node sim/test/pieces.mjs
//
// Before any of this existed the claw was blind in a way that read as the
// opposite: chad::claw_dist is not one of the localization beams, so nothing
// wrote its bus slot, and clawDistance() saw a zero -- which the library
// correctly treats as "something is against the jaws". clawSees() was true on
// an empty field and auto-grab fired the instant it was switched on.
//
// The counts in the first section are the manual's own: 36 Cups and 37 Pins
// start on the Field, the rest are Match Loads and Preloads. If they stop
// matching, the layout has drifted from Figure FO-2.
//
// The pick-and-place case drives by hand, so it measures the claw and nothing
// else. The last check of the compiled routine is red, and says why: the cause
// is a pose reset, not anything in this file.
import createChadSim from '../web/chad_sim.mjs';
import { installRapier } from '../web/rapier_pieces.js';

const M = await createChadSim();
// Rapier drives the pieces here, as it does in the browser. It has to be in
// place before the first tick: pieces.cpp asks once per step whether a solver
// is installed and falls back to its own kinematic model if not, so a test
// that forgot this would quietly measure the fallback instead.
await installRapier(M);
const c = (n, r, a) => M.cwrap(n, r, a);
const api = {
  init: c('sim_init', null, []),
  reset: c('sim_reset', null, ['number', 'number', 'number']),
  step: c('sim_step', null, ['number']),
  setDrive: c('sim_set_drive', null, ['number', 'number']),
  setLoc: c('sim_set_localization', null, ['number']),
  setClaw: c('sim_set_claw', null, ['number']),
  setPreload: c('sim_set_preload', null, ['number']),
  togglesJson: c('sim_toggles_json', 'string', []),
  toggleReset: c('sim_toggle_reset', null, []),
  piecesExternal: c('sim_pieces_external', 'number', []),
  setParam: c('sim_set_param', null, ['string', 'number']),
  stateJson: c('sim_state_json', 'string', []),
  piecesJson: c('sim_pieces_json', 'string', []),
  pieceClear: c('sim_piece_clear', null, []),
  pieceDefault: c('sim_piece_default', null, []),
  pieceAdd: c('sim_piece_add', 'number', ['number', 'number', 'number', 'number']),
  pieceRemove: c('sim_piece_remove', null, ['number']),
  piecePose: c('sim_piece_set_pose', null, ['number', 'number', 'number', 'number']),
  pieceFlags: c('sim_piece_set_flags', null,
                ['number', 'number', 'number', 'number']),
  autonCount: c('sim_auton_count', 'number', []),
  autonName: c('sim_auton_name', 'string', ['number']),
  runAuton: c('sim_run_auton', null, ['number']),
  stopAuton: c('sim_stop_auton', null, []),
  routeRunning: c('sim_route_running', 'number', []),
};

const pj = () => JSON.parse(api.piecesJson());
const st = () => JSON.parse(api.stateJson());
const run = (ms) => { for (let i = 0; i < ms; i += 5) api.step(5); };
const piece = (id) => pj().pieces.find(q => q.id === id);
const tg = () => JSON.parse(api.togglesJson());
const tgState = (wall) => tg().find(t => t.wall === wall).state;

// The nine goals, in the order gamespecific/config.cpp registers them.
const RED = [0, 1], BLUE = [2, 3], TALL = 4, SHORT = [5, 6, 7, 8];

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};

api.init();

check('rapier is the one driving the pieces', api.piecesExternal() === 1,
      api.piecesExternal() === 1 ? '' : 'fell back to the kinematic model');

// ---------------------------------------------------------------------------
console.log('\n== Figure FO-2 ==');
api.reset(-48, -60, 90);
check('nothing is out until it is asked for', pj().pieces.length === 0,
      `${pj().pieces.length} pieces`);

api.pieceDefault();
{
  const p = pj().pieces;
  const pins = p.filter(q => q.kind === 0);
  const cups = p.filter(q => q.kind === 1);
  check('36 cups start on the field', cups.length === 36, `${cups.length}`);
  check('and 37 pins', pins.length === 37, `${pins.length}`);

  // 4 red/blue, 8 red/yellow, 8 blue/yellow, 17 yellow/yellow.
  const byColor = [0, 1, 2, 3].map(k => pins.filter(q => q.color === k).length);
  check('the colour split is the manual\'s',
        byColor[0] === 8 && byColor[1] === 8 && byColor[2] === 17 && byColor[3] === 4,
        `r/y ${byColor[0]}, b/y ${byColor[1]}, y/y ${byColor[2]}, r/b ${byColor[3]}`);

  // Sixteen pins lie on their side around the four cups on the Autonomous Line.
  check('sixteen pins start lying down',
        pins.filter(q => !q.up).length === 16,
        `${pins.filter(q => !q.up).length}`);

  const seated = pins.filter(q => q.goal >= 0);
  check('a yellow pin stands in each of the five neutral goals',
        seated.length === 5 &&
        seated.every(q => q.goal === TALL || SHORT.includes(q.goal)),
        seated.map(q => `goal ${q.goal}`).join(', '));
  check('and the four alliance goals start empty',
        !seated.some(q => RED.includes(q.goal) || BLUE.includes(q.goal)));

  // Twelve cups start clear side up: the eight against the walls that hold a
  // pin, and the four at the corners of the midfield diamond.
  check('twelve cups start clear side up',
        cups.filter(q => q.clear).length === 12,
        `${cups.filter(q => q.clear).length}`);

  check('every nested pin has a cup to be nested in',
        p.filter(q => q.cup >= 0).every(q => p.some(v => v.id === q.cup)));
}

// A reset has to put the same field back, or a second run of a routine is
// measuring where the first one happened to leave things.
api.reset(-48, -60, 90);
check('and a reset restores it', pj().pieces.length === 73,
      `${pj().pieces.length} pieces`);

// ---------------------------------------------------------------------------
console.log('\n== true sizes ==');
{
  const p = pj().pieces;
  const pinR = p.find(q => q.kind === 0).r;
  const cupR = p.find(q => q.kind === 1).r;
  check('a pin is 1.6 in across', Math.abs(pinR - 0.8) < 0.01, `r ${pinR}`);
  check('a cup is 3.16 in across', Math.abs(cupR - 1.58) < 0.01, `r ${cupR}`);
  const goals = JSON.parse(M.cwrap('sim_field_json', 'string', [])()).circles;
  check('a goal is 5.7 in across', goals.every(g => Math.abs(g[2] - 2.85) < 0.01),
        `r ${goals[0][2]}`);
}

// ---------------------------------------------------------------------------
console.log('\n== the claw sensor ==');
api.pieceClear();
api.reset(0, 0, 90);            // VEX 90 faces +x, so the claw faces -x
run(300);
const bare = pj();
check('it reads the far wall on a bare field', bare.clawRange > 50,
      `${bare.clawRange.toFixed(1)} in`);
check('and the jaws are where the mount says', Math.abs(bare.clawX + 8) < 0.1,
      `claw at ${bare.clawX.toFixed(1)}, ${bare.clawY.toFixed(1)}`);

const pin = api.pieceAdd(0, -14, 0, 0);
run(200);
check('a pin in front of it shortens the reading', pj().clawRange < 7,
      `${pj().clawRange.toFixed(1)} in`);

api.pieceRemove(pin);
run(200);
check('and removing it opens the reading back up', pj().clawRange > 50,
      `${pj().clawRange.toFixed(1)} in`);

// ---------------------------------------------------------------------------
console.log('\n== a pin on its side is 6.5 in of obstacle ==');
// Offset 2.5 in off the beam: the 1.6 in body of an upright pin is nowhere
// near it, the 6.5 in body of a lying one crosses it. Publishing a lying pin
// as one circle instead of three is exactly the bug this catches.
api.pieceClear();
api.reset(0, 0, 90);
const lying = api.pieceAdd(0, -20, 2.5, 0);     // VEX 0 points +y
run(200);
check('upright, it misses the beam', pj().clawRange > 50,
      `${pj().clawRange.toFixed(1)} in`);
api.pieceFlags(lying, 2, 0, 0);
run(200);
check('lying across it, it does not', pj().clawRange < 14,
      `${pj().clawRange.toFixed(1)} in`);
api.pieceClear();

// ---------------------------------------------------------------------------
console.log('\n== the localization beams do not see them ==');
// A 1.6 in pin and a half-transparent cup do not fill enough of a V5 distance
// sensor's cone at range to return anything. The beams see walls, goals,
// edited geometry and the other robots -- and a sim where they also see every
// loose cup is one where the filter fights returns the robot never gets.
api.reset(30, -50, 90);         // facing +x, the east wall about 40 in away
run(400);
const clearRange = st().beams.find(b => b.i === 0).r;
api.pieceAdd(1, 44, -50, 0);    // a cup straight ahead
run(400);
check('a cup in the beam changes nothing',
      Math.abs(st().beams.find(b => b.i === 0).r - clearRange) < 0.5,
      `${clearRange.toFixed(1)} in -> ${st().beams.find(b => b.i === 0).r.toFixed(1)} in`);

// The switch is there because where the sensors sit is a hardware question.
api.setParam('distSeePieces', 1);
run(400);
check('unless you say the sensors are low enough to',
      st().beams.find(b => b.i === 0).r < clearRange - 5,
      `${st().beams.find(b => b.i === 0).r.toFixed(1)} in`);
api.setParam('distSeePieces', 0);
run(400);
api.pieceClear();

// A goal is a specified obstacle, so it always stops one.
// Facing +x, so the rear sensor looks -x. Its mount is 5.5 in to the left, so
// the robot sits 5.5 low for that beam to run down y = -23.5 into the goal.
api.reset(-36, -29, 90);
run(400);
check('a goal still stops the beam looking at it',
      st().beams.find(b => b.i === 2).r < 8,
      `back beam ${st().beams.find(b => b.i === 2).r.toFixed(1)} in`);

// And exhaustively: with the whole starting layout out, every beam has to read
// exactly what it reads on a bare field. Seventy-three objects, several of them
// parked against the walls the beams range off, and not one may shift a
// reading. Noise off, so "identical" means identical.
api.setParam('distNoiseMm', 0);
{
  const poses = [
    [-48, -60, 90], [0, -60, 0], [-60, 0, 270], [30, -50, 90],
    [-23.5, 55, 0], [20, 20, 45], [66, -23.5, 180],
  ];
  let worst = 0, worstAt = '';
  for (const [x, y, h] of poses) {
    api.pieceClear();
    api.reset(x, y, h);
    run(300);
    const bare = st().beams.map(b => b.r);
    api.pieceDefault();
    run(300);
    const laid = st().beams.map(b => b.r);
    for (let i = 0; i < bare.length; i++) {
      const d = Math.abs(bare[i] - laid[i]);
      if (d > worst) { worst = d; worstAt = `beam ${i} at ${x},${y},${h}`; }
    }
  }
  check('the whole layout is invisible to them, at every pose', worst < 1e-3,
        worst < 1e-3 ? '7 poses, 4 beams each' : `${worst.toFixed(3)} in on ${worstAt}`);
}
api.pieceClear();
api.setParam('distNoiseMm', 5);


// ---------------------------------------------------------------------------
console.log('\n== the robot plows them ==');
// <SG6a> allows plowing, and a route that drives through a cup without
// touching it is not the route that runs.
// y = -35 is a clear lane: at y = -50 the goal on (23.5, -47) stops the robot
// two inches after it first touches the cup, which measures the goal rather
// than the plough.
api.reset(0, -35, 90);
const cup = api.pieceAdd(1, 20, -35, 0);
run(200);
api.setDrive(80, 80); run(1500); api.setDrive(0, 0); run(300);
{
  const p = piece(cup);
  const tr = st()['true'];
  check('a cup in the way gets pushed', p.x > 22,
        `x 20.0 -> ${p.x.toFixed(1)}`);
  check('and is carried along in front rather than run over',
        p.x > 60 && p.x > tr.x,
        `cup ${p.x.toFixed(1)}, robot ${tr.x.toFixed(1)}`);
}
api.pieceClear();

// What the solver is for. The kinematic model moved the piece the robot hit
// and nothing else, so a plough gathered exactly one thing.
api.reset(0, -35, 90);
const first = api.pieceAdd(1, 20, -35, 0);
const second = api.pieceAdd(1, 27, -35.4, 0);
run(200);
api.setDrive(80, 80); run(900); api.setDrive(0, 0); run(400);
check('and the one it hits shifts the one beyond it',
      piece(second).x > 29,
      `second cup 27.0 -> ${piece(second).x.toFixed(1)}, ` +
      `first ${piece(first).x.toFixed(1)}`);
api.pieceClear();

// ---------------------------------------------------------------------------
console.log('\n== dragging one ==');
api.pieceDefault();
{
  const seatedPin = pj().pieces.find(q => q.goal >= 0);
  api.piecePose(seatedPin.id, 10, 10, 0);
  run(100);
  const moved = piece(seatedPin.id);
  check('it goes where it is put', Math.hypot(moved.x - 10, moved.y - 10) < 0.1,
        `${moved.x.toFixed(1)}, ${moved.y.toFixed(1)}`);
  check('and comes off the goal it was in', moved.goal === -1);

  // A pin nested in a cup rides it.
  const nested = pj().pieces.find(q => q.cup >= 0);
  api.piecePose(nested.cup, -5, -5, 0);
  run(100);
  const rider = piece(nested.id);
  check('a nested pin follows its cup',
        Math.hypot(rider.x + 5, rider.y + 5) < 0.2,
        `${rider.x.toFixed(1)}, ${rider.y.toFixed(1)}`);
}

// ---------------------------------------------------------------------------
console.log('\n== pick one up and put it in a goal ==');
// Driven by hand rather than by a routine, so this measures the claw and
// nothing else. Back the jaws onto a pin, close, drive to a goal, open.
api.pieceClear();
api.setLoc(2);
// Facing +x, so the jaws point -x, down the row at the red goal on (-47, -23.5)
// -- 16 in of carry away, well clear of it at the start.
api.reset(-20, -23.5, 90);
const target = api.pieceAdd(0, -31, -23.5, 0);
run(200);
check('the pin starts clear of the goal',
      Math.hypot(-31 + 47, 0) > 5, '16.0 in away');

api.setClaw(1);                 // the claw grips extended
run(200);
check('it takes the pin in front of the jaws', pj().held === target,
      `holding ${pj().held}`);
check('and the pin rides in the jaws',
      Math.abs(piece(target).x - pj().clawX) < 4,
      `pin at ${piece(target).x.toFixed(1)}, jaws at ${pj().clawX.toFixed(1)}`);

// Reverse until the goal stops the body, which is how a routine arrives at one.
api.setDrive(-60, -60); run(1400); api.setDrive(0, 0); run(300);
check('carrying it moves it', piece(target).x < -40,
      `x -31.0 -> ${piece(target).x.toFixed(1)}`);

api.setClaw(0);
run(200);
{
  const p = piece(target);
  check('opening against a goal nests it there', p.goal === 0,
        `goal ${p.goal}, at ${p.x.toFixed(1)}, ${p.y.toFixed(1)}`);
  check('and the claw is empty again', pj().held === -1);
}
api.pieceClear();

// ---------------------------------------------------------------------------
console.log('\n== the preload ==');
// <SG5>: one pin, touching the robot, and nothing else. Red loads red/yellow
// and blue loads blue/yellow, taken from which side of the Autonomous Line the
// robot was placed on rather than from being told which alliance you are.
api.pieceClear();
api.setPreload(1);
api.reset(-61.5, 0, 270);       // x + y < 0, so this is red's half
run(200);
{
  const p = pj().pieces;
  check('a reset puts one pin in front of the jaws', p.length === 1,
        `${p.length} pieces`);
  check('and it is red/yellow on red' + String.fromCharCode(39) + 's half',
        p[0] && p[0].color === 0, `colour ${p[0] && p[0].color}`);
  const d = p[0] ? Math.hypot(p[0].x - pj().clawX, p[0].y - pj().clawY) : 99;
  check('close enough for the claw to shut on it', d < 4.5,
        `${d.toFixed(2)} in from the jaws`);
}
api.setClaw(1);
run(200);
check('and closing takes it', pj().held === pj().pieces[0].id,
      `holding ${pj().held}`);

// Resetting with the claw shut has to arrive holding it. You put the robot on
// the field with the preload already gripped; it is not something it finds a
// tick later once the sensor notices it.
const wasId = pj().held;
api.reset(-61.5, 0, 270);
check('a reset with the claw shut arrives holding it', pj().held === wasId,
      `holding ${pj().held} before a single tick`);

// Five more. The preload used to be removed and re-added every time, so its id
// walked up by one per reset and the HUD named a different pin each run.
for (let i = 0; i < 5; i++) { api.reset(-61.5, 0, 270); run(20); }
check('and stays the same pin however many resets',
      pj().held === wasId && pj().pieces.length === 1,
      `id ${pj().held} after six resets, ${pj().pieces.length} pieces`);

// And it goes where the robot goes.
api.reset(0, 0, 0);
run(50);
{
  const p = pj().pieces.find(q => q.id === pj().held);
  check('a reset somewhere else brings it along',
        p && Math.hypot(p.x, p.y + 10.8) < 1.0,
        p ? `${p.x.toFixed(1)}, ${p.y.toFixed(1)}` : 'gone');
}

api.setClaw(0);
api.reset(-61.5, 0, 270);
run(50);
check('with the claw open it is set down in front of the jaws instead',
      pj().held === -1 && pj().pieces.length === 1,
      `held ${pj().held}, ${pj().pieces.length} pieces`);

// Blue's half loads the other colour.
api.reset(61.5, 0, 90);
run(200);
check('blue' + String.fromCharCode(39) + 's half loads blue/yellow',
      pj().pieces[0] && pj().pieces[0].color === 1,
      `colour ${pj().pieces[0] && pj().pieces[0].color}`);

api.setPreload(0);
api.reset(-61.5, 0, 270);
run(100);
check('off, the jaws start empty', pj().pieces.length === 0,
      `${pj().pieces.length} pieces`);
api.pieceClear();

// ---------------------------------------------------------------------------
console.log('\n== the rear aligner ==');
// All-omni drive, so there is no lateral friction for the arms to fight: a
// goal caught in the mouth pulls the robot across until it sits in the throat.
api.pieceClear();
api.toggleReset();
api.reset(-34, -20.5, 90);      // facing +x, so the back faces the goal at -47
run(200);
api.setDrive(-55, -55); run(1400); api.setDrive(0, 0); run(300);
{
  const tr = st()['true'];
  check('backing onto a goal centres the robot on it',
        Math.abs(tr.y + 23.5) < 1.0,
        `y -20.5 -> ${tr.y.toFixed(2)} (goal at -23.5)`);
}

// Turned off, the same approach stays where it was aimed.
api.setParam('alignerWidthIn', 0);
api.reset(-34, -20.5, 90);
run(200);
api.setDrive(-55, -55); run(1400); api.setDrive(0, 0); run(300);
{
  const tr = st()['true'];
  check('and with no aligner fitted it does not', Math.abs(tr.y + 20.5) < 1.0,
        `y -20.5 -> ${tr.y.toFixed(2)}`);
}
api.setParam('alignerWidthIn', 7);

// ---------------------------------------------------------------------------
console.log('\n== toggles ==');
api.toggleReset();
api.reset(0, 48, 0);            // VEX 0 faces +y, at the middle of the north wall
run(200);
check('they start neutral', tg().every(t => t.state === 0), `${tg().length} toggles`);

api.setDrive(70, 70); run(1200); api.setDrive(0, 0); run(200);
check('ramming the front into the wall turns it one face', tgState(0) === 1,
      `north is ${tgState(0)}`);

// Held against it, it must not keep spinning.
api.setDrive(70, 70); run(1500); api.setDrive(0, 0); run(200);
check('leaning on it is still only one flip', tgState(0) === 1,
      `north is ${tgState(0)}`);

// Back off and come in again.
api.setDrive(-70, -70); run(700); api.setDrive(0, 0); run(200);
api.setDrive(70, 70); run(900); api.setDrive(0, 0); run(200);
check('backing off and ramming again turns the next face', tgState(0) === 2,
      `north is ${tgState(0)}`);
check('and the other three are untouched',
      tg().filter(t => t.wall !== 0).every(t => t.state === 0));

// Off the end of the bar is just wall.
api.toggleReset();
api.reset(40, 48, 0);
run(200);
api.setDrive(70, 70); run(1200); api.setDrive(0, 0); run(200);
check('the rest of the wall is not a toggle', tgState(0) === 0,
      `north is ${tgState(0)}`);

// Red's two walls are the same roller mounted a half turn round, so a ram
// walks the colours the other way: yellow, blue, red.
api.toggleReset();
api.reset(0, -48, 180);         // VEX 180 faces -y, at the south wall
run(200);
api.setDrive(70, 70); run(1200); api.setDrive(0, 0); run(200);
check('red side turns to blue first, not red', tgState(2) === 2,
      `south is ${tgState(2)}`);
api.setDrive(-70, -70); run(700); api.setDrive(0, 0); run(200);
api.setDrive(70, 70); run(900); api.setDrive(0, 0); run(200);
check('and red comes second', tgState(2) === 1, `south is ${tgState(2)}`);

// The claw is on the back; reversing into one must not count.
api.toggleReset();
api.reset(0, 48, 180);          // facing -y, so the BACK is against the wall
run(200);
api.setDrive(-70, -70); run(1200); api.setDrive(0, 0); run(200);
check('backing into one does not flip it', tgState(0) === 0,
      `north is ${tgState(0)}`);
api.toggleReset();

// ---------------------------------------------------------------------------
console.log('\n== a compiled routine, on the real starting layout ==');
api.pieceDefault();
const names = [];
for (let i = 0; i < api.autonCount(); i++) names.push(api.autonName(i));
const idx = names.indexOf('Red 3 Pin');
check('the routine is there to run', idx >= 0, names.join(', '));

api.reset(-61.5, 0, 270);
api.setLoc(2);
api.runAuton(idx);

let sawClosed = false, sawHeld = false, maxLift = 0, maxErr = 0;
let guard = 0;
while (api.routeRunning() === 1 && guard++ < 2500) {
  run(20);
  const p = pj();
  if (p.closed) sawClosed = true;
  if (p.held >= 0) sawHeld = true;
  if (p.liftDeg > maxLift) maxLift = p.liftDeg;
  const e = st().err.dist;
  if (e > maxErr) maxErr = e;
}
api.stopAuton();

check('the claw closes during the run', sawClosed);
check('the lift actually travels', maxLift > 100, `${maxLift.toFixed(0)}°`);

check('it has hold of something while the claw is shut', sawHeld,
      `worst pose error during the run ${maxErr.toFixed(1)} in`);

// The one below is RED, and the cause is not in this file. The routine's first
// autons::resetFromBeamsNear() relocates the pose by about 89 in, and every
// motion after it aims at the wrong place. Measured 2026-09-27; it reproduces
// unchanged with the goals back at their old 2.0 in radius and with no pieces
// on the field at all, so it predates the Override layout. solveMapped returns
// an alias of the post lattice and reports it verified, which API.md already
// describes; RclTracking::latestEstimate() also freezes at its last fix when a
// cycle has no votes, so the jump gate that would otherwise catch it is
// measuring against a stale prior.
{
  const scored = pj().pieces.filter(q => q.goal >= 0 && q.level > 0);
  check('a pin ends up in a goal above the bottom level', scored.length > 0,
        scored.map(q => `#${q.id}@goal${q.goal}/L${q.level}`).join(' ') || 'none');
}

api.pieceClear();

console.log(failures === 0 ? '\nALL CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
