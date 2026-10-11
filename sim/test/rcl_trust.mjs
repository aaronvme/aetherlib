// RCL per-sensor reading trust.
//
//   node sim/test/rcl_trust.mjs
//
// The innovation gate (maxWallInnovation, 8 in) only rejects a reading that
// misses the predicted wall by more than 8 in. A game element a few inches in
// front of the wall passes it, and with one sensor on that axis -- the usual
// case, the far one being out of range -- that sensor is the whole axis
// estimate, so it drags the pose onto the element. Trust is per sensor: a run
// of short readings drops it out of the vote and out of the displacement
// evidence, and a run of consistent ones brings it back. Everything is checked
// against ground truth.
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
  teleport: c('sim_teleport', null, ['number', 'number', 'number']),
  actorAdd: c('sim_actor_add', 'number', ['number', 'number', 'number', 'number']),
  actorClear: c('sim_actor_clear', null, []),
  setFar: c('sim_set_far_side_gate', null, ['number', 'number']),
  distFault: c('sim_distance_fault', null, ['number', 'number']),
  pieceClear: c('sim_piece_clear', null, []),
  pieceAdd: c('sim_piece_add', 'number', ['number', 'number', 'number', 'number']),
  pieceRemove: c('sim_piece_remove', null, ['number']),
  piecesJson: c('sim_pieces_json', 'string', []),
};

const WALL = 70.25;           // Field::FIELD_HALF_WALL
const BACK = 2;               // sensor slot ranging -x when facing +x
const CUP = 1;                // sim PieceKind::Cup
const STEP_MS = 5;

const st = () => JSON.parse(api.stateJson());
const run = (ms) => { for (let i = 0; i < ms; i += STEP_MS) api.step(STEP_MS); };
const sensor = (s, i) => s.filters.find(f => f.i === i);
const beam = (s, i) => s.beams.find(b => b.i === i);

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};

const seePieces0 = api.getParam('distSeePieces');

/// RCL only, no field pieces, beams that do see pieces placed by the test.
const begin = (x, y, h) => {
  for (let i = 0; i < 8; i++) api.distFault(i, 0);
  api.actorClear();
  api.setFar(0, 0);
  api.setPods(0);
  api.init();
  api.reset(x, y, h);
  api.pieceClear();
  api.setParam('distSeePieces', 1);
  api.setLoc(2);
  run(600);
};

/// A cup on the back beam's axis whose near face is `shortIn` in front of the
/// west wall.
const cupInBackBeam = (shortIn) => {
  const b = beam(st(), BACK);
  const id = api.pieceAdd(CUP, 0, b.oy, 0);
  const r = JSON.parse(api.piecesJson()).pieces.find(p => p.id === id)?.r ?? 1.58;
  api.pieceRemove(id);
  // The beam points -x, so the face it meets is the east one: centre + r.
  return api.pieceAdd(CUP, -WALL + shortIn - r, b.oy, 0);
};

/// Steps ms, tracking worst error, the back sensor's trust, its votes, and
/// whether a displacement was declared.
const watch = (ms, i = BACK) => {
  let worst = 0, minTrust = Infinity, maxTrust = -Infinity, fresh = 0, voted = 0;
  let displaced = false, displacedMs = 0, lastVoteT = -1;
  for (let t = 0; t < ms; t += STEP_MS) {
    api.step(STEP_MS);
    const s = st();
    worst = Math.max(worst, s.err.dist);
    displaced = displaced || s.rcl2.displaced;
    if (s.rcl2.displaced) displacedMs += STEP_MS;
    const f = sensor(s, i);
    minTrust = Math.min(minTrust, f.trust);
    maxTrust = Math.max(maxTrust, f.trust);
    if (f.fresh) {
      fresh++;
      if (f.rclValid) { voted++; lastVoteT = t; }
    }
  }
  return { worst, minTrust, maxTrust, fresh, voted, displaced, displacedMs, lastVoteT,
           trust: sensor(st(), i).trust, err: st().err.dist };
};

api.init();

// ---------------------------------------------------------------------------
console.log('\n== clean walls: trust stays full and tracking is unchanged ==');
begin(-48, -60, 90);
{
  let minTrust = Infinity;
  api.setDrive(70, 70);
  for (let t = 0; t < 1500; t += STEP_MS) {
    api.step(STEP_MS);
    for (const f of st().filters) if (f.fresh && f.rclValid) minTrust = Math.min(minTrust, f.trust);
  }
  api.setDrive(0, 0); run(800);
  const err = st().err.dist;
  check('RCL alone tracks a straight drive', err < 1.5, `${err.toFixed(2)} in`);
  check('no voting sensor lost trust on clean walls', minTrust >= 0.99,
        `min trust while voting ${minTrust.toFixed(2)}`);
}
begin(-48, -48, 90);
{
  const w = watch(2000);
  check('sitting still, the back sensor keeps full trust and votes',
        w.minTrust >= 0.99 && w.voted > 0.8 * w.fresh,
        `trust ${w.minTrust.toFixed(2)}, voted ${w.voted}/${w.fresh}`);
  check('and the pose holds', w.worst < 0.5, `${w.worst.toFixed(2)} in`);
}

// ---------------------------------------------------------------------------
console.log('\n== a game element 5 in in front of the wall, robot sitting ==');
// At (-48,-48) facing +x the back beam is the only x reading (the front one is
// out of range). A cup 5 in off the west wall shortens it by 5 in: inside the
// 8 in gate, so without trust it is a clean wall hit and the whole x axis.
begin(-48, -48, 90);
{
  const clearRange = beam(st(), BACK).r;
  const err0 = st().err.dist;
  const id = cupInBackBeam(5.0);
  run(100);
  const blocked = beam(st(), BACK).r;
  check('the beam really stops on the cup, inside the innovation gate',
        clearRange - blocked > 3.5 && clearRange - blocked < 7.5,
        `${clearRange.toFixed(1)} in -> ${blocked.toFixed(1)} in`);
  const w = watch(1900);
  check('the pose stays within 1 in of the truth', w.worst < err0 + 1.0,
        `${err0.toFixed(2)} in -> worst ${w.worst.toFixed(2)} in`);
  check("the sensor's trust drops below the floor", w.minTrust < 0.5,
        `min trust ${w.minTrust}`);
  check('and it stops voting', w.voted <= 3,
        `voted ${w.voted} of ${w.fresh} fresh samples`);

  api.pieceRemove(id);
  const back = watch(1500);
  check('with the cup gone, trust comes back', back.trust >= 0.9,
        `trust ${back.trust}`);
  check('not instantly: the first ~150 ms of clean readings are a hold',
        back.minTrust < 0.5, `min trust after removal ${back.minTrust}`);
  check('and the sensor votes again', back.voted > 0.5 * back.fresh,
        `voted ${back.voted}/${back.fresh}`);
  check('and the pose is still right', back.err < 0.5, `${back.err.toFixed(2)} in`);
}

// ---------------------------------------------------------------------------
console.log('\n== driving away from the wall with the element still in the beam ==');
// The cup stays on the beam's axis as the robot drives along it, so the
// reading grows with the wall's and stays short by the same amount.
begin(-56, -48, 90);
{
  const err0 = st().err.dist;
  cupInBackBeam(5.5);
  run(300);
  api.setDrive(45, 45);
  const w = watch(1200);
  api.setDrive(0, 0);
  const s = watch(600);
  check('the pose stays within 1 in of the truth while driving',
        Math.max(w.worst, s.worst) < err0 + 1.0,
        `${err0.toFixed(2)} in -> worst ${Math.max(w.worst, s.worst).toFixed(2)} in`);
  check('the blocked sensor stays out of the vote', w.voted + s.voted <= 3,
        `voted ${w.voted + s.voted} of ${w.fresh + s.fresh}`);
}

// ---------------------------------------------------------------------------
console.log('\n== a robot parked in front of one sensor for 2 s ==');
// At (-24,-48) facing +x the back beam is again the only x reading, and a
// robot pulled up behind reads ~40 in short. That is gross enough to have
// counted as displacement evidence on its own.
begin(-24, -48, 90);
{
  const err0 = st().err.dist;
  api.actorAdd(-38, -48, 90, 0);
  const w = watch(2000);
  check('the pose stays within 1 in of the truth', w.worst < err0 + 1.0,
        `${err0.toFixed(2)} in -> worst ${w.worst.toFixed(2)} in`);
  check("the sensor's trust drops below the floor", w.minTrust < 0.5,
        `min trust ${w.minTrust}`);
  // The onset is gross enough to count for displaceCycles samples (see
  // rcl_guard.mjs), so one short recovery is declared; the distrusted beam
  // cannot vote in it. This read "never declared" while a displacement was
  // cleared in the same update() that declared it.
  check('a displacement from that one beam clears within 300 ms',
        w.displacedMs <= 300 && !st().rcl2.displaced, `${w.displacedMs} ms displaced`);
  api.actorClear();
  const back = watch(1000);
  check('after it leaves, trust recovers', back.trust >= 0.9, `trust ${back.trust}`);
  check('and the sensor contributes again', back.voted > 0.5 * back.fresh,
        `voted ${back.voted}/${back.fresh}`);
  check('and the pose is still right', back.err < 0.5, `${back.err.toFixed(2)} in`);
}

// ---------------------------------------------------------------------------
console.log('\n== real pose errors are still corrected ==');
// A shove the wheels never saw. Away from the wall the back beam reads long:
// that cannot be an obstruction, so it costs no trust and is corrected. A
// small one toward the wall is inside the consistent band.
const shove = (dx, label) => {
  begin(-48, -48, 90);
  const t = st().true;
  api.teleport(t.x + dx, t.y, 90);
  run(50);
  const before = st().err.dist;
  const w = watch(2500);
  check(label, w.err < 0.5 && w.minTrust >= 0.99,
        `${before.toFixed(2)} in -> ${w.err.toFixed(2)} in, min trust ${w.minTrust.toFixed(2)}`);
};
shove(6.0, 'a 6 in shove away from the wall is corrected, trust untouched');
shove(-2.0, 'a 2 in shove toward it is corrected, trust untouched');

api.setParam('distSeePieces', seePieces0);
console.log(failures === 0 ? '\nALL CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
