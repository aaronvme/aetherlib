// Wall contact on an all-omni drive: the body slides along what it hits.
//
//   node sim/test/glide.mjs
//
// The drive is all omni wheels, so nothing resists motion along a wall: an
// angled push keeps the component along the surface and loses only the one
// into it. These cases check that against ground truth, that the tracking pods
// see the slide (the vertical one the body-forward part, the horizontal one
// the lateral part), that the body never overlaps a wall or a post, that a
// wedged robot still gets out, and that wallSlideFrac = 0 is the old hard stop.
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
  stateJson: c('sim_state_json', 'string', []),
  setParam: c('sim_set_param', null, ['string', 'number']),
  getParam: c('sim_get_param', 'number', ['string']),
  actorClear: c('sim_actor_clear', null, []),
  pieceClear: c('sim_piece_clear', null, []),
};

const STEP_MS = 5;
// Field::loadDefault(): perimeter at +-70.2 in, nine goals of radius 2.85.
const WALL = 70.2;
const POST_R = 2.85;
const POSTS = [[-47, -23.5], [-23.5, -47], [23.5, 47], [47, 23.5], [0, 0],
               [-47, 23.5], [23.5, -47], [-23.5, 47], [47, -23.5]];

const st = () => JSON.parse(api.stateJson());
const run = (ms) => { for (let i = 0; i < ms; i += STEP_MS) api.step(STEP_MS); };

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};

api.init();
const HALF_L = 0.5 * api.getParam('robotLengthIn');
const HALF_W = 0.5 * api.getParam('robotWidthIn');

/// Odometry only (no wall resets), no pieces. pods: 0 = the robot's tracking
/// wheels, 2 = drive encoders only. Friction, square-up and bounce are off:
/// this suite is the frictionless glide; contact.mjs covers the defaults.
const CONTACT = ['wallFriction', 'squareUpRate', 'wallRestitution'];
const begin = (x, y, h, pods = 0, slideFrac = 1) => {
  api.actorClear();
  api.setPods(pods);
  api.setParam('wallSlideFrac', slideFrac);
  for (const k of CONTACT) api.setParam(k, 0);
  api.reset(x, y, h);
  api.pieceClear();
  api.setLoc(0);
  api.setDrive(0, 0);
  run(200);
};

/// VEX heading (0 = +y, clockwise) -> body axes in the world.
const axes = (hDeg) => {
  const t = (90 - hDeg) * Math.PI / 180;
  return { fx: Math.cos(t), fy: Math.sin(t) };
};

/// Smallest clearance between the body rectangle and the perimeter or any
/// goal; negative means the body overlaps something.
const clearance = (s) => {
  const { fx, fy } = axes(s.true.h);
  const ex = HALF_L * Math.abs(fx) + HALF_W * Math.abs(fy);
  const ey = HALF_L * Math.abs(fy) + HALF_W * Math.abs(fx);
  let best = Math.min(WALL - (s.true.x + ex), (s.true.x - ex) + WALL,
                      WALL - (s.true.y + ey), (s.true.y - ey) + WALL);
  for (const [px, py] of POSTS) {
    const dx = px - s.true.x, dy = py - s.true.y;
    const lx = dx * fx + dy * fy, ly = -dx * fy + dy * fx;
    const qx = Math.max(-HALF_L, Math.min(HALF_L, lx));
    const qy = Math.max(-HALF_W, Math.min(HALF_W, ly));
    best = Math.min(best, Math.hypot(lx - qx, ly - qy) - POST_R);
  }
  return best;
};

/// Steps ms under a constant drive, sampling truth and odometry every tick.
const trace = (ms, l, r) => {
  api.setDrive(l, r);
  const out = [];
  for (let t = 0; t < ms; t += STEP_MS) {
    api.step(STEP_MS);
    const s = st();
    out.push({ x: s.true.x, y: s.true.y, h: s.true.h, ox: s.odom.x, oy: s.odom.y,
               err: s.err.dist, hErr: s.err.heading, hit: s.colliding,
               clear: clearance(s) });
  }
  return out;
};
const span = (a) => Math.max(...a) - Math.min(...a);
const worst = (tr, f) => Math.max(...tr.map(f));
const minClear = (tr) => Math.min(...tr.map(p => p.clear));

// ---------------------------------------------------------------------------
console.log('\n== wallSlideFrac is a sim param, omni by default ==');
check('defaults to 1 (all omni)', api.getParam('wallSlideFrac') === 1,
      `${api.getParam('wallSlideFrac')}`);

// ---------------------------------------------------------------------------
// Heading 330: 30 deg off the wall, driving north-west into the west wall. The
// lane x ~ -60, y in [-12, 15] stays clear of the goals at (-47, +-23.5).
const GLIDE = [-56, -12, 330];
const glideRun = (pods, frac = 1) => {
  begin(...GLIDE, pods, frac);
  const tr = trace(1800, 4, 4);
  api.setDrive(0, 0);
  const first = tr.findIndex(p => p.hit);
  return { tr, first, contact: first < 0 ? [] : tr.slice(first + 2) };
};

console.log('\n== 30 deg into the west wall glides along it ==');
{
  const { tr, first, contact } = glideRun(0);
  check('reaches the wall', first >= 0 && contact.length > 100,
        first < 0 ? 'never touched' : `contact at x=${tr[first].x.toFixed(2)}`);
  if (contact.length > 1) {
    const { fx, fy } = axes(GLIDE[2]);
    const ex = HALF_L * Math.abs(fx) + HALF_W * Math.abs(fy);
    const pinned = -WALL + ex;
    const xs = contact.map(p => p.x);
    const dy = contact[contact.length - 1].y - contact[0].y;
    const secs = contact.length * STEP_MS / 1000;
    check('keeps moving along the wall', dy > 8 * secs,
          `${dy.toFixed(2)} in north in ${secs.toFixed(2)} s on the wall`);
    check('perpendicular coordinate pinned at the wall',
          span(xs) < 0.1 && Math.abs(xs[0] - pinned) < 0.3,
          `x ${Math.min(...xs).toFixed(3)}..${Math.max(...xs).toFixed(3)}, ` +
          `wall-flush ${pinned.toFixed(3)}`);
    check('heading unchanged', span(contact.map(p => p.h)) < 0.05,
          `${span(contact.map(p => p.h)).toFixed(3)} deg`);
    check('body never overlaps the wall', minClear(tr) > -1e-3,
          `min clearance ${minClear(tr).toFixed(4)} in`);
    // The drive wheels roll along the heading; the slide is lateral to it, so
    // only the horizontal pod can see it.
    check('odometry on the tracking pods follows the slide',
          worst(tr, p => p.err) < 1.0,
          `worst ${worst(tr, p => p.err).toFixed(3)} in, ` +
          `heading err ${worst(tr, p => Math.abs(p.hErr)).toFixed(3)} deg`);
  }
}
{
  const { tr, contact } = glideRun(2);
  const dy = contact.length ? contact[contact.length - 1].y - contact[0].y : 0;
  check('drive encoders alone do not see the slide (no lateral wheel)',
        dy > 5 && tr[tr.length - 1].err > 3,
        `${dy.toFixed(2)} in slid, err ${tr[tr.length - 1].err.toFixed(2)} in`);
}

// ---------------------------------------------------------------------------
console.log('\n== square-on push stops dead, no jitter ==');
{
  begin(-50, 0, 270);
  const tr = trace(1500, 4, 4);
  const first = tr.findIndex(p => p.hit);
  const held = first < 0 ? [] : tr.slice(first + 2);
  check('reaches the wall', held.length > 100);
  if (held.length > 1) {
    check('x and y do not move while pushing',
          span(held.map(p => p.x)) < 0.01 && span(held.map(p => p.y)) < 0.01,
          `x span ${span(held.map(p => p.x)).toFixed(4)}, ` +
          `y span ${span(held.map(p => p.y)).toFixed(4)}`);
    check('flush with the wall', Math.abs(held[0].x - (-WALL + HALF_L)) < 0.3,
          `x=${held[0].x.toFixed(3)}`);
    check('heading unchanged', span(held.map(p => p.h)) < 0.01);
  }
  check('body never overlaps the wall', minClear(tr) > -1e-3);
}

// ---------------------------------------------------------------------------
console.log('\n== glancing a goal post slides round it ==');
{
  // Heading east past the centre goal with the body's right edge 1.4 in inside
  // its radius: the corner catches the post and has to slide off it.
  const y0 = HALF_W + POST_R - 1.4;
  begin(-20, y0, 90);
  const tr = trace(2000, 4, 4);
  const hit = tr.some(p => p.hit);
  const last = tr[tr.length - 1];
  check('contacts the post', hit);
  check('gets past it', last.x > 12, `x=${last.x.toFixed(2)} y=${last.y.toFixed(2)}`);
  check('pushed off sideways, not through', last.y >= y0 + 1.0,
        `y ${y0.toFixed(2)} -> ${last.y.toFixed(2)}`);
  check('never overlaps the post', minClear(tr) > -1e-3,
        `min clearance ${minClear(tr).toFixed(4)} in`);
  check('pods follow it', worst(tr, p => p.err) < 1.0,
        `worst ${worst(tr, p => p.err).toFixed(3)} in`);
}
{
  // Full speed, corner just catching the post: the tangent step must not
  // tunnel into it at any tick.
  begin(-30, HALF_W + POST_R - 0.5, 90);
  const tr = trace(1500, 12, 12);
  const end = tr[tr.length - 1];
  check('full speed glance gets round without overlapping',
        minClear(tr) > -1e-3 && tr.some(p => p.hit) && end.x > 10,
        `min clearance ${minClear(tr).toFixed(4)} in, ` +
        `end ${end.x.toFixed(1)}, ${end.y.toFixed(1)}`);
  // A face on the post: a differential drive moves along its heading, which
  // is the face normal, so there is nothing to slide along.
  begin(-20, 0, 90);
  const head = trace(1500, 12, 12);
  check('dead-centre hit stops on the post', head[head.length - 1].x < 0 &&
        minClear(head) > -1e-3, `x=${head[head.length - 1].x.toFixed(2)}`);
}

// ---------------------------------------------------------------------------
console.log('\n== wedged in a corner, it can still get out ==');
{
  // Heading 200: mostly south, a little west, so it glides west along the
  // south wall into the corner and stops there.
  const wedge = () => {
    begin(-50, -50, 200);
    const tr = trace(2500, 6, 6);
    return { tr, end: tr[tr.length - 1] };
  };
  let { tr, end } = wedge();
  check('wedged in the south-west corner', end.hit && end.x < -57 && end.y < -57,
        `at ${end.x.toFixed(2)}, ${end.y.toFixed(2)}`);
  check('never overlaps either wall', minClear(tr) > -1e-3);
  const back = trace(500, -6, -6);
  const out = back[back.length - 1];
  check('reverses out', Math.hypot(out.x - end.x, out.y - end.y) > 8 && !out.hit,
        `to ${out.x.toFixed(2)}, ${out.y.toFixed(2)}`);
  ({ end } = wedge());
  const turn = trace(800, -6, 6);
  check('can turn while wedged', Math.abs(turn[turn.length - 1].h - end.h) > 5 &&
        minClear(turn) > -1e-3, `h ${end.h.toFixed(1)} -> ${turn[turn.length - 1].h.toFixed(1)}`);
}

// ---------------------------------------------------------------------------
console.log('\n== wallSlideFrac = 0 is the old hard stop ==');
{
  const { contact } = glideRun(0, 0);
  const dy = contact.length ? contact[contact.length - 1].y - contact[0].y : NaN;
  check('stops dead on the wall', contact.length > 100 && Math.abs(dy) < 0.05,
        `${dy.toFixed(3)} in along the wall`);
  check('reads back', api.getParam('wallSlideFrac') === 0);
}
{
  const { contact } = glideRun(0, 0.5);
  const full = glideRun(0, 1).contact;
  const rate = (cs) => (cs[cs.length - 1].y - cs[0].y) / cs.length;
  check('0.5 slides at about half the rate', contact.length > 10 && full.length > 10 &&
        Math.abs(rate(contact) / rate(full) - 0.5) < 0.1,
        `${(rate(contact) / rate(full)).toFixed(3)} of omni`);
}
api.setParam('wallSlideFrac', 1);
api.setDrive(0, 0);

console.log(failures ? `\n${failures} FAILED` : '\nALL PASSED');
process.exit(failures ? 1 : 0);
