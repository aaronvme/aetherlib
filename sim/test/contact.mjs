// Wall and post contact on an all-omni drive, against what the real robot does.
//
//   node sim/test/contact.mjs
//
// Hit a wall at an angle and the corner that lands first pivots the body flush
// (square-up), the contact scrubs along the surface instead of gliding
// (Coulomb drag, wallFriction), the slide dies once the face is flat, and a
// fast hit knocks the body back a little before it settles (wallRestitution).
// The rotation is real, so the IMU sees it; every displacement is real, so the
// tracking pods see it. glide.mjs keeps the frictionless model reachable with
// the three params at 0.
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
const WALL = 70.2;
const POST_R = 2.85;
const POSTS = [[-47, -23.5], [-23.5, -47], [23.5, 47], [47, 23.5], [0, 0],
               [-47, 23.5], [23.5, -47], [-23.5, 47], [47, -23.5]];
const PARAMS = ['wallFriction', 'squareUpRate', 'wallRestitution'];

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
const DEFAULTS = Object.fromEntries(PARAMS.map(k => [k, api.getParam(k)]));
/// Free wheel surface speed at v volts, in/s.
const speedAt = (v) => v / 12 * api.getParam('maxWheelRpm') / 60 *
                       Math.PI * api.getParam('wheelDiameterIn');

/// Odometry only, no pieces. pods: 0 = the robot's tracking wheels. over: param
/// overrides on top of the defaults.
const begin = (x, y, h, over = {}) => {
  api.actorClear();
  api.setPods(0);
  api.setParam('wallSlideFrac', 1);
  for (const k of PARAMS) api.setParam(k, k in over ? over[k] : DEFAULTS[k]);
  api.reset(x, y, h);
  api.pieceClear();
  api.setLoc(0);
  api.setDrive(0, 0);
  run(200);
};

const axes = (hDeg) => {
  const t = (90 - hDeg) * Math.PI / 180;
  return { fx: Math.cos(t), fy: Math.sin(t) };
};
/// Signed smallest angle a - b, degrees.
const dAng = (a, b) => ((a - b + 540) % 360) - 180;

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
/// Overlap bound: the state JSON rounds h to 0.01 deg, ~0.0013 in at a corner.
const NO_OVERLAP = -2e-3;
/// In contact. `colliding` is the last 1 ms physics tick of each 5 ms sample,
/// so a hit that bounces straight off can be missed by it.
const touching = (p) => p.hit || p.clear < 0.02;
const lastMs = (tr, ms) => tr.slice(-Math.round(ms / STEP_MS));

// ---------------------------------------------------------------------------
console.log('\n== contact params ==');
for (const k of PARAMS) {
  check(`${k} is a sim param with a positive default`,
        Number.isFinite(DEFAULTS[k]) && DEFAULTS[k] > 0, `${DEFAULTS[k]}`);
}
api.setParam('wallFriction', 0.25);
check('reads back what was set', Math.abs(api.getParam('wallFriction') - 0.25) < 1e-6);
api.setParam('wallFriction', DEFAULTS.wallFriction);

// ---------------------------------------------------------------------------
// Heading 300: 30 deg off square to the west wall. The front-left corner lands
// first and the reaction there turns the body CCW until the front face is flat.
console.log('\n== 30 deg off square into the west wall: squares up and stops ==');
{
  const V = 4, v = speedAt(V);
  begin(-50, -5, 300);
  const tr = trace(1500, V, V);
  const first = tr.findIndex(touching);
  const contact = first < 0 ? [] : tr.slice(first);
  check('reaches the wall', first >= 0 && contact.length > 100);
  if (contact.length > 100) {
    const end = tr[tr.length - 1];
    check('rotates to flush (heading 270) within 2 deg', Math.abs(dAng(end.h, 270)) < 2,
          `h ${tr[first].h.toFixed(1)} -> ${end.h.toFixed(2)}`);
    const off = contact.map(p => Math.abs(dAng(p.h, 270)));
    const flushAt = off.findIndex(o => o < 2);
    check('squares up within 0.5 s of contact', flushAt >= 0 && flushAt * STEP_MS <= 500,
          `${flushAt * STEP_MS} ms`);
    check('turns only toward flush, no overshoot or jitter',
          off.every((o, i) => i === 0 || o <= off[i - 1] + 0.02),
          `max step back ${Math.max(0, ...off.slice(1).map((o, i) => o - off[i])).toFixed(3)} deg`);
    // The leading corner is the pivot: it barely moves along the wall, where a
    // frictionless glide would carry it at half the drive speed.
    const corner = (p) => {
      const { fx, fy } = axes(p.h);
      return p.y + HALF_L * fy + HALF_W * fx;   // front-left, y only
    };
    const cornerSlide = Math.abs(corner(contact[contact.length - 1]) - corner(contact[0]));
    check('contact corner scrubs, it does not glide',
          cornerSlide < 1.5, `${cornerSlide.toFixed(2)} in along the wall ` +
          `(frictionless would be ~${(0.5 * v * 0.3).toFixed(1)} in in 0.3 s)`);
    const tail = lastMs(tr, 500);
    check('once flat, no creep along the wall', span(tail.map(p => p.y)) < 0.02,
          `y span ${span(tail.map(p => p.y)).toFixed(4)} in over 0.5 s`);
    check('pinned flush against the wall',
          Math.abs(end.x - (-WALL + HALF_L)) < 0.05 && span(tail.map(p => p.x)) < 0.005,
          `x=${end.x.toFixed(3)}, flush ${(-WALL + HALF_L).toFixed(3)}`);
    check('body never overlaps the wall', minClear(tr) > NO_OVERLAP,
          `min clearance ${minClear(tr).toFixed(4)} in`);
    check('pods and IMU follow the pivot',
          worst(tr, p => p.err) < 1.0 && worst(tr, p => Math.abs(p.hErr)) < 1.0,
          `worst ${worst(tr, p => p.err).toFixed(3)} in, ` +
          `heading ${worst(tr, p => Math.abs(p.hErr)).toFixed(3)} deg`);
  }
}

// ---------------------------------------------------------------------------
// Heading 330: 30 deg off parallel. The same corner lands, but the reaction
// now turns the body CW, laying the left face along the wall; it then drives
// on along it. Friction makes the run in slower than a glide would be.
console.log('\n== 30 deg off parallel into the west wall: lays flat, scrubs ==');
{
  const V = 4, v = speedAt(V);
  const along = (over) => {
    begin(-56, -12, 330, over);
    const tr = trace(1800, V, V);
    const first = tr.findIndex(touching);
    return { tr, first };
  };
  const { tr, first } = along({});
  const zero = along({ wallFriction: 0, squareUpRate: 0, wallRestitution: 0 });
  check('reaches the wall', first >= 0 && first < tr.length - 100);
  if (first >= 0 && zero.first >= 0) {
    const end = tr[tr.length - 1];
    check('ends parallel to the wall (heading 0) within 2 deg',
          Math.abs(dAng(end.h, 0)) < 2, `h ${tr[first].h.toFixed(1)} -> ${end.h.toFixed(2)}`);
    const win = Math.round(300 / STEP_MS);
    const prog = (t, f) => t[Math.min(f + win, t.length - 1)].y - t[f].y;
    const withF = prog(tr, first), glide = prog(zero.tr, zero.first);
    check('slower along the wall than the frictionless glide while it squares up',
          withF < 0.85 * glide,
          `${withF.toFixed(2)} vs ${glide.toFixed(2)} in in the first 0.3 s of contact`);
    const sp = tr.slice(first + 1).map((p, i) => (p.y - tr[first + i].y) / (STEP_MS / 1000));
    check('never faster along the wall than it drives', Math.max(...sp) <= v * 1.02,
          `peak ${Math.max(...sp).toFixed(1)} in/s, drive ${v.toFixed(1)} in/s`);
    // Not yet exactly flush (the turn slows as the push into the wall fades),
    // so the side is on the wall rather than at the square-on x.
    check('drives on with its side on the wall', end.clear < 0.02 && end.x < -62,
          `x=${end.x.toFixed(3)}, clearance ${end.clear.toFixed(4)} in`);
    check('body never overlaps the wall', minClear(tr) > NO_OVERLAP,
          `min clearance ${minClear(tr).toFixed(4)} in`);
    check('pods and IMU follow it',
          worst(tr, p => p.err) < 1.0 && worst(tr, p => Math.abs(p.hErr)) < 1.0,
          `worst ${worst(tr, p => p.err).toFixed(3)} in, ` +
          `heading ${worst(tr, p => Math.abs(p.hErr)).toFixed(3)} deg`);
  }
}

// ---------------------------------------------------------------------------
console.log('\n== square hits: a fast one bounces, a slow one does not ==');
const squareHit = (V, ms = 1500) => {
  begin(-35, 0, 270);
  const tr = trace(ms, V, V);
  const flush = -WALL + HALF_L;
  // A fast hit can bounce clear within one sample, so contact is "within an
  // inch", and the knock-back is the largest retreat from the deepest point.
  const first = tr.findIndex(p => p.x - flush < 1.0);
  const after = first < 0 ? [] : tr.slice(first);
  const retreat = (f) => {
    let low = Infinity, best = 0;
    for (const p of after) { low = Math.min(low, f(p)); best = Math.max(best, f(p) - low); }
    return after.length ? best : NaN;
  };
  return { tr, first, flush, after, rebound: retreat(p => p.x), odomRebound: retreat(p => p.ox) };
};
{
  const { tr, first, flush, after, rebound, odomRebound } = squareHit(12);
  check('full speed reaches the wall', first >= 0 && after.length > 150);
  if (after.length > 150) {
    check('knocks back a bounded small amount', rebound >= 0.3 && rebound <= 1.5,
          `${rebound.toFixed(3)} in`);
    const tail = lastMs(tr, 300);
    check('then settles flush, no jitter',
          span(tail.map(p => p.x)) < 0.005 && Math.abs(tail[tail.length - 1].x - flush) < 0.02,
          `x span ${span(tail.map(p => p.x)).toFixed(4)}, x=${tail[tail.length - 1].x.toFixed(3)}`);
    check('settles within 0.5 s of the hit',
          after.slice(Math.round(500 / STEP_MS)).every(p => Math.abs(p.x - flush) < 0.02));
    check('square hit does not rotate', span(after.map(p => p.h)) < 0.05,
          `${span(after.map(p => p.h)).toFixed(3)} deg`);
    check('body never overlaps the wall', minClear(tr) > NO_OVERLAP,
          `min clearance ${minClear(tr).toFixed(4)} in`);
    check('the pods see the knock-back', odomRebound > 0.5 * rebound,
          `odom ${odomRebound.toFixed(3)} of ${rebound.toFixed(3)} in`);
    const settled = tr[tr.length - 1];
    check('odometry agrees once settled', settled.err < 0.3, `${settled.err.toFixed(3)} in`);
  }
}
{
  const slow = squareHit(2.5, 2500);
  check('slow contact barely bounces', slow.first >= 0 && slow.rebound < 0.1,
        `${slow.rebound.toFixed(3)} in at ${speedAt(2.5).toFixed(1)} in/s`);
}

// ---------------------------------------------------------------------------
console.log('\n== goal posts ==');
{
  const y0 = HALF_W + POST_R - 1.4;
  begin(-20, y0, 90);
  const tr = trace(2500, 4, 4);
  const last = tr[tr.length - 1];
  check('glancing hit contacts the post', tr.some(touching));
  check('slides round it and gets past', last.x > 8 && last.y >= y0 + 1.0,
        `end ${last.x.toFixed(2)}, ${last.y.toFixed(2)}`);
  check('never overlaps the post', minClear(tr) > NO_OVERLAP,
        `min clearance ${minClear(tr).toFixed(4)} in`);
  check('pods follow it', worst(tr, p => p.err) < 1.0,
        `worst ${worst(tr, p => p.err).toFixed(3)} in`);
}
{
  begin(-30, HALF_W + POST_R - 0.5, 90);
  const tr = trace(1500, 12, 12);
  const end = tr[tr.length - 1];
  check('full speed glance gets round without overlapping',
        minClear(tr) > NO_OVERLAP && tr.some(touching) && end.x > 10,
        `min clearance ${minClear(tr).toFixed(4)} in, end ${end.x.toFixed(1)}, ${end.y.toFixed(1)}`);
  begin(-20, 0, 90);
  const head = trace(1500, 12, 12);
  const e = head[head.length - 1];
  check('dead-centre hit at full speed stops on the post, no overlap',
        e.x < 0 && minClear(head) > NO_OVERLAP && span(lastMs(head, 300).map(p => p.x)) < 0.005,
        `x=${e.x.toFixed(2)}, min clearance ${minClear(head).toFixed(4)}`);
}

// ---------------------------------------------------------------------------
console.log('\n== corner: squares into it and still gets out ==');
{
  const wedge = () => {
    begin(-50, -50, 240);
    const tr = trace(3000, 6, 6);
    return { tr, end: tr[tr.length - 1] };
  };
  let { tr, end } = wedge();
  check('ends in the south-west corner, square to the west wall',
        end.hit && end.x < -62 && end.y < -62 && Math.abs(dAng(end.h, 270)) < 2,
        `at ${end.x.toFixed(2)}, ${end.y.toFixed(2)}, h ${end.h.toFixed(2)}`);
  check('never overlaps either wall', minClear(tr) > NO_OVERLAP,
        `min clearance ${minClear(tr).toFixed(4)} in`);
  const back = trace(500, -6, -6);
  const out = back[back.length - 1];
  check('reverses out', Math.hypot(out.x - end.x, out.y - end.y) > 8 && !out.hit,
        `to ${out.x.toFixed(2)}, ${out.y.toFixed(2)}`);
  ({ end } = wedge());
  const turn = trace(800, -6, 6);
  check('can turn while square in the corner',
        Math.abs(dAng(turn[turn.length - 1].h, end.h)) > 5 && minClear(turn) > NO_OVERLAP,
        `h ${end.h.toFixed(1)} -> ${turn[turn.length - 1].h.toFixed(1)}, ` +
        `min clearance ${minClear(turn).toFixed(4)}`);
  // The same turn in open field reads up to 1.6 deg behind (IMU sample lag at
  // ~230 deg/s), so the bound is that plus margin, not the 1 deg used above.
  check('pods and IMU follow the turn out',
        worst(turn, p => p.err) < 1.0 && worst(turn, p => Math.abs(p.hErr)) < 2.5,
        `worst ${worst(turn, p => p.err).toFixed(3)} in, heading ${worst(turn, p => Math.abs(p.hErr)).toFixed(3)} deg`);
}

// ---------------------------------------------------------------------------
console.log('\n== flush on a wall, a turn in place walks off it ==');
{
  begin(-50, 0, 270);
  trace(800, 4, 4);
  const turn = trace(600, -5, 5);
  const e = turn[turn.length - 1];
  check('turns', Math.abs(dAng(e.h, 270)) > 10 && minClear(turn) > NO_OVERLAP,
        `h -> ${e.h.toFixed(1)}, min clearance ${minClear(turn).toFixed(4)}`);
}

for (const k of PARAMS) api.setParam(k, DEFAULTS[k]);
api.setDrive(0, 0);

console.log(failures ? `\n${failures} FAILED` : '\nALL PASSED');
process.exit(failures ? 1 : 0);
