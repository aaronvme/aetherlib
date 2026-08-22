// Checks the experimental MPC/RTI follower and solveTvlqr still do what the
// commit message claims. Run with: node sim/test/mpc.mjs
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
const here = dirname(fileURLToPath(import.meta.url));
const createChadSim = (await import('file://' + join(here, '..', 'web', 'chad_sim.mjs'))).default;

const M = await createChadSim();
// sim/web/chad_sim.* are untracked build output, so they survive a branch switch
// and a build from a branch without MPC leaves a binary that silently lacks it.
if (typeof M._sim_mpc_run !== 'function') {
  console.error('The built sim has no MPC exports.');
  console.error('sim/web/chad_sim.* is untracked build output, so switching');
  console.error('branches does not rebuild it. Run ./sim/build.sh and retry.');
  process.exit(1);
}
const c = (n, r, a) => M.cwrap(n, r, a);
const api = {
  init: c('sim_init', null, []), reset: c('sim_reset', null, ['number','number','number']),
  step: c('sim_step', null, ['number']), setLoc: c('sim_set_localization', null, ['number']),
  stateJson: c('sim_state_json', 'string', []), setParam: c('sim_set_param', null, ['string','number']),
  collideTicks: c('sim_collide_ticks', 'number', []),
  trajClear: c('sim_mpc_traj_clear', null, []),
  trajPush: c('sim_mpc_traj_push', null, ['number','number','number','number','number']),
  defaults: c('sim_mpc_defaults', null, []),
  set: c('sim_mpc_set', null, ['string','number']),
  run: c('sim_mpc_run', null, []), running: c('sim_mpc_running', 'number', []),
  bench: c('sim_mpc_bench_json', 'string', ['number','number','number','number']),
  check: c('sim_mpc_check_json', 'string', ['number']),
};
api.init();
const st = () => JSON.parse(api.stateJson());

let fails = 0;
const ok = (cond, label, detail) => {
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
  if (!cond) fails++;
};

const PATHS = {
  'gentle S': u => [-58 + 52*u, -52 + 13*Math.sin(u*Math.PI)],
  'hairpin':  u => [-56 + 30*Math.sin(u*Math.PI*0.95), -58 + 40*u],
};

// The follower indexes its reference by time, so the trajectory has to be
// sampled at dt. Feeding a geometric path straight in races the robot.
function timeTrajectory(pts, dt = 0.01, cruise = 40, accel = 50, latAccel = 60) {
  const n = pts.length, cum = [0];
  for (let i = 1; i < n; i++)
    cum.push(cum[i-1] + Math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1]));
  const total = cum[n-1], curv = new Array(n).fill(0);
  for (let i = 1; i < n-1; i++) {
    const [ax,ay]=pts[i-1], [bx,by]=pts[i], [cx,cy]=pts[i+1];
    const a=Math.hypot(bx-ax,by-ay), b=Math.hypot(cx-bx,cy-by), d=Math.hypot(cx-ax,cy-ay);
    const cross=(bx-ax)*(cy-ay)-(by-ay)*(cx-ax);
    curv[i] = (a*b*d > 1e-9) ? 2*cross/(a*b*d) : 0;
  }
  curv[0]=curv[1]; curv[n-1]=curv[n-2];
  const v = curv.map(k => Math.min(cruise, Math.abs(k)>1e-6 ? Math.sqrt(latAccel/Math.abs(k)) : cruise));
  v[0]=0; v[n-1]=0;
  for (let i=1;i<n;i++) v[i]=Math.min(v[i], Math.sqrt(v[i-1]**2 + 2*accel*(cum[i]-cum[i-1])));
  for (let i=n-2;i>=0;i--) v[i]=Math.min(v[i], Math.sqrt(v[i+1]**2 + 2*accel*(cum[i+1]-cum[i])));
  const out = []; let s = 0;
  for (let step = 0; step < 2000; step++) {
    let lo=0, hi=n-1;
    if (s >= total) { lo = n-2; }
    else while (hi-lo > 1) { const m=(lo+hi)>>1; if (cum[m] <= s) lo=m; else hi=m; }
    const seg = cum[lo+1]-cum[lo], f = s>=total ? 1 : (seg>1e-9 ? (s-cum[lo])/seg : 0);
    const j = Math.min(lo+1,n-1), k = Math.max(lo-1,0);
    const x = pts[lo][0] + f*(pts[lo+1][0]-pts[lo][0]);
    const y = pts[lo][1] + f*(pts[lo+1][1]-pts[lo][1]);
    const th = Math.atan2(pts[j][1]-pts[k][1], pts[j][0]-pts[k][0]);
    const sp = Math.max(v[lo] + f*(v[j]-v[lo]), step === 0 ? 0 : 4);
    out.push([x, y, th, sp, sp*(curv[lo] + f*(curv[j]-curv[lo]))]);
    if (s >= total) break;
    s += sp*dt;
  }
  return out;
}

function follow(name, cfg) {
  const fn = PATHS[name], pts = [];
  for (let i=0;i<=70;i++) pts.push(fn(i/70));
  api.trajClear();
  for (const [x,y,th,v,w] of timeTrajectory(pts)) api.trajPush(x,y,th,v,w);
  api.defaults();
  for (const [k,v] of Object.entries(cfg)) api.set(k, v);
  const [x0,y0]=pts[0], [x1,y1]=pts[1];
  api.reset(x0, y0, 90 - Math.atan2(y1-y0, x1-x0)*180/Math.PI);
  api.setLoc(5); api.setParam('collideObstacles', 0);
  for (let i=0;i<60;i++) api.step(10);
  api.run();
  let t=0, peak=0, sum=0, n=0;
  for (let i=0;i<5;i++) { api.step(10); t+=10; }
  while (api.running() && t < 9000) {
    api.step(10); t+=10;
    const s = st();
    let best=1e9; for (const [px,py] of pts) best=Math.min(best, Math.hypot(s.true.x-px, s.true.y-py));
    peak=Math.max(peak,best); sum+=best; n++;
  }
  for (let i=0;i<30;i++) api.step(10);
  return { t: t/1000, peak, mean: n?sum/n:0, collide: api.collideTicks() };
}

console.log('== MPC solves what it claims: first move equals the LQR move ==');
for (const h of [10, 15]) {
  const r = JSON.parse(api.check(h));
  ok(r.worstRelErr < 1e-3, `horizon ${h} matches unconstrained LQR`,
     `rel err ${r.worstRelErr.toExponential(2)}`);
}

console.log('== solver fits a control tick ==');
for (const [h, s, it] of [[10,0,1],[20,1,50]]) {
  const r = JSON.parse(api.bench(h, s, it, 200));
  ok(r.usPerSolve < 100, `horizon ${h} solver ${s}`, `${r.usPerSolve.toFixed(2)}us per solve`);
}

console.log('== follower tracks closed loop ==');
for (const name of ['gentle S', 'hairpin']) {
  for (const [lbl, cfg] of [['RTI h10', {mode:1, horizon:10, rtiIters:1}],
                            ['full h15', {mode:0, horizon:15, maxIters:50}]]) {
    const r = follow(name, cfg);
    ok(r.t < 8.9 && r.mean < 1.5 && r.collide === 0, `${name} ${lbl}`,
       `${r.t.toFixed(2)}s  mean ${r.mean.toFixed(2)}in  peak ${r.peak.toFixed(2)}in`);
  }
}

console.log(fails ? `\n${fails} CHECK(S) FAILED` : '\nALL CHECKS PASSED');
process.exit(fails ? 1 : 0);
