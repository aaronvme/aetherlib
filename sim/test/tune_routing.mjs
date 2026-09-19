// Coordinate-descent sweeps over the constants that decide whether a route
// lands where it was drawn.
//
//   node sim/test/tune_routing.mjs                every section, one pass
//   node sim/test/tune_routing.mjs lateral        one section
//   node sim/test/tune_routing.mjs all 2          two passes, so a knob swept
//                                                 before its confounder gets a
//                                                 second look
//
// Scored by sim/test/accuracy.mjs, so a sweep and the gate cannot disagree
// about what "accurate" means. Lower cost is better; the columns show which
// bound is driving it.
//
// These are the SIMULATOR's constants, and they are not meant to match
// gamespecific/config.cpp. The sim's plant is a first-order lag with no mass,
// so a gain that is right here would be optimistic on the robot. What they are
// for is making the route you drew and the route the sim drives the same route.
//
// The starting point is read out of the wasm rather than written down here, so
// this converges on whatever kSimTuning in sim/bridge/bridge.cpp already holds
// instead of silently re-testing the library defaults.
import { boot, BATTERY, runCase, score } from './accuracy.mjs';

const api = await boot();

// Every key any section touches, so a trial can reassert all of them. sim_reset
// does not restore motion or filter config: without this a trial inherits the
// previous row's answer and the sweep reports a value it never tested. The
// control section proves that is not happening.
const KEYS = [
  'lateralKp', 'lateralKi', 'lateralKd', 'lateralDFilt', 'lateralWindup',
  'lateralSlew',
  'angularKp', 'angularKi', 'angularKd', 'angularDFilt', 'angularWindup',
  'targetTolIn', 'targetTolDeg', 'bearingHold', 'settleTimeMs',
  'mtpTolerance', 'mtpBearingHold', 'mtpLead', 'driveTolerance', 'turnTolerance',
  'boomLead', 'boomLeadDecay', 'boomMinSpeed', 'boomTolerance', 'boomHeadingTol',
  'arcTolerance', 'arcMinSpeed',
  'ramseteB', 'ramseteZeta', 'ramseteMaxAccel', 'ramseteLatAccel',
  'ramseteMaxSpeed', 'ramseteMinSpeed', 'ramseteTolerance', 'ramseteHeadingTol',
  'ramseteStall', 'ramseteSmooth', 'ramseteSettleMs', 'ramseteVelCtrl',
  'ramseteFfKv', 'ramseteFfKs', 'ramseteFfKp',
];

const BASE = {};
for (const k of KEYS) {
  const v = api.getParam(k);
  if (Number.isNaN(v)) {
    console.error(`no getter for "${k}" -- trials would not be independent`);
    process.exit(2);
  }
  BASE[k] = v;
}
const kept = { ...BASE };

function apply(cfg) {
  for (const [k, v] of Object.entries(kept)) api.setParam(k, v);
  for (const [k, v] of Object.entries(cfg)) api.setParam(k, v);
}

/// Run a slice of the battery under one config. `only` is a substring filter,
/// so a RAMSETE sweep does not spend its time re-proving that turns work.
function trial(cfg, only) {
  const cases = only
    ? BATTERY.filter(c => only.some(o => c.name.includes(o)))
    : BATTERY;
  const rows = cases.map(cs => { apply(cfg); return runCase(api, cs); });
  return { rows, ...score(rows) };
}

const pad = (s, w) => String(s).padEnd(w);
const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v))
  ? '   – ' : v.toFixed(d);

/// Sweep one key, print a row per value, keep the best.
function sweep(key, values, only) {
  // Whatever is in place counts as a candidate, or a sweep whose list happens
  // to miss the current value can only make things worse.
  // Float readback means 1.15 comes out of the wasm as 1.149999976, so the
  // comparison is a tolerance rather than an equality -- otherwise every
  // sweep prints its current value twice.
  const near = v => Math.abs(v - kept[key]) <= Math.abs(kept[key]) * 1e-6 + 1e-9;
  const list = values.some(near) ? values : [kept[key], ...values];
  const results = list.map(v => [v, trial({ [key]: v }, only)]);
  const best = results.reduce((a, b) => (b[1].cost < a[1].cost ? b : a));
  for (const [v, t] of results) {
    const a = t.agg;
    console.log(`    ${pad(key + '=' + v, 26)} cost ${t.cost.toFixed(2).padStart(8)}`
      + `  end ${fmt(a.endIn)}in ${fmt(a.endDeg)}deg`
      + `  xt ${fmt(a.xtIn)}  ${(a.ms / 1000).toFixed(1)}s`
      + `  ${t.fails.length} over`
      + (v === best[0] ? '   <-- best' : ''));
  }
  kept[key] = best[0];
  return best[0];
}

const POINTS = ['drive', 'moveToPoint'];
const TURNS = ['turn', 'swing'];
const PATHS = ['path'];

const SECTIONS = {
  /// The same config twice has to give the same numbers, or nothing below means
  /// anything.
  control() {
    console.log('== control: trials must be independent ==');
    const a = trial({}, PATHS), b = trial({}, PATHS);
    const same = Math.abs(a.cost - b.cost) < 1e-9;
    console.log(`    run A cost ${a.cost.toFixed(6)}`);
    console.log(`    run B cost ${b.cost.toFixed(6)}`);
    console.log(same ? '    [PASS] repeatable'
                     : '    [FAIL] state leaks between trials -- results are junk');
    if (!same) process.exitCode = 1;
  },

  // --- how hard the along-track loop pulls ---------------------------------
  lateral() {
    console.log('== lateral PID: the terminal approach is what sets end error ==');
    sweep('lateralKp', [9.012, 14, 17, 20, 24, 30], POINTS);
    sweep('lateralKd', [0, 1.15, 2, 3, 6], POINTS);
    sweep('lateralDFilt', [0.1, 0.2, 0.3, 0.45, 0.6], POINTS);
    sweep('lateralSlew', [0, 450, 700, 900], POINTS);
  },

  // --- the settle test is the other half ----------------------------------
  tolerance() {
    console.log('== settle tolerances: with no odometry noise these can be tight,');
    console.log('   but every one of them is also a delay ==');
    sweep('targetTolIn', [1.0, 0.5, 0.25, 0.12, 0.05], POINTS);
    sweep('settleTimeMs', [0, 60, 110, 200, 300], POINTS);
    sweep('bearingHold', [0.5, 1.0, 1.5, 2.0], POINTS);
  },

  angular() {
    console.log('== angular PID and turn tolerance ==');
    sweep('angularKp', [140, 176.607, 210, 250, 300], TURNS);
    sweep('angularKd', [6, 9.933, 13, 16], TURNS);
    sweep('angularDFilt', [0.3, 0.5, 0.7], TURNS);
    sweep('targetTolDeg', [1.5, 0.75, 0.3, 0.1], TURNS);
  },

  boomerang() {
    console.log('== boomerang: the carrot collapses onto the target, so the endgame');
    console.log('   is the final-heading blend against a speed scale already near');
    console.log('   zero. lead and leadDecay first: they decide whether the heading');
    console.log('   converges at all, and until it does no tolerance binds ==');
    sweep('boomLead', [0.3, 0.5, 0.6, 0.7, 0.85], ['boomerang']);
    sweep('boomLeadDecay', [0, 0.15, 0.3, 0.5], ['boomerang']);
    sweep('boomMinSpeed', [0, 12, 25, 40, 60], ['boomerang']);
    sweep('boomTolerance', [-1, 0.5, 0.25, 0.12], ['boomerang']);
    sweep('boomHeadingTol', [0.5, 1, 2, 4], ['boomerang']);
  },

  arc() {
    console.log('== arc ==');
    sweep('arcTolerance', [-1, 0.5, 0.25, 0.12], ['arc']);
    sweep('arcMinSpeed', [0, 8, 20], ['arc']);
  },

  ramsete() {
    // The exit condition confounds everything else. Swept first, a tolerance
    // tight enough to time out costs 25 penalty points, and every gain row
    // after it reports whether it happened to squeeze inside that tolerance
    // rather than how well it tracked. So the gains are swept against a
    // tolerance the follower can comfortably meet, and the tolerance is
    // tightened afterwards against the gains that won.
    console.log('== RAMSETE tracking, measured with a tolerance loose enough that');
    console.log('   a gain difference shows up as cross-track and not as a timeout ==');
    const loose = { ramseteTolerance: 0.75, ramseteSettleMs: 110 };
    const pinned = (k, vals) => {
      const saved = { ...kept };
      Object.assign(kept, loose);
      sweep(k, vals, PATHS);
      const won = kept[k];
      Object.assign(kept, saved);
      kept[k] = won;
      return won;
    };
    pinned('ramseteB', [24, 48, 72, 96, 144]);
    pinned('ramseteZeta', [0.7, 0.85, 1.0]);
    pinned('ramseteMaxAccel', [30, 40, 50, 70, 100]);
    pinned('ramseteLatAccel', [40, 60, 90, 120]);
    pinned('ramseteMinSpeed', [6, 12, 20]);
    console.log('   now the exit conditions, against the gains that won');
    sweep('ramseteTolerance', [-1, 1.0, 0.5, 0.25, 0.12], PATHS);
    sweep('ramseteHeadingTol', [1, 2, 4, 8], PATHS);
    sweep('ramseteStall', [0, 0.75, 1.5, 3], PATHS);
    sweep('ramseteSettleMs', [0, 60, 110, 200], PATHS);
    // smoothSpacingIn is deliberately NOT swept. It is pinned to the header
    // default by sim/test/auton_cpp.mjs, because the browser draws the preview
    // at that spacing -- tuning it here would make the sim drive a curve it did
    // not draw. Coarsening it to 4 in did score better, which is worth knowing
    // and is not worth a preview that lies.
  },

  // Closing the velocity loop is meant to be worth more on a curve than any
  // gain change -- if its own gains are right. Swept together, because kP
  // trades against the motor response time and kV is what it corrects around.
  velctrl() {
    console.log('== RAMSETE velocity controller: open loop against closed ==');
    const open = trial({ ramseteVelCtrl: 0 }, PATHS);
    console.log(`    open loop                  cost ${open.cost.toFixed(2)}`);
    let best = ['open loop', open.cost, { ramseteVelCtrl: 0 }];
    for (const kv of [5.6, 6.2, 6.8]) {
      for (const kp of [2, 6, 12]) {
        const t = trial({ ramseteVelCtrl: 1, ramseteFfKv: kv, ramseteFfKp: kp },
                        PATHS);
        console.log(`    closed kV=${kv} kP=${String(kp).padStart(2)}       `
                    + `cost ${t.cost.toFixed(2)}  ${t.fails.length} over`);
        if (t.cost < best[1]) {
          best = [`closed kV=${kv} kP=${kp}`, t.cost,
                  { ramseteVelCtrl: 1, ramseteFfKv: kv, ramseteFfKp: kp }];
        }
      }
    }
    Object.assign(kept, best[2]);
    console.log(`    keeping: ${best[0]}`);
  },
};

const ORDER = ['lateral', 'tolerance', 'angular', 'boomerang', 'arc', 'ramsete',
               'velctrl'];

const which = process.argv[2] && process.argv[2] !== 'all' ? process.argv[2] : null;
const passes = parseInt(process.argv[3] || '1', 10);

SECTIONS.control();
console.log('');

for (let p = 0; p < passes; p++) {
  if (passes > 1) console.log(`######## pass ${p + 1} of ${passes} ########\n`);
  for (const name of ORDER) {
    if (which && which !== name) continue;
    SECTIONS[name]();
    console.log('');
  }
}

// The whole battery under everything the sweeps kept, so a section that helped
// its own cases at another's expense cannot hide.
console.log('== full battery under the kept values ==');
const final = trial({});
for (const r of final.rows) {
  console.log(`   ${pad(r.name, 22)} end ${fmt(r.endIn)}in ${fmt(r.endDeg)}deg`
    + `  xt ${fmt(r.xtIn)}  ${String(r.ms).padStart(5)}ms`
    + `${r.cut ? ' CUTOFF' : ''}${r.hit ? ' HIT' : ''}`);
}
console.log(`   cost ${final.cost.toFixed(2)}  worst/bound ${final.worst.toFixed(2)}`
            + `   total ${(final.agg.ms / 1000).toFixed(1)}s`);
for (const f of final.fails) console.log(`   [over] ${f}`);
console.log('');
console.log('changed from what the wasm was built with:');
let any = false;
for (const [k, v] of Object.entries(kept)) {
  if (BASE[k] !== v) { console.log(`  ${pad(k, 20)} ${BASE[k]}  ->  ${v}`); any = true; }
}
if (!any) console.log('  (nothing -- the built-in values won every sweep)');
