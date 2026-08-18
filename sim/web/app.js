// riku-sim — browser front end.
//
// All simulation and every algorithm under test live in the WASM module; this
// file only drives the clock, renders state, and marshals control input.
import createRikuSim from './riku_sim.mjs';
import { parseRoute, moveWaypoint, buildPath, DEFAULT_ROUTE } from './route_dsl.js';

const M = await createRikuSim();

// ============================================================================
// WASM API
// ============================================================================

const w = (n, r, a) => M.cwrap(n, r, a);
const api = {
  init:         w('sim_init', null, []),
  reset:        w('sim_reset', null, ['number', 'number', 'number']),
  step:         w('sim_step', null, ['number']),
  teleport:     w('sim_teleport', null, ['number', 'number', 'number']),
  setDrive:     w('sim_set_drive', null, ['number', 'number']),
  setLoc:       w('sim_set_localization', null, ['number']),
  getParam:     w('sim_get_param', 'number', ['string']),
  setOdomPods:  w('sim_set_odom_pods', null, ['number']),
  setParam:     w('sim_set_param', null, ['string', 'number']),
  stateJson:    w('sim_state_json', 'string', []),
  fieldJson:    w('sim_field_json', 'string', []),
  particlePtr:  w('sim_particles_ptr', 'number', []),
  particleCnt:  w('sim_particle_count', 'number', []),
  trailTruePtr: w('sim_trail_true_ptr', 'number', []),
  trailTrueCnt: w('sim_trail_true_count', 'number', []),
  trailOdomPtr: w('sim_trail_odom_ptr', 'number', []),
  trailOdomCnt: w('sim_trail_odom_count', 'number', []),
  autonCount:   w('sim_auton_count', 'number', []),
  autonName:    w('sim_auton_name', 'string', ['number']),
  runAuton:     w('sim_run_auton', null, ['number']),
  stopAuton:    w('sim_stop_auton', null, []),
  routeRunning: w('sim_route_running', 'number', []),
  routeClear:   w('sim_route_clear', null, []),
  routePush:    w('sim_route_push', null, ['number', 'number', 'number', 'number', 'number']),
  routeRun:     w('sim_route_run', null, []),
  routeIndex:   w('sim_route_index', 'number', []),
  pathClear:    w('sim_path_clear', null, []),
  pathPush:     w('sim_path_push', null, ['number', 'number', 'number']),
  recPtr:       w('sim_record_ptr', 'number', []),
  recCount:     w('sim_record_count', 'number', []),
  recStride:    w('sim_record_stride', 'number', []),
  recClear:     w('sim_record_clear', null, []),
  fieldClear:   w('sim_field_clear', null, []),
  fieldDefault: w('sim_field_default', null, []),
  fieldAddSeg:  w('sim_field_add_segment', null, ['number', 'number', 'number', 'number']),
  fieldAddCirc: w('sim_field_add_circle', null, ['number', 'number', 'number']),
};

const START = { x: -48, y: -60, h: 90 };
api.init();
api.reset(START.x, START.y, START.h);

let fieldGeom = JSON.parse(api.fieldJson());

// ============================================================================
// Canvas
// ============================================================================

const canvas = document.getElementById('field');
const ctx = canvas.getContext('2d');

// SIZE is the drawing side in CSS pixels and SCALE the inches-to-CSS-pixel
// factor; both are recomputed by fitCanvas() whenever the stage resizes. The
// backing store is sized in DEVICE pixels and the context pre-scaled by the
// pixel ratio, so all drawing below stays in CSS pixels -- line widths and dot
// radii keep their intended weight instead of thinning out on a HiDPI screen.
// Sized to the element rather than fixed at 900, so the field is rendered at the
// resolution it is displayed at rather than upscaled from a stale bitmap.
let SIZE = canvas.width;
const MARGIN_FRAC = 22 / 900;   // the old 22 px margin, as a share of the side
let MARGIN = SIZE * MARGIN_FRAC;
let SCALE = (SIZE - MARGIN * 2) / 144;

function fitCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const side = Math.max(1, Math.min(rect.width, rect.height));
  const store = Math.round(side * dpr);
  if (canvas.width !== store) { canvas.width = store; canvas.height = store; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  SIZE = store / dpr;
  MARGIN = SIZE * MARGIN_FRAC;
  SCALE = (SIZE - MARGIN * 2) / 144;
}

// Arrow functions close over the bindings, not the values, so these follow
// fitCanvas() without re-creation.
const toPx = (x, y) => [SIZE / 2 + x * SCALE, SIZE / 2 - y * SCALE];
const toIn = (px, py) => [(px - SIZE / 2) / SCALE, (SIZE / 2 - py) / SCALE];

new ResizeObserver(fitCanvas).observe(canvas);
// devicePixelRatio changes when the window moves between monitors or the page is
// zoomed; neither fires a resize on the canvas itself.
window.addEventListener('resize', fitCanvas);
fitCanvas();

// ============================================================================
// Layers
// ============================================================================

const LAYERS = [
  ['field',     'field',         true],
  ['grid',      'tile grid',     true],
  ['obstacles', 'goals & posts', true],
  ['robot',     'robot (true)',  true],
  ['odom',      'odom pose',     true],
  ['mcl',       'mcl pose',      true],
  ['particles', 'MCL particles', true],
  ['beams',     'sensor beams',  true],
  ['cones',     'sensor cones',  false],
  ['truePath',  'true path',     true],
  ['odomPath',  'odom path',     true],
  ['plan',      'route preview', true],
  ['rcl',       'rcl pose & rays', true],
  ['ghost',     'replay ghost',  true],
];
const layers = Object.fromEntries(LAYERS.map(([k, , v]) => [k, v]));

const layersEl = document.getElementById('layers');
for (const [key, label, on] of LAYERS) {
  const l = document.createElement('label');
  l.innerHTML = `<input type="checkbox" ${on ? 'checked' : ''}> ${label}`;
  l.querySelector('input').addEventListener('change', e => { layers[key] = e.target.checked; });
  layersEl.appendChild(l);
}

// ============================================================================
// Parameters
// ============================================================================

function buildParams(hostId, defs) {
  const host = document.getElementById(hostId);
  for (const d of defs) {
    // Start from what the library actually holds. The panel used to publish its
    // own markup default on load, which overwrote the tuned gains in
    // gamespecific/config.cpp the moment the page opened.
    const live = api.getParam(d.key);
    const value = Number.isFinite(live) ? live : d.value;

    const row = document.createElement('div');
    row.className = 'param';
    row.innerHTML =
      `<span title="${d.help || ''}">${d.label}</span>` +
      `<input class="pslide" type="range" min="${d.min}" max="${d.max}" step="${d.step}" value="${value}">` +
      `<input class="pnum" type="number" step="${d.step}" value="${+value.toFixed(4)}">`;
    const slide = row.querySelector('.pslide');
    const num = row.querySelector('.pnum');

    // The box is authoritative and deliberately unclamped: a slider range is a
    // convenience, not a limit, and typing 34 into a kP that sliders to 30 has to
    // work. The slider just saturates.
    const push = (v, from) => {
      if (!Number.isFinite(v)) return;
      if (from !== 'slide') slide.value = v;
      if (from !== 'num') num.value = +v.toFixed(4);
      api.setParam(d.key, v);
    };
    slide.addEventListener('input', () => push(parseFloat(slide.value), 'slide'));
    // 'change' not 'input', so a half-typed "0.0" is not pushed as 0 mid-edit.
    num.addEventListener('change', () => push(parseFloat(num.value), 'num'));
    num.addEventListener('keydown', e => { if (e.key === 'Enter') num.blur(); });

    host.appendChild(row);
  }
}

buildParams('params-pid', [
  { key: 'lateralKp', label: 'lateral kP', min: 0, max: 60,  step: 0.5, value: 34 },
  { key: 'lateralKi', label: 'lateral kI', min: 0, max: 10,  step: 0.1, value: 0,
    help: '0 is tuned; a little helps against static friction on a real robot' },
  { key: 'lateralKd', label: 'lateral kD', min: 0, max: 20,  step: 0.1, value: 2 },
  { key: 'angularKp', label: 'angular kP', min: 0, max: 260, step: 1,   value: 140,
    help: 'angular error is in RADIANS, so 140 is a ~52 deg proportional band' },
  { key: 'angularKi', label: 'angular kI', min: 0, max: 40,  step: 0.5, value: 0 },
  { key: 'angularKd', label: 'angular kD', min: 0, max: 40,  step: 0.5, value: 8 },
  { key: 'lateralDFilt', label: 'lateral D filter', min: 0.05, max: 1, step: 0.05, value: 0.6,
    help: 'derivative smoothing; low values delay kD by dt/alpha and cost damping' },
  { key: 'angularDFilt', label: 'angular D filter', min: 0.05, max: 1, step: 0.05, value: 0.5 },
  { key: 'settleTimeMs', label: 'settle hold ms', min: 0, max: 400, step: 10, value: 110,
    help: 'how long tolerance must hold; 0 exits while still carrying velocity' },
]);

buildParams('params-sensor', [
  { key: 'distNoiseMm',  label: 'dist noise mm', min: 0, max: 60, step: 1, value: 5 },
  { key: 'distMaxRangeMm', label: 'dist range mm', min: 500, max: 4000, step: 50, value: 2000,
    help: 'a real V5 distance sensor tops out near 2000 mm' },
  { key: 'distFovDeg', label: 'cone FOV °', min: 0, max: 45, step: 1, value: 25,
    help: 'the sensor integrates a cone, so it sees posts off-axis' },
  { key: 'distMaxIncidenceDeg', label: 'max incidence °', min: 30, max: 89, step: 1, value: 75,
    help: 'past this angle from the surface normal the return is lost' },
  { key: 'imuDriftDegPerSec', label: 'IMU drift °/s', min: -2, max: 2, step: 0.05, value: 0 },
  { key: 'imuNoiseDeg', label: 'IMU noise °', min: 0, max: 2, step: 0.05, value: 0 },
  { key: 'imuUnderReport', label: 'IMU under-report', min: 0.9, max: 1.1, step: 0.001, value: 354.25 / 360,
    help: 'cancels ScaledIMU(360, 354.25) exactly at the default' },
]);

buildParams('params-drive', [
  { key: 'encoderSlip',   label: 'wheel slip',   min: 0, max: 0.4, step: 0.01, value: 0,
    help: 'fraction of travel the encoders never see — the main odometry drift source' },
  { key: 'leftScale',     label: 'left scale',   min: 0.8, max: 1.2, step: 0.01, value: 1 },
  { key: 'rightScale',    label: 'right scale',  min: 0.8, max: 1.2, step: 0.01, value: 1 },
  { key: 'responseTauMs', label: 'drive lag ms', min: 0, max: 300, step: 5, value: 70 },
]);

buildParams('params-rate', [
  { key: 'distPeriodMs',    label: 'distance ms', min: 5, max: 100, step: 5, value: 30 },
  { key: 'imuPeriodMs',     label: 'IMU ms',      min: 1, max: 50,  step: 1, value: 10 },
  { key: 'encoderPeriodMs', label: 'encoder ms',  min: 1, max: 50,  step: 1, value: 10 },
]);

// ============================================================================
// Route DSL
// ============================================================================

const editor = document.getElementById('editor');
const routeStatus = document.getElementById('route-status');
editor.value = localStorage.getItem('riku-sim.route') || DEFAULT_ROUTE;

let plan = [];
let sampledPaths = [];      // spline points, exactly what gets uploaded

function refreshPlan() {
  const r = parseRoute(editor.value);
  if (r.error) {
    routeStatus.textContent = r.error;
    routeStatus.className = 'status err';
    return null;
  }
  sampledPaths = (r.paths || []).map(vias => buildPath(vias));
  const pts = sampledPaths.reduce((n, p) => n + p.length, 0);
  routeStatus.textContent = `${r.cmds.length} commands`
    + (sampledPaths.length ? ` · ${sampledPaths.length} path, ${pts} pts` : '');
  routeStatus.className = 'status';
  plan = r.plan;
  return r;
}

editor.addEventListener('input', () => {
  localStorage.setItem('riku-sim.route', editor.value);
  refreshPlan();
});
refreshPlan();

document.getElementById('btn-run-route').addEventListener('click', () => {
  const r = refreshPlan();
  if (!r) return;
  // Upload the sampled trajectories first, then rewrite each followPath
  // command's (index) into the (start, count) slice it should follow. The
  // points drawn on the field are the same array moveRAMSETE receives.
  api.pathClear();
  const slices = [];
  let cursor = 0;
  for (const pts of sampledPaths) {
    for (const q of pts) api.pathPush(q.x, q.y, q.h);
    slices.push([cursor, pts.length]);
    cursor += pts.length;
  }

  api.routeClear();
  for (const cmd of r.cmds) {
    let [op, a, b, c, d] = cmd;
    if (op === 13) {                       // OP_FOLLOW_PATH
      const slice = slices[a];
      if (!slice) continue;
      [a, b] = slice;
    }
    api.routePush(op, a, b, c, d);
  }
  api.recClear();
  api.routeRun();
  routeStatus.textContent = `running ${r.cmds.length} commands`;
  routeStatus.className = 'status ok';
  running = true;
  btnPlay.textContent = 'Pause';
});

// --- saved routes -----------------------------------------------------------

const savedSel = document.getElementById('saved');
const routeName = document.getElementById('route-name');

const loadSaved = () => JSON.parse(localStorage.getItem('riku-sim.saved') || '{}');
function refreshSaved() {
  const all = loadSaved();
  savedSel.innerHTML = '<option value="">— saved routes —</option>';
  for (const k of Object.keys(all).sort()) {
    const o = document.createElement('option');
    o.value = k; o.textContent = k;
    savedSel.appendChild(o);
  }
}
refreshSaved();

document.getElementById('btn-save').addEventListener('click', () => {
  const all = loadSaved();
  all[routeName.value || 'untitled'] = editor.value;
  localStorage.setItem('riku-sim.saved', JSON.stringify(all));
  refreshSaved();
});

savedSel.addEventListener('change', () => {
  if (!savedSel.value) return;
  editor.value = loadSaved()[savedSel.value];
  routeName.value = savedSel.value;
  localStorage.setItem('riku-sim.route', editor.value);
  refreshPlan();
});

// ============================================================================
// Autons
// ============================================================================

const autonSel = document.getElementById('auton');
for (let i = 0; i < api.autonCount(); i++) {
  const o = document.createElement('option');
  o.value = i; o.textContent = api.autonName(i);
  autonSel.appendChild(o);
}
document.getElementById('btn-run-auton').addEventListener('click', () => {
  api.recClear();
  api.runAuton(parseInt(autonSel.value, 10));
  running = true;
  btnPlay.textContent = 'Pause';
});
document.getElementById('btn-stop').addEventListener('click', () => api.stopAuton());

// ============================================================================
// Transport
// ============================================================================

let running = true;
let speed = 1.0;

const btnPlay = document.getElementById('btn-play');
btnPlay.addEventListener('click', () => {
  running = !running;
  btnPlay.textContent = running ? 'Pause' : 'Play';
});
document.getElementById('btn-step').addEventListener('click', () => {
  running = false;
  btnPlay.textContent = 'Play';
  api.step(10);
});

const startFields = ['start-x', 'start-y', 'start-h'].map(id => document.getElementById(id));
const readStart = () => startFields.map(f => parseFloat(f.value) || 0);
const doReset = () => { api.reset(...readStart()); api.recClear(); };
document.getElementById('btn-reset').addEventListener('click', doReset);
document.getElementById('btn-place').addEventListener('click', doReset);

{
  const hudEl = document.getElementById('hud');
  const btn = document.getElementById('hud-toggle');
  btn.addEventListener('click', () => {
    const off = hudEl.classList.toggle('collapsed');
    btn.innerHTML = off ? '+' : '&minus;';
  });
}

document.getElementById('chk-collide').addEventListener('change', e => {
  api.setParam('collideObstacles', e.target.checked ? 1 : 0);
});
// Localization is four independent switches. The bridge still takes a bitmask, so
// it is assembled here rather than enumerated as one option per combination.
const locBits = { 'loc-mcl': 1, 'loc-rcl': 2, 'loc-ekf': 4 };
const locLabelEl = document.getElementById('loc-label');

function applyLoc() {
  let mode = 0;
  for (const [id, bit] of Object.entries(locBits)) {
    if (document.getElementById(id).checked) mode |= bit;
  }
  const names = [];
  if (mode & 1) names.push('MCL');
  if (mode & 2) names.push('RCL');
  if (mode & 4) names.push('EKF');
  locLabelEl.textContent = names.length ? names.join(' + ') : 'dead reckoning';
  // The EKF owns the pose while it is up, so say that rather than implying the
  // other two are still steering it.
  locLabelEl.title = (mode & 4)
    ? 'EKF owns the pose; MCL and RCL feed it'
    : 'nothing ticked = IMU + motor encoders only';
  api.setLoc(mode);
}
for (const id of Object.keys(locBits)) {
  document.getElementById(id).addEventListener('change', applyLoc);
}
document.getElementById('loc-pods').addEventListener('change', e => {
  api.setOdomPods(e.target.checked ? 1 : 0);
});

const speedInput = document.getElementById('speed');
const speedVal = document.getElementById('speed-val');
speedInput.addEventListener('input', () => {
  speed = parseFloat(speedInput.value);
  speedVal.textContent = speed.toFixed(1) + '×';
});

// --- tabs -------------------------------------------------------------------

for (const tab of document.querySelectorAll('.tab')) {
  tab.addEventListener('click', () => {
    document.querySelectorAll('.tab').forEach(t => t.classList.toggle('active', t === tab));
    document.querySelectorAll('.tabpane').forEach(p =>
      p.classList.toggle('active', p.dataset.pane === tab.dataset.tab));
  });
}

// --- keyboard drive ---------------------------------------------------------

const keys = new Set();
addEventListener('keydown', e => {
  if (e.target === editor || e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)) e.preventDefault();
  keys.add(e.key.toLowerCase());
});
addEventListener('keyup', e => keys.delete(e.key.toLowerCase()));

function applyKeyboardDrive() {
  if (api.routeRunning()) return;
  const fwd = (keys.has('w') || keys.has('arrowup')) - (keys.has('s') || keys.has('arrowdown'));
  const turn = (keys.has('d') || keys.has('arrowright')) - (keys.has('a') || keys.has('arrowleft'));
  api.setDrive((fwd + turn * 0.6) * 9, (fwd - turn * 0.6) * 9);
}

// --- pointer: drag to kidnap, shift-click to append a waypoint ---------------

let dragging = false;       // dragging the robot (kidnap)
let dragWaypoint = -1;      // index into plan, or -1

const pointerIn = e => {
  const r = canvas.getBoundingClientRect();
  return toIn((e.clientX - r.left) * (SIZE / r.width), (e.clientY - r.top) * (SIZE / r.height));
};

/// Nearest editable waypoint within grabRadius inches, or -1.
function waypointAt(x, y, grabRadius = 4) {
  let best = -1, bestD = grabRadius;
  plan.forEach((p, i) => {
    if (!p.xRange || !p.yRange) return;      // not rewritable, so not draggable
    const d = Math.hypot(p.x - x, p.y - y);
    if (d < bestD) { bestD = d; best = i; }
  });
  return best;
}

canvas.addEventListener('pointerdown', e => {
  const [x, y] = pointerIn(e);

  if (e.shiftKey) {
    const line = `moveTo(x: ${x.toFixed(1)}, y: ${y.toFixed(1)});\n`;
    editor.value = editor.value.replace(/\s*$/, '\n') + line;
    localStorage.setItem('riku-sim.route', editor.value);
    refreshPlan();
    return;
  }

  // Grabbing a numbered waypoint edits the route text; anything else kidnaps
  // the robot.
  const wp = layers.plan ? waypointAt(x, y) : -1;
  if (wp >= 0) {
    dragWaypoint = wp;
  } else {
    dragging = true;
  }
  canvas.classList.add('dragging');
  canvas.setPointerCapture(e.pointerId);
});

canvas.addEventListener('pointermove', e => {
  if (!dragging && dragWaypoint < 0) return;
  const [x, y] = pointerIn(e);

  if (dragWaypoint >= 0) {
    const wp = plan[dragWaypoint];
    if (!wp) return;
    editor.value = moveWaypoint(editor.value, wp, x, y);
    localStorage.setItem('riku-sim.route', editor.value);
    refreshPlan();          // re-parse so the ranges follow the new text
    return;
  }
  api.teleport(x, y, lastState ? lastState.true.h : START.h);
});

const endDrag = () => {
  dragging = false;
  dragWaypoint = -1;
  canvas.classList.remove('dragging');
};
canvas.addEventListener('pointerup', endDrag);
canvas.addEventListener('pointercancel', endDrag);

// ============================================================================
// Field editor
// ============================================================================

const fieldJsonEl = document.getElementById('field-json');
const fieldFacts = document.getElementById('field-facts');

function showFieldGeom() {
  fieldGeom = JSON.parse(api.fieldJson());
  fieldFacts.innerHTML =
    `<div class="k">segments</div><div class="v">${fieldGeom.segments.length}</div>` +
    `<div class="k">circles</div><div class="v">${fieldGeom.circles.length}</div>`;
  fieldJsonEl.value = JSON.stringify({
    segments: fieldGeom.segments.map(s => s.slice(0, 4)),
    circles: fieldGeom.circles.map(c => c.slice(0, 3)),
  }, null, 1);
}
showFieldGeom();

document.getElementById('btn-field-apply').addEventListener('click', () => {
  try {
    const g = JSON.parse(fieldJsonEl.value);
    api.fieldClear();
    for (const s of g.segments || []) api.fieldAddSeg(s[0], s[1], s[2], s[3]);
    for (const c of g.circles || []) api.fieldAddCirc(c[0], c[1], c[2]);
    showFieldGeom();
  } catch (err) {
    alert('field JSON: ' + err.message);
  }
});
document.getElementById('btn-field-reset').addEventListener('click', () => {
  api.fieldDefault();
  showFieldGeom();
});

// ============================================================================
// Plots
// ============================================================================

const SIGNALS = [
  ['errDist',  'odom error in', 10, '#f87171', true],
  ['errHead',  'heading err °', 11, '#fb923c', true],
  ['vLin',     'lin vel in/s',  12, '#4ade80', false],
  ['vAng',     'ang vel °/s',   13, '#5aa9ff', false],
  ['trueX',    'true x',        1,  '#94a3b8', false],
  ['trueY',    'true y',        2,  '#64748b', false],
  ['mclX',     'mcl x',         7,  '#f472b6', false],
  ['mclY',     'mcl y',         8,  '#e879f9', false],
];
const sigOn = Object.fromEntries(SIGNALS.map(([k, , , , on]) => [k, on]));

const picks = document.getElementById('plot-picks');
for (const [key, label, , color, on] of SIGNALS) {
  const l = document.createElement('label');
  l.innerHTML = `<input type="checkbox" ${on ? 'checked' : ''}> <span style="color:${color}">${label}</span>`;
  l.querySelector('input').addEventListener('change', e => { sigOn[key] = e.target.checked; });
  picks.appendChild(l);
}

const plotCanvas = document.getElementById('plot');
const pctx = plotCanvas.getContext('2d');

// The plot keeps a fixed logical coordinate system so the layout maths below is
// unaffected; only the backing store follows the element's real size and pixel
// ratio, which is what keeps 1 px gridlines and 10 px labels sharp at any width.
const PW = 640, PH = 900;

function fitPlot() {
  const dpr = window.devicePixelRatio || 1;
  const cssW = Math.max(1, plotCanvas.getBoundingClientRect().width);
  const storeW = Math.round(cssW * dpr);
  const storeH = Math.round(cssW * (PH / PW) * dpr);
  if (plotCanvas.width !== storeW || plotCanvas.height !== storeH) {
    plotCanvas.width = storeW;
    plotCanvas.height = storeH;
  }
  const k = storeW / PW;
  pctx.setTransform(k, 0, 0, k, 0, 0);
}
new ResizeObserver(fitPlot).observe(plotCanvas);
window.addEventListener('resize', fitPlot);
fitPlot();

function drawPlots() {
  const n = api.recCount();
  const stride = api.recStride();
  fitPlot();
  pctx.clearRect(0, 0, PW, PH);
  if (n < 2) return;

  const ptr = api.recPtr() >> 2;
  const buf = M.HEAPF32.subarray(ptr, ptr + n * stride);

  const active = SIGNALS.filter(([k]) => sigOn[k]);
  if (!active.length) return;

  const W = PW;
  const rowH = PH / active.length;
  const t0 = buf[0], t1 = buf[(n - 1) * stride];
  const span = Math.max(t1 - t0, 1e-6);

  active.forEach(([, label, col, color], row) => {
    const top = row * rowH;
    let lo = Infinity, hi = -Infinity;
    for (let i = 0; i < n; i++) {
      const v = buf[i * stride + col];
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    if (hi - lo < 1e-6) { hi = lo + 1; lo -= 1; }
    const pad = (hi - lo) * 0.12;
    lo -= pad; hi += pad;

    // frame
    pctx.strokeStyle = '#2f3540';
    pctx.lineWidth = 1;
    pctx.strokeRect(38, top + 14, W - 46, rowH - 28);

    // zero line
    if (lo < 0 && hi > 0) {
      const zy = top + 14 + (1 - (0 - lo) / (hi - lo)) * (rowH - 28);
      pctx.strokeStyle = '#3a4150';
      pctx.setLineDash([3, 3]);
      pctx.beginPath(); pctx.moveTo(38, zy); pctx.lineTo(W - 8, zy); pctx.stroke();
      pctx.setLineDash([]);
    }

    // trace
    pctx.strokeStyle = color;
    pctx.lineWidth = 1.4;
    pctx.beginPath();
    for (let i = 0; i < n; i++) {
      const x = 38 + ((buf[i * stride] - t0) / span) * (W - 46);
      const v = buf[i * stride + col];
      const y = top + 14 + (1 - (v - lo) / (hi - lo)) * (rowH - 28);
      i === 0 ? pctx.moveTo(x, y) : pctx.lineTo(x, y);
    }
    pctx.stroke();

    // replay cursor
    if (replayRow) {
      const cx = 38 + ((replayRow.t - t0) / span) * (W - 46);
      pctx.strokeStyle = '#5aa9ff';
      pctx.lineWidth = 1;
      pctx.beginPath();
      pctx.moveTo(cx, top + 14);
      pctx.lineTo(cx, top + rowH - 14);
      pctx.stroke();
    }

    // labels
    pctx.fillStyle = color;
    pctx.font = '11px ui-monospace, Consolas, monospace';
    pctx.fillText(label, 40, top + 11);
    pctx.fillStyle = '#8b93a1';
    pctx.textAlign = 'right';
    pctx.fillText(hi.toFixed(1), 35, top + 20);
    pctx.fillText(lo.toFixed(1), 35, top + rowH - 16);
    pctx.textAlign = 'left';
    const last = buf[(n - 1) * stride + col];
    pctx.fillStyle = color;
    pctx.fillText(last.toFixed(2), W - 60, top + 11);
  });
}

// --- replay scrubbing -------------------------------------------------------
//
// The recorder already holds every pose; scrubbing just reads a row out of it
// and draws a ghost. The sim itself is untouched, so you can scrub back, look,
// and resume without disturbing the run.

const replayOn = document.getElementById('replay-on');
const replayPos = document.getElementById('replay-pos');
const replayT = document.getElementById('replay-t');
let replayRow = null;      // decoded row while scrubbing, else null

function recordRow(i) {
  const n = api.recCount();
  if (n === 0) return null;
  const stride = api.recStride();
  const idx = Math.max(0, Math.min(n - 1, i));
  const ptr = (api.recPtr() >> 2) + idx * stride;
  const r = M.HEAPF32.subarray(ptr, ptr + stride);
  return {
    t: r[0],
    true: { x: r[1], y: r[2], h: r[3] },
    odom: { x: r[4], y: r[5], h: r[6] },
    mcl:  { x: r[7], y: r[8], h: r[9] },
    err:  { dist: r[10], heading: r[11] },
  };
}

function updateReplay() {
  if (!replayOn.checked) {
    replayRow = null;
    replayT.textContent = 'live';
    replayPos.disabled = true;
    return;
  }
  replayPos.disabled = false;
  const n = api.recCount();
  if (n === 0) { replayRow = null; replayT.textContent = 'no data'; return; }
  const idx = Math.round((parseFloat(replayPos.value) / 1000) * (n - 1));
  replayRow = recordRow(idx);
  replayT.textContent = replayRow ? `${replayRow.t.toFixed(2)} s` : 'live';
}

replayOn.addEventListener('change', () => {
  if (replayOn.checked) { running = false; btnPlay.textContent = 'Play'; }
  updateReplay();
});
replayPos.addEventListener('input', updateReplay);

// Clicking the plot seeks there.
plotCanvas.addEventListener('pointerdown', e => {
  const n = api.recCount();
  if (n < 2) return;
  const r = plotCanvas.getBoundingClientRect();
  const x = (e.clientX - r.left) * (PW / r.width);
  const frac = Math.max(0, Math.min(1, (x - 38) / (PW - 46)));
  replayOn.checked = true;
  running = false;
  btnPlay.textContent = 'Play';
  replayPos.value = String(Math.round(frac * 1000));
  updateReplay();
});

document.getElementById('btn-clear-rec').addEventListener('click', () => {
  api.recClear();
  updateReplay();
});

document.getElementById('btn-export').addEventListener('click', () => {
  const n = api.recCount(), stride = api.recStride();
  const ptr = api.recPtr() >> 2;
  const buf = M.HEAPF32.subarray(ptr, ptr + n * stride);
  const cols = ['t', 'trueX', 'trueY', 'trueH', 'odomX', 'odomY', 'odomH',
                'mclX', 'mclY', 'mclH', 'errDist', 'errHeading', 'vLin', 'vAng'];
  let csv = cols.join(',') + '\n';
  for (let i = 0; i < n; i++) {
    const row = [];
    for (let c = 0; c < stride; c++) row.push(buf[i * stride + c].toFixed(4));
    csv += row.join(',') + '\n';
  }
  const url = URL.createObjectURL(new Blob([csv], { type: 'text/csv' }));
  const a = document.createElement('a');
  a.href = url; a.download = 'riku-sim-run.csv'; a.click();
  URL.revokeObjectURL(url);
});

// ============================================================================
// Field rendering
// ============================================================================

function drawGrid() {
  ctx.strokeStyle = '#20242c';
  ctx.lineWidth = 1;
  for (let i = -3; i <= 3; i++) {
    let [a, b] = toPx(i * 24, -72), [c, d] = toPx(i * 24, 72);
    ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
    [a, b] = toPx(-72, i * 24); [c, d] = toPx(72, i * 24);
    ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
  }
}

function drawField() {
  const [x0, y0] = toPx(-72, 72);
  ctx.fillStyle = '#171a20';
  ctx.fillRect(x0, y0, 144 * SCALE, 144 * SCALE);

  for (const s of fieldGeom.segments) {
    const kind = s[4] ?? 0;
    ctx.strokeStyle = kind === 0 ? '#3a4150' : '#8b93a1';
    ctx.lineWidth = kind === 0 ? 2.5 : 2;
    const [p1x, p1y] = toPx(s[0], s[1]);
    const [p2x, p2y] = toPx(s[2], s[3]);
    ctx.beginPath(); ctx.moveTo(p1x, p1y); ctx.lineTo(p2x, p2y); ctx.stroke();
  }
}

function drawObstacles() {
  for (const c of fieldGeom.circles) {
    const [px, py] = toPx(c[0], c[1]);
    ctx.beginPath();
    ctx.arc(px, py, c[2] * SCALE, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(148,163,184,0.32)';
    ctx.fill();
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }
}

function drawParticles() {
  const n = api.particleCnt();
  if (n === 0) return;
  const ptr = api.particlePtr() >> 2;
  const buf = M.HEAPF32.subarray(ptr, ptr + n * 4);
  let maxW = 0;
  for (let i = 0; i < n; i++) maxW = Math.max(maxW, buf[i * 4 + 3]);
  if (maxW <= 0) maxW = 1;
  for (let i = 0; i < n; i++) {
    const [px, py] = toPx(buf[i * 4], buf[i * 4 + 1]);
    ctx.fillStyle = `rgba(244,114,182,${0.12 + 0.75 * Math.min(1, buf[i * 4 + 3] / maxW)})`;
    ctx.fillRect(px - 1, py - 1, 2, 2);
  }
}

function drawTrail(ptrFn, cntFn, color) {
  const n = cntFn();
  if (n < 2) return;
  const ptr = ptrFn() >> 2;
  const buf = M.HEAPF32.subarray(ptr, ptr + n * 2);
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  // A NaN pair is a break marker, pushed whenever the robot is repositioned
  // rather than driven there. Without it the trail draws a straight line across
  // the field from wherever the last run ended to the new start pose.
  let pen = false;
  for (let i = 0; i < n; i++) {
    const x = buf[i * 2], y = buf[i * 2 + 1];
    if (!Number.isFinite(x) || !Number.isFinite(y)) { pen = false; continue; }
    const [px, py] = toPx(x, y);
    if (!pen) { ctx.moveTo(px, py); pen = true; } else ctx.lineTo(px, py);
  }
  ctx.stroke();
}

function drawBeams(s) {
  for (const b of s.beams) {
    const [ox, oy] = toPx(b.ox, b.oy);

    if (layers.cones) {
      const dir = b.dir * Math.PI / 180;
      const half = (b.fov / 2) * Math.PI / 180;
      const R = b.r * SCALE;
      ctx.fillStyle = 'rgba(125,211,252,0.07)';
      ctx.beginPath();
      ctx.moveTo(ox, oy);
      ctx.arc(ox, oy, R, -dir - half, -dir + half);
      ctx.closePath();
      ctx.fill();
    }

    // ex/ey is where the on-axis laser actually terminates. The reported range
    // can come from an off-axis ray in the cone, so hx/hy is drawn separately --
    // otherwise a post picked up off-axis makes the beam stop in empty space.
    const [ex, ey] = toPx(b.ex, b.ey);
    const a = b.hit ? 0.25 + 0.6 * (b.conf / 63) : 0.12;
    ctx.strokeStyle = `rgba(125,211,252,${a})`;
    ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ex, ey); ctx.stroke();

    const [hx, hy] = toPx(b.hx, b.hy);
    if (b.hit && b.obs) {
      // A post owns this return, so every filter should be discarding it. Drawn
      // as a cross at the real intersection, not as a range dot.
      ctx.strokeStyle = '#f87171';
      ctx.lineWidth = 1.6;
      const k = 4;
      ctx.beginPath();
      ctx.moveTo(hx - k, hy - k); ctx.lineTo(hx + k, hy + k);
      ctx.moveTo(hx + k, hy - k); ctx.lineTo(hx - k, hy + k);
      ctx.stroke();
    } else if (b.hit) {
      ctx.fillStyle = '#7dd3fc';
      ctx.beginPath(); ctx.arc(hx, hy, 2.5, 0, Math.PI * 2); ctx.fill();
    }
  }
}

function drawPlan() {
  // Sampled RAMSETE trajectories, drawn as the continuous curve they are.
  for (const pts of sampledPaths) {
    if (pts.length < 2) continue;
    ctx.strokeStyle = 'rgba(167,139,250,0.85)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    pts.forEach((q, i) => {
      const [px, py] = toPx(q.x, q.y);
      i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
    });
    ctx.stroke();

    // Heading ticks every few samples, so a reversed or twisted spline is
    // obvious before you run it.
    ctx.strokeStyle = 'rgba(167,139,250,0.45)';
    ctx.lineWidth = 1;
    for (let i = 0; i < pts.length; i += 4) {
      const q = pts[i];
      const rad = (90 - q.h) * Math.PI / 180;
      const [px, py] = toPx(q.x, q.y);
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(px + Math.cos(rad) * 7, py - Math.sin(rad) * 7);
      ctx.stroke();
    }
  }

  if (plan.length < 1) return;
  ctx.strokeStyle = 'rgba(90,169,255,0.55)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([5, 4]);
  ctx.beginPath();
  plan.forEach((p, i) => {
    const [px, py] = toPx(p.x, p.y);
    i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
  });
  ctx.stroke();
  ctx.setLineDash([]);

  plan.forEach((p, i) => {
    const [px, py] = toPx(p.x, p.y);
    ctx.fillStyle = p.kind === 'start' ? '#4ade80'
                  : p.kind === 'back' ? '#fb923c'
                  : p.kind === 'via' ? '#a78bfa' : '#5aa9ff';
    ctx.beginPath(); ctx.arc(px, py, 4, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#0f1115';
    ctx.font = 'bold 9px ui-monospace, monospace';
    ctx.textAlign = 'center';
    ctx.fillText(String(i), px, py + 3);
    ctx.textAlign = 'left';
  });
}

function drawRobot(pose, color, filled) {
  const rad = (90 - pose.h) * Math.PI / 180;
  const [px, py] = toPx(pose.x, pose.y);
  const L = 15 * SCALE, W = 15 * SCALE;
  ctx.save();
  ctx.translate(px, py);
  ctx.rotate(-rad);
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  if (filled) {
    ctx.fillStyle = color.replace('rgb(', 'rgba(').replace(')', ',0.14)');
    ctx.fillRect(-L / 2, -W / 2, L, W);
  }
  ctx.strokeRect(-L / 2, -W / 2, L, W);
  ctx.beginPath();
  ctx.moveTo(L / 2, 0); ctx.lineTo(L / 2 - 6, -5);
  ctx.moveTo(L / 2, 0); ctx.lineTo(L / 2 - 6, 5);
  ctx.stroke();
  ctx.restore();
}

function drawPoseMarker(pose, color) {
  const [px, py] = toPx(pose.x, pose.y);
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(px, py, 5, 0, Math.PI * 2); ctx.stroke();
  const rad = (90 - pose.h) * Math.PI / 180;
  ctx.beginPath();
  ctx.moveTo(px, py);
  ctx.lineTo(px + Math.cos(rad) * 14, py - Math.sin(rad) * 14);
  ctx.stroke();
}

function drawRclRays(s) {
  if (!s.filters) return;
  for (const f of s.filters) {
    if (!f.rclValid) continue;
    const [ox, oy] = toPx(f.rx, f.ry);
    const [ex, ey] = toPx(f.rx + f.rdx * f.rclIn, f.ry + f.rdy * f.rclIn);
    ctx.strokeStyle = 'rgba(52,211,153,0.7)';
    ctx.lineWidth = 1.6;
    ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ex, ey); ctx.stroke();
    ctx.fillStyle = '#34d399';
    ctx.beginPath(); ctx.arc(ex, ey, 3, 0, Math.PI * 2); ctx.fill();
  }
}

function drawGhost(row) {
  // Where the robot actually was at the scrub cursor, plus what odometry
  // believed at that instant.
  drawRobot(row.true, 'rgb(148,163,184)', false);
  const [px, py] = toPx(row.odom.x, row.odom.y);
  ctx.strokeStyle = 'rgba(251,191,36,0.55)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([3, 3]);
  ctx.beginPath(); ctx.arc(px, py, 5, 0, Math.PI * 2); ctx.stroke();
  ctx.setLineDash([]);
}

function render(s) {
  ctx.clearRect(0, 0, SIZE, SIZE);
  if (layers.field) drawField();
  if (layers.grid) drawGrid();
  if (layers.obstacles) drawObstacles();
  if (layers.plan) drawPlan();
  if (layers.truePath) drawTrail(api.trailTruePtr, api.trailTrueCnt, 'rgba(74,222,128,0.5)');
  if (layers.odomPath) drawTrail(api.trailOdomPtr, api.trailOdomCnt, 'rgba(251,191,36,0.5)');
  if (layers.particles) drawParticles();
  if (layers.beams) drawBeams(s);
  if (layers.rcl && (s.locMode & 2)) { drawRclRays(s); drawPoseMarker(s.rcl, '#34d399'); }
  if (layers.odom) drawPoseMarker(s.odom, '#fbbf24');
  if (layers.mcl && (s.locMode & 1)) drawPoseMarker(s.mcl, '#f472b6');
  if (layers.ghost && replayRow) drawGhost(replayRow);
  if (layers.robot) drawRobot(s.true, s.colliding ? 'rgb(248,113,113)' : 'rgb(74,222,128)', true);
}

// ============================================================================
// HUD
// ============================================================================

const hud = Object.fromEntries(
  ['t', 'true', 'odom', 'mcl', 'ekf', 'ekfsig', 'err', 'speed', 'sensors',
   'neff', 'tasks']
    .map(k => [k, document.getElementById('hud-' + k)]));

const diagEl = document.getElementById('sensor-diag');

function updateDiag(s) {
  if (!s.filters) return;
  const cell = (txt, cls) => `<div class="${cls}">${txt}</div>`;
  let html = cell('#', 'hdr') + cell('sim', 'hdr r') + cell('mcl', 'hdr r') + cell('rcl', 'hdr r');

  for (const f of s.filters) {
    const beam = s.beams.find(b => b.i === f.i);
    const simTxt = beam && beam.hit ? `${beam.r.toFixed(1)}"` : '—';
    // Deadzones are judged per particle, so a beam is not simply used or
    // dropped: mclDead is the share of the cloud that ignored it. Reading only
    // mclValid hid them completely and made the obstacles look inert.
    const dead = f.mclDead || 0;
    let mclTxt, mclCls;
    if (!(s.locMode & 1))   { mclTxt = '–';    mclCls = 'off'; }
    else if (!f.mclValid)   { mclTxt = 'drop'; mclCls = 'bad'; }
    else if (dead > 0.60)   { mclTxt = 'dead ' + Math.round(dead * 100) + '%'; mclCls = 'bad'; }
    else if (dead > 0.05)   { mclTxt = 'part ' + Math.round(dead * 100) + '%'; mclCls = 'warn'; }
    else                    { mclTxt = 'use';  mclCls = 'ok'; }
    const rclCls = (s.locMode & 2) ? (f.rclValid ? 'ok' : 'bad') : 'off';
    html += cell(f.i, 's')
          + cell(simTxt, 'r')
          + cell(mclTxt, `r ${mclCls}`)
          + cell((s.locMode & 2) ? (f.rclValid ? 'use' : 'drop') : '–', `r ${rclCls}`);
  }
  diagEl.innerHTML = html;
}

const fmtPose = p => `${p.x.toFixed(2)}, ${p.y.toFixed(2)} @ ${p.h.toFixed(1)}°`;

function updateHud(s) {
  hud.t.textContent = s.t.toFixed(2) + ' s';
  hud.true.textContent = fmtPose(s.true);
  hud.odom.textContent = fmtPose(s.odom);
  hud.mcl.textContent = (s.locMode & 1) ? fmtPose(s.mcl) : '—';
  hud.ekf.textContent = (s.locMode & 4) ? fmtPose(s.ekf) : '—';
  hud.ekfsig.textContent = (s.locMode & 4)
    ? `${s.ekf.sx.toFixed(2)} / ${s.ekf.sy.toFixed(2)} in  ${s.ekf.st.toFixed(2)}°`
      + `  ${s.ekf.beams} beam${s.ekf.beams === 1 ? '' : 's'}`
      + (s.ekf.reloc > 0 ? `  ${s.ekf.reloc} reloc` : '')
    : '—';
  hud.err.textContent = `${s.err.dist.toFixed(2)} in / ${s.err.heading.toFixed(1)}°`;
  hud.speed.textContent = `${s.vel.lin.toFixed(1)} in/s  ${s.vel.ang.toFixed(0)} °/s`;
  const valid = s.beams.filter(b => b.hit).length;
  hud.sensors.textContent = `${valid}/${s.beams.length} valid`;
  // Injection above zero means the beam likelihoods stopped fitting -- the
  // cloud is being redrawn across the field to find the robot again.
  hud.neff.textContent = (s.locMode & 1)
    ? `${s.mcl.nEff.toFixed(0)} / 1024`
      + (s.mclInject > 0.001 ? `  inject ${(s.mclInject * 100).toFixed(0)}%` : '')
    : '—';
  const ri = api.routeIndex();
  hud.tasks.textContent = `${s.tasks} live` + (ri >= 0 ? ` · cmd ${ri}` : '');
  updateDiag(s);
}

// ============================================================================
// Main loop
// ============================================================================

let lastState = null;
let lastFrame = performance.now();
let plotTick = 0;

function frame(now) {
  const wallDt = Math.min(now - lastFrame, 100);
  lastFrame = now;

  if (running) {
    applyKeyboardDrive();
    api.step(wallDt * speed);
  }

  lastState = JSON.parse(api.stateJson());
  render(lastState);
  updateHud(lastState);

  // Plots are cheap but not free; refresh a few times a second.
  if (++plotTick % 6 === 0) { if (replayOn.checked) updateReplay(); drawPlots(); }

  requestAnimationFrame(frame);
}

requestAnimationFrame(frame);

// Drivetrain facts, straight from the compiled config.
document.getElementById('dt-facts').innerHTML = [
  ['track width', '10.4 in'],
  ['wheel', '3.25 in'],
  ['drive RPM', '450'],
  ['sensors', '4 × Distance'],
  ['particles', '1024'],
].map(([k, v]) => `<div class="k">${k}</div><div class="v">${v}</div>`).join('');
