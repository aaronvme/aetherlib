// Does the browser front end actually boot?
//
//   node sim/test/app_boot.mjs
//
// app.js had no test at all, and the gap bit exactly once: a rename left
// drawPlan() referencing a variable that no longer existed, frame() threw on
// its FIRST call, and the page came up with a dead clock and an empty field.
// Every suite still passed, because none of them load app.js.
//
// So this is deliberately shallow. It does not check what was drawn -- it
// checks that module init, a handful of frames, and each button's handler run
// without throwing, against a DOM stubbed just far enough to get through them.
// A stale reference anywhere on those paths is a hard failure here.

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};

// ============================================================================
// DOM stub
// ============================================================================

const listeners = new Map();          // `${id}:${type}` -> [fn]
const elements = new Map();

// Canvas context: every drawing call is a no-op, every property readable.
const ctxStub = new Proxy({}, {
  get(_, prop) {
    if (prop === 'canvas') return elements.get('field');
    if (prop === 'measureText') return () => ({ width: 10 });
    if (prop === 'createLinearGradient') {
      return () => ({ addColorStop() {} });
    }
    return typeof prop === 'string' && /^[a-z]/.test(prop) ? () => {} : undefined;
  },
  set() { return true; },
});

function mkEl(id) {
  const el = {
    id,
    value: '',
    textContent: '',
    innerHTML: '',
    className: '',
    checked: false,
    disabled: false,
    width: 900,
    height: 900,
    style: {
      vars: {},
      setProperty(k, v) { this.vars[k] = v; },
      removeProperty(k) { delete this.vars[k]; },
    },
    dataset: {},
    children: [],
    classList: { add() {}, remove() {}, toggle() {}, contains: () => false },
    addEventListener(type, fn) {
      const key = `${id}:${type}`;
      if (!listeners.has(key)) listeners.set(key, []);
      listeners.get(key).push(fn);
    },
    removeEventListener() {},
    setPointerCapture() {},
    releasePointerCapture() {},
    appendChild(c) { el.children.push(c); return c; },
    getBoundingClientRect: () => ({ left: 0, top: 0, width: 900, height: 900 }),
    getContext: () => ctxStub,
    querySelector: () => mkEl(id + ' >'),
    querySelectorAll: () => [],
    getAttribute: () => null,
    setAttribute() {},
    focus() {},
    select() {},
    scrollIntoView() {},
  };
  return el;
}

const el = id => {
  if (!elements.has(id)) elements.set(id, mkEl(id));
  return elements.get(id);
};

const fire = (id, type, event = {}) => {
  for (const fn of listeners.get(`${id}:${type}`) ?? []) fn(event);
};

globalThis.document = {
  getElementById: el,
  createElement: tag => mkEl('<' + tag + '>'),
  querySelector: () => mkEl('?'),
  querySelectorAll: () => [],
  addEventListener() {},
  body: mkEl('body'),
  documentElement: el('html'),
};
globalThis.window = globalThis;
globalThis.devicePixelRatio = 1;
globalThis.innerWidth = 1600;
globalThis.getComputedStyle = () => ({
  getPropertyValue: k => el('html').style.vars[k] ?? '0',
});
globalThis.addEventListener = () => {};
globalThis.ResizeObserver = class { observe() {} disconnect() {} };
// The field backdrop is loaded as an <img>; nothing here decodes one, so the
// stub only has to let the load path run and report a miss.
globalThis.Image = class {
  set src(v) { this._src = v; if (this.onerror) this.onerror(); }
  get src() { return this._src; }
};
globalThis.URL.createObjectURL = () => 'blob:stub';
globalThis.performance = { now: () => nowMs };
Object.defineProperty(globalThis.navigator, 'clipboard', {
  value: { writeText: async () => {} }, configurable: true,
});

let nowMs = 0;
const frames = [];
globalThis.requestAnimationFrame = fn => { frames.push(fn); return frames.length; };

const store = new Map();
globalThis.localStorage = {
  getItem: k => (store.has(k) ? store.get(k) : null),
  setItem: (k, v) => store.set(k, String(v)),
  removeItem: k => store.delete(k),
};

// ============================================================================
// Boot
// ============================================================================

console.log('\n== module init ==');

let booted = false;
try {
  await import('../web/app.js');
  booted = true;
  check('app.js loads and initialises', true);
} catch (e) {
  check('app.js loads and initialises', false, `${e.constructor.name}: ${e.message}`);
  console.log((e.stack || '').split('\n').slice(1, 5).join('\n'));
}

if (!booted) {
  console.log('\n1 CHECK(S) FAILED\n');
  process.exit(1);
}

check('the default route parsed into a plan',
      !/^\s*$/.test(el('route-status').textContent) &&
      !el('route-status').className.includes('err'),
      `status: ${el('route-status').textContent}`);

check('the editor was seeded with auton source',
      /chassis\.\w+\(/.test(el('editor').value),
      el('editor').value.split('\n').find(l => l.includes('chassis.')) ?? '(none)');

// The drivetrain panel asks the wasm for these rather than asserting them, and
// every other check here only proves nothing threw -- which is how it printed a
// 3.25 in wheel on a 2.6738 in drivetrain for as long as it did. A missing or
// misspelled getter returns NaN and formats as "NaN", so the values are checked
// without naming any of them: hardcoding the robot's constants here is the
// mistake being guarded against.
{
  const facts = el('dt-facts').innerHTML;
  const values = [...facts.matchAll(/<div class="v">([^<]*)<\/div>/g)].map(m => m[1]);
  check('the drivetrain panel filled in', values.length === 6,
        `${values.length} values: ${values.join(' | ')}`);
  // Positive, not merely non-NaN: a getter that exists but reads zero is the
  // other way this goes wrong, and it is how "0 × Distance" shipped once.
  check('every drivetrain fact is a positive number',
        values.length > 0 && values.every(v => parseFloat(v) > 0),
        values.join(' | '));
}

// ============================================================================
// Frames
// ============================================================================

console.log('\n== the render loop ==');

const runFrames = (n) => {
  for (let i = 0; i < n; i++) {
    const fn = frames.shift();
    if (!fn) return `no frame queued at ${i}`;
    nowMs += 16;
    fn(nowMs);
  }
  return null;
};

let err = null;
try {
  err = runFrames(1);
} catch (e) {
  err = `${e.constructor.name}: ${e.message}` +
        '\n      ' + (e.stack || '').split('\n')[1]?.trim();
}
check('the first frame renders', err === null, err ?? '');

try {
  err = runFrames(20);
} catch (e) {
  err = `${e.constructor.name}: ${e.message}` +
        '\n      ' + (e.stack || '').split('\n')[1]?.trim();
}
check('it keeps rendering, and re-arms itself each time', err === null, err ?? '');

// ============================================================================
// Handlers
// ============================================================================
//
// Every button, because a handler that throws is invisible until someone clicks
// it -- and half of them touch the route parser.

console.log('\n== button handlers ==');

const clickable = [
  'btn-run-route', 'btn-stop', 'btn-copy', 'btn-save',
  'btn-run-auton', 'btn-play', 'btn-step', 'btn-reset', 'btn-place',
  'btn-clear-rec', 'btn-field-apply', 'btn-field-reset', 'hud-toggle',
];

for (const id of clickable) {
  const had = (listeners.get(`${id}:click`) ?? []).length > 0;
  if (!had) { check(`${id} has a handler`, false, 'no click listener registered'); continue; }
  let e = null;
  try { fire(id, 'click'); } catch (ex) {
    e = `${ex.constructor.name}: ${ex.message}`;
  }
  check(`${id} click`, e === null, e ?? '');
}

// Running a route then letting frames run exercises the report path, which only
// renders on the falling edge of routeRunning.
console.log('\n== a route runs and reports ==');

try {
  fire('btn-run-route', 'click');
  err = runFrames(400);
} catch (e) {
  err = `${e.constructor.name}: ${e.message}` +
        '\n      ' + (e.stack || '').split('\n')[1]?.trim();
}
check('frames keep rendering while a route runs', err === null, err ?? '');

try {
  fire('btn-stop', 'click');
  err = runFrames(5);
} catch (e) {
  err = `${e.constructor.name}: ${e.message}`;
}
check('stopping it renders the report without throwing', err === null, err ?? '');

// The editor's own input handler re-parses on every keystroke, including while
// the text is a half-typed statement.
console.log('\n== editing ==');

const partials = [
  '',
  'chassis.',
  'chassis.moveToPoint(',
  'chassis.moveToPoint(1.0f,',
  'chassis.moveToPoint(1.0f, 2.0f);',
  'void autonX() {\n    chassis.moveDistance(5.0f);\n}',
  'std::vector<Pose> p = {',
  '{{{{',
  '}}}}',
  '/* unterminated',
];
let editErr = null;
for (const text of partials) {
  el('editor').value = text;
  try { fire('editor', 'input'); } catch (e) {
    editErr = `${JSON.stringify(text.slice(0, 24))} -> ${e.constructor.name}: ${e.message}`;
    break;
  }
}
check('half-typed source never throws, only reports', editErr === null, editErr ?? '');

// Shift-click appends a statement; a plain drag kidnaps the robot.
try {
  el('editor').value = 'void autonX() {\n    chassis.moveDistance(5.0f);\n}\n';
  fire('editor', 'input');
  fire('field', 'pointerdown', { shiftKey: true, clientX: 400, clientY: 300, pointerId: 1 });
  err = /moveToPoint/.test(el('editor').value) ? null : 'no statement was appended';
} catch (e) {
  err = `${e.constructor.name}: ${e.message}`;
}
check('shift-click appends a moveToPoint to the source', err === null,
      err ?? el('editor').value.split('\n').filter(l => l.includes('moveToPoint'))[0]?.trim());

try {
  fire('field', 'pointerdown', { shiftKey: false, clientX: 300, clientY: 300, pointerId: 2 });
  fire('field', 'pointermove', { clientX: 320, clientY: 320, pointerId: 2 });
  fire('field', 'pointerup', { pointerId: 2 });
  err = null;
} catch (e) {
  err = `${e.constructor.name}: ${e.message}`;
}
check('dragging on the field does not throw', err === null, err ?? '');

console.log('\n== panel resizing ==');

// The stub reports every element as 900 px wide, so dragging 300 px left
// should land the panel on 600 -- below the 720 px cap, so the clamp does
// not mask the arithmetic.
{
  const vars = el('html').style.vars;
  fire('gutter-left', 'pointerdown',
       { preventDefault() {}, pointerId: 1, clientX: 900 });
  fire('gutter-left', 'pointermove', { clientX: 600 });
  check('dragging a gutter resizes the panel', vars['--left-w'] === '600px',
        `--left-w = ${vars['--left-w']}`);

  // Past the 45% cap, which exists so the field cannot be squeezed away.
  fire('gutter-left', 'pointermove', { clientX: 3000 });
  check('and is clamped to a fraction of the window',
        parseFloat(vars['--left-w']) <= innerWidth * 0.45 + 0.5,
        `--left-w = ${vars['--left-w']}, cap ${innerWidth * 0.45}`);

  // And to a floor, so it cannot be dragged shut and become ungrabbable.
  fire('gutter-left', 'pointermove', { clientX: -3000 });
  check('and to a floor', parseFloat(vars['--left-w']) >= 150,
        `--left-w = ${vars['--left-w']}`);

  fire('gutter-left', 'pointerup', {});
  check('the width is remembered',
        store.get('chad-sim.panel.left') === String(parseFloat(vars['--left-w'])),
        `stored ${store.get('chad-sim.panel.left')}`);

  fire('gutter-left', 'dblclick', {});
  check('double-click restores the default',
        vars['--left-w'] === undefined &&
        !store.has('chad-sim.panel.left'),
        `--left-w = ${vars['--left-w']}`);
}

// A long route fills the report until it buries the run buttons under it.
{
  const card = el('report-card');
  let collapsed = false;
  card.classList.toggle = (name, on) => { collapsed = on; return on; };
  card.classList.contains = () => collapsed;
  fire('report-toggle', 'click', {});
  check('the run report collapses', collapsed === true, `collapsed=${collapsed}`);
  fire('report-toggle', 'click', {});
  check('and reopens', collapsed === false, `collapsed=${collapsed}`);
}

console.log(`\n${failures === 0 ? 'ALL CHECKS PASSED' : failures + ' CHECK(S) FAILED'}\n`);
process.exit(failures === 0 ? 0 : 1);
