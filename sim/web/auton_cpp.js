// chad-sim — auton source parser.
//
// Parses real auton C++ into the opcode list the WASM bridge executes against
// the real Chassis API. The editor holds the same text you paste into
// src/gamespecific/autons/, so there is no second language to learn and no
// translation step to get wrong:
//
//   void autonBlueLeft() {
//       chassis.setPose(Pose::fromVex(-48.0f, -60.0f, 90.0f));
//       chassis.moveToPoint(-24.0f, -60.0f, 2500);
//       chassis.turnToHeading(0.0f, 1200, {.maxSpeed = 90.0f});
//       chassis.moveDistance(-12.0f, 1500);
//       pros::delay(300);
//   }
//
// This is a subset, not a C++ front end. It understands chassis calls,
// pros::delay, and a std::vector<Pose> of control points for moveRAMSETE.
// Anything else
// -- a lift call, a loop, a local variable -- is SKIPPED and listed in `notes`
// rather than dropped silently, because a route that quietly ignored half your
// auton would be worse than no preview at all.
//
// Two kinds of mistake are reported as errors rather than skipped, because both
// are compile errors on the robot and finding them here is free:
//
//   * a designated initializer naming a field the struct does not have
//   * designated initializers out of declaration order (C++20 requires the
//     order to match, and the compiler's message for it is not obvious)
//
// Waypoints carry the character range of their x and y literals in the ORIGINAL
// text, so dragging a point on the field splices new numbers into the source
// without reformatting anything else.

import { OP } from './route_dsl.js';

export { OP };

// ============================================================================
// The params structs, as the headers declare them
// ============================================================================
//
// Order matters: C++20 designated initializers must appear in declaration
// order, so this doubles as the check for that. sim/test/auton_cpp.mjs reads
// include/chad/movement_control/*.hpp and asserts these lists still match,
// which is the only thing stopping them going stale.

export const PARAM_FIELDS = {
  MoveDistanceParams: ['forwards', 'maxSpeed', 'minSpeed', 'targetTolerance',
                       'earlyExitRange'],
  MoveToPointParams: ['forwards', 'maxSpeed', 'minSpeed', 'targetTolerance',
                      'earlyExitRange', 'lead', 'bearingHoldDist'],
  TurntoHeadingParams: ['direction', 'maxSpeed', 'minSpeed', 'targetTolerance'],
  TurnToPointParams: ['direction', 'maxSpeed', 'minSpeed', 'targetTolerance',
                      'offset'],
  SwingParams: ['direction', 'maxSpeed', 'minSpeed', 'targetTolerance'],
  ArcParams: ['forwards', 'maxSpeed', 'minSpeed', 'targetTolerance', 'lead'],
  BoomerangParams: ['forwards', 'maxSpeed', 'minSpeed', 'targetTolerance',
                    'headingTolerance', 'lead', 'leadDecay'],
  RAMSETEParams: ['b', 'zeta', 'smoothSpacingIn', 'maxSpeed', 'minSpeed',
                  'targetTolerance',
                  'headingTolerance', 'stallSpeed', 'lookaheadDist',
                  'useVelocityProfile', 'maxAccel', 'maxLateralAccel',
                  'useVelocityController', 'ffGains', 'settleTimeMs'],
};

// Fields the sim cannot carry across the C API, which takes one float per
// field. Named so the report can say so instead of appearing to honour them.
const NON_SCALAR_FIELDS = new Set(['ffGains']);

const ENUMS = {
  'TurnDirection::CCW': 0,
  'TurnDirection::CW': 1,
  'TurnDirection::SHORTEST': 2,
  'SwingSide::Left': 0,
  'SwingSide::Right': 1,
};

// ============================================================================
// Lexical helpers
// ============================================================================

/// Blank comments and preprocessor lines, preserving length so every offset
/// captured afterwards still points at the right character of the original.
function blankNonCode(text) {
  let out = text.split('');
  const blank = (from, to) => {
    for (let i = from; i < to && i < out.length; i++) {
      if (out[i] !== '\n') out[i] = ' ';
    }
  };

  // Tracked rather than re-derived: testing "is this the start of a line" by
  // slicing the prefix makes the whole scan quadratic in the file length.
  let atLineStart = true;

  for (let i = 0; i < text.length; i++) {
    if (text[i] === '/' && text[i + 1] === '/') {
      let j = text.indexOf('\n', i);
      if (j === -1) j = text.length;
      blank(i, j);
      i = j;
      atLineStart = true;
      continue;
    } else if (text[i] === '/' && text[i + 1] === '*') {
      let j = text.indexOf('*/', i + 2);
      j = j === -1 ? text.length : j + 2;
      blank(i, j);
      i = j - 1;
      continue;
    } else if (text[i] === '#' && atLineStart) {
      let j = text.indexOf('\n', i);
      if (j === -1) j = text.length;
      blank(i, j);
      i = j;
      atLineStart = true;
      continue;
    } else if (text[i] === '"') {
      let j = i + 1;
      while (j < text.length && !(text[j] === '"' && text[j - 1] !== '\\')) j++;
      blank(i, Math.min(j + 1, text.length));
      i = j;
    }
    if (text[i] === '\n') atLineStart = true;
    else if (!/\s/.test(text[i] ?? '')) atLineStart = false;
  }
  return out.join('');
}

/// Split source into statements, respecting nesting.
///
/// A statement ends at a `;` outside every bracket, or at a bare `{` / `}`
/// which is structural (a function body or a namespace). Splitting on `;`
/// alone would cut a `std::vector<Pose> p = {{1,2,3}, {4,5,6}};` in half.
function splitStatements(scan) {
  const out = [];
  let depth = 0;         // () and [] and <> we do not track; {} we do
  let paren = 0;
  let start = 0;

  for (let i = 0; i < scan.length; i++) {
    const ch = scan[i];
    if (ch === '(') paren++;
    else if (ch === ')') paren--;
    else if (ch === '{') {
      if (paren === 0 && depth === 0 && !/[=,({\[]\s*$/.test(scan.slice(start, i))) {
        // A brace that does not continue an expression opens a block.
        out.push({ text: scan.slice(start, i), start, structural: true });
        start = i + 1;
        continue;
      }
      depth++;
    } else if (ch === '}') {
      if (depth === 0 && paren === 0) {
        out.push({ text: scan.slice(start, i), start, structural: true });
        start = i + 1;
        continue;
      }
      depth--;
    } else if (ch === ';' && paren === 0 && depth === 0) {
      out.push({ text: scan.slice(start, i), start, structural: false });
      start = i + 1;
    }
  }
  if (start < scan.length) {
    out.push({ text: scan.slice(start), start, structural: true });
  }
  return out;
}

/// Split an argument list on top-level commas, keeping each piece's absolute
/// range in the original text.
function splitArgs(text, base) {
  const parts = [];
  let depth = 0, start = 0;
  for (let i = 0; i < text.length; i++) {
    const ch = text[i];
    if (ch === '(' || ch === '{' || ch === '[') depth++;
    else if (ch === ')' || ch === '}' || ch === ']') depth--;
    else if (ch === ',' && depth === 0) {
      parts.push({ text: text.slice(start, i), start: base + start });
      start = i + 1;
    }
  }
  if (text.slice(start).trim()) {
    parts.push({ text: text.slice(start), start: base + start });
  }
  return parts;
}

/// Absolute [from, to) of the trimmed content of an argument.
function argRange(arg) {
  const lead = arg.text.length - arg.text.trimStart().length;
  const from = arg.start + lead;
  return [from, from + arg.text.trim().length];
}

// ============================================================================
// Expression evaluation
// ============================================================================
//
// Real code writes `static_cast<float>(M_PI) / 2.0f`, not `1.5708`. A tiny
// recursive-descent evaluator handles the arithmetic that actually turns up in
// a pose literal, without handing the editor's contents to eval().

const FUNCS = {
  degToRad: v => v * Math.PI / 180,
  radToDeg: v => v * 180 / Math.PI,
  vexToStdRad: v => wrapRad((90 - v) * Math.PI / 180),
  stdRadToVexDeg: v => wrapDeg(90 - v * 180 / Math.PI),
  fabs: Math.abs,
  abs: Math.abs,
  sqrt: Math.sqrt,
  sin: Math.sin,
  cos: Math.cos,
};

const CONSTS = { M_PI: Math.PI, PI: Math.PI, M_PI_2: Math.PI / 2 };

export const wrapRad = r => {
  let x = (r + Math.PI) % (2 * Math.PI);
  if (x < 0) x += 2 * Math.PI;
  return x - Math.PI;
};
export const wrapDeg = d => ((d % 360) + 360) % 360;

/// Evaluate a C++ numeric expression. Returns NaN if it is not one.
export function evalExpr(src) {
  const text = String(src)
    .replace(/static_cast\s*<\s*[A-Za-z_:<>\s]*\s*>/g, '')
    .replace(/\bchad::/g, '')
    .replace(/\bpros::/g, '');

  let i = 0;
  const ws = () => { while (i < text.length && /\s/.test(text[i])) i++; };

  const primary = () => {
    ws();
    if (text[i] === '(') {
      i++;
      const v = expr();
      ws();
      if (text[i] !== ')') return NaN;
      i++;
      return v;
    }
    if (text[i] === '-') { i++; return -primary(); }
    if (text[i] === '+') { i++; return primary(); }

    const name = /^[A-Za-z_]\w*/.exec(text.slice(i));
    if (name) {
      i += name[0].length;
      ws();
      if (text[i] === '(') {
        i++;
        const v = expr();
        ws();
        if (text[i] !== ')') return NaN;
        i++;
        const fn = FUNCS[name[0]];
        return fn ? fn(v) : NaN;
      }
      return name[0] in CONSTS ? CONSTS[name[0]] : NaN;
    }

    const num = /^(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?[fFlLuU]*/.exec(text.slice(i));
    if (!num) return NaN;
    i += num[0].length;
    return parseFloat(num[0]);
  };

  const term = () => {
    let v = primary();
    for (;;) {
      ws();
      if (text[i] === '*') { i++; v *= primary(); }
      else if (text[i] === '/') { i++; v /= primary(); }
      else return v;
    }
  };

  const expr = () => {
    let v = term();
    for (;;) {
      ws();
      if (text[i] === '+') { i++; v += term(); }
      else if (text[i] === '-') { i++; v -= term(); }
      else return v;
    }
  };

  const value = expr();
  ws();
  return i === text.length ? value : NaN;
}

/// Evaluate a value that may be a bool, an enum, or a number.
function evalValue(src) {
  const t = String(src).trim().replace(/\bchad::/g, '');
  if (t === 'true') return 1;
  if (t === 'false') return 0;
  if (t in ENUMS) return ENUMS[t];
  return evalExpr(t);
}

// ============================================================================
// Method table
// ============================================================================
//
// `pos` names the leading positional arguments. `timeoutAt` is the index the
// timeout sits at; everything after it is the params block and then the async
// flag, exactly as the Chassis overloads are declared.

const METHODS = {
  setPose:         { op: OP.SET_POSE, pos: ['pose'] },
  moveDistance:    { op: OP.DRIVE, pos: ['d'], timeoutAt: 1, params: 'MoveDistanceParams' },
  moveToPoint:     { op: OP.MOVE_TO, pos: ['x', 'y'], timeoutAt: 2, params: 'MoveToPointParams' },
  turnToHeading:   { op: OP.TURN_HEADING, pos: ['h'], timeoutAt: 1, params: 'TurntoHeadingParams' },
  turnToPoint:     { op: OP.TURN_POINT, pos: ['x', 'y'], timeoutAt: 2, params: 'TurnToPointParams' },
  moveArc:         { op: OP.ARC, pos: ['x', 'y', 'r'], timeoutAt: 3, params: 'ArcParams' },
  moveBoomerang:   { op: OP.BOOMERANG, pos: ['x', 'y', 'h'], timeoutAt: 3, params: 'BoomerangParams' },
  swingToHeading:  { op: OP.SWING, pos: ['h', 'side'], timeoutAt: 2, params: 'SwingParams' },
  moveRAMSETE:     { op: OP.FOLLOW_PATH, pos: ['path'], timeoutAt: 1, params: 'RAMSETEParams' },
  waitUntilSettled:{ op: OP.WAIT_SETTLED, pos: [] },
  setLateralGains: { op: OP.LATERAL_GAINS, pos: ['gains'] },
  setAngularGains: { op: OP.ANGULAR_GAINS, pos: ['gains'] },
};

// Real chassis calls the sim knows about but cannot route. Skipped with a note
// rather than an error -- unlike a misspelled method, which is a typo worth
// catching here instead of at the next compile.
const UNROUTABLE = new Set([
  'calibrate', 'setPosition', 'setHeading', 'getPose', 'tank', 'arcade',
  'curvature', 'isSettled', 'cancelMotion', 'abortMotion', 'waitUntilDist',
  'odom', 'rcl',
  'mcl', 'ekf', 'startTracking', 'stopTracking', 'setPidTuning',
  'setThrottleCurve', 'setSteerCurve', 'setBrakeMode', 'setDistanceSensors',
  'setDryRun', 'isDryRun', 'dryRunPath', 'resetDryRunPath', 'setTelemetry',
  'config', 'setConfig',
]);

// ============================================================================
// Parsing
// ============================================================================

/**
 * @returns {{cmds, opts, plan, paths, notes, fnName} | {error: string}}
 */
export function parseAuton(text) {
  const src = String(text);
  const scan = blankNonCode(src);

  const cmds = [];
  const opts = [];
  const plan = [];
  const paths = [];
  const notes = [];
  const pathVars = new Map();     // name -> index into paths
  let fnName = null;

  for (const stmt of splitStatements(scan)) {
    const body = stmt.text.trim();
    if (!body) continue;

    // --- structure we simply step over --------------------------------------
    const fn = /^(?:void|static\s+void)\s+([A-Za-z_]\w*)\s*\(\s*(?:void)?\s*\)$/.exec(body);
    if (fn) { fnName ??= fn[1]; continue; }
    if (/^namespace\b/.test(body) || /^using\b/.test(body)) continue;
    if (/^extern\b/.test(body)) continue;

    // --- std::vector<Pose> path = { ... } -----------------------------------
    const vec =
      /^(?:const\s+)?std::vector\s*<\s*(?:chad::)?Pose\s*>\s*([A-Za-z_]\w*)\s*=\s*\{[\s\S]*\}$/
        .exec(body);
    if (vec) {
      // The initializer braces are nested, so splitStatements kept the whole
      // declaration together and the list can be read straight out of it.
      const braceStart = scan.indexOf('{', stmt.start);
      const braceEnd = matchBrace(scan, braceStart);
      if (braceStart === -1 || braceEnd === -1) {
        return { error: `unterminated pose list for ${vec[1]}` };
      }

      // These are CONTROL points, not the path. moveRAMSETE resamples them
      // along a spline before the controller sees anything, so three points
      // describe a curve -- on the robot as well as here, which is the whole
      // reason the smoothing lives in the library and not in this file.
      const poses = parsePoseList(scan, braceStart + 1, braceEnd);
      if (poses.error) return poses;
      pathVars.set(vec[1], paths.length);
      paths.push(poses.list);
      for (const q of poses.list) plan.push({ ...q, kind: 'via' });
      continue;
    }

    // --- pros::delay(ms) ----------------------------------------------------
    const delay = /^(?:pros::)?delay\s*\(([\s\S]*)\)$/.exec(body);
    if (delay) {
      const ms = evalExpr(delay[1]);
      if (!Number.isFinite(ms)) return { error: `delay(): ${delay[1].trim()}` };
      cmds.push([OP.DELAY, ms, 0, 0, 0]);
      opts.push({});
      continue;
    }

    // --- chassis.method(...) ------------------------------------------------
    const call = /^(?:chad::)?chassis\s*\.\s*([A-Za-z_]\w*)\s*\(([\s\S]*)\)$/.exec(body);
    if (!call) {
      notes.push(summarise(body));
      continue;
    }

    const name = call[1];
    if (UNROUTABLE.has(name)) { notes.push(summarise(body)); continue; }
    const spec = METHODS[name];
    if (!spec) return { error: `chassis has no method ${name}()` };

    const parsed = parseCall(spec, call[2], scan.indexOf('(', stmt.start) + 1,
                             pathVars);
    if (parsed.error) return parsed;

    cmds.push(parsed.cmd);
    opts.push(parsed.opt);
    if (parsed.waypoint) plan.push(parsed.waypoint);
    if (parsed.note) notes.push(parsed.note);
  }

  return { cmds, opts, plan, paths, notes, fnName };
}

/// Index of the `}` matching the `{` at `open`, or -1.
function matchBrace(text, open) {
  let depth = 0;
  for (let i = open; i < text.length; i++) {
    if (text[i] === '{') depth++;
    else if (text[i] === '}' && --depth === 0) return i;
  }
  return -1;
}

/// Parse `{ {x, y, h}, {x, y, h} }` into waypoints carrying source ranges.
function parsePoseList(scan, from, to) {
  const list = [];
  let i = from;
  while (i < to) {
    const open = scan.indexOf('{', i);
    if (open === -1 || open >= to) break;
    const close = matchBrace(scan, open);
    if (close === -1 || close > to) return { error: 'unterminated pose in the list' };

    const args = splitArgs(scan.slice(open + 1, close), open + 1);
    if (args.length < 2) return { error: 'a Pose needs at least x and y' };
    const x = evalExpr(args[0].text);
    const y = evalExpr(args[1].text);
    const theta = args.length > 2 ? evalExpr(args[2].text) : 0;
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(theta)) {
      return { error: `cannot read the pose {${scan.slice(open + 1, close).trim()}}` };
    }
    list.push({
      x, y,
      // Poses carry standard-math radians; the sim wants a VEX bearing.
      h: wrapDeg(90 - theta * 180 / Math.PI),
      xRange: argRange(args[0]),
      yRange: argRange(args[1]),
    });
    i = close + 1;
  }
  if (list.length < 2) return { error: 'a path needs at least two poses' };
  return { list };
}

function parseCall(spec, argsText, argsBase, pathVars) {
  const args = splitArgs(argsText, argsBase);
  const cmd = [spec.op, 0, 0, 0, 0];
  const opt = {};
  let waypoint = null;
  let note = null;

  const nPos = spec.pos.length;

  // --- the leading positional arguments --------------------------------------
  const need = (idx, what) => {
    if (idx >= args.length) return { error: `${what} is missing` };
    const v = evalValue(args[idx].text);
    if (!Number.isFinite(v)) {
      return { error: `cannot read ${what}: ${args[idx].text.trim()}` };
    }
    return { v };
  };

  if (spec.pos[0] === 'pose') {
    // setPose({x, y, thetaRad}) or setPose(Pose::fromVex(x, y, vexDeg)).
    const inner = /^\s*(?:chad::)?Pose\s*::\s*fromVex\s*\(([\s\S]*)\)\s*$/.exec(argsText);
    const braced = /^\s*\{([\s\S]*)\}\s*$/.exec(argsText);
    let parts, vexDegrees;
    if (inner) { parts = splitArgs(inner[1], argsBase + argsText.indexOf('(', inner.index) + 1); vexDegrees = true; }
    else if (braced) { parts = splitArgs(braced[1], argsBase + argsText.indexOf('{') + 1); vexDegrees = false; }
    else return { error: 'setPose() wants {x, y, theta} or Pose::fromVex(x, y, deg)' };

    if (parts.length < 3) return { error: 'setPose() wants three values' };
    const x = evalExpr(parts[0].text), y = evalExpr(parts[1].text);
    const th = evalExpr(parts[2].text);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(th)) {
      return { error: `cannot read the start pose: ${argsText.trim()}` };
    }
    // Pose.theta is standard-math radians; the opcode carries a VEX bearing.
    cmd[1] = x;
    cmd[2] = y;
    cmd[3] = vexDegrees ? wrapDeg(th) : wrapDeg(90 - th * 180 / Math.PI);
    waypoint = { x, y, kind: 'start', xRange: argRange(parts[0]), yRange: argRange(parts[1]) };
    return { cmd, opt, waypoint };
  }

  if (spec.pos[0] === 'gains') {
    const braced = /^\s*\{([\s\S]*)\}\s*$/.exec(argsText);
    if (!braced) return { error: 'gains want {kp, ki, kd}' };
    const parts = splitArgs(braced[1], argsBase + argsText.indexOf('{') + 1);
    for (let i = 0; i < 3; i++) {
      const v = i < parts.length ? evalExpr(parts[i].text) : 0;
      if (!Number.isFinite(v)) return { error: `cannot read gain ${i}` };
      cmd[i + 1] = v;
    }
    return { cmd, opt };
  }

  if (spec.pos[0] === 'path') {
    const varName = args[0]?.text.trim();
    if (!pathVars.has(varName)) {
      return { error: `moveRAMSETE(${varName || '?'}): no std::vector<Pose> by that name above it` };
    }
    cmd[1] = pathVars.get(varName);
  } else {
    // Every method's positional arguments land in cmd[1..3] in order, which is
    // what the bridge's opcode layout was built around.
    for (let i = 0; i < nPos; i++) {
      const key = spec.pos[i];
      const got = need(i, key === 'd' ? 'distance' : key);
      if (got.error) return got;
      cmd[i + 1] = got.v;
    }
    if (spec.op === OP.MOVE_TO) cmd[3] = 1;      // forward unless .forwards says otherwise
  }

  // --- timeout ---------------------------------------------------------------
  if (spec.timeoutAt !== undefined && args.length > spec.timeoutAt) {
    const t = evalExpr(args[spec.timeoutAt].text);
    if (!Number.isFinite(t)) {
      return { error: `cannot read the timeout: ${args[spec.timeoutAt].text.trim()}` };
    }
    cmd[4] = t;
  }

  // --- params block and the async flag ---------------------------------------
  for (let i = (spec.timeoutAt ?? nPos - 1) + 1; i < args.length; i++) {
    const t = args[i].text.trim();
    if (/^\{/.test(t)) {
      const res = parseParams(t, spec.params);
      if (res.error) return res;
      Object.assign(opt, res.opt);
      if (res.note) note = res.note;
    } else if (t === 'true') {
      opt._async = true;
    } else if (t !== 'false' && t !== '') {
      return { error: `unexpected argument ${t}` };
    }
  }

  // --- the waypoint, once .forwards is known --------------------------------
  if (spec.op === OP.MOVE_TO || spec.op === OP.BOOMERANG || spec.op === OP.ARC ||
      spec.op === OP.TURN_POINT) {
    const back = opt.forwards === 0;
    waypoint = {
      x: cmd[1], y: cmd[2],
      kind: spec.op === OP.TURN_POINT ? 'face' : (back ? 'back' : 'move'),
      xRange: argRange(args[0]),
      yRange: argRange(args[1]),
    };
  }

  return { cmd, opt, waypoint, note };
}

/// Parse a designated-initializer block, checking names and order.
function parseParams(text, structName) {
  const inner = /^\{([\s\S]*)\}$/.exec(text);
  if (!inner) return { error: `cannot read the params block ${text.slice(0, 30)}` };
  const fields = PARAM_FIELDS[structName] ?? [];
  const opt = {};
  let note = null;
  let lastIdx = -1;

  for (const part of splitArgs(inner[1], 0)) {
    const kv = /^\s*\.\s*([A-Za-z_]\w*)\s*=\s*([\s\S]*)$/.exec(part.text);
    if (!kv) {
      if (!part.text.trim()) continue;
      return {
        error: `${structName} takes designated initializers ` +
               `(.field = value), got ${part.text.trim()}`,
      };
    }
    const [, key, raw] = kv;
    const idx = fields.indexOf(key);
    if (idx === -1) {
      return { error: `${structName} has no field .${key}` };
    }
    if (idx < lastIdx) {
      return {
        error: `.${key} must come before .${fields[lastIdx]} — C++ wants ` +
               `designated initializers in declaration order`,
      };
    }
    lastIdx = idx;

    if (NON_SCALAR_FIELDS.has(key)) {
      note = `.${key} is not a number, so the sim leaves it at its default`;
      continue;
    }
    const v = evalValue(raw);
    if (!Number.isFinite(v)) {
      return { error: `cannot read .${key} = ${raw.trim()}` };
    }
    opt[key] = v;
  }
  return { opt, note };
}

const summarise = body => {
  const one = body.replace(/\s+/g, ' ').trim();
  return one.length > 48 ? one.slice(0, 45) + '...' : one;
};

// ============================================================================
// Writing source back out
// ============================================================================

/// Format a number the way the surrounding code does.
///
/// Real auton code writes `-24.0f`, and rewriting it as `-24` would compile to
/// a double and read as a different style on every drag. The `f` suffix is kept
/// when the literal being replaced had one.
export function fmtLike(original, value) {
  const rounded = Math.round(value * 10) / 10;
  const hasF = /[fF]\s*$/.test(original ?? '');
  const body = Number.isInteger(rounded) ? rounded.toFixed(1) : String(rounded);
  return hasF ? `${body}f` : body;
}

/**
 * Rewrite a waypoint's x/y literals in the source.
 * Splices the later range first so the earlier offset stays valid.
 */
export function moveWaypoint(text, waypoint, x, y) {
  if (!waypoint?.xRange || !waypoint?.yRange) return text;
  const edits = [
    { range: waypoint.xRange, value: x },
    { range: waypoint.yRange, value: y },
  ].sort((a, b) => b.range[0] - a.range[0]);

  let out = text;
  for (const e of edits) {
    const was = out.slice(e.range[0], e.range[1]);
    out = out.slice(0, e.range[0]) + fmtLike(was, e.value) + out.slice(e.range[1]);
  }
  return out;
}

/**
 * Append a statement inside the auton body.
 *
 * Appending at the end of the text would put it after the function's closing
 * brace, where it is neither run nor valid C++. This finds the last line that
 * is only closing braces and inserts above it, matching the indentation of the
 * statement before.
 */
export function appendStatement(text, statement) {
  const lines = text.split('\n');

  // Trailing blank lines and closing braces are two different things. Folding
  // them into one test made a body with no function wrapper look like one that
  // ended in a brace, and the new statement came out indented under nothing.
  let end = lines.length;
  while (end > 0 && lines[end - 1].trim() === '') end--;

  let closers = end;
  while (closers > 0 && /^\s*\}\s*(\/\/.*)?$/.test(lines[closers - 1])) closers--;

  if (closers === end) {                    // no brace to insert above
    const sep = /\n$/.test(text) ? '' : '\n';
    return `${text}${sep}${statement}\n`;
  }

  const prev = lines.slice(0, closers).reverse().find(l => l.trim() !== '');
  const indent = /^\s*/.exec(prev ?? '')[0] || '    ';
  lines.splice(closers, 0, indent + statement);
  return lines.join('\n');
}

const f = v => {
  const r = Math.round(v * 10) / 10;
  return `${Number.isInteger(r) ? r.toFixed(1) : r}f`;
};

/// The statement a shift-click on the field should add.
export const moveToStatement = (x, y, timeoutMs = 2000) =>
  `chassis.moveToPoint(${f(x)}, ${f(y)}, ${Math.round(timeoutMs)});`;

/**
 * Wrap the editor's body in a compilable file.
 *
 * The point of the whole exercise: what comes out of here is pasted into
 * src/gamespecific/autons/ unchanged.
 */
export function emitAuton(text, name = 'autonFromSim') {
  const already = /\bvoid\s+[A-Za-z_]\w*\s*\(\s*\)\s*\{/.test(text);
  const body = already
    ? text.trim()
    : ['void ' + name + '() {',
       ...text.trim().split('\n').map(l => (l.trim() ? '    ' + l.trim() : '')),
       '}'].join('\n');

  return [
    '#include "gamespecific/autons.hpp"',
    '',
    '#include "chad/api.hpp"',
    '',
    'namespace chad {',
    '',
    body,
    '',
    '}  // namespace chad',
    '',
  ].join('\n');
}

// ============================================================================
// The starting point
// ============================================================================

export const DEFAULT_AUTON = `// chad-sim route — this IS auton source. Paste it into
// src/gamespecific/autons/ when it drives the way you want.
//
// shift-click the field to append a moveToPoint; drag a numbered point to move
// it. Anything the sim cannot route (a lift call, a loop) is listed under the
// editor and skipped, not silently ignored.

void autonFromSim() {
    chassis.setPose(Pose::fromVex(-48.0f, -60.0f, 90.0f));

    chassis.moveToPoint(-24.0f, -60.0f, 2500);
    chassis.turnToHeading(0.0f, 1200);
    chassis.moveToPoint(-24.0f, -24.0f, 2500, {.maxSpeed = 90.0f});
    chassis.turnToPoint(0.0f, 0.0f, 1500);

    chassis.moveDistance(12.0f, 1500);
    pros::delay(300);
    chassis.moveDistance(-12.0f, 1500);

    // Control points for a RAMSETE path. The sim splines through them and
    // drives the smooth curve -- drag the purple points to reshape it.
    std::vector<Pose> path = {
        {-24.0f, -24.0f, 0.0f},
        {0.0f, -40.0f, 0.0f},
        {24.0f, -24.0f, 0.0f},
    };
    chassis.moveRAMSETE(path, 5000);
}
`;
