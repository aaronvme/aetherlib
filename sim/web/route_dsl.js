// chad-sim — route DSL.
//
// Parses route text into a flat opcode list the WASM bridge executes against
// the real Chassis API, plus a waypoint list for the field preview.
//
// Grammar is one call per statement, semicolon separated, `//` comments:
//
//   setPose(x: -48, y: -60, h: 90);
//   moveTo(x: -24, y: -60, t: 2500);
//   moveTo(x: -48, y: 24, back: true);
//   turnTo(heading: 140);        turnTo(x: 10, y: 20);
//   drive(distance: 12);         drive(distance: -8);
//   boomerang(x: 10, y: 20, h: 90);
//   arc(x: 10, y: 20, radius: 24);
//   swing(heading: 90, side: right);
//   delay(ms: 300);              waitSettled();
//   tank(left: -50, right: -50);   open loop, holds until the next one
//   maxSpeed(80);
//   lateralGains(kp: 5, ki: 0, kd: 5);
//   angularGains(kp: 50, ki: 0, kd: 6);
//
// `t:` (alias `timeout:`) sets a per-motion timeout in ms on any motion.
//
// Waypoints carry the character range of their x and y literals in the ORIGINAL
// text, so dragging a point on the field can splice new numbers back in without
// reformatting the rest of the route. Comments are blanked rather than removed
// so every offset stays valid.
//
// Opcode numbers must stay in step with RouteOp in sim/bridge/bridge.cpp.

export const OP = {
  SET_POSE: 0, MOVE_TO: 1, TURN_HEADING: 2, TURN_POINT: 3, DRIVE: 4,
  BOOMERANG: 5, ARC: 6, SWING: 7, DELAY: 8, WAIT_SETTLED: 9,
  LATERAL_GAINS: 10, ANGULAR_GAINS: 11, MAX_SPEED: 12, FOLLOW_PATH: 13,
  CLAW: 14, TANK: 15,
};

const truthy = v => v === 'true' || v === '1';

/// Blank out `//` comments, preserving length so offsets stay meaningful.
const blankComments = t => t.replace(/\/\/[^\n]*/g, m => ' '.repeat(m.length));

/**
 * @returns {{cmds: number[][], plan: object[]} | {error: string}}
 */
export function parseRoute(text) {
  const src = String(text);
  const scan = blankComments(src);

  const cmds = [];
  const plan = [];
  const paths = [];        // one entry per followPath, each a list of via points
  let vias = [];           // vias accumulated since the last followPath

  let cursor = 0;
  while (cursor < scan.length) {
    const semi = scan.indexOf(';', cursor);
    const end = semi === -1 ? scan.length : semi;
    const raw = scan.slice(cursor, end);

    if (raw.trim()) {
      const res = parseStatement(raw, cursor);
      if (res.error) return res;

      if (res.via) {
        vias.push(res.via);
        plan.push(res.via);
        if (semi === -1) break;
        cursor = end + 1;
        continue;                       // via emits no opcode of its own
      }

      if (res.cmd[0] === OP.FOLLOW_PATH) {
        if (vias.length < 2) {
          return { error: 'followPath needs at least two via() points' };
        }
        // a/b are filled in by the caller once the spline is sampled and
        // uploaded; the index here just names which path this command wants.
        res.cmd[1] = paths.length;
        paths.push(vias);
        vias = [];
      }

      cmds.push(res.cmd);
      if (res.waypoint) plan.push(res.waypoint);
    }

    if (semi === -1) break;
    cursor = end + 1;
  }
  return { cmds, plan, paths };
}

function parseStatement(raw, base) {
  const m = raw.match(/^(\s*)([A-Za-z_][A-Za-z0-9_]*)\s*\(([\s\S]*)\)\s*$/);
  if (!m) return { error: `cannot parse: ${raw.trim().slice(0, 40)}` };

  const name = m[2];
  const argsText = m[3];
  const argsOffset = base + m[1].length + m[2].length +
                     raw.slice(m[1].length + m[2].length).indexOf('(') + 1;

  // Named args, each with the absolute range of its value literal.
  const args = {};
  const ranges = {};
  const positional = [];

  // Walk the comma-separated parts by hand, tracking how far into argsText
  // each one starts, so a value's range can be mapped back to the source.
  let seen = 0;
  for (const part of argsText.split(',')) {
    const partStart = argsOffset + seen;
    seen += part.length + 1;                       // +1 for the comma

    // [\s\S] rather than . so a value can carry a trailing newline: in JS `$`
    // without the m flag will not match before one, which silently dropped the
    // last argument of a multi-line call.
    const kv = part.match(/^\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*([\s\S]*)$/);
    if (kv) {
      const value = kv[2].trimEnd();               // keep the leading offset exact
      const valStart = partStart + part.length - kv[2].length;
      args[kv[1]] = value.trim();
      ranges[kv[1]] = [valStart, valStart + value.length];
    } else if (part.trim()) {
      positional.push(part.trim());
    }
  }

  const num = (...names) => {
    for (const n of names) if (n in args) return parseFloat(args[n]);
    return undefined;
  };
  const pos = i => (positional[i] !== undefined ? parseFloat(positional[i]) : undefined);
  const t = num('t', 'timeout') ?? 0;
  const xy = () => [num('x') ?? 0, num('y') ?? 0];
  const wp = (kind) => {
    const [x, y] = xy();
    return { x, y, kind, xRange: ranges.x ?? null, yRange: ranges.y ?? null };
  };

  switch (name) {
    case 'setPose': {
      const [x, y] = xy();
      return { cmd: [OP.SET_POSE, x, y, num('h', 'heading') ?? 0, 0], waypoint: wp('start') };
    }
    case 'moveTo': {
      const [x, y] = xy();
      const back = truthy(args.back ?? args.backward ?? args.reverse ?? '');
      return { cmd: [OP.MOVE_TO, x, y, back ? -1 : 1, t], waypoint: wp(back ? 'back' : 'move') };
    }
    case 'turnTo': {
      if ('x' in args || 'y' in args) {
        const [x, y] = xy();
        return { cmd: [OP.TURN_POINT, x, y, 0, t], waypoint: wp('face') };
      }
      return { cmd: [OP.TURN_HEADING, num('h', 'heading') ?? pos(0) ?? 0, 0, 0, t] };
    }
    case 'drive':
      return { cmd: [OP.DRIVE, num('distance', 'd') ?? pos(0) ?? 0, 0, 0, t] };
    case 'boomerang': {
      const [x, y] = xy();
      return { cmd: [OP.BOOMERANG, x, y, num('h', 'heading') ?? 0, t], waypoint: wp('boomerang') };
    }
    case 'arc': {
      const [x, y] = xy();
      return { cmd: [OP.ARC, x, y, num('radius', 'r') ?? 24, t], waypoint: wp('arc') };
    }
    case 'swing': {
      const side = String(args.side ?? 'right').toLowerCase().startsWith('r') ? 1 : 0;
      return { cmd: [OP.SWING, num('h', 'heading') ?? 0, side, 0, t] };
    }
    case 'via': {
      const [x, y] = xy();
      return { via: { x, y, kind: 'via', xRange: ranges.x ?? null, yRange: ranges.y ?? null } };
    }
    case 'followPath':
      // [1] is the path index, patched to (start, count) before upload.
      return { cmd: [OP.FOLLOW_PATH, 0, 0, 0, t] };
    case 'delay':
      return { cmd: [OP.DELAY, num('ms') ?? pos(0) ?? 0, 0, 0, 0] };
    case 'waitSettled':
      return { cmd: [OP.WAIT_SETTLED, 0, 0, 0, 0] };
    case 'tank':
      // Open loop. It holds until the next tank(), so it is almost always
      // followed by a delay.
      return { cmd: [OP.TANK, num('left', 'l') ?? pos(0) ?? 0,
                     num('right', 'r') ?? pos(1) ?? 0, 0, 0] };
    case 'maxSpeed':
      return { cmd: [OP.MAX_SPEED, num('speed', 'v') ?? pos(0) ?? 127, 0, 0, 0] };
    case 'lateralGains':
      return { cmd: [OP.LATERAL_GAINS, num('kp') ?? 0, num('ki') ?? 0, num('kd') ?? 0, 0] };
    case 'angularGains':
      return { cmd: [OP.ANGULAR_GAINS, num('kp') ?? 0, num('ki') ?? 0, num('kd') ?? 0, 0] };
    default:
      return { error: `unknown command: ${name}` };
  }
}

/**
 * Rewrite a waypoint's x/y literals in the route text.
 * Splices the later range first so the earlier offset stays valid.
 */
export function moveWaypoint(text, waypoint, x, y) {
  if (!waypoint?.xRange || !waypoint?.yRange) return text;

  const edits = [
    { range: waypoint.xRange, value: fmt(x) },
    { range: waypoint.yRange, value: fmt(y) },
  ].sort((a, b) => b.range[0] - a.range[0]);

  let out = text;
  for (const e of edits) {
    out = out.slice(0, e.range[0]) + e.value + out.slice(e.range[1]);
  }
  return out;
}

const fmt = v => (Math.round(v * 10) / 10).toString();

/**
 * Sample a Catmull-Rom spline through the via points.
 *
 * The browser samples it and uploads the exact points the route will follow,
 * so the drawn preview and what moveRAMSETE actually receives are the same
 * array -- there is no second implementation to drift.
 *
 * @returns [{x, y, h}] with h a VEX bearing in degrees, tangent to the curve.
 */
export function buildPath(vias, spacingIn = 2) {
  if (!vias || vias.length < 2) return [];

  // Duplicate the endpoints so the curve reaches the first and last via.
  const p = [vias[0], ...vias, vias[vias.length - 1]];
  const pts = [];

  const at = (a, b, c, d, t) => {
    const t2 = t * t, t3 = t2 * t;
    return 0.5 * ((2 * b) + (-a + c) * t +
                  (2 * a - 5 * b + 4 * c - d) * t2 +
                  (-a + 3 * b - 3 * c + d) * t3);
  };

  for (let i = 1; i < p.length - 2; i++) {
    const [p0, p1, p2, p3] = [p[i - 1], p[i], p[i + 1], p[i + 2]];
    const chord = Math.hypot(p2.x - p1.x, p2.y - p1.y);
    const steps = Math.max(2, Math.ceil(chord / spacingIn));
    for (let sIdx = 0; sIdx < steps; sIdx++) {
      const t = sIdx / steps;
      pts.push({ x: at(p0.x, p1.x, p2.x, p3.x, t), y: at(p0.y, p1.y, p2.y, p3.y, t) });
    }
  }
  pts.push({ x: vias[vias.length - 1].x, y: vias[vias.length - 1].y });

  // Heading from the local tangent, converted to a VEX bearing.
  for (let i = 0; i < pts.length; i++) {
    const a = pts[Math.max(0, i - 1)];
    const b = pts[Math.min(pts.length - 1, i + 1)];
    pts[i].h = 90 - Math.atan2(b.y - a.y, b.x - a.x) * 180 / Math.PI;
  }
  return pts;
}

export const DEFAULT_ROUTE = `// chad-sim route — runs the real Chassis API.
// setPose / moveTo / turnTo / drive / boomerang / arc / swing
// delay / waitSettled / maxSpeed / lateralGains / angularGains
// optional t: <ms> timeout on any motion.
// shift-click the field to append a moveTo; drag a numbered point to move it.

setPose(x: -48, y: -60, h: 90);

moveTo(x: -24, y: -60, t: 2500);
turnTo(heading: 0);
moveTo(x: -24, y: -24, t: 2500);
turnTo(x: 0, y: 0);
drive(distance: 12, t: 1500);
delay(ms: 300);
drive(distance: -12, t: 1500);

// A RAMSETE path: via() points define a spline, followPath() drives it.
via(x: -24, y: -24);
via(x: 0, y: -40);
via(x: 24, y: -24);
followPath(t: 5000);
`;
