// Route DSL parser tests.
//
//   node sim/test/dsl.mjs
//
// The opcode numbers here are the contract with RouteOp in
// sim/bridge/bridge.cpp; the last check reads that enum back out of the C++
// source so the two cannot drift apart silently.
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

import { parseRoute, moveWaypoint, buildPath, OP, DEFAULT_ROUTE } from '../web/route_dsl.js';

const here = dirname(fileURLToPath(import.meta.url));

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};
const eq = (a, b) => JSON.stringify(a) === JSON.stringify(b);

console.log('\n== parsing ==');

let r = parseRoute('setPose(x: -48, y: -60, h: 90);');
check('setPose', eq(r.cmds, [[OP.SET_POSE, -48, -60, 90, 0]]), JSON.stringify(r.cmds));

r = parseRoute('moveTo(x: 12.5, y: -3, t: 2500);');
check('moveTo with timeout', eq(r.cmds, [[OP.MOVE_TO, 12.5, -3, 1, 2500]]), JSON.stringify(r.cmds));

r = parseRoute('moveTo(x: 1, y: 2, back: true);');
check('moveTo backward sets the direction flag',
      eq(r.cmds, [[OP.MOVE_TO, 1, 2, -1, 0]]), JSON.stringify(r.cmds));

r = parseRoute('turnTo(heading: 140);');
check('turnTo heading', eq(r.cmds, [[OP.TURN_HEADING, 140, 0, 0, 0]]), JSON.stringify(r.cmds));

r = parseRoute('turnTo(x: 10, y: 20);');
check('turnTo point picks a different opcode',
      eq(r.cmds, [[OP.TURN_POINT, 10, 20, 0, 0]]), JSON.stringify(r.cmds));

r = parseRoute('drive(distance: -8);');
check('drive keeps the sign for reverse',
      eq(r.cmds, [[OP.DRIVE, -8, 0, 0, 0]]), JSON.stringify(r.cmds));

r = parseRoute('swing(heading: 90, side: left);');
check('swing side left', eq(r.cmds, [[OP.SWING, 90, 0, 0, 0]]), JSON.stringify(r.cmds));

r = parseRoute('maxSpeed(80); waitSettled(); delay(ms: 250);');
check('bare and positional forms',
      eq(r.cmds, [[OP.MAX_SPEED, 80, 0, 0, 0],
                  [OP.WAIT_SETTLED, 0, 0, 0, 0],
                  [OP.DELAY, 250, 0, 0, 0]]), JSON.stringify(r.cmds));

r = parseRoute('lateralGains(kp: 5, ki: 0.1, kd: 5);');
check('gains', eq(r.cmds, [[OP.LATERAL_GAINS, 5, 0.1, 5, 0]]), JSON.stringify(r.cmds));

console.log('\n== comments, whitespace, errors ==');

r = parseRoute(`
  // a comment
  drive(distance: 5);   // trailing comment
`);
check('comments are stripped', eq(r.cmds, [[OP.DRIVE, 5, 0, 0, 0]]), JSON.stringify(r.cmds));

r = parseRoute('moveTo(\n  x: 1,\n  y: 2\n);');
check('multi-line calls parse', eq(r.cmds, [[OP.MOVE_TO, 1, 2, 1, 0]]), JSON.stringify(r.cmds));

check('empty input is valid', eq(parseRoute('').cmds, []));
check('unknown command is an error', !!parseRoute('teleport(x: 1);').error,
      parseRoute('teleport(x: 1);').error);
check('malformed statement is an error', !!parseRoute('moveTo x: 1').error,
      parseRoute('moveTo x: 1').error);

console.log('\n== plan preview ==');
r = parseRoute(DEFAULT_ROUTE);
check('default route parses', !r.error && r.cmds.length === 9, `${r.cmds?.length} commands`);
check('preview starts at the setPose waypoint',
      r.plan[0].kind === 'start' && r.plan[0].x === -48 && r.plan[0].y === -60,
      JSON.stringify(r.plan[0]));
check('preview collects the moveTo waypoints and the vias', r.plan.length === 7,
      r.plan.map(p => p.kind).join(', '));
check('default route builds one path', r.paths.length === 1,
      `${r.paths.length} path of ${r.paths[0]?.length} vias`);

console.log('\n== waypoint source ranges ==');

const wpText = 'moveTo(x: -24, y: -60, t: 2500);';
r = parseRoute(wpText);
const wp = r.plan[0];
check('x range points at its literal',
      wpText.slice(wp.xRange[0], wp.xRange[1]) === '-24',
      JSON.stringify(wpText.slice(wp.xRange[0], wp.xRange[1])));
check('y range points at its literal',
      wpText.slice(wp.yRange[0], wp.yRange[1]) === '-60',
      JSON.stringify(wpText.slice(wp.yRange[0], wp.yRange[1])));

check('moveWaypoint rewrites both literals and nothing else',
      moveWaypoint(wpText, wp, 12.34, -5) === 'moveTo(x: 12.3, y: -5, t: 2500);',
      moveWaypoint(wpText, wp, 12.34, -5));

// Offsets have to survive comments, which are blanked rather than stripped.
const commented = '// lead-in comment\nmoveTo(x: 1, y: 2);\n';
const cwp = parseRoute(commented).plan[0];
check('ranges stay correct after a comment',
      commented.slice(cwp.xRange[0], cwp.xRange[1]) === '1' &&
      commented.slice(cwp.yRange[0], cwp.yRange[1]) === '2',
      JSON.stringify(commented.slice(cwp.xRange[0], cwp.xRange[1] + 0)));

// A multi-waypoint route must keep each range distinct after an edit.
const multi = 'setPose(x: -48, y: -60, h: 90);\nmoveTo(x: 10, y: 20);\nmoveTo(x: 30, y: 40);';
const mp = parseRoute(multi);
const edited = moveWaypoint(multi, mp.plan[2], 33, 44);
check('editing the third waypoint leaves the others intact',
      edited === 'setPose(x: -48, y: -60, h: 90);\nmoveTo(x: 10, y: 20);\nmoveTo(x: 33, y: 44);',
      edited.split('\n').pop());

check('an edited route still parses to the same opcodes',
      eq(parseRoute(edited).cmds.map(c => c[0]), mp.cmds.map(c => c[0])));

console.log('\n== paths ==');

r = parseRoute('via(x: 0, y: 0); via(x: 10, y: 10); via(x: 20, y: 0); followPath(t: 4000);');
check('via emits no opcode of its own', r.cmds.length === 1, JSON.stringify(r.cmds));
check('followPath names its path index',
      r.cmds[0][0] === OP.FOLLOW_PATH && r.cmds[0][1] === 0, JSON.stringify(r.cmds[0]));
check('vias are grouped into one path', r.paths.length === 1 && r.paths[0].length === 3,
      `${r.paths.length} x ${r.paths[0].length}`);

r = parseRoute('via(x:0,y:0); via(x:5,y:5); followPath(); via(x:9,y:9); via(x:12,y:0); followPath();');
check('a second followPath starts a fresh path',
      r.paths.length === 2 && r.cmds[1][1] === 1,
      `${r.paths.length} paths, second index ${r.cmds[1]?.[1]}`);

check('followPath with one via is rejected',
      !!parseRoute('via(x:0,y:0); followPath();').error,
      parseRoute('via(x:0,y:0); followPath();').error);

const sp = buildPath([{ x: 0, y: 0 }, { x: 10, y: 10 }, { x: 20, y: 0 }], 2);
check('spline hits both endpoints exactly',
      Math.hypot(sp[0].x, sp[0].y) < 1e-6 &&
      Math.hypot(sp.at(-1).x - 20, sp.at(-1).y) < 1e-6,
      `(${sp[0].x},${sp[0].y}) -> (${sp.at(-1).x},${sp.at(-1).y})`);
check('spline samples near the requested spacing',
      sp.length > 10 && sp.length < 40, `${sp.length} points`);
check('spline headings are tangent to travel',
      Math.abs(sp[0].h - 45) < 20, `first heading ${sp[0].h.toFixed(1)} deg (expect ~45)`);
check('buildPath refuses a single via', buildPath([{ x: 0, y: 0 }]).length === 0);

console.log('\n== opcodes match the C++ bridge ==');
const src = readFileSync(join(here, '..', 'bridge', 'bridge.cpp'), 'utf8');
const body = src.slice(src.indexOf('enum RouteOp'), src.indexOf('};', src.indexOf('enum RouteOp')));
const names = body.split('\n').slice(1)
  .map(l => l.trim().replace(/\/\/.*$/, '').replace(/=.*$/, '').replace(/,.*$/, '').trim())
  .filter(n => n.startsWith('OP_'));

const expected = ['OP_SET_POSE', 'OP_MOVE_TO', 'OP_TURN_TO_HEADING', 'OP_TURN_TO_POINT',
                  'OP_DRIVE', 'OP_BOOMERANG', 'OP_ARC', 'OP_SWING', 'OP_DELAY',
                  'OP_WAIT_SETTLED', 'OP_LATERAL_GAINS', 'OP_ANGULAR_GAINS', 'OP_MAX_SPEED',
                  'OP_FOLLOW_PATH'];
check('bridge.cpp declares the ops in the expected order', eq(names, expected),
      names.join(' '));

const jsOrder = Object.entries(OP).sort((a, b) => a[1] - b[1]).map(([k]) => k);
check('JS opcode values are 0..n in the same order',
      jsOrder.length === expected.length &&
      jsOrder.every((_, i) => OP[jsOrder[i]] === i),
      jsOrder.join(' '));

console.log(`\n${failures === 0 ? 'ALL CHECKS PASSED' : failures + ' CHECK(S) FAILED'}\n`);
process.exit(failures === 0 ? 0 : 1);
