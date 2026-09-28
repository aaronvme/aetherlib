// Auton source parser tests.
//
//   node sim/test/auton_cpp.mjs
//
// Pure JS -- no wasm, so this runs in a second and does not need emsdk.
//
// The last two sections are contracts rather than unit tests: the params field
// lists are read back out of include/chad/movement_control/*.hpp, and the
// autons that actually ship in src/gamespecific/autons/ are parsed as-is. If
// either drifts, routing silently stops matching the robot, which is the whole
// thing this module exists to prevent.
import { readFileSync, readdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

import {
  parseAuton, evalExpr, moveWaypoint, appendStatement, emitAuton,
  moveToStatement, fmtLike, PARAM_FIELDS, OP,
} from '../web/auton_cpp.js';


const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, '..', '..');

let failures = 0;
const check = (label, cond, detail = '') => {
  if (!cond) failures++;
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
};
const eq = (a, b) => JSON.stringify(a) === JSON.stringify(b);

const p = src => parseAuton(src);
const one = src => {
  const r = p(src);
  if (r.error) return { error: r.error };
  return { cmd: r.cmds[0], opt: r.opts[0], r };
};

console.log('\n== motions ==');

let g = one('chassis.moveToPoint(-24.0f, -60.0f, 2500);');
check('moveToPoint', eq(g.cmd, [OP.MOVE_TO, -24, -60, 1, 2500]),
      JSON.stringify(g.cmd ?? g.error));

// tank/delay pairs are how the match routines drive. They used to land in
// notes, so an auton pasted into the editor ran a different route from the
// compiled function it came from.
g = one('chassis.tank(-50, -50);');
check('tank', eq(g.cmd, [OP.TANK, -50, -50, 0, 0]),
      JSON.stringify(g.cmd ?? g.error));
g = one('chassis.tank(60, -60);');
check('tank turns in place too', eq(g.cmd, [OP.TANK, 60, -60, 0, 0]),
      JSON.stringify(g.cmd ?? g.error));
check('and it is not a waypoint', !g.r || !g.r.plan || g.r.plan.length === 0,
      `${g.r && g.r.plan ? g.r.plan.length : 0} plan entries`);

g = one('chassis.moveDistance(-12.0f, 1500);');
check('moveDistance keeps the sign, as Chassis::moveDistance does',
      eq(g.cmd, [OP.DRIVE, -12, 0, 0, 1500]), JSON.stringify(g.cmd ?? g.error));

g = one('chassis.turnToHeading(140.0f, 1200);');
check('turnToHeading', eq(g.cmd, [OP.TURN_HEADING, 140, 0, 0, 1200]),
      JSON.stringify(g.cmd ?? g.error));

g = one('chassis.turnToPoint(10.0f, 20.0f);');
check('turnToPoint with no timeout', eq(g.cmd, [OP.TURN_POINT, 10, 20, 0, 0]),
      JSON.stringify(g.cmd ?? g.error));

g = one('chassis.moveArc(24.0f, 24.0f, 20.0f, 3000);');
check('moveArc puts the radius third', eq(g.cmd, [OP.ARC, 24, 24, 20, 3000]),
      JSON.stringify(g.cmd ?? g.error));

g = one('chassis.moveBoomerang(48.0f, 24.0f, 180.0f, 3500);');
check('moveBoomerang', eq(g.cmd, [OP.BOOMERANG, 48, 24, 180, 3500]),
      JSON.stringify(g.cmd ?? g.error));

g = one('chassis.swingToHeading(270.0f, SwingSide::Left, 1500);');
check('swingToHeading resolves the side enum',
      eq(g.cmd, [OP.SWING, 270, 0, 0, 1500]), JSON.stringify(g.cmd ?? g.error));

g = one('chassis.swingToHeading(270.0f, chad::SwingSide::Right, 1500);');
check('a chad:: qualified enum resolves too',
      eq(g.cmd, [OP.SWING, 270, 1, 0, 1500]), JSON.stringify(g.cmd ?? g.error));

g = one('chassis.waitUntilSettled();');
check('waitUntilSettled', eq(g.cmd, [OP.WAIT_SETTLED, 0, 0, 0, 0]),
      JSON.stringify(g.cmd ?? g.error));

g = one('pros::delay(300);');
check('pros::delay', eq(g.cmd, [OP.DELAY, 300, 0, 0, 0]),
      JSON.stringify(g.cmd ?? g.error));

g = one('chassis.setLateralGains({5.0f, 0.1f, 5.0f});');
check('setLateralGains', eq(g.cmd, [OP.LATERAL_GAINS, 5, 0.1, 5, 0]),
      JSON.stringify(g.cmd ?? g.error));

console.log('\n== setPose carries radians, the opcode carries a bearing ==');

g = one('chassis.setPose({0.0f, 0.0f, 0.0f});');
check('theta 0 rad is VEX 90 (facing +X)', eq(g.cmd, [OP.SET_POSE, 0, 0, 90, 0]),
      JSON.stringify(g.cmd ?? g.error));

g = one('chassis.setPose({1.0f, 2.0f, static_cast<float>(M_PI) / 2.0f});');
check('theta pi/2 rad is VEX 0', g.cmd && Math.abs(g.cmd[3]) < 1e-4,
      JSON.stringify(g.cmd ?? g.error));

g = one('chassis.setPose(Pose::fromVex(-48.0f, -60.0f, 90.0f));');
check('Pose::fromVex is taken as degrees, not radians',
      eq(g.cmd, [OP.SET_POSE, -48, -60, 90, 0]), JSON.stringify(g.cmd ?? g.error));

console.log('\n== params blocks ==');

g = one('chassis.moveToPoint(1.0f, 2.0f, 2000, {.forwards = false, .maxSpeed = 90.0f, .lead = 4.0f});');
check('designated initializers become options',
      eq(g.opt, { forwards: 0, maxSpeed: 90, lead: 4 }),
      JSON.stringify(g.opt ?? g.error));

g = one('chassis.moveToPoint(1.0f, 2.0f, 2000, {.forwards = false}, true);');
check('a trailing true is the async flag',
      g.opt?._async === true, JSON.stringify(g.opt ?? g.error));

g = one('chassis.turnToHeading(90.0f, 1000, {.direction = TurnDirection::CW});');
check('an enum field resolves to its integer value',
      eq(g.opt, { direction: 1 }), JSON.stringify(g.opt ?? g.error));

let r = p('chassis.moveToPoint(1.0f, 2.0f, 2000, {.maxSpeed = 90.0f, .forwards = false});');
check('out-of-order initializers are an error, as the compiler would say',
      !!r.error && /declaration order/.test(r.error), r.error);

r = p('chassis.moveToPoint(1.0f, 2.0f, 2000, {.speed = 90.0f});');
check('a field the struct does not have is an error',
      !!r.error && /no field \.speed/.test(r.error), r.error);

// The path has to be declared, or this passes on the missing-variable error
// instead of on the thing it claims to test.
r = p('std::vector<Pose> path = {{0.0f, 0.0f, 0.0f}, {10.0f, 0.0f, 0.0f}};\n' +
      'chassis.moveRAMSETE(path, 5000, {.maxSpeed = 90.0f, .ffGains = {}});');
check('a non-scalar field is noted, not silently honoured',
      !r.error && r.notes.some(n => /ffGains/.test(n)) &&
      eq(r.opts[0], { maxSpeed: 90 }),
      r.error ?? `notes ${JSON.stringify(r.notes)} opts ${JSON.stringify(r.opts[0])}`);

console.log('\n== a RAMSETE path is a real pose list ==');

const ramsete = `
  std::vector<Pose> path = {
      {0.0f, 0.0f, 0.0f},
      {24.0f, 0.0f, 0.0f},
      {48.0f, 24.0f, static_cast<float>(M_PI) / 2.0f}
  };
  chassis.moveRAMSETE(path, 5000, {.b = 48.0f, .zeta = 0.85f});
`;
r = p(ramsete);
check('the pose list becomes a path', r.paths?.length === 1 && r.paths[0].length === 3,
      r.error ?? `${r.paths?.length} path of ${r.paths?.[0]?.length}`);
check('moveRAMSETE names its path index',
      eq(r.cmds, [[OP.FOLLOW_PATH, 0, 0, 0, 5000]]),
      r.error ?? JSON.stringify(r.cmds));
check('pose theta is converted to a VEX bearing',
      r.paths && Math.abs(r.paths[0][0].h - 90) < 1e-4 &&
      Math.abs(r.paths[0][2].h - 0) < 1e-4,
      r.paths ? r.paths[0].map(q => q.h.toFixed(1)).join(', ') : r.error);
check('RAMSETE params come through', eq(r.opts?.[0], { b: 48, zeta: 0.85 }),
      JSON.stringify(r.opts?.[0]));

r = p('chassis.moveRAMSETE(nosuchpath, 5000);');
check('a path variable that was never declared is an error',
      !!r.error && /no std::vector<Pose>/.test(r.error), r.error);

console.log('\n== what is skipped, and what is not ==');

r = p('lift.setHeight(12.0f);\nchassis.moveDistance(5.0f);');
check('a subsystem call is skipped, not fatal',
      !r.error && r.cmds.length === 1 && r.notes.length === 1,
      r.error ?? `${r.cmds.length} cmd, note: ${r.notes[0]}`);

// arcade is still unroutable -- there is no opcode for it, and unlike tank the
// match routines do not use it.
r = p('chassis.arcade(80.0f, 20.0f);');
check('a real-but-unroutable chassis call is skipped with a note',
      !r.error && r.cmds.length === 0 && r.notes.length === 1,
      r.error ?? r.notes[0]);

r = p('chassis.moveToPint(1.0f, 2.0f);');
check('a misspelled chassis method is an error, not a skip',
      !!r.error && /no method moveToPint/.test(r.error), r.error);

r = p('// nothing but a comment\n');
check('comment-only input is valid and empty', !r.error && r.cmds.length === 0,
      r.error ?? `${r.cmds.length} commands`);

r = p('#include "chad/api.hpp"\nnamespace chad {\nvoid autonX() {\nchassis.moveDistance(5.0f);\n}\n}');
check('includes, namespace and the function wrapper are stepped over',
      !r.error && r.cmds.length === 1 && r.fnName === 'autonX',
      r.error ?? `${r.cmds.length} cmd, fn ${r.fnName}, notes ${r.notes.length}`);

console.log('\n== expressions ==');

check('float suffix', evalExpr('12.5f') === 12.5);
check('M_PI', Math.abs(evalExpr('M_PI') - Math.PI) < 1e-9);
check('static_cast and division',
      Math.abs(evalExpr('static_cast<float>(M_PI) / 2.0f') - Math.PI / 2) < 1e-9);
check('vexToStdRad(90) is 0 rad', Math.abs(evalExpr('vexToStdRad(90.0f)')) < 1e-9,
      String(evalExpr('vexToStdRad(90.0f)')));
check('a negated parenthesised sum', evalExpr('-(2.0f + 3.0f)') === -5);
check('not-a-number is NaN, not a guess', Number.isNaN(evalExpr('someVariable')));
check('a trailing operator is NaN', Number.isNaN(evalExpr('2.0f +')));

console.log('\n== editing the source ==');

const wpText = 'chassis.moveToPoint(-24.0f, -60.0f, 2500);';
r = p(wpText);
const wp = r.plan[0];
check('x range points at its literal',
      wpText.slice(wp.xRange[0], wp.xRange[1]) === '-24.0f',
      JSON.stringify(wpText.slice(wp.xRange[0], wp.xRange[1])));
check('dragging rewrites both literals and keeps the f suffix',
      moveWaypoint(wpText, wp, 12.34, -5) ===
        'chassis.moveToPoint(12.3f, -5.0f, 2500);',
      moveWaypoint(wpText, wp, 12.34, -5));

check('an integer literal stays without a suffix', fmtLike('-24', 12.34) === '12.3');
check('a suffixed literal keeps one', fmtLike('-24.0f', 12) === '12.0f');

const edited = moveWaypoint(wpText, wp, 1, 2);
check('an edited statement still parses to the same opcode',
      eq(p(edited).cmds[0], [OP.MOVE_TO, 1, 2, 1, 2500]),
      JSON.stringify(p(edited).cmds[0]));

const wrapped = 'void autonX() {\n    chassis.moveDistance(5.0f);\n}\n';
const appended = appendStatement(wrapped, moveToStatement(-24, -60));
check('a new statement lands inside the function body',
      appended === 'void autonX() {\n    chassis.moveDistance(5.0f);\n' +
                   '    chassis.moveToPoint(-24.0f, -60.0f, 2000);\n}\n',
      JSON.stringify(appended));
check('the appended statement parses', p(appended).cmds.length === 2,
      `${p(appended).cmds.length} commands`);

const bare = 'chassis.moveDistance(5.0f);\n';
check('with no function wrapper it just appends',
      appendStatement(bare, 'chassis.waitUntilSettled();') ===
        'chassis.moveDistance(5.0f);\nchassis.waitUntilSettled();\n',
      JSON.stringify(appendStatement(bare, 'chassis.waitUntilSettled();')));

console.log('\n== emitting a file ==');

const emitted = emitAuton('chassis.moveDistance(5.0f);', 'autonTest');
check('emit wraps a bare body in a compilable file',
      /#include "chad\/api.hpp"/.test(emitted) &&
      /namespace chad \{/.test(emitted) &&
      /void autonTest\(\) \{/.test(emitted) &&
      /\}  \/\/ namespace chad/.test(emitted),
      emitted.split('\n').filter(Boolean).slice(0, 2).join(' | '));
check('emit does not double-wrap an existing function',
      (emitAuton(wrapped).match(/void autonX/g) || []).length === 1);
check('what emit produces parses back', p(emitted).cmds.length === 1,
      `${p(emitted).cmds.length} commands`);

console.log('\n== control points are splined for you ==');

// A pose list is CONTROL points, the way via() always was: place three and
// the trajectory comes out smooth without asking for it. The parser hands
// back the controls; buildPath() in route_dsl.js samples the curve.
const ctlSrc = 'std::vector<Pose> path = {\n' +
  '    {0.0f, 0.0f, 0.0f},\n' +
  '    {10.0f, 10.0f, 0.0f},\n' +
  '    {20.0f, 0.0f, 0.0f},\n' +
  '};\n' +
  'chassis.moveRAMSETE(path, 4000);';
r = p(ctlSrc);
check('the control points come through as written',
      r.paths?.[0]?.length === 3, r.error ?? `${r.paths?.[0]?.length} points`);
check('they are drawn as vias', r.plan?.every(q => q.kind === 'via'),
      r.plan?.map(q => q.kind).join(','));
check('moveRAMSETE names the list', eq(r.cmds, [[OP.FOLLOW_PATH, 0, 0, 0, 4000]]),
      JSON.stringify(r.cmds));

check('the smoothing is not done here', r.paths?.[0]?.length === 3,
      'the parser must hand over controls, not a curve');

// Dragging a control point rewrites the control, which is what makes the
// curve reshape under the cursor.
const ctrl = p(ctlSrc).plan[1];
const moved = moveWaypoint(ctlSrc, ctrl, 12, -3);
check('dragging a control point rewrites its literal',
      moved.includes('{12.0f, -3.0f, 0.0f}'),
      moved.split('\n').find(l => l.includes('12.0f')));
check('the dragged source still parses',
      p(moved).paths?.[0]?.[1]?.x === 12 && p(moved).paths[0][1].y === -3,
      JSON.stringify(p(moved).paths?.[0]?.[1]));

console.log('\n== contract: params fields match the headers ==');

const mc = join(root, 'include', 'chad', 'movement_control');
const headers = readdirSync(mc).filter(f => f.endsWith('.hpp'))
  .map(f => readFileSync(join(mc, f), 'utf8')).join('\n');

for (const [name, expected] of Object.entries(PARAM_FIELDS)) {
  const m = new RegExp(`struct\\s+${name}\\s*\\{([\\s\\S]*?)\\n\\};`).exec(headers);
  if (!m) { check(`${name} found in the headers`, false); continue; }
  const body = m[1].replace(/\/\/[^\n]*/g, '').replace(/\/\*[\s\S]*?\*\//g, '');
  const found = [...body.matchAll(
    /^\s*(?:[A-Za-z_][\w:<>]*\s+)+?([A-Za-z_]\w*)\s*(?:\{[^}]*\}|=[^;]*)?;/gm)]
    .map(x => x[1]);
  check(`${name} (${found.length} fields)`, eq(found, expected),
        eq(found, expected) ? '' : `header: ${found.join(',')}  js: ${expected.join(',')}`);
}

console.log('\n== contract: the preview spacing matches the header ==');

// The browser draws the curve at RAMSETEParams::smoothSpacingIn, read back
// from the wasm. If app.js's fallback and the header's default disagree, the
// drawn curve is sampled differently from the one that gets driven.
const ramseteHpp = readFileSync(
  join(root, 'include', 'chad', 'movement_control', 'ramsete.hpp'), 'utf8');
const headerSpacing = parseFloat(
  /float\s+smoothSpacingIn\s*\{\s*([\d.]+)f?\s*\}/.exec(ramseteHpp)?.[1]);
const appJs = readFileSync(join(root, 'sim', 'web', 'app.js'), 'utf8');
const appSpacing = parseFloat(
  /RAMSETE_SPACING_IN\s*=\s*([\d.]+)/.exec(appJs)?.[1]);
check('RAMSETEParams declares smoothSpacingIn', Number.isFinite(headerSpacing),
      String(headerSpacing));
check('app.js uses the same default as the header',
      headerSpacing === appSpacing, `header ${headerSpacing}, app ${appSpacing}`);

// And route_dsl.js's buildPath, which still draws the old via() preview, has
// to agree too or the two previews disagree with each other.
const dslJs = readFileSync(join(root, 'sim', 'web', 'route_dsl.js'), 'utf8');
const dslSpacing = parseFloat(
  /buildPath\(vias,\s*spacingIn\s*=\s*([\d.]+)\)/.exec(dslJs)?.[1]);
check('the DSL preview samples at the same spacing',
      dslSpacing === headerSpacing, `dsl ${dslSpacing}, header ${headerSpacing}`);

console.log('\n== contract: the autons that ship actually parse ==');

const autonDir = join(root, 'src', 'gamespecific', 'autons');
const files = [];
const walk = dir => {
  for (const e of readdirSync(dir, { withFileTypes: true })) {
    if (e.isDirectory()) walk(join(dir, e.name));
    else if (e.name.endsWith('.cpp')) files.push(join(dir, e.name));
  }
};
walk(autonDir);

// The tuning routines are C++ the sim has no business routing -- loops, screen
// output, calls into Characterizer. Only the match and example autons are
// expected to round-trip, and example.cpp is the interesting one: it exercises
// every motion type with a full params block on each.
// A mirrored auton is one line -- `void autonBlue3Pin() { reflect(autonRed3Pin); }`
// -- and has no motions of its own to round-trip. Demanding that it parse to
// something asked it for a route it does not contain, and the red one it
// mirrors is checked here anyway. sim/auton_sources.py skips them for the same
// reason, so this keeps the two in step.
const REFLECT_ONLY = /\breflect\s*\(\s*auton\w*\s*\)\s*;/;
const routable = files
  .filter(f => /example\.cpp$|[\\/]match[\\/]/.test(f))
  .filter(f => !REFLECT_ONLY.test(readFileSync(f, 'utf8')));
check('found the autons to check', routable.length >= 2,
      routable.map(f => f.split(/[\\/]/).pop()).join(', '));

for (const file of routable) {
  const src = readFileSync(file, 'utf8');
  const parsed = parseAuton(src);
  const label = file.split(/[\\/]/).pop();
  if (parsed.error) {
    check(`${label} parses`, false, parsed.error);
    continue;
  }
  check(`${label} parses`, parsed.cmds.length > 0,
        `${parsed.cmds.length} commands, ${parsed.notes.length} skipped` +
        (parsed.notes.length ? ` (${parsed.notes[0]})` : ''));
}

const ex = parseAuton(readFileSync(join(autonDir, 'example.cpp'), 'utf8'));
check('example.cpp covers every motion opcode',
      new Set(ex.cmds?.map(c => c[0])).size >= 9,
      `${new Set(ex.cmds?.map(c => c[0])).size} distinct opcodes`);
check('example.cpp loses nothing to the skip list',
      ex.notes?.length === 0, JSON.stringify(ex.notes));

// sim/build.sh lifts these same functions out for the browser, so the front
// end can draw a compiled auton's route. Skipped when the sim has not been
// built -- the module is generated, not committed.
let AUTON_SOURCES = null;
try {
  ({ AUTON_SOURCES } = await import('../web/auton_sources.js'));
} catch { /* not generated yet */ }

if (AUTON_SOURCES === null) {
  console.log('  [skip] auton_sources.js not generated; run sim/build.sh');
} else {
  const emitted = Object.keys(AUTON_SOURCES);
  check('the generated module carries every routable auton',
        routable.every(f => emitted.some(k => readFileSync(f, 'utf8').includes(k))),
        emitted.join(', '));
  for (const [name, src] of Object.entries(AUTON_SOURCES)) {
    const p = parseAuton(src);
    check(`${name} survives the trip through the generator`,
          !p.error && p.cmds.length > 0,
          p.error ?? `${p.cmds.length} commands`);
  }
}

console.log('\n== for loops ==');

// Unrolled into the flat opcode list. The old behaviour was worse than
// refusing: the header was noted as skipped but the body still parsed once,
// so a three-lap loop previewed and drove as a single lap.
{
  const wrap = inner => `void a() { ${inner} }`;
  const cmds = src => { const r = parseAuton(wrap(src)); return r.error ?? r.cmds; };

  const thrice = cmds('for (int i = 0; i < 3; i++)'
                      + ' { chassis.moveDistance(12, 500); }');
  check('a loop body runs once per iteration', thrice.length === 3,
        `${thrice.length} commands`);

  const counted = cmds('for (int i = 0; i < 3; i++)'
                       + ' { chassis.moveToPoint(i * 24, 0, 1000); }');
  check('the counter reaches the arguments',
        counted.map(c => c[1]).join(',') === '0,24,48',
        counted.map(c => c[1]).join(','));

  check('an empty body is a no-op',
        cmds('for (int i = 0; i < 5; i++) {}').length === 0);

  check('<= includes the bound',
        cmds('for (int i = 1; i <= 3; i++) { chassis.moveDistance(i, 500); }')
          .map(c => c[1]).join(',') === '1,2,3');
  check('a += step is honoured',
        cmds('for (int i = 0; i < 6; i += 2) { chassis.moveDistance(i, 500); }')
          .map(c => c[1]).join(',') === '0,2,4');
  check('counting down works',
        cmds('for (int i = 3; i > 0; i--) { chassis.moveDistance(i, 500); }')
          .map(c => c[1]).join(',') === '3,2,1');

  // Refused rather than unrolled, because either would take the browser down.
  check('a runaway loop is refused',
        String(cmds('for (int i = 0; i < 99999; i++)'
                    + ' { chassis.moveDistance(1, 500); }')).includes('200'),
        String(cmds('for (int i = 0; i < 99999; i++)'
                    + ' { chassis.moveDistance(1, 500); }')));
  check('a range-for is refused, not silently skipped',
        typeof cmds('for (auto q : path) { chassis.moveDistance(1, 500); }')
          === 'string');

  // Its coordinates are computed, so there is no literal in the source to
  // rewrite and the point must not offer itself for dragging.
  const plan = parseAuton(wrap('for (int i = 0; i < 2; i++)'
                               + ' { chassis.moveToPoint(i * 24, 0, 1000); }')).plan;
  check('a computed waypoint is not draggable',
        plan.length === 2 && plan.every(p => !p.xRange),
        `${plan.filter(p => p.xRange).length} of ${plan.length} draggable`);

  // This one is literal in the source, so dragging it should still edit that
  // literal -- and move every iteration, which is what the source says.
  const fixed = parseAuton(wrap('for (int i = 0; i < 2; i++)'
                                + ' { chassis.moveToPoint(24, 0, 1000); }')).plan;
  check('a literal waypoint stays draggable',
        fixed.length === 2 && fixed.every(p => !!p.xRange),
        `${fixed.filter(p => p.xRange).length} of ${fixed.length} draggable`);
}

console.log('\n== the claw ==');

{
  const r = parseAuton('void a() { claw.extend(); claw.retract(); claw.toggle(); }');
  check('claw calls route instead of being skipped',
        !r.error && r.cmds.length === 3 && r.notes.length === 0,
        r.error ?? `${r.cmds.length} commands, ${r.notes.length} skipped`);
  check('and carry which way they move',
        !r.error && r.cmds.map(c => c[1]).join(',') === '1,0,2',
        r.error ?? r.cmds.map(c => c[1]).join(','));

  // A different subsystem still goes to notes rather than erroring.
  const lift = parseAuton('void a() { liftToPin(0, 1000); }');
  check('other subsystems are still listed, not routed',
        !lift.error && lift.cmds.length === 0 && lift.notes.length === 1,
        lift.error ?? lift.notes.join(' | '));
}

console.log(`\n${failures === 0 ? 'ALL CHECKS PASSED' : failures + ' CHECK(S) FAILED'}\n`);
process.exit(failures === 0 ? 0 : 1);
