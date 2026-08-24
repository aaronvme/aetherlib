// Shared loader and assertion helpers for the chad-sim test suites.
//
// Exists for two reasons. CHAD_SIM_MJS lets a suite run against a private
// build, so concurrent builds do not race on the single shared sim/web/ pair.
// And requiring a build to declare the exports it needs turns "this binary is
// stale" into one clear message instead of a TypeError deep inside a test --
// sim/web/chad_sim.* is untracked, so a branch switch leaves a stale binary.
import { fileURLToPath, pathToFileURL } from 'node:url';
import { dirname, join, resolve } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));

export const simPath = () =>
  process.env.CHAD_SIM_MJS
    ? resolve(process.env.CHAD_SIM_MJS)
    : join(here, '..', 'web', 'chad_sim.mjs');

/// Instantiate the built sim. `needs` names bridge exports without their
/// leading underscore; a build missing any of them exits non-zero here.
export async function loadSim(needs = []) {
  const target = simPath();
  let create;
  try {
    create = (await import(pathToFileURL(target).href)).default;
  } catch (e) {
    console.error(`cannot load ${target}`);
    console.error('run ./sim/build.sh (or set CHAD_SIM_MJS) and retry');
    console.error(String(e.message ?? e));
    process.exit(1);
  }
  const M = await create();
  const missing = needs.filter((n) => typeof M['_' + n] !== 'function');
  if (missing.length) {
    console.error(`this build lacks: ${missing.join(', ')}`);
    console.error(`built from ${target}`);
    console.error('sim/web/chad_sim.* is untracked build output, so switching');
    console.error('branches does not rebuild it. Run ./sim/build.sh and retry.');
    process.exit(1);
  }
  return M;
}

/// cwrap a whole API table at once: wrap(M, { step: [null, ['number']] })
/// binds sim_step. Pass a 3-element array to name the symbol explicitly.
export function wrap(M, table) {
  const api = {};
  for (const [key, spec] of Object.entries(table)) {
    const [name, ret, args] =
      spec.length === 3 ? spec : ['sim_' + key.replace(/[A-Z]/g, (c) => '_' + c.toLowerCase()), spec[0], spec[1]];
    api[key] = M.cwrap(name, ret, args);
  }
  return api;
}

/// A counting checker. `finish()` prints the tally and returns an exit code, so
/// a suite ends with `process.exit(t.finish())` and CI sees the failure.
export function tally(title = '') {
  let failures = 0;
  let checks = 0;
  return {
    section(name) { console.log(`\n== ${name} ==`); },
    check(label, cond, detail = '') {
      checks++;
      if (!cond) failures++;
      console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${label}${detail ? '  ' + detail : ''}`);
      return !!cond;
    },
    get failures() { return failures; },
    finish() {
      console.log();
      if (failures) console.log(`${failures} of ${checks} CHECK(S) FAILED${title ? ' in ' + title : ''}`);
      else console.log(`ALL ${checks} CHECKS PASSED${title ? ' in ' + title : ''}`);
      return failures ? 1 : 0;
    },
  };
}
