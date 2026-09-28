// Rapier, driving the pins and the cups.
//
// The sim's own piece model is kinematic: a piece is where it was put, and the
// robot shoves it out of the way along the shortest exit. That is fine for
// "does a beam stop on it" and wrong for everything else -- pieces do not roll,
// do not knock each other over, and a plough gathers nothing.
//
// This hands them to Rapier instead. The arrangement is:
//
//   * pieces        dynamic bodies, damped hard because a foam tile is not ice
//   * the robot     a kinematic body, driven from the sim's ground truth
//   * other robots  the same
//   * goals, walls  fixed
//   * a held piece  switched to kinematic and flown to the jaws, then handed
//                   back to the solver when the claw opens
//
// Rapier owns where a piece IS. It does not own the game: which piece the claw
// bites, and whether opening it counts as nesting, stay in pieces.cpp, because
// those are rules rather than mechanics.
//
// Everything crosses the boundary through two heap buffers rather than a call
// per piece, because this runs on the 10 ms physics tick and not per frame.
//
// Units: Rapier is tuned around metres and the sim thinks in inches, so the
// conversion happens here rather than anywhere a person reads numbers.

const M_PER_IN = 0.0254;
const IN_PER_M = 1 / M_PER_IN;

/// Interior wall-to-wall, matching sim::Field.
const WALL_IN = 70.2;

/// Kinds, matching sim::PieceKind.
const KIND_CUP = 1;

/// True dimensions, from the manual. A pin lying down is a capsule, not a
/// circle -- 6.5 in of it, which is most of why they are worth simulating.
const PIN_R_IN = 0.80;
const PIN_LEN_IN = 6.5;
const CUP_R_IN = 1.58;

/// A pin is a hollow plastic peg and a cup is a thinner one. The numbers only
/// have to be right relative to each other and to the robot, which is immovable.
const PIN_DENSITY = 1.0;
const CUP_DENSITY = 0.6;

/// Top-down friction stands in for the tile. Without it a nudged piece slides
/// until it hits something, which is not what a pin on foam does.
const LINEAR_DAMPING = 3.2;
const ANGULAR_DAMPING = 4.0;

export async function installRapier(Module) {
  const RAPIER = (await import('./vendor/rapier2d-compat-0.21.0.mjs')).default;
  await RAPIER.init();

  const world = new RAPIER.World({ x: 0, y: 0 });
  // One physics substep per sim tick. The sim ticks at 10 ms, which is already
  // finer than Rapier's 16 ms default, so nothing is gained by subdividing it.
  world.integrationParameters.dt = 0.01;

  // --- the field, built once ------------------------------------------------
  const wall = WALL_IN * M_PER_IN;
  const thick = 0.25;
  const staticBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  for (const [x, y, hx, hy] of [
    [0, wall + thick, wall + thick, thick],
    [0, -wall - thick, wall + thick, thick],
    [wall + thick, 0, thick, wall + thick],
    [-wall - thick, 0, thick, wall + thick],
  ]) {
    world.createCollider(
      RAPIER.ColliderDesc.cuboid(hx, hy).setTranslation(x, y), staticBody);
  }

  let goalColliders = [];
  function setGoals(goals) {
    for (const c of goalColliders) world.removeCollider(c, false);
    goalColliders = goals.map(g => world.createCollider(
      RAPIER.ColliderDesc.ball(g.r * M_PER_IN)
        .setTranslation(g.x * M_PER_IN, g.y * M_PER_IN)
        .setFriction(0.5),
      staticBody));
  }
  setGoals([]);

  // --- the robot and the other robots --------------------------------------
  const robot = world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased());
  let robotCollider = null;
  let robotHalf = [0, 0];
  function sizeRobot(halfLenIn, halfWidIn) {
    if (robotHalf[0] === halfLenIn && robotHalf[1] === halfWidIn) return;
    robotHalf = [halfLenIn, halfWidIn];
    if (robotCollider) world.removeCollider(robotCollider, false);
    robotCollider = world.createCollider(
      RAPIER.ColliderDesc.cuboid(halfLenIn * M_PER_IN, halfWidIn * M_PER_IN)
        .setFriction(0.7), robot);
  }

  const actors = [];   // { body, collider, half: [len, wid] }
  function sizeActors(n) {
    while (actors.length < n) {
      const body = world.createRigidBody(
        RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, -1000));
      actors.push({ body, collider: null, half: [0, 0] });
    }
    // Extra ones are parked far away rather than destroyed: a stress test adds
    // and removes robots constantly and churning bodies is how handles go stale.
    for (let i = n; i < actors.length; i++) {
      actors[i].body.setNextKinematicTranslation({ x: 0, y: -1000 });
    }
  }

  // --- pieces ---------------------------------------------------------------
  // id -> { body, collider, kind, upright }
  const pieces = new Map();

  function makeCollider(kind, upright) {
    if (kind === KIND_CUP) {
      return RAPIER.ColliderDesc.ball(CUP_R_IN * M_PER_IN).setDensity(CUP_DENSITY);
    }
    if (upright) {
      return RAPIER.ColliderDesc.ball(PIN_R_IN * M_PER_IN).setDensity(PIN_DENSITY);
    }
    // Lying down: a capsule along the pin's own axis.
    const half = (PIN_LEN_IN / 2 - PIN_R_IN) * M_PER_IN;
    return RAPIER.ColliderDesc.capsule(half, PIN_R_IN * M_PER_IN)
      .setDensity(PIN_DENSITY);
  }

  function addPiece(p) {
    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(p.x * M_PER_IN, p.y * M_PER_IN)
        .setRotation(p.theta)
        .setLinearDamping(LINEAR_DAMPING)
        .setAngularDamping(ANGULAR_DAMPING)
        // Never sleep. The robot is a kinematic body, and one that has come to
        // rest against a wall stops generating the contact that would wake a
        // sleeping neighbour -- so a piece it is leaning on goes to sleep
        // half-inside it and stays there, which reads as the solver ignoring
        // the robot entirely.
        .setCanSleep(false));
    const collider = world.createCollider(
      makeCollider(p.kind, p.upright).setFriction(0.55).setRestitution(0.05),
      body);
    pieces.set(p.id, { body, collider, kind: p.kind, upright: p.upright });
  }

  function dropPiece(id) {
    const e = pieces.get(id);
    if (!e) return;
    world.removeRigidBody(e.body);
    pieces.delete(id);
  }

  /// Rebuild to match the list C++ holds. Called whenever the set changes --
  /// a load, a clear, a drag, or a pin standing up out of a cup, any of which
  /// changes a body's shape or whether it exists at all.
  function sync(list) {
    const seen = new Set();
    for (const p of list) {
      seen.add(p.id);
      const e = pieces.get(p.id);
      if (e && e.kind === p.kind && e.upright === p.upright) {
        // Same body, told where it is now. Velocity is cleared because a piece
        // that was picked up and put down did not travel between the two.
        e.body.setTranslation({ x: p.x * M_PER_IN, y: p.y * M_PER_IN }, true);
        e.body.setRotation(p.theta, true);
        e.body.setLinvel({ x: 0, y: 0 }, true);
        e.body.setAngvel(0, true);
        continue;
      }
      if (e) dropPiece(p.id);
      addPiece(p);
    }
    for (const id of [...pieces.keys()]) if (!seen.has(id)) dropPiece(id);
  }

  let heldId = -1;
  function setHeld(id) {
    if (heldId === id) return;
    const was = pieces.get(heldId);
    if (was) {
      was.body.setBodyType(RAPIER.RigidBodyType.Dynamic, true);
      was.body.setLinvel({ x: 0, y: 0 }, true);
      was.body.setAngvel(0, true);
    }
    const now = pieces.get(id);
    if (now) now.body.setBodyType(RAPIER.RigidBodyType.KinematicPositionBased, true);
    heldId = id;
  }

  // --- the tick -------------------------------------------------------------
  // Input, in floats:
  //   0 robotX  1 robotY  2 robotTh  3 halfLen  4 halfWid
  //   5 heldId  6 heldX   7 heldY    8 heldTh
  //   9 actorCount, then 5 floats each: x, y, th, halfLen, halfWid
  Module.rapierStep = (dtMs, inPtr, inCount, outPtr, cap) => {
    const H = Module.HEAPF32;
    const IN = H.subarray(inPtr, inPtr + inCount);

    sizeRobot(IN[3], IN[4]);
    robot.setNextKinematicTranslation({ x: IN[0] * M_PER_IN, y: IN[1] * M_PER_IN });
    robot.setNextKinematicRotation(IN[2]);

    const nActors = IN[9] | 0;
    sizeActors(nActors);
    for (let i = 0; i < nActors; i++) {
      const b = 10 + i * 5;
      const a = actors[i];
      if (a.half[0] !== IN[b + 3] || a.half[1] !== IN[b + 4]) {
        a.half = [IN[b + 3], IN[b + 4]];
        if (a.collider) world.removeCollider(a.collider, false);
        a.collider = world.createCollider(
          RAPIER.ColliderDesc.cuboid(a.half[0] * M_PER_IN, a.half[1] * M_PER_IN)
            .setFriction(0.7), a.body);
      }
      a.body.setNextKinematicTranslation({ x: IN[b] * M_PER_IN, y: IN[b + 1] * M_PER_IN });
      a.body.setNextKinematicRotation(IN[b + 2]);
    }

    setHeld(IN[5] | 0);
    const held = pieces.get(heldId);
    if (held) {
      held.body.setNextKinematicTranslation(
        { x: IN[6] * M_PER_IN, y: IN[7] * M_PER_IN });
      held.body.setNextKinematicRotation(IN[8]);
    }

    // Rapier's own dt is fixed; run as many substeps as the tick is worth.
    let left = dtMs / 1000;
    let guard = 0;
    while (left > 1e-6 && guard++ < 8) {
      world.integrationParameters.dt = Math.min(left, 0.01);
      world.step();
      left -= world.integrationParameters.dt;
    }

    let n = 0;
    const OUT = Module.HEAPF32;
    for (const [id, e] of pieces) {
      if (n * 4 + 4 > cap) break;
      const t = e.body.translation();
      OUT[outPtr + n * 4 + 0] = id;
      OUT[outPtr + n * 4 + 1] = t.x * IN_PER_M;
      OUT[outPtr + n * 4 + 2] = t.y * IN_PER_M;
      OUT[outPtr + n * 4 + 3] = e.body.rotation();
      n++;
    }
    return n;
  };

  // The goals ride along with the piece list rather than being configured
  // separately: the field is editable at runtime, and a solver holding last
  // week's goals is worse than one holding none.
  Module.rapierSync = (json) => {
    try {
      const payload = JSON.parse(json);
      setGoals(payload.goals || []);
      sync(payload.pieces || []);
    } catch { /* a malformed list must not take the tick down with it */ }
  };

  return { world, RAPIER };
}
