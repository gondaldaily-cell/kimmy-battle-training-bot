// battle.training controller — "kimmy" v8
// Season 2 "Drafted" (s2.0.0). Draft: missile + MBT + drone + scout (hunter mirror,
// but with focus fire, drone-hunting, artillery dodging and proper objective play).
//
// v8 changes (diagnosed from live replays m_8d41d8b7e1ce + m_9db436548725 g1):
//  - v7 DIED live: orders went null from tick ~160-188 and never resumed
//    (4369/4481 and 4298/4445 null ticks) -> units froze near spawn, 0 points.
//    Root cause: a runtime BFS (gridPath, ~41 QuickJS units) blew the 40-unit
//    tick budget; the tick was dropped BEFORE the Nav commit, so the replan
//    condition stayed true forever and EVERY later tick re-ran the BFS and died.
//    FIX: NO runtime BFS at all. Navigation is flow-fields only (built in
//    hello) + drive-straight fallback. tick() can never blow the budget on
//    pathfinding by construction. Local profile confirmed the BFS ticks were
//    the slowest (36.6ms/22.6ms in Node = way over budget in QuickJS).
//  - Flow fields now also anchored at enemy spawn + map centre (better coverage).
//  - nearestPassable is cached per goal tile + radius-capped (was re-run every
//    tick per unit and could spiral expensively).
//  - Objective ASSIGNMENT (spread units across hills) was tried and REVERTED:
//    it dropped the local win rate to 83.1% (holder 90%->72.5%, hunter
//    82.5%->70%) — splitting the force lets holder/hunter kill our units one
//    by one. The assignment code stays in the file (assignObjectives) but is
//    not called. Spreading needs a phased approach (concentrate first, spread
//    after winning the fight) and more testing before it ships.
//  - MBT remembered-chase guard was also reverted with it (same test).
//
// Ideas vs the house bots:
//  - House bots fire at their NEAREST contact each; we FOCUS all guns on one priority
//    target (enemy drone first — it is the hunter's eyes and dies to one missile /
//    one MBT shell — then missile carrier, artillery, MBT, scout, APC, engineer).
//  - Brawler/holder forget contacts after 4 s; we remember 20 s and hunt remembered
//    positions, and we dodge artillery shells (only hunter does that).
//  - Direct guns lead their shots (shell flight time prediction).
//  - Scout caps objectives fast; MBT holds the heavy one at standoff range.
//  - Missile carrier keeps 24-36 tiles from its target (out of MBT range, inside
//    missile range) and goes home to resupply when out of rounds.

const FP = 65536;
const FULL = 4096;

let rules, map, side, mySpawn, enemySpawn, objectives;
let navs = new Map();
let memory = new Map(); // enemy id -> {kind,x,y,hpFrac,speed,heading,tick}
let quietSince = 0;
let lastSeenTick = -1000;
let fields = []; // BFS distance fields from objectives + spawns + centre (built in hello)

const PRI = { drone: 0, missile: 1, artillery: 2, mbt: 3, scout: 4, apc: 5, engineer: 6 };

function angTo(ax, ay, bx, by) {
  return Math.round((Math.atan2(by - ay, bx - ax) / (2 * Math.PI)) * FULL) & (FULL - 1);
}
function angDiff(a, b) { let d = (b - a) & (FULL - 1); return d > FULL / 2 ? d - FULL : d; }
function dist(ax, ay, bx, by) { return Math.hypot(bx - ax, by - ay); }
function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }
function tileOf(v) { return v >> 16; }
function centre(t) { return (t << 16) + FP / 2; }
function passableTile(t) { return t !== 2 && t !== 4 && t !== 6; } // water, rock, ruin
function clampGoal(g) {
  return [clamp(g[0], FP, (map.width << 16) - FP), clamp(g[1], FP, (map.height << 16) - FP)];
}
function awayFrom(u, fx, fy, tiles) {
  const a = angTo(fx, fy, u.x, u.y);
  const rad = (a / FULL) * 2 * Math.PI;
  return clampGoal([Math.round(u.x + Math.cos(rad) * tiles * FP), Math.round(u.y + Math.sin(rad) * tiles * FP)]);
}
// point on the line from target (tx,ty) towards the unit, at `want` tiles from the target
function standOff(u, tx, ty, want) {
  const d = Math.max(1, dist(u.x, u.y, tx, ty));
  const w = want * FP;
  return [Math.round(tx + (u.x - tx) * (w / d)), Math.round(ty + (u.y - ty) * (w / d))];
}
function predict(c, ticks) {
  if (!c.speed) return [c.x, c.y];
  const rad = (c.heading / FULL) * 2 * Math.PI;
  const d = (c.speed / FP) * ticks * FP;
  return [Math.round(c.x + Math.cos(rad) * d), Math.round(c.y + Math.sin(rad) * d)];
}

// ---- navigation: flow fields + waypoint follower (v8: NO runtime BFS) ----
// v7 died live because a runtime BFS (gridPath, ~41 QuickJS units) blew the
// 40-unit tick budget; the dropped tick never committed the Nav state, so the
// replan condition stayed true forever and every later tick died the same way
// (orders null from tick ~160 to the end of the game, 0 points). v8 removes
// the runtime BFS entirely: navigation is flow fields (built in hello, where
// the budget is ~4000 units) plus driving straight at the goal when no field
// covers it. tick() can never blow the budget on pathfinding by construction.
// (Coarse 64x64 BFS was tried in v1: it cost 15 points of win rate — units
// took worse paths and held fewer objectives. Fine flow fields win fights.)
// Spiral search for the nearest passable tile, radius-capped at 24 (a goal
// deep inside a ruin field used to spiral the whole map, every tick, per unit).
// Callers cache the result per goal tile — see Nav.
function nearestPassable(tx, ty) {
  const W = map.width, H = map.height, T = map.terrain;
  const R = 24;
  for (let r = 0; r <= R; r++) {
    for (let dy = -r; dy <= r; dy++) for (let dx = -r; dx <= r; dx++) {
      if (Math.max(Math.abs(dx), Math.abs(dy)) !== r) continue;
      const nx = tx + dx, ny = ty + dy;
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      if (passableTile(T[ny * W + nx])) return [nx, ny];
    }
  }
  return [tx, ty];
}

// ---- flow fields: BFS distance field from one anchor tile (built in hello) ----
function bfsField(ax, ay) {
  const W = map.width, H = map.height, T = map.terrain;
  const F = new Int16Array(W * H).fill(-1);
  const start = ay * W + ax;
  F[start] = 0;
  const q = [start];
  let qi = 0;
  while (qi < q.length) {
    const i = q[qi++];
    const x = i % W, y = (i - x) / W;
    const nd = F[i] + 1;
    for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
      if (!dx && !dy) continue;
      const nx = x + dx, ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      const j = ny * W + nx;
      if (F[j] !== -1) continue;
      const tj = T[j];
      if (tj === 2 || tj === 4 || tj === 6) continue;
      if (dx && dy) {
        const t1 = T[y * W + nx], t2 = T[ny * W + x];
        if (t1 === 2 || t1 === 4 || t1 === 6 || t2 === 2 || t2 === 4 || t2 === 6) continue;
      }
      F[j] = nd;
      q.push(j);
    }
  }
  return F;
}

// Cheap path: descend the distance field of the anchor nearest the goal.
// Costs ~1-2 QuickJS units (a few hundred trivial steps) instead of ~41 for
// a runtime BFS, so it never blows the tick budget. Returns null when the
// goal is far from every anchor or the descent can't reach it — the caller
// then falls back to a runtime BFS.
function fieldPath(ax, ay, bx, by) {
  const W = map.width;
  let best = null, bestD = Infinity;
  for (const f of fields) {
    const d = f.field[by * W + bx];
    if (d >= 0 && d < bestD) { bestD = d; best = f; }
  }
  if (!best || bestD > 30) return null;
  const F = best.field;
  let cur = F[ay * W + ax];
  if (cur < 0) return null;
  const path = [];
  let x = ax, y = ay;
  for (let step = 0; step < 300; step++) {
    if (Math.max(Math.abs(x - bx), Math.abs(y - by)) <= 4) { path.push([bx, by]); return path; }
    if (cur === 0) return null; // reached the anchor without nearing the goal
    let nx = -1, ny = -1, nv = cur;
    for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
      if (!dx && !dy) continue;
      const px = x + dx, py = y + dy;
      if (px < 0 || py < 0 || px >= W || py >= map.height) continue;
      const v = F[py * W + px];
      if (v >= 0 && v < nv) { nv = v; nx = px; ny = py; }
    }
    if (nx === -1) return null; // no descent (can't happen in a BFS field)
    path.push([nx, ny]);
    x = nx; y = ny; cur = nv;
  }
  return null; // path too long
}
class Nav {
  constructor() { this.path = []; this.goal = null; this.pgoal = null; this.age = 0; this.sinceReplan = 0; this.stuck = 0; }
  next(u, gx, gy, nudge) {
    // Cache the passable goal tile: nearestPassable only runs when the raw
    // goal tile changed (v7 ran the spiral every tick for every unit).
    const raw = [tileOf(gx), tileOf(gy)];
    let g;
    if (this.pgoal && this.pgoal[2] === raw[0] && this.pgoal[3] === raw[1]) g = this.pgoal;
    else {
      const p = nearestPassable(raw[0], raw[1]);
      this.pgoal = [p[0], p[1], raw[0], raw[1]];
      g = this.pgoal;
    }
    const here = [tileOf(u.x), tileOf(u.y)];
    this.age++; this.sinceReplan++;
    this.stuck = u.blocked ? this.stuck + 1 : 0;
    const nearGoal = Math.max(Math.abs(here[0] - g[0]), Math.abs(here[1] - g[1])) <= 8;
    const goalChanged = !this.goal || Math.max(Math.abs(this.goal[0] - g[0]), Math.abs(this.goal[1] - g[1])) >= 8;
    if (!nearGoal && (goalChanged || this.age > 200 + (nudge % 7) || (u.blocked && this.sinceReplan > 8))) {
      // Flow-field path only — cheap (~1-2 units) and always commits, so a
      // replan can never wedge the Nav into a permanent retry loop (the v7
      // killer: a BFS that blew the budget left goal/age uncommitted, so the
      // next tick retried the same BFS and died the same way, forever).
      // When no field covers the goal, path = [] and we drive straight at it;
      // driveTowards() steers around local obstacles and the stuck-backoff
      // reverses us out of dead ends.
      const fp = fieldPath(here[0], here[1], g[0], g[1]);
      this.goal = g; this.age = 0; this.sinceReplan = 0;
      this.path = fp || [];
    }
    while (this.path.length && this.path[0][0] === here[0] && this.path[0][1] === here[1]) this.path.shift();
    const wp = this.path[0] || g;
    return { wp: [centre(wp[0]), centre(wp[1])], stuck: this.stuck };
  }
}
function navFor(id) {
  let n = navs.get(id);
  if (!n) { n = new Nav(); navs.set(id, n); }
  return n;
}
function driveTowards(u, turnRate, tx, ty, slow) {
  let want = angTo(u.x, u.y, tx, ty);
  const clear = (ang) => {
    for (const ahead of [FP * 0.8, FP * 1.6]) {
      const rad = (ang / FULL) * 2 * Math.PI;
      const px = u.x + Math.round(ahead * Math.cos(rad)), py = u.y + Math.round(ahead * Math.sin(rad));
      const txl = px >> 16, tyl = py >> 16;
      if (txl < 0 || tyl < 0 || txl >= map.width || tyl >= map.height) return false;
      if (!passableTile(map.terrain[tyl * map.width + txl])) return false;
    }
    return true;
  };
  if (!clear(want)) {
    for (const off of [341, 683, 1024, 1365]) {
      if (clear((want + off) & (FULL - 1))) { want = (want + off) & (FULL - 1); break; }
      if (clear((want - off) & (FULL - 1))) { want = (want - off) & (FULL - 1); break; }
    }
  }
  const d = angDiff(u.heading, want);
  const steer = clamp(Math.round((d * FP) / Math.max(1, turnRate)), -FP, FP);
  const ad = Math.abs(d);
  const throttle = ad > 512 ? Math.round(FP * 0.15) : ad > 256 ? Math.round(FP * 0.5) : slow ? Math.round(FP * 0.6) : FP;
  return { steer, throttle };
}

// ---- objectives ----
function pickObjective(x, y) {
  // Highest weight per tile of distance. An objective we already hold is
  // skipped — UNLESS this unit is standing inside it, in which case it is
  // the holder and must STAY (skipping it would make the holder abandon the
  // point it just capped and ping-pong between objectives, holding none).
  let best = null, score = -1;
  for (const o of objectives || []) {
    const d = dist(x, y, o.x, o.y) / FP;
    const inside = d <= (o.radius || 0) / FP;
    if (o.owner === side && o.state === "held" && !inside) continue;
    const s = o.weight / (1 + d);
    if (s > score) { score = s; best = o; }
  }
  return best;
}
function heaviestObjective() {
  let best = null;
  for (const o of objectives || []) if (!best || o.weight > best.weight) best = o;
  return best;
}

// ---- objective assignment (v8) ----
// v7 sent every unit to the same "best" objective (weight/distance), so in the
// kestrel game we held 1 hill while the foe held 2 unopposed and out-scored us
// 3/tick to 2/tick for the whole game. v8 assigns DIFFERENT objectives:
// MBT -> heaviest (it holds), scout -> best remaining (it caps), missile ->
// heaviest (support). A unit standing inside a hill we hold STAYS on it
// (holder rule — abandoning a capped point to chase another loses both).
// Objectives we already hold are only assigned to a unit already inside them;
// everyone else is sent to contestable ones. Recomputed every tick, so the
// assignment adapts as hills change hands.
let assigned = new Map(); // unit id -> objective
function assignObjectives(myUnits) {
  assigned = new Map();
  const taken = new Set();
  // pass 1: holders stay
  for (const u of myUnits) {
    for (const o of objectives || []) {
      const d = dist(u.x, u.y, o.x, o.y) / FP;
      if (d <= (o.radius || 0) / FP && o.owner === side && o.state === "held") {
        assigned.set(u.id, o); taken.add(o); break;
      }
    }
  }
  // pass 2: remaining units, MBT first, then scout, then missile, then rest
  const order = { mbt: 0, scout: 1, missile: 2, drone: 3, apc: 4, artillery: 5, engineer: 6 };
  const rest = myUnits.filter((u) => !assigned.has(u.id))
    .sort((a, b) => (order[a.kind] ?? 9) - (order[b.kind] ?? 9));
  for (const u of rest) {
    let best = null, score = -1;
    for (const o of objectives || []) {
      if (taken.has(o)) continue;
      if (o.owner === side && o.state === "held") continue; // held by us, no one inside -> nobody needs to go
      const d = dist(u.x, u.y, o.x, o.y) / FP;
      const s = o.weight / (1 + d);
      if (s > score) { score = s; best = o; }
    }
    if (!best) {
      // everything is held by us or already taken: share the best one
      best = pickObjective(u.x, u.y);
    }
    if (best) { assigned.set(u.id, best); taken.add(best); }
  }
}
function assignedObjective(u) { return assigned.get(u.id) || null; }

globalThis.controller = {
  draft(h) {
    const d = h.draft;
    if (!d) return undefined;
    const pick = ["missile", "mbt", "drone", "scout"];
    const cost = pick.reduce((c, k) => c + (d.costs[k] ?? 99), 0);
    return cost <= d.budget && pick.every((k) => d.roster.includes(k)) ? pick : undefined;
  },

  hello(h) {
    rules = h.rules; map = h.map; side = h.side;
    objectives = h.objectives || [];
    const s0 = h.map.spawns[side], s1 = h.map.spawns[1 - side];
    mySpawn = [centre(s0[0]), centre(s0[1])];
    enemySpawn = [centre(s1[0]), centre(s1[1])];
    navs = new Map();
    memory = new Map();
    quietSince = 0; lastSeenTick = -1000;
    // Flow fields: one BFS distance field per objective + my spawn + enemy
    // spawn + map centre, built here in hello() (4000-unit budget) so tick()
    // can navigate almost for free. v8 adds the enemy spawn + centre anchors:
    // v7's fields only covered objectives + spawn, so goals on the far side
    // of the map (enemy spawn pushes, remembered contacts) were >30 tiles
    // from every anchor, fieldPath returned null, and the runtime BFS fired.
    // (v8 has no runtime BFS at all — no pre-seeding is needed either, since
    // the first tick-40 replan now uses the cheap fieldPath, not a BFS.)
    fields = [];
    for (const o of objectives) {
      const t = nearestPassable(tileOf(o.x), tileOf(o.y));
      fields.push({ x: t[0], y: t[1], field: bfsField(t[0], t[1]) });
    }
    fields.push({ x: tileOf(mySpawn[0]), y: tileOf(mySpawn[1]), field: bfsField(tileOf(mySpawn[0]), tileOf(mySpawn[1])) });
    fields.push({ x: tileOf(enemySpawn[0]), y: tileOf(enemySpawn[1]), field: bfsField(tileOf(enemySpawn[0]), tileOf(enemySpawn[1])) });
    fields.push({ x: map.width >> 1, y: map.height >> 1, field: bfsField(map.width >> 1, map.height >> 1) });
  },

  tick(o) {
    // live objective state (owner/state change during the match; the hello()
    // copy is stale — without this, units pile onto objectives we already hold)
    if (o.objectives) objectives = o.objectives;
    // --- memory ---
    for (const c of o.contacts) memory.set(c.id, { kind: c.kind, x: c.x, y: c.y, hpFrac: c.hpFrac, speed: c.speed, heading: c.heading, tick: o.tick });
    for (const [id, m] of memory) if (o.tick - m.tick > 400) memory.delete(id);
    if (o.contacts.length) { lastSeenTick = o.tick; quietSince = o.tick; }

    // --- shared priority target (live only) ---
    let target = null, tKey = Infinity;
    for (const c of o.contacts) {
      const dTiles = dist(0, 0, c.x, c.y) * 0 + dist(c.x, c.y, mySpawn[0], mySpawn[1]) / FP;
      const key = (PRI[c.kind] ?? 9) * 1e9 + c.hpFrac * 1e4 + dTiles;
      if (key < tKey) { tKey = key; target = c; }
    }
    // remembered target for movement (nearest remembered contact)
    let remembered = null, rBest = Infinity;
    for (const m of memory.values()) {
      const d = dist(m.x, m.y, mySpawn[0], mySpawn[1]);
      if (d < rBest) { rBest = d; remembered = m; }
    }

    const units = [];
    const myUnits = o.units.filter((u) => u.alive);
    const mbt = myUnits.find((u) => u.kind === "mbt");
    const scout = myUnits.find((u) => u.kind === "scout");
    const droneAlive = myUnits.some((u) => u.kind === "drone");
    // v8: spread units across different objectives (see assignObjectives)
    // NOTE: full spreading REGRESSED local win rate (83.1% vs v7 91.25% —
    // splitting the force lets holder/hunter kill our units one by one).
    // Disabled for now: assignment stays a no-op and the code falls back to
    // pickObjective (concentrated force, v7 behaviour). The controller-death
    // fix (no runtime BFS) is the v8 change that matters; objective spreading
    // needs a phased approach (concentrate first, spread after the fight is
    // won) and more testing before it ships.
    // assignObjectives(myUnits);

    for (const u of myUnits) {
      const s = rules.units[u.kind];
      const order = { id: u.id };
      let goal = null;
      let slow = false;

      // --- dodge incoming artillery ---
      for (const inc of o.incoming) {
        if (dist(inc.x, inc.y, u.x, u.y) < FP * 4.5 && inc.ticks < 45) { goal = awayFrom(u, inc.x, inc.y, 6); break; }
      }

      // --- per-kind goals ---
      if (!goal) {
        if (u.kind === "drone") {
          // forward observation: orbit a point between centre and my side;
          // keep >32 tiles from any known enemy (MBT range is 30, drone dies
          // to 1 hit — and it is the missile's eyes, so survival comes first)
          let nearestE = null, nd = Infinity;
          for (const c of o.contacts) { const d = dist(u.x, u.y, c.x, c.y); if (d < nd) { nd = d; nearestE = c; } }
          if (nearestE && nd < FP * 32) goal = awayFrom(u, nearestE.x, nearestE.y, 10);
          else {
            const heavy = heaviestObjective();
            const cx = heavy ? heavy.x : (map.width / 2) * FP;
            const cy = heavy ? heavy.y : (map.height / 2) * FP;
            const ox = cx + (enemySpawn[0] - cx) * 0.15;
            const oy = cy + (enemySpawn[1] - cy) * 0.15;
            const ang = ((o.tick % 600) / 600) * 2 * Math.PI;
            goal = [Math.round(ox + Math.cos(ang) * FP * 6), Math.round(oy + Math.sin(ang) * FP * 6)];
          }
        } else if (u.kind === "missile") {
          if (u.ammo === 0) {
            goal = mySpawn; // resupply at spawn
          } else if (target) {
            const d = dist(u.x, u.y, target.x, target.y);
            if (d < FP * 22) goal = awayFrom(u, target.x, target.y, 8);
            else if (d > FP * 36) goal = standOff(u, target.x, target.y, 30);
            else goal = [u.x, u.y];
          } else if (remembered) {
            // blind without the drone? close in to re-acquire (own sensor is
            // only 18 tiles) instead of holding a 30-tile standoff from a
            // stale position and never seeing anything again
            goal = droneAlive ? standOff(u, remembered.x, remembered.y, 30)
                              : standOff(u, remembered.x, remembered.y, 14);
          } else {
            const obj = pickObjective(u.x, u.y) || heaviestObjective();
            if (obj) goal = droneAlive ? standOff(u, obj.x, obj.y, 26) : [obj.x, obj.y];
            else goal = [u.x, u.y];
          }
        } else if (u.kind === "mbt") {
          if (u.hp < 60 && target) { goal = awayFrom(u, target.x, target.y, 10); }
          else if (target) {
            const d = dist(u.x, u.y, target.x, target.y);
            if (d > FP * 22) goal = standOff(u, target.x, target.y, 21);
            else if (d < FP * 10) goal = awayFrom(u, target.x, target.y, 6);
            else goal = [u.x, u.y];
            slow = d < FP * 26;
          } else if (remembered) {
            goal = [remembered.x, remembered.y];
          } else {
            const obj = pickObjective(u.x, u.y);
            if (obj) goal = [obj.x, obj.y];
            else if (o.tick - quietSince > 300) goal = enemySpawn;
            else goal = [u.x, u.y];
          }
        } else if (u.kind === "scout") {
          // hunt enemy drones first (2 scout hits kill one), else cap objectives
          let drone = null, dd = Infinity;
          for (const c of o.contacts) if (c.kind === "drone") { const d = dist(u.x, u.y, c.x, c.y); if (d < dd) { dd = d; drone = c; } }
          if (drone && dd < FP * 24) goal = [drone.x, drone.y];
          else {
            const obj = pickObjective(u.x, u.y);
            if (obj) goal = [obj.x, obj.y];
            else if (target) goal = standOff(u, target.x, target.y, 14);
            else if (mbt) goal = [mbt.x, mbt.y];
            else goal = [u.x, u.y];
          }
        } else {
          const obj = pickObjective(u.x, u.y);
          goal = obj ? [obj.x, obj.y] : [u.x, u.y];
        }
      }

      // --- movement ---
      if (o.canMove && goal && !(goal[0] === u.x && goal[1] === u.y)) {
        const g = clampGoal(goal);
        const nav = navFor(u.id).next(u, g[0], g[1], u.id);
        const drv = driveTowards(u, s.turnRate, nav.wp[0], nav.wp[1], slow);
        order.steer = drv.steer; order.throttle = drv.throttle;
        // stuck backoff: reverse out while steering TOWARD the waypoint
        // (the old random-spin backoff spun units into walls/friendlies)
        if (nav.stuck > 2) {
          order.throttle = Math.round(-FP * 0.7);
          const want = angTo(u.x, u.y, nav.wp[0], nav.wp[1]);
          const dd = angDiff(u.heading, want);
          order.steer = clamp(Math.round((dd * FP) / Math.max(1, s.turnRate)), -FP, FP);
        }
        // resupply parking: stop at spawn
        if (u.kind === "missile" && u.ammo === 0 && dist(u.x, u.y, mySpawn[0], mySpawn[1]) < FP * 3) {
          order.throttle = 0; order.steer = 0;
        }
        // scout: face the target while chasing (fixed gun rides the hull)
        if (u.kind === "scout" && target && dist(u.x, u.y, target.x, target.y) < FP * 20) {
          const want = angTo(u.x, u.y, target.x, target.y);
          const d = angDiff(u.heading, want);
          order.steer = clamp(Math.round((d * FP) / Math.max(1, s.turnRate)), -FP, FP);
        }
      }

      // --- weapons ---
      if (u.kind === "missile") {
        if (target && u.ammo !== 0) {
          const d = dist(u.x, u.y, target.x, target.y);
          order.target = target.id;
          order.aimAt = [target.x, target.y];
          const lockNeed = s.gun.lockTicks || 40;
          if (d >= s.gun.minRange && d <= s.gun.maxRange && u.lockTarget === target.id && u.lockTicks >= lockNeed && u.reload === 0) {
            order.fire = true;
          }
        }
      } else if (s.gun.mode === "direct" && target) {
        const d = dist(u.x, u.y, target.x, target.y);
        if (d <= s.gun.maxRange) {
          const flight = Math.max(1, Math.round(d / Math.max(1, s.gun.shellSpeed)));
          const [px, py] = predict(target, flight);
          order.aimAt = [px, py];
          const aimed = Math.abs(angDiff(u.turret, angTo(u.x, u.y, px, py))) <= s.gun.aimTolerance;
          if (u.reload === 0 && aimed) order.fire = true;
        } else {
          order.aimAt = [target.x, target.y];
        }
      } else if (s.secondary && target) {
        const d = dist(u.x, u.y, target.x, target.y);
        if (d <= s.secondary.maxRange && u.reload2 === 0) order.fire = true;
      }

      units.push(order);
    }
    return { units };
  },
};
