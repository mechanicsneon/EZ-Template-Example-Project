// Finds EZ-Template PID gains for the 17021V drivetrain with no robot needed.
//
// Every candidate set of gains is run through a list of drives / turns / swings on many
// simulated robots (different weights, batteries, friction, motor wear...). The winner is
// the set that's fastest to settle on target across ALL of them, with extra weight on the
// worst cases so it doesn't fall apart on a robot that's heavier or a battery that's low.
//
//   node tools/pid-tuner/tune.mjs            tune everything
//   node tools/pid-tuner/tune.mjs --check    just score EZ-Template's default gains
//
// Note: the swing gains in autons.cpp (20 / 220) were picked by hand from a speed-vs-overshoot
// comparison. The search's own swing pick is a bit faster but swings 10-19 deg past the target
// on light, low-friction robots, which can hit walls or game pieces.

import { makeRobot, simDrive, simTurn, simSwing } from "./model.mjs";

/////
// What we don't know about the robot, as ranges. Narrow these once you can weigh/measure it.
/////
const RANGES = {
  massLb: [10, 18],        // Robot weight
  trackIn: [10, 13],       // Left wheels to right wheels, center to center
  gyrationIn: [4.5, 6.5],  // How spread out the weight is (sets turning inertia)
  batteryV: [11.2, 12.6],  // Max motor voltage: low vs fresh battery
  mu: [0.7, 1.1],          // Wheel grip on foam tiles
  rollFriction: [0.06, 0.14],  // Drivetrain drag, as a fraction of max push (needs ~8-15 power to start moving)
  scrub: [0.15, 0.45],     // Wheel scrub when turning in place
  strengthL: [0.94, 1.06], // Worn / mismatched motors per side
  strengthR: [0.94, 1.06],
  lagLoops: [1, 3],        // 10ms loops between the robot moving and the code seeing it (V5 is ~20-30ms)
  imuNoise: [0.005, 0.04],   // IMU reading jitter, degrees (a still V5 IMU barely moves)
};

const EXIT_DRIVE = { smallTime: 90, smallError: 1, velocityTime: 500, mATime: 500 };
const EXIT_TURN = { smallTime: 90, smallError: 3, velocityTime: 500, mATime: 500 };

// EZ-Template's defaults (what autons.cpp had before tuning), to compare against
const CURRENT = {
  drive: { kp: 20, kd: 100, hkp: 11, hkd: 20 },
  turn: { kp: 3, ki: 0.05, kd: 20, startI: 15 },
  swing: { kp: 6, kd: 65 },
};

// Motions like the ones autons use. DRIVE_SPEED 110, TURN_SPEED 90, SWING_SPEED 110
const DRIVES = [
  { target: 3, speed: 110, slew: false },
  { target: 6, speed: 110, slew: false },
  { target: 12, speed: 110, slew: false },
  { target: 24, speed: 110, slew: true },
  { target: 48, speed: 110, slew: true },
  { target: -12, speed: 110, slew: false },
  { target: -24, speed: 110, slew: true },
  { target: 24, speed: 60, slew: false },
];
const TURNS = [
  { from: 0, to: 15, speed: 90 },
  { from: 0, to: 45, speed: 90 },
  { from: 0, to: 90, speed: 90 },
  { from: 0, to: 180, speed: 90 },
  { from: 90, to: 0, speed: 90 },
  { from: 45, to: -45, speed: 90 },
  { from: 0, to: -90, speed: 90 },
];
const SWINGS = [
  { to: 45, speed: 110, opposite: 0 },
  { to: 90, speed: 110, opposite: 0 },
  { to: 45, speed: 110, opposite: 45 },
  { to: 90, speed: 110, opposite: 45 },
];

/////
// Simulated robots: a spread over the ranges plus the nasty corners
/////
function rng(seed) {
  return () => {
    seed |= 0;
    seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function robots(count, seed) {
  const rand = rng(seed);
  const keys = Object.keys(RANGES);
  // Latin hypercube: each range gets evenly covered
  const slots = Object.fromEntries(keys.map((k) => [k, [...Array(count).keys()].sort(() => rand() - 0.5)]));
  const list = [];
  for (let i = 0; i < count; i++) {
    const p = {};
    for (const k of keys) {
      const [lo, hi] = RANGES[k];
      const u = (slots[k][i] + rand()) / count;
      p[k] = k === "lagLoops" ? Math.round(lo + u * (hi - lo)) : lo + u * (hi - lo);
    }
    p.startHeading = (rand() - 0.5) * 4;  // Previous turn ended up to 2 deg off
    p.imuDrift = 0;
    p.seed = seed * 1000 + i + 1;
    list.push(p);
  }
  // Worst cases: heavy + weak battery + grippy, and light + fresh battery + slippery
  const mid = (k) => (RANGES[k][0] + RANGES[k][1]) / 2;
  const base = Object.fromEntries(Object.keys(RANGES).map((k) => [k, mid(k)]));
  list.push({ ...base, massLb: 18, batteryV: 11.2, mu: 1.1, rollFriction: 0.14, scrub: 0.45, lagLoops: 3, strengthL: 0.94, strengthR: 1.06, startHeading: 2, imuDrift: 0, imuNoise: 0.04, seed: 7 });
  list.push({ ...base, massLb: 10, batteryV: 12.6, mu: 0.7, rollFriction: 0.06, scrub: 0.15, lagLoops: 3, strengthL: 1.06, strengthR: 0.94, startHeading: -2, imuDrift: 0, imuNoise: 0.04, seed: 8 });
  return list.map(makeRobot);
}

const TRAIN = robots(16, 17021);
const TEST = robots(40, 5);  // Different robots, to check the result isn't overfit to TRAIN

/////
// Scoring: seconds until EZ says the motion is done, plus how far off it ended up.
// Lower is better. An exit that isn't "small" (stalled/timed out) costs an extra second.
// Jitter (motor output jumping around) is charged too: on a real robot it heats motors,
// shakes the chassis and means the gains are living on the edge.
/////
const MISS = 1.0;
const JITTER = 0.0003;  // Seconds of cost per unit of output change (a clean move is ~250)

function driveCost(r) {
  return r.time + 0.3 * Math.abs(r.error) + 0.3 * Math.max(0, r.overshoot - 0.3) +
         0.1 * Math.abs(r.headingError) + 0.3 * Math.abs(r.drift) + JITTER * r.jitter + (r.kind === "small" ? 0 : MISS);
}
function angleCost(r) {
  return r.time + 0.03 * Math.abs(r.error) + 0.06 * Math.max(0, r.overshoot - 2) + JITTER * r.jitter + (r.kind === "small" ? 0 : MISS);
}

// Average, plus half the average of the worst 20% (so the bad robots matter more)
function robust(costs) {
  const sorted = [...costs].sort((a, b) => b - a);
  const tail = sorted.slice(0, Math.max(1, Math.round(sorted.length * 0.2)));
  const avg = (a) => a.reduce((x, y) => x + y, 0) / a.length;
  return avg(costs) + 0.5 * avg(tail);
}

function scoreDrive(g, set = TRAIN) {
  const costs = [];
  for (const robot of set)
    for (const m of DRIVES) costs.push(driveCost(simDrive(robot, { ...g, exit: EXIT_DRIVE }, { ...m, startHeading: robot.startHeading })));
  return robust(costs);
}
function scoreTurn(g, set = TRAIN) {
  const costs = [];
  for (const robot of set) for (const m of TURNS) costs.push(angleCost(simTurn(robot, { ...g, exit: EXIT_TURN }, m)));
  return robust(costs);
}
function scoreSwing(g, drive, set = TRAIN) {
  const costs = [];
  for (const robot of set) for (const m of SWINGS) costs.push(angleCost(simSwing(robot, { ...g, exit: EXIT_TURN }, drive, m)));
  return robust(costs);
}

/////
// Nelder-Mead search in log space (gains are always positive and span orders of magnitude)
/////
function nelderMead(f, x0, { iters = 250, step = 0.5 } = {}) {
  const n = x0.length;
  let simplex = [x0, ...x0.map((_, i) => x0.map((v, j) => (i === j ? v + step : v)))].map((x) => ({ x, f: f(x) }));
  for (let it = 0; it < iters; it++) {
    simplex.sort((a, b) => a.f - b.f);
    const best = simplex[0], worst = simplex[n], second = simplex[n - 1];
    const centroid = x0.map((_, j) => simplex.slice(0, n).reduce((s, p) => s + p.x[j], 0) / n);
    const along = (t) => centroid.map((c, j) => c + t * (worst.x[j] - c));
    const reflect = { x: along(-1) };
    reflect.f = f(reflect.x);
    if (reflect.f < best.f) {
      const expand = { x: along(-2) };
      expand.f = f(expand.x);
      simplex[n] = expand.f < reflect.f ? expand : reflect;
    } else if (reflect.f < second.f) {
      simplex[n] = reflect;
    } else {
      const contract = { x: along(reflect.f < worst.f ? -0.5 : 0.5) };
      contract.f = f(contract.x);
      if (contract.f < Math.min(worst.f, reflect.f)) simplex[n] = contract;
      else simplex = simplex.map((p, i) => (i === 0 ? p : { x: p.x.map((v, j) => best.x[j] + 0.5 * (v - best.x[j])) })).map((p) => (p.f !== undefined ? p : { ...p, f: f(p.x) }));
    }
    const spread = Math.max(...simplex.map((p) => Math.abs(p.f - simplex[0].f)));
    if (spread < 1e-5) break;
  }
  simplex.sort((a, b) => a.f - b.f);
  return simplex[0];
}

// Search from several starting points and keep the best
function search(label, keys, starts, score) {
  const toGains = (x) => Object.fromEntries(keys.map((k, i) => [k, Math.exp(x[i])]));
  let best = null;
  for (const s of starts) {
    const x0 = keys.map((k) => Math.log(s[k]));
    const r = nelderMead((x) => score(toGains(x)), x0);
    process.stdout.write(`  ${label} start ${JSON.stringify(round(s))} -> ${r.f.toFixed(3)}\n`);
    if (!best || r.f < best.f) best = r;
  }
  return toGains(best.x);
}

const round = (g) => Object.fromEntries(Object.entries(g).map(([k, v]) => [k, +v.toPrecision(3)]));

/////
// Reports
/////
function report(name, run, oldG, newG) {
  const rows = run.map(({ label, sim }) => {
    const per = (g) => {
      const res = TEST.map((robot) => sim(robot, g));
      const t = res.map((r) => r.time);
      const avgT = t.reduce((a, b) => a + b, 0) / t.length;
      const worst = Math.max(...t);
      const err = res.reduce((a, r) => a + Math.abs(r.error), 0) / res.length;
      const misses = res.filter((r) => r.kind !== "small").length;
      return { avgT, worst, err, misses };
    };
    return { label, old: per(oldG), new: per(newG) };
  });
  console.log(`\n${name}  (avg / worst time to finish on ${TEST.length} test robots, avg final error, stalls)`);
  console.log("  motion".padEnd(24) + "CURRENT".padEnd(34) + "TUNED");
  const fmt = (r, unit) => `${r.avgT.toFixed(2)}s / ${r.worst.toFixed(2)}s  ${r.err.toFixed(2)}${unit}  ${r.misses ? r.misses + " missed" : ""}`.padEnd(34);
  for (const r of rows) console.log("  " + r.label.padEnd(22) + fmt(r.old, r.unit ?? "") + fmt(r.new, ""));
}

const check = process.argv.includes("--check");
const t0 = Date.now();

console.log("Current gains, training score (lower is better):");
console.log(`  drive ${scoreDrive(CURRENT.drive).toFixed(3)}   turn ${scoreTurn(CURRENT.turn).toFixed(3)}   swing ${scoreSwing(CURRENT.swing, CURRENT.drive).toFixed(3)}`);
if (check) process.exit(0);

console.log("\nSearching drive + heading gains...");
const drive = search("drive", ["kp", "kd", "hkp", "hkd"], [CURRENT.drive, { kp: 10, kd: 40, hkp: 5, hkd: 10 }, { kp: 35, kd: 200, hkp: 15, hkd: 40 }], scoreDrive);

console.log("\nSearching turn gains...");
const turn = search("turn", ["kp", "ki", "kd", "startI", "turnMin"], [{ ...CURRENT.turn, turnMin: 30 }, { kp: 5, ki: 0.1, kd: 35, startI: 15, turnMin: 40 }, { kp: 4, ki: 0.05, kd: 25, startI: 10, turnMin: 50 }], (g) => scoreTurn({ ...g, turnMin: Math.min(127, g.turnMin) }));
// Also try no I at all; keep whichever wins
const turnNoI = search("turn (no I)", ["kp", "kd"], [{ kp: 3, kd: 20 }, { kp: 5, kd: 35 }], (g) => scoreTurn({ ...g, ki: 0, startI: 0 }));
const turnBest = scoreTurn({ ...turnNoI, ki: 0, startI: 0 }) < scoreTurn(turn) ? { ...turnNoI, ki: 0, startI: 0 } : turn;

console.log("\nSearching swing gains...");
const swing = search("swing", ["kp", "kd"], [CURRENT.swing, { kp: 4, kd: 35 }, { kp: 9, kd: 90 }], (g) => scoreSwing(g, drive));

const result = { drive: round(drive), turn: round(turnBest), swing: round(swing) };

report("DRIVE", DRIVES.map((m) => ({
  label: `${m.target > 0 ? "fwd" : "back"} ${Math.abs(m.target)}in @${m.speed}${m.slew ? " slew" : ""}`,
  sim: (robot, g) => simDrive(robot, { ...g, exit: EXIT_DRIVE }, { ...m, startHeading: robot.startHeading }),
})), CURRENT.drive, result.drive);
report("TURN", TURNS.map((m) => ({ label: `${m.from} -> ${m.to} deg`, sim: (robot, g) => simTurn(robot, { ...g, exit: EXIT_TURN }, m) })), CURRENT.turn, result.turn);
report("SWING", SWINGS.map((m) => ({ label: `swing ${m.to} opp ${m.opposite}`, sim: (robot, g) => simSwing(robot, { ...g, exit: EXIT_TURN }, g === CURRENT.swing ? CURRENT.drive : result.drive, m) })), CURRENT.swing, result.swing);

console.log("\nTest-robot scores (lower is better):");
console.log(`  drive  ${scoreDrive(CURRENT.drive, TEST).toFixed(3)} -> ${scoreDrive(result.drive, TEST).toFixed(3)}`);
console.log(`  turn   ${scoreTurn(CURRENT.turn, TEST).toFixed(3)} -> ${scoreTurn(result.turn, TEST).toFixed(3)}`);
console.log(`  swing  ${scoreSwing(CURRENT.swing, CURRENT.drive, TEST).toFixed(3)} -> ${scoreSwing(result.swing, result.drive, TEST).toFixed(3)}`);

const d = result.drive, t = result.turn, s = result.swing;
console.log("\nPaste into default_constants() in src/autons.cpp:");
console.log(`  chassis.pid_drive_constants_set(${d.kp}, 0.0, ${d.kd});`);
console.log(`  chassis.pid_heading_constants_set(${d.hkp}, 0.0, ${d.hkd});`);
console.log(`  chassis.pid_turn_constants_set(${t.kp}, ${t.ki}, ${t.kd}, ${t.startI});`);
if (t.turnMin) console.log(`  chassis.pid_turn_min_set(${Math.round(t.turnMin)});`);
console.log(`  chassis.pid_swing_constants_set(${s.kp}, 0.0, ${s.kd});`);
console.log(`\n${JSON.stringify(result)}`);
console.log(`(${((Date.now() - t0) / 1000).toFixed(0)}s)`);
