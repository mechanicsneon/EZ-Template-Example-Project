// Finds lift PID gains for src/lift.cpp with no robot needed.
//
// The DR4B is modeled at the motor shaft (that's what lift.cpp measures, in degrees): two V5
// motors, gravity pulling down, friction, and a hard stop at the bottom. Almost everything about
// the lift is unknown, so the ranges are wide: either cartridge, any gear ratio, from rubber
// bands doing most of the work to none at all.
//
//   node tools/pid-tuner/lift-tune.mjs

import { PID, v5Motor } from "./model.mjs";

const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));

const RANGES = {
  rpm: [100, 200],            // Lift motor cartridge (red or green)
  gravity: [0.05, 0.45],      // Torque to hold the lift up, as a fraction of both motors' max (rubber bands lower this)
  spinUp: [0.06, 0.3],        // Seconds for the lift to reach full speed with no load (bigger = heavier / more geared)
  friction: [0.03, 0.1],      // Fraction of max torque
  batteryV: [11.2, 12.6],
  lagLoops: [1, 3],
};

// Must match lift.cpp
const SETTLED_DEG = 15, SETTLED_MS = 100, TIMEOUT_MS = 2500;
const CURRENT = { kp: 0.5, ki: 0, kd: 2, startI: 0, downMax: 127 };

const MOVES = [
  { from: 0, to: 150 },
  { from: 0, to: 400 },
  { from: 0, to: 800 },
  { from: 400, to: 800 },
  { from: 800, to: 400 },
  { from: 800, to: 0 },
];

function simLift(robot, g, { from, to }) {
  const m = v5Motor(robot.rpm);
  const maxT = 2 * m.torqueCap;
  const J = maxT / (m.freeSpeed / robot.spinUp);  // Inertia at the motor shaft
  const pid = new PID(g.kp, g.ki, g.kd, g.startI);
  pid.target = to;
  pid.prevCur = from;
  let pos = (from * Math.PI) / 180, vel = 0, volts = 0, seed = robot.seed;
  const history = [];
  let closeSince = null, maxOver = 0, jitter = 0, last = 0;
  const noise = () => ((seed = (seed * 1664525 + 1013904223) % 4294967296) / 4294967296 - 0.5) * 0.5;

  for (let ms = 0; ms < TIMEOUT_MS; ms += 10) {
    const seen = history[Math.max(0, history.length - 1 - robot.lagLoops)] ?? from;
    // downMax = DOWN_SPEED in lift.cpp: gravity already helps going down, so full power overshoots
    const out = Math.trunc(clamp(pid.compute(seen), -g.downMax, 127));
    jitter += Math.abs(out - last);
    last = out;
    for (let i = 0; i < 10; i++) {
      volts += (clamp((out * 12) / 127, -robot.batteryV, robot.batteryV) - volts) * 0.2;
      const motor = 2 * clamp(m.torque0 * (volts / 12 - vel / m.freeSpeed), -m.torqueCap, m.torqueCap);
      let t = motor - robot.gravity * maxT;
      const fr = robot.friction * maxT;
      t = Math.abs(vel) > 0.05 ? t - fr * Math.sign(vel) : Math.abs(t) < fr * 1.2 ? 0 : t - fr * Math.sign(t);
      vel += (t / J) * 0.001;
      if (Math.abs(vel) <= 0.05 && t === 0) vel *= 0.9;
      pos += vel * 0.001;
      if (pos < 0) { pos = 0; vel = Math.max(0, vel); }  // Bottom hard stop
    }
    const deg = (pos * 180) / Math.PI;
    history.push(deg + noise());
    maxOver = Math.max(maxOver, (deg - to) * Math.sign(to - from));
    if (Math.abs(to - deg) < SETTLED_DEG) {
      closeSince ??= ms;
      if (ms - closeSince >= SETTLED_MS) return { time: ms / 1000, error: deg - to, overshoot: maxOver, jitter, done: true };
    } else closeSince = null;
  }
  return { time: TIMEOUT_MS / 1000, error: (pos * 180) / Math.PI - to, overshoot: maxOver, jitter, done: false };
}

function lifts(count, seed) {
  let s = seed;
  const rand = () => ((s = (s * 1664525 + 1013904223) % 4294967296) / 4294967296);
  const list = [];
  for (let i = 0; i < count; i++) {
    const p = { seed: seed * 100 + i };
    for (const [k, [lo, hi]] of Object.entries(RANGES)) p[k] = k === "rpm" ? (rand() < 0.5 ? 100 : 200) : k === "lagLoops" ? Math.round(lo + rand() * (hi - lo)) : lo + rand() * (hi - lo);
    list.push(p);
  }
  // Heavy lift on a weak battery, light fast lift
  list.push({ rpm: 200, gravity: 0.45, spinUp: 0.3, friction: 0.1, batteryV: 11.2, lagLoops: 3, seed: 1 });
  list.push({ rpm: 200, gravity: 0.05, spinUp: 0.06, friction: 0.03, batteryV: 12.6, lagLoops: 3, seed: 2 });
  list.push({ rpm: 100, gravity: 0.45, spinUp: 0.3, friction: 0.1, batteryV: 11.2, lagLoops: 3, seed: 3 });
  return list;
}
const TRAIN = lifts(30, 17021), TEST = lifts(60, 5);

const cost = (r) => r.time + 0.01 * Math.abs(r.error) + 0.03 * Math.max(0, r.overshoot - 10) + 0.0003 * r.jitter + (r.done ? 0 : 1);
function score(g, set = TRAIN) {
  const c = [];
  for (const l of set) for (const mv of MOVES) c.push(cost(simLift(l, g, mv)));
  c.sort((a, b) => b - a);
  const avg = (a) => a.reduce((x, y) => x + y, 0) / a.length;
  return avg(c) + 0.5 * avg(c.slice(0, Math.round(c.length * 0.2)));
}

// Coarse grid, then refine around the best
function search(keys, grid, scoreFn) {
  let best = null;
  // Try every combination in `values`, keeping the best seen so far
  const walk = (values, i = 0, g = {}) => {
    if (i === keys.length) {
      const f = scoreFn(g);
      if (!best || f < best.f) best = { g: { ...g }, f };
      return;
    }
    for (const v of values[keys[i]]) walk(values, i + 1, { ...g, [keys[i]]: v });
  };
  walk(grid);
  for (let round = 0; round < 4; round++) {
    const start = best;
    walk(Object.fromEntries(keys.map((k) => [k, [0.75, 0.88, 1, 1.14, 1.33].map((f) => best.g[k] * f)])));
    if (best === start) break;
  }
  return best;
}

const capped = (g) => ({ ...g, downMax: Math.min(127, g.downMax) });
const withI = search(["kp", "ki", "kd", "startI", "downMax"], { kp: [0.5, 1, 2, 3], ki: [0.005, 0.02, 0.05], kd: [2, 5, 10, 20], startI: [60, 120], downMax: [50, 80, 127] }, (g) => score(capped(g)));
const noI = search(["kp", "kd", "downMax"], { kp: [0.5, 1, 2, 3, 5, 8], kd: [2, 5, 10, 20, 40], downMax: [50, 80, 127] }, (g) => score(capped({ ...g, ki: 0, startI: 0 })));
const best = capped(withI.f < noI.f ? withI.g : { ...noI.g, ki: 0, startI: 0 });
const r3 = (g) => Object.fromEntries(Object.entries(g).map(([k, v]) => [k, +v.toPrecision(3)]));

console.log(`train score  current ${score(CURRENT).toFixed(3)}  with I ${withI.f.toFixed(3)}  no I ${noI.f.toFixed(3)}`);
console.log(`\nLIFT on ${TEST.length} test lifts (avg / worst time, avg error, worst overshoot, didn't settle)`);
for (const [name, g] of [["CURRENT", CURRENT], ["TUNED", best]]) {
  const res = TEST.flatMap((l) => MOVES.map((mv) => simLift(l, g, mv)));
  const n = res.length;
  console.log(`  ${name.padEnd(8)} ${(res.reduce((a, r) => a + r.time, 0) / n).toFixed(2)}s / ${Math.max(...res.map((r) => r.time)).toFixed(2)}s  ${(res.reduce((a, r) => a + Math.abs(r.error), 0) / n).toFixed(1)} deg  ${Math.max(...res.map((r) => r.overshoot)).toFixed(0)} deg  ${res.filter((r) => !r.done).length}/${n}`);
}
const b = r3(best);
console.log(`\nPaste into src/lift.cpp:\n  ez::PID lift_pid{${b.kp}, ${b.ki}, ${b.kd}, ${b.startI}, "Lift"};`);
if (b.downMax < 127) console.log(`  ...and cap the lift's downward output at -${Math.round(b.downMax)} in lift_task()`);

// Where the worst overshoots happen
const all = TEST.flatMap((l) => MOVES.map((mv) => ({ l, mv, r: simLift(l, best, mv) }))).sort((x, y) => y.r.overshoot - x.r.overshoot);
console.log("\nWorst overshoots:");
for (const { l, mv, r } of all.slice(0, 4))
  console.log(`  ${mv.from}->${mv.to}  ${r.overshoot.toFixed(0)} deg  (${l.rpm} rpm, gravity ${l.gravity.toFixed(2)}, spin-up ${l.spinUp.toFixed(2)}s, lag ${l.lagLoops * 10}ms)`);
const over = all.map((a) => a.r.overshoot).sort((x, y) => x - y);
console.log(`  90% of moves overshoot less than ${over[Math.floor(over.length * 0.9)].toFixed(0)} deg`);
