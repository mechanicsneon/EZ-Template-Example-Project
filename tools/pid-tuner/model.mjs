// Physics model of the 17021V drivetrain + a line-for-line copy of EZ-Template 3.2.2's
// PID, slew, exit conditions and drive/turn/swing tasks, so gains tuned here behave the
// same way on the robot.
//
// Units: inches, degrees, seconds unless a name says otherwise. EZ outputs are -127..127.

export const sgn = (x) => (x > 0 ? 1 : x < 0 ? -1 : 0);
const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
const IN = 0.0254;  // meters per inch
const G = 9.81;

/////
// EZ-Template clones (src: EZ-Template/PID.cpp, slew.cpp, drive/pid_tasks.cpp)
/////

export class PID {
  constructor(kp, ki = 0, kd = 0, startI = 0) {
    Object.assign(this, { kp, ki, kd, startI });
    this.target = 0;
    this.error = 0;
    this.cur = 0;
    this.prevCur = 0;
    this.integral = 0;
    this.derivative = 0;
    this.output = 0;
  }
  compute(current) {
    this.error = this.target - current;
    this.cur = current;
    // Derivative on measurement
    this.derivative = this.cur - this.prevCur;
    if (this.ki !== 0) {
      if (Math.abs(this.error) < this.startI) this.integral += this.error;
      // EZ compares the error's sign to the previous *position*, not the previous error
      if (sgn(this.error) !== sgn(this.prevCur)) this.integral = 0;
    }
    this.output = this.error * this.kp + this.integral * this.ki - this.derivative * this.kd;
    this.prevCur = this.cur;
    return this.output;
  }
}

// PID::exit_condition with a motor list. Note: in EZ the "big error" exit is in an
// `else if` behind the small-error check, so it never runs when a small error is set.
export class Exit {
  constructor({ smallTime, smallError, velocityTime, mATime }) {
    Object.assign(this, { smallTime, smallError, velocityTime, mATime });
    this.j = this.k = this.l = 0;
  }
  check(pid, overCurrent, dt = 10) {
    if (this.mATime) {
      if (overCurrent) {
        this.l += dt;
        if (this.l > this.mATime) return "mA";
      } else this.l = 0;
    }
    if (Math.abs(pid.error) < this.smallError) {
      this.j += dt;
      if (this.j > this.smallTime) return "small";
    } else this.j = 0;
    if (this.velocityTime) {
      if (Math.abs(pid.derivative) <= 0.05) {
        this.k += dt;
        if (this.k > this.velocityTime) return "velocity";
      } else this.k = 0;
    }
    return null;
  }
}

export class Slew {
  constructor(distance, minSpeed) {
    Object.assign(this, { distance, minSpeed });
  }
  init(enabled, maxSpeed, target, current) {
    this.enabled = maxSpeed < this.minSpeed ? false : enabled;
    this.maxSpeed = maxSpeed;
    this.sign = sgn(target - current);
    this.xInt = current + this.distance * this.sign;
    this.yInt = maxSpeed * this.sign;
    this.slope = (this.sign * this.minSpeed - this.yInt) / (this.xInt - current);
    this.last = maxSpeed;
  }
  iterate(current) {
    if (this.enabled) {
      const error = this.xInt - current;
      if (sgn(error) !== this.sign) this.enabled = false;
      else this.last = (this.slope * error + this.yInt) * this.sign;
    } else this.last = this.maxSpeed;
    return this.last;
  }
}

/////
// Robot physics
/////

// V5 11W motor, output-shaft numbers for a cartridge. The current limit (2.5A) caps torque
// at the rated stall torque; without it the torque line would hit twice that.
export function v5Motor(rpm) {
  const freeSpeed = (rpm * 2 * Math.PI) / 60;  // rad/s at 12V
  const torqueCap = 2.1 * (100 / rpm);          // N*m, 2.1 N*m on the 100 rpm cartridge
  return { freeSpeed, torqueCap, torque0: 2 * torqueCap, ticksPerRev: { 100: 1800, 200: 900, 600: 300 }[rpm] };
}

// One drivetrain with everything we don't know about it spread across ranges.
// Known: 2 motors per side, blue 600 rpm cartridges geared to 450 rpm, 3.25" wheels.
export function makeRobot(p) {
  const motor = v5Motor(600);
  const gear = 600 / 450;  // Motor turns per wheel turn
  const r = (3.25 / 2) * IN;
  const mass = p.massLb * 0.4536;
  const track = p.trackIn * IN;
  const inertia = mass * (p.gyrationIn * IN) ** 2;
  return { ...p, motor, gear, r, mass, track, inertia, ticksPerInch: (motor.ticksPerRev * gear) / (Math.PI * 3.25) };
}

// Simulates the robot for one EZ loop (10ms) in 1ms steps.
// cmd = [left, right] in -127..127 (already truncated to int like private_drive_set).
export function stepRobot(robot, s, cmdL, cmdR) {
  const { motor, gear, r, mass, track, inertia } = robot;
  // How jumpy the motor commands are (real motors heat up and the chassis shakes when this is high)
  s.jitter += (Math.abs(cmdL - s.lastL) + Math.abs(cmdR - s.lastR)) / 2;
  s.lastL = cmdL;
  s.lastR = cmdR;
  const vmax = robot.batteryV;
  const forceMax = 2 * (motor.torqueCap * gear) / r;               // One side, both motors, at the current limit
  const traction = (robot.mu * mass * G) / 2;                        // One side
  const fricLin = robot.rollFriction * forceMax;                     // Coulomb friction per side
  const scrub = robot.scrub * mass * G * (track / 2) * 0.5;          // Coulomb torque resisting rotation
  let overL = false, overR = false;

  for (let i = 0; i < 10; i++) {
    // Motor voltage follows the command with the motor's own ~5ms response
    const wantL = clamp((cmdL * 12) / 127, -vmax, vmax) * robot.strengthL;
    const wantR = clamp((cmdR * 12) / 127, -vmax, vmax) * robot.strengthR;
    s.vL += (wantL - s.vL) * 0.2;
    s.vR += (wantR - s.vR) * 0.2;

    const vLeft = s.v + (s.w * track) / 2;  // Ground speed of each side (m/s), w > 0 = clockwise
    const vRight = s.v - (s.w * track) / 2;
    const motorTorque = (volts, wheelSpeed) => {
      const omega = (wheelSpeed / r) * gear;
      const t = motor.torque0 * (volts / 12 - omega / motor.freeSpeed);
      return clamp(t, -motor.torqueCap, motor.torqueCap);
    };
    const tL = motorTorque(s.vL, vLeft);
    const tR = motorTorque(s.vR, vRight);
    overL = Math.abs(tL) >= motor.torqueCap * 0.999;
    overR = Math.abs(tR) >= motor.torqueCap * 0.999;
    let fL = clamp((2 * tL * gear) / r, -traction, traction);
    let fR = clamp((2 * tR * gear) / r, -traction, traction);

    // Friction: while a side is (nearly) still it cancels any push up to the breakaway
    // force, so small outputs don't move the robot (that's what makes PID stall short)
    const fric = (vel, f, limit) => (Math.abs(vel) > 0.004 ? -limit * sgn(vel) : -clamp(f, -limit * 1.2, limit * 1.2));
    fL += fric(vLeft, fL, fricLin);
    fR += fric(vRight, fR, fricLin);

    const dt = 0.001;
    let torque = ((fL - fR) * track) / 2;
    torque += Math.abs(s.w) > 0.03 ? -scrub * sgn(s.w) : -clamp(torque, -scrub * 1.2, scrub * 1.2);
    s.v += ((fL + fR) / (mass * 1.1)) * dt;  // +10% for spinning wheels/motors
    s.w += (torque / (inertia * 1.1)) * dt;
    // Stuck by friction: bleed off the leftover creep instead of sliding forever
    const stuckL = Math.abs(vLeft) <= 0.004 && Math.abs(fL) < 1e-9;
    const stuckR = Math.abs(vRight) <= 0.004 && Math.abs(fR) < 1e-9;
    if (stuckL && stuckR) {
      s.v *= 0.9;
      s.w *= 0.9;
    }
    s.encL += ((s.v + (s.w * track) / 2) * dt) / IN;
    s.encR += ((s.v - (s.w * track) / 2) * dt) / IN;
    s.y += (s.v * Math.sin((s.heading * Math.PI) / 180) * dt) / IN;  // Sideways drift, inches
    s.heading += (s.w * dt * 180) / Math.PI;
  }
  // What the brain reads: noisy encoder ticks -> inches, noisy IMU heading to 0.01 deg
  const q = (x, step) => Math.round(x / step) * step;
  s.sensorHistory.push({
    l: q(s.encL + gauss(s) * 0.01, 1 / robot.ticksPerInch),
    r: q(s.encR + gauss(s) * 0.01, 1 / robot.ticksPerInch),
    h: q(s.heading + robot.imuDrift * s.t + gauss(s) * robot.imuNoise, 0.01),
    overL, overR,
  });
  s.t += 0.01;
}

export function newState(heading = 0, seed = 1) {
  return { v: 0, w: 0, vL: 0, vR: 0, encL: 0, encR: 0, y: 0, heading, t: 0, sensorHistory: [], jitter: 0, lastL: 0, lastR: 0, seed };
}

// Repeatable random noise (same robot + same motion = same noise, so the search is fair)
function gauss(s) {
  const u = () => {
    s.seed = (s.seed * 1664525 + 1013904223) % 4294967296;
    return (s.seed + 0.5) / 4294967296;
  };
  return Math.sqrt(-2 * Math.log(u())) * Math.cos(2 * Math.PI * u());
}

// Sensor values the code sees, `lag` loops old
function sense(s, lag) {
  const h = s.sensorHistory;
  return h[Math.max(0, h.length - 1 - lag)] ?? { l: s.encL, r: s.encR, h: s.heading, overL: false, overR: false };
}

/////
// EZ motions
/////

const POST_EXIT_MS = 300;  // EZ keeps running the PID after pid_wait() returns, until the next motion
const TIMEOUT_MS = 5000;

// pid_drive_set(target, speed, slew) + pid_wait()
export function simDrive(robot, gains, { target, speed, slew, startHeading = 0 }) {
  const s = newState(startHeading, robot.seed);
  const lag = robot.lagLoops;
  const left = new PID(gains.kp, 0, gains.kd);
  const right = new PID(gains.kp, 0, gains.kd);
  const heading = new PID(gains.hkp, 0, gains.hkd);
  left.target = target;
  right.target = target;
  heading.target = 0;
  heading.prevCur = startHeading;
  const slewL = new Slew(3, 70), slewR = new Slew(3, 70);
  slewL.init(slew, speed, target, 0);
  slewR.init(slew, speed, target, 0);
  const exitL = new Exit(gains.exit), exitR = new Exit(gains.exit);

  let doneL = null, doneR = null, exitT = null, maxPos = -Infinity, minPos = Infinity;
  for (let ms = 0; ms < TIMEOUT_MS + POST_EXIT_MS; ms += 10) {
    const z = sense(s, lag);
    left.compute(z.l);
    right.compute(z.r);
    heading.compute(z.h);
    slewL.iterate(z.l);
    slewR.iterate(z.r);
    let l = left.output, r = right.output;
    let maxSlew = Math.max(slewL.last, slewR.last);
    let faster = Math.max(Math.abs(l), Math.abs(r));
    if (faster > maxSlew) { l *= maxSlew / faster; r *= maxSlew / faster; }
    l += heading.output;
    r -= heading.output;
    maxSlew = Math.max(slewL.last, slewR.last);
    faster = Math.max(Math.abs(l), Math.abs(r));
    if (faster > maxSlew) { l *= maxSlew / faster; r *= maxSlew / faster; }
    stepRobot(robot, s, Math.trunc(l), Math.trunc(r));

    const pos = (s.encL + s.encR) / 2;
    maxPos = Math.max(maxPos, pos);
    minPos = Math.min(minPos, pos);
    if (exitT === null && ms >= 10) {
      doneL ??= exitL.check(left, z.overL);
      doneR ??= exitR.check(right, z.overR);
      if (doneL && doneR) exitT = { ms, kind: doneL === "small" && doneR === "small" ? "small" : doneL !== "small" ? doneL : doneR };
    }
    if (exitT && ms >= exitT.ms + POST_EXIT_MS) break;
  }
  exitT ??= { ms: TIMEOUT_MS, kind: "timeout" };
  const pos = (s.encL + s.encR) / 2;
  const overshoot = target >= 0 ? maxPos - target : target - minPos;
  return { time: exitT.ms / 1000, kind: exitT.kind, error: pos - target, overshoot, headingError: s.heading, drift: s.y, jitter: s.jitter };
}

// pid_turn_set(target, speed) + pid_wait(), starting from startHeading
export function simTurn(robot, gains, { from, to, speed }) {
  const s = newState(from, robot.seed);
  const lag = robot.lagLoops;
  const pid = new PID(gains.kp, gains.ki, gains.kd, gains.startI);
  pid.target = to;
  pid.prevCur = from;
  const exit = new Exit(gains.exit);
  const turnMin = gains.turnMin ?? 30;  // pid_turn_min_set(), EZ default 30

  let exitT = null, maxOver = 0;
  const dir = sgn(to - from);
  for (let ms = 0; ms < TIMEOUT_MS + POST_EXIT_MS; ms += 10) {
    const z = sense(s, lag);
    pid.compute(z.h);
    let out = clamp(pid.output, -speed, speed);
    if (pid.ki !== 0 && Math.abs(pid.target) > pid.startI && Math.abs(pid.error) < pid.startI) out = clamp(out, -turnMin, turnMin);
    out = Math.trunc(out);
    stepRobot(robot, s, out, -out);
    maxOver = Math.max(maxOver, (s.heading - to) * dir);
    if (exitT === null && ms >= 10) {
      const k = exit.check(pid, z.overL || z.overR);
      if (k) exitT = { ms, kind: k };
    }
    if (exitT && ms >= exitT.ms + POST_EXIT_MS) break;
  }
  exitT ??= { ms: TIMEOUT_MS, kind: "timeout" };
  return { time: exitT.ms / 1000, kind: exitT.kind, error: s.heading - to, overshoot: maxOver, jitter: s.jitter };
}

// pid_swing_set(LEFT_SWING, target, speed, opposite) + pid_wait(). Left side drives, right side
// either holds still with the drive PID (opposite = 0) or runs at a scaled speed.
export function simSwing(robot, gains, driveGains, { to, speed, opposite }) {
  const s = newState(0, robot.seed);
  const lag = robot.lagLoops;
  const pid = new PID(gains.kp, gains.ki ?? 0, gains.kd, gains.startI ?? 0);
  const right = new PID(driveGains.kp, 0, driveGains.kd);
  pid.target = to;
  right.target = 0;
  const exit = new Exit(gains.exit);

  let exitT = null, maxOver = 0;
  for (let ms = 0; ms < TIMEOUT_MS + POST_EXIT_MS; ms += 10) {
    const z = sense(s, lag);
    pid.compute(z.h);
    right.compute(z.r);
    const out = clamp(pid.output, -speed, speed);
    const other = opposite === 0 ? right.output : opposite * (out / speed);
    stepRobot(robot, s, Math.trunc(out), Math.trunc(other));
    maxOver = Math.max(maxOver, (s.heading - to) * sgn(to));
    if (exitT === null && ms >= 10) {
      const k = exit.check(pid, z.overL);
      if (k) exitT = { ms, kind: k };
    }
    if (exitT && ms >= exitT.ms + POST_EXIT_MS) break;
  }
  exitT ??= { ms: TIMEOUT_MS, kind: "timeout" };
  return { time: exitT.ms / 1000, kind: exitT.kind, error: s.heading - to, overshoot: maxOver, jitter: s.jitter };
}
