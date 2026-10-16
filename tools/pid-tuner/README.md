# PID tuner (no robot needed)

Finds EZ-Template drive / heading / turn / swing gains by simulating the robot.

- `model.mjs`: a physics model of the drivetrain (V5 motor torque curve and current limit,
  450 rpm on 3.25" wheels, friction, wheel scrub, sensor delay and noise), plus exact copies of
  EZ-Template 3.2.2's PID, slew, exit conditions and drive/turn/swing loops, quirks included.
- `tune.mjs`: runs every candidate set of gains through drives, turns and swings on 18 simulated
  robots (weights 10-18 lb, low to full battery, grippy to slippery...), searches for the set that
  finishes fastest and most accurately on all of them, then checks it on 42 different robots.

```
node tools/pid-tuner/tune.mjs
```

Takes about a minute and prints a before/after table plus lines to paste into
`default_constants()` in `src/autons.cpp`.

## Make it sharper

The ranges at the top of `tune.mjs` (`RANGES`) are guesses that cover most VRC robots. Every one
you measure and narrow makes the result fit your robot better:

| Range | How to measure |
|---|---|
| `massLb` | Weigh the robot |
| `trackIn` | Center of left wheels to center of right wheels |
| `batteryV` | Leave as is (batteries vary through a match) |
| `rollFriction` | Smallest drive power that makes the robot start rolling, divided by ~127 |
| `scrub` | Smallest turn power that makes it start turning in place (~20 power = 0.3) |

## What it can't know

Things the model leaves out: chassis flex, gear slop, a bent axle rubbing, tile seams. Expect to
nudge the gains a little on the real robot. The EZ PID tuner (press X in driver control, not on a
competition switch) is the quickest way.

Known EZ-Template quirk the model copies: turn integral resets every loop when turning *toward*
0 deg from the other side (it compares the error's sign with the heading's), so those turns rely on
P alone and can stop a few degrees short on a robot with lots of wheel scrub.

## Lift

```
node tools/pid-tuner/lift-tune.mjs
```

Tunes `lift_pid` in `src/lift.cpp` across red/green cartridges, any gear ratio and lift weight.
The old guess (0.5, 0, 2) stalled short of the target on about 2 out of 3 simulated lifts; the
tuned gains reach it on all of them in about 1 second. Overshoot stays under 7 deg on 90% of moves;
the big ones only happen on very heavy lifts with no rubber bands (needing 40%+ power just to
hold still). Rubber bands help the PID a lot.
