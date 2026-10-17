#pragma once

// =====================================================================================
// ROBOT SETTINGS: everything you might need to change at the robot, in one place.
// Change a number, build, upload. Lines marked TODO still need checking on the real robot.
// (Drive ports are in src/main.cpp, lift + drive PID gains in src/lift.cpp and src/autons.cpp)
// =====================================================================================

namespace cfg {

/////
// Claw
/////

// How the claw grabs: pick ONE
enum class ClawType {
  MOTOR,        // A smart motor opens/closes it
  PISTON,       // One piston opens/closes it
  TWO_PISTONS,  // Two pistons (e.g. one per side) that always move together
};
constexpr ClawType CLAW_TYPE = ClawType::PISTON;  // TODO: set to the real claw

// Set to true once the claw type and ports below match the robot (the pre-match check shows red until then)
constexpr bool CLAW_CONFIGURED = false;

// MOTOR claw
constexpr int CLAW_MOTOR_PORT = 2;  // TODO: smart port (negative reverses it, so + power closes)
constexpr int CLAW_POWER = 100;     // Out of 127, while opening/closing
constexpr int CLAW_HOLD = 20;       // Keeps squeezing while holding something (0 = off)

// PISTON / TWO_PISTONS claw
constexpr char CLAW_PISTON_PORT = 'A';      // TODO: 3-wire port A-H
constexpr char CLAW_PISTON_2_PORT = 'B';    // TODO: TWO_PISTONS only
constexpr bool CLAW_CLOSED_WHEN_ON = true;  // Flip if the claw opens when the piston fires

constexpr int CLAW_MOVE_MS = 250;  // How long opening/closing takes (autons wait this long)

// Optional extra claw pistons, toggled with LEFT (tilt) and RIGHT (flip) on the controller.
// Set to 0 if the robot doesn't have one. (If they're motors, tell the programmer.)
constexpr char TILT_PISTON_PORT = 0;  // TODO: e.g. 'C'
constexpr char FLIP_PISTON_PORT = 0;  // TODO: e.g. 'D'

// Motor that rotates the claw. Setup finds both of its ends by turning slowly until it stalls
constexpr int CLAW_ROTATE_PORT = 0;     // TODO: smart port (0 = the robot doesn't have one)
constexpr int ROTATE_CAL_POWER = 40;    // Out of 127, while finding the ends. Low so nothing breaks
constexpr double ROTATE_HOME_DEG = 10;  // Where it sits after setup: degrees from the first end it finds

/////
// Lift HARD STOP: keeps the claw's rotating gears off the field tiles
/////

// The lift can NEVER be driven below where it was when the program started: not by the driver,
// a macro, an auton, or setup. So before starting the program, set the lift by hand to
// 2-3" above the field (use the same block every time so it's always the same spot).
// To change the spot: restart the program with the lift at the new spot.
constexpr double LIFT_FLOOR_DEG = 0;       // Motor degrees above the start spot (0 = exactly there)
constexpr double LIFT_SLOW_ZONE_DEG = 60;  // Lowering slows down over this many degrees above the stop

/////
// Setup: calibrate + load the Preload. Runs when the program starts (if the robot is enabled),
// or tap CHECKS -> SETUP on the brain. Any controller button or stick cancels it.
//   1. Lift up slowly until it stops: that's the top (the lift won't be driven past it after)
//   2. Claw rotate motor: finds both ends, then goes home. Pistons: each one out and back
//   3. Lift down to the hard stop
//   4. Lift to the Preload spot, claw opens: put the Preload in. It closes by itself after
//      PRELOAD_CLOSE_MS, then lifts to height 1
// The field disables the robot before a match (nothing can move), so run setup BEFORE plugging
// into the field. Every match + both kinds of Skills get exactly one Preload Pin (SG5, RSC2).
/////

constexpr bool SETUP_AT_START = true;   // false = only run it from the SETUP button
constexpr int LIFT_CAL_POWER = 60;      // Out of 127, going up to find the top
constexpr double LIFT_TOP_MARGIN = 20;  // Motor degrees under the top where the lift stops after
constexpr int CAL_TIMEOUT_MS = 4000;    // Give up finding an end after this long
constexpr int PRELOAD_LOAD_HEIGHT = 0;  // Where the lift waits for the Preload: 0 = hard stop, 1-3 = saved heights
constexpr int PRELOAD_CLOSE_MS = 3000;  // Time to put the Preload in before the claw closes. HANDS OUT by then
constexpr int PRELOAD_HOLD_HEIGHT = 1;  // Where it holds the Preload until the match: 1 = height 1

/////
// Autons
/////

// Robot center to the middle of the claw's grip, in inches
constexpr double CLAW_REACH = 13;  // TODO: measure

// Lift level used to take a Pin off the top of a Cup (Cups are 6.5" tall). 1-3 = saved heights
constexpr int PICK_HEIGHT = 2;  // TODO: check

// Toggles: the robot backs its rear hook into the wall; each bump turns the Toggle one color
constexpr int TOGGLE_BUMPS = 1;  // TODO: 1 if one bump turns it yellow -> red, 2 if it goes to blue first
constexpr int BUMP_POWER = 50;   // Out of 127. TODO: enough to turn it without slamming
constexpr int BUMP_MS = 450;     // TODO: long enough to reach the wall and turn it

// Driver control: A+B backs into the Toggle behind the robot once, then pulls away this long
// (a Toggle only counts when no robot is touching it)
constexpr int TOGGLE_PULL_POWER = 50;  // Out of 127
constexpr int TOGGLE_PULL_MS = 300;

// Skills: time for the drive team to drop a Match Load into the Loader
constexpr int LOADER_WAIT_MS = 400;

}  // namespace cfg
