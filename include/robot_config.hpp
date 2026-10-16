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
