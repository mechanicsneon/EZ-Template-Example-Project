#pragma once

// Driver assists: the robot does a precise move for the driver on a button press.
// Driving with the sticks always cancels an assist.
namespace assist {

// A+B: back the rear hook into the Toggle behind the robot once, then pull away. Line the robot
// up first, back facing the wall in front of the Toggle. Press again for another color step.
void bump_toggle();

void opcontrol();    // Call every loop in opcontrol() (cancels an assist when the sticks move)
bool drive_busy();   // True while an assist is driving: opcontrol() must not run joystick drive

}  // namespace assist
