#pragma once

// Claw that grabs Pins / Cups, with optional tilt and flip pistons.
// Set the claw type and ports in robot_config.hpp.
//
// Driver controls (see claw::opcontrol):
//   B      grab / let go          (A+B = Toggle bump, see assist.hpp)
//   LEFT   tilt (if the robot has a tilt piston)
//   RIGHT  flip (if the robot has a flip piston)
namespace claw {

void initialize();  // Call in initialize()
void opcontrol();   // Call every loop in opcontrol()

// For autons and macros: these wait until the claw has finished moving
void grab();
void release();

bool holding();  // True while the claw is closed

}  // namespace claw
