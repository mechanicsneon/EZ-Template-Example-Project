#pragma once

#include <string>

// Laptop link: talk to a laptop over USB (a cable to the brain, or to the controller, which passes
// it on wirelessly). The brain sends its status 10 times a second as one line of JSON starting with
// '@', and takes one text command per line:
//   hello                  list the autons + which one is picked
//   select N               pick auton N (same as tapping it on the brain)
//   stop                   stop whatever the laptop started, right now
//   setup                  run setup (lift + claw calibration, Preload)
//   lift N                 lift to saved height N (0 = the hard stop)
//   claw open | claw close
//   bump                   Toggle bump (same as A+B)
//   zero X Y H             tell odometry where the robot is (inches, degrees)
//   auton                  run the picked auton
//   route STEP;STEP;...    run a list of auton steps (see route_step() in autons.cpp)
// Anything that moves the robot only works in the pits: never while it's plugged into a field or
// competition switch. Moving a stick on the controller cancels whatever the laptop started.
namespace laptop {

void initialize();                    // Call in initialize()
void opcontrol();                     // Call every loop in opcontrol()
bool drive_busy();                    // True while the laptop is driving: opcontrol must not run joystick drive
void event(const std::string &text);  // Pass a log event on to the laptop

}  // namespace laptop
