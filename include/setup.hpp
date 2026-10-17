#pragma once

#include <string>

// Start-up setup: calibrate the lift + claw, then load the Preload. Steps and settings are in
// robot_config.hpp. Runs when the program starts (if the robot is enabled), or from the
// SETUP button on the brain's pre-match screen. Any controller button or stick cancels it.
namespace setup {

enum State { NOT_RUN, RUNNING, DONE, FAILED, CANCELLED };

void initialize();  // Call at the end of initialize()
void start();       // Run it now (the SETUP button). Does nothing if it's running or the robot is disabled
void opcontrol();   // Call every loop in opcontrol(): watches for the cancel

bool running();      // True while setup moves things: opcontrol must leave the lift + claw alone
State state();
std::string status_text();  // Short, for the pre-match check

}  // namespace setup
