#pragma once

// Driver HUD: a status readout on the controller screen and a dashboard on the brain.
//  - Controller: match timer + battery, warnings or selected auton, hottest motor temps
//  - Brain: battery, controller battery, IMU, every motor's temp, selected auton, warnings
// Tap "PICK AUTON" on the brain to get to EZ-Template's auton selector, and "HUD" to come back.
namespace hud {

// Call once at the end of initialize(), after ez::as::initialize()
void initialize();

// Call at the top of opcontrol() so the 1:45 countdown starts exactly with driver control
void driver_timer_start();

}  // namespace hud
