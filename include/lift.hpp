#pragma once

#include <string>

// DR4B lift with saved heights and a one-button "place" macro.
//
// Driver controls (see lift::opcontrol):
//   L1 / L2    manual up / down (always wins, cancels any macro)
//   R1         PLACE: lift to the next height, release, back up, lower. Next press goes one height higher
//   A + R1     TEACH: save where the lift is right now as the next height (stored on the SD card)
//   R2         cancel + lower the lift to the bottom
//   A + R2     the lift is at the bottom right now: reset it to 0
//   UP / DOWN  change which height is next
//   Y          go back to height 1
//   A + Y      lift to the height for taking a Pin off a Cup (PICK_HEIGHT in robot_config.hpp)
//
// The lift counts 0 as wherever it is when the program starts, so start with it all the way down
// (or lower it by hand and press A + R2).
namespace lift {

// Saved heights match the three Goal heights in Override (autons use these too):
//   H1 = Alliance Goal (3.25" tall), H2 = neutral Goal (5.8"), H3 = center Goal (8.8")
// so save H1 at the right height for a red Goal, H2 for a black Quadrant Goal, H3 for the center
constexpr int HEIGHT_COUNT = 3;

void initialize();  // Call in initialize()
void opcontrol();   // Call every loop in opcontrol()

// True while a macro is driving the robot. opcontrol() must not run joystick drive then.
bool drive_busy();

// For autons: move to a saved height (1..HEIGHT_COUNT) or 0 for the bottom, and wait for it
void move_to_height(int height);
void wait();

// Short text for the HUD, like "NEXT H2" or "SAVED H2 640"
std::string status_text();
bool status_is_new();  // True for a second or so after something changes, so the HUD shows it right away
int next_height();     // 1..HEIGHT_COUNT
int heights_saved();   // How many of the heights have been saved (0..HEIGHT_COUNT)
double position();     // Motor degrees, 0 = bottom

}  // namespace lift
