#pragma once

#include <string>

// Match logs on the SD card, to see what went wrong between matches.
// While the robot is enabled it writes a row every 0.1 s to log_NNN.csv on the SD card (a new file
// each time the program starts): time, mode, position, heading, lift, claw, battery, hottest
// drive motor, and any event. Open the file in Excel or Google Sheets. No SD card = no log.
namespace logger {

void initialize();                      // Call in initialize()
void event(const std::string &text);    // Something worth noting ("place GOAL_RED_LEFT")

}  // namespace logger
