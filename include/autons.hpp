#pragma once

void default_constants();

// PID tests
void drive_example();
void turn_example();
void swing_example();

// Match autons (15 seconds). Blue = the red route turned 180 degrees
void red_left_toggle();
void red_bottom_toggle();
void red_left_two_goals();
void blue_right_toggle();
void blue_top_toggle();
void blue_right_two_goals();

// Backups: Toggle only, no claw needed
void red_left_toggle_only();
void red_bottom_toggle_only();
void blue_right_toggle_only();
void blue_top_toggle_only();

// Skills (60 seconds)
void skills_quick();
void skills_toggles();
