#include <cmath>

#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  // Drive/heading/turn/swing gains come from tools/pid-tuner (simulated across a range of robot
  // weights, batteries and friction). Re-run it after measuring the robot to sharpen them.
  chassis.pid_drive_constants_set(20.7, 0.0, 190.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(10.5, 0.0, 0.25);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(5.0, 0.127, 36.0, 21.5);    // Turn in place constants
  chassis.pid_turn_min_set(58);                              // Max turn speed once within Start I (EZ default 30 can stall)
  chassis.pid_swing_constants_set(20.0, 0.0, 220.0);         // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

/////
// PID tests: simple moves for checking the PID gains on the real robot
/////

void drive_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void turn_example() {
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

void swing_example() {
  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

/////
// V5RC Override field map (Game Manual v2.1, Appendix A), in inches
//
// (0, 0) is the center of the field. The red Alliance Station is on the left (x = -70.2) and
// the audience is at the bottom. Headings: 0 = facing +y (away from the audience), 90 = facing +x.
// Red's side of the Autonomous Line (the double diagonal from the top-left corner to the
// bottom-right corner) is the left and bottom Quadrants. Blue's side is red's turned 180
// degrees, so blue routes run the red route with every point turned 180 degrees.
//
// Every route below was checked against the v2.1 rules: legal start (SG1), never past the
// Autonomous Line (SG7), never touching blue or neutral Goals (SG9/SG10), nothing hit, and
// match routes end off the wall (Autonomous Win Point). The waypoints thread between field
// objects with very little room, so re-check the whole path if you change one.
/////

namespace {

struct Pt {
  double x, y;
};
struct Pose {
  Pt p;
  double heading;
};

// Goals and the lift level to place at: 1 = Alliance Goal (3.25"), 2 = neutral (5.8"), 3 = center (8.8")
struct Goal {
  Pt p;
  int height;
};
const Goal GOAL_RED_LEFT = {{-47.1, -23.55}, 1};
const Goal GOAL_RED_BOTTOM = {{-23.55, -47.1}, 1};
const Goal GOAL_NEUTRAL_LEFT = {{-47.1, 23.55}, 2};  // Skills only: in matches it starts with a Pin in it
const Goal GOAL_CENTER = {{0, 0}, 3};                // Skills only: in matches it starts with a Pin in it

// Upright Pins sitting on a Cup. We take the Pin; the Cup stays
const Pt MID_WEST = {-23.55, 0};             // Red half up. On the Autonomous Line, so either alliance can take it
const Pt MID_SOUTH = {0, -23.55};            // Red half up. On the Autonomous Line
const Pt STACK_RED_INNER = {-23.55, -23.55};  // Yellow/yellow, on our side

// The rest of the field, for adding new routes. Blue side: skills only
const Goal GOAL_NEUTRAL_BOTTOM = {{23.55, -47.1}, 2};
[[maybe_unused]] const Goal GOAL_BLUE_TOP = {{23.55, 47.1}, 1};
[[maybe_unused]] const Goal GOAL_BLUE_RIGHT = {{47.1, 23.55}, 1};
[[maybe_unused]] const Goal GOAL_NEUTRAL_TOP = {{-23.55, 47.1}, 2};
[[maybe_unused]] const Goal GOAL_NEUTRAL_RIGHT = {{47.1, -23.55}, 2};
[[maybe_unused]] const Pt MID_NORTH = {0, 23.55};  // Blue half up
[[maybe_unused]] const Pt MID_EAST = {23.55, 0};   // Blue half up
const Pt STACK_RED_OUTER = {-47.1, -47.1};
[[maybe_unused]] const Pt STACK_BLUE_INNER = {23.55, 23.55};
[[maybe_unused]] const Pt STACK_BLUE_OUTER = {47.1, 47.1};
[[maybe_unused]] const Pt LOADER_RED_BOTTOM = {-66.5, -58.8};  // Skills: Match Loads come out the bottom
[[maybe_unused]] const Pt LOADER_RED_TOP = {-66.5, 58.8};

// Yellow/yellow Pins on the middle Cup of each wall group, either side of the Toggle
const Pt WALL_BOTTOM_LEFT = {-23.55, -68.6};
const Pt WALL_BOTTOM_RIGHT = {23.55, -68.6};
[[maybe_unused]] const Pt WALL_LEFT_LOW = {-68.6, -23.55};
[[maybe_unused]] const Pt WALL_LEFT_HIGH = {-68.6, 23.55};

// Toggles. Our robot sets one by backing the hook on its back into the wall from `spot`.
// Each bump turns the Toggle one color. A set Toggle makes every visible yellow Pin half in
// its Quadrant count for us (10 each), including the yellow Pin already in the neutral Goal
struct ToggleSpot {
  Pt spot;
  double heading;  // Facing away from the wall
};
const ToggleSpot TOGGLE_LEFT = {{-55.2, 0}, 90};
const ToggleSpot TOGGLE_BOTTOM = {{0, -55.2}, 0};
constexpr double WALL_GAP = 61.2;    // Field center to robot center when its back is flat on a wall

// Legal starts (SG1): against the wall, centered in front of the Toggle, between the wall Cups.
// The two red robots must be in different Quadrants, so agree with your partner who takes which
const Pose START_LEFT = {{-61.2, 0}, 90};
const Pose START_BOTTOM = {{0, -61.2}, 0};

// Claw reach, pick height, Toggle bumps etc. live in include/robot_config.hpp
using cfg::BUMP_MS;
using cfg::BUMP_POWER;
using cfg::CLAW_REACH;
using cfg::LOADER_WAIT_MS;
using cfg::PICK_HEIGHT;
using cfg::TOGGLE_BUMPS;

/////
// Route helpers
/////

bool blue = false;  // Blue routes: every point turned 180 degrees about the center

Pt side(Pt p) { return blue ? Pt{-p.x, -p.y} : p; }

void start_at(Pose s) {
  claw::grab();  // Grip the Preload: the claw starts open when the program starts
  Pt p = side(s.p);
  chassis.odom_enable(true);
  chassis.odom_xyt_set(p.x, p.y, blue ? s.heading + 180 : s.heading);
}

void drive_forward(double inches) {
  chassis.pid_drive_set(inches, DRIVE_SPEED);
  chassis.pid_wait();
}

void back(double inches) {
  chassis.pid_drive_set(-inches, DRIVE_SPEED);
  chassis.pid_wait();
}

// Face a field spot, then drive straight until the robot is `stop_short` inches from it
void face_and_drive(Pt target, double stop_short) {
  target = side(target);
  chassis.pid_turn_set(ez::pose{target.x, target.y}, ez::fwd, TURN_SPEED);
  chassis.pid_wait();

  double dist = std::hypot(target.x - chassis.odom_x_get(), target.y - chassis.odom_y_get()) - stop_short;
  if (dist < 0.5) return;
  chassis.pid_drive_set(dist, DRIVE_SPEED, dist > 12);
  chassis.pid_wait();
}

void drive_to(Pt spot) { face_and_drive(spot, 0); }

// Place the Pin in the claw on a Goal, then back straight off
void place(Goal goal, double back_off) {
  logger::event("auton: place at (" + std::to_string((int)goal.p.x) + ", " + std::to_string((int)goal.p.y) + ")");
  lift::move_to_height(goal.height);  // Rises while the robot drives
  face_and_drive(goal.p, CLAW_REACH);
  lift::wait();
  claw::release();
  back(back_off);
  lift::move_to_height(0);
}

// Take the Pin off the top of a Cup, then back straight off
void pick(Pt spot, double back_off) {
  logger::event("auton: pick at (" + std::to_string((int)spot.x) + ", " + std::to_string((int)spot.y) + ")");
  lift::move_to_height(PICK_HEIGHT);
  face_and_drive(spot, CLAW_REACH);
  lift::wait();
  claw::grab();
  back(back_off);
}

// Set a Toggle: drive to the spot in front of it, turn the back to the wall, and bump the hook
// into it once per color step. Flat against the wall we know exactly where the robot is, so
// odometry gets corrected there too. Then pull away: a Toggle only counts when nothing touches it
void set_toggle(ToggleSpot t) {
  logger::event("auton: Toggle bump");
  drive_to(t.spot);
  double heading = blue ? t.heading + 180 : t.heading;
  chassis.pid_turn_set(heading, TURN_SPEED);
  chassis.pid_wait();

  double to_wall = WALL_GAP - std::max(std::fabs(t.spot.x), std::fabs(t.spot.y));
  for (int i = 0; i < TOGGLE_BUMPS; i++) {
    chassis.drive_set(-BUMP_POWER, -BUMP_POWER);
    pros::delay(BUMP_MS);
    chassis.drive_set(0, 0);

    Pt here = side({chassis.odom_x_get(), chassis.odom_y_get()});  // Into the red-side frame
    if (t.spot.x != 0)
      here.x = t.spot.x < 0 ? -WALL_GAP : WALL_GAP;
    else
      here.y = t.spot.y < 0 ? -WALL_GAP : WALL_GAP;
    Pt field = side(here);
    chassis.odom_xyt_set(field.x, field.y, heading);

    drive_forward(to_wall);
  }
}

// Skills only: take a Match Load from the bottom of a red Loader, then back straight off
[[maybe_unused]] void load(Pt loader, double back_off) {
  lift::move_to_height(0);  // TODO: the Loader opening may need a lift height
  face_and_drive(loader, CLAW_REACH);
  lift::wait();
  pros::delay(LOADER_WAIT_MS);
  claw::grab();
  back(back_off);
}

/////
// Match autons (15 seconds). Each sets our Quadrant's Toggle to red first: that makes the yellow
// Pin already sitting in that Quadrant's neutral Goal count for us (+10), and the Toggle bump
// re-zeros odometry against the wall. Then they score the Preload in our own Alliance Goal.
// Preload in the claw red half up = 5 (safe even if the Toggle gets flipped later), or yellow
// half up = 10 while our Toggle stays red.
// The Autonomous Win Point needs 6 scored Pin Halves in 2+ Goals from the whole alliance, which
// takes stacking with Cups: plan it with your partner.
/////

// Left wall start: left Toggle, Preload into the left red Goal. About 4 seconds, 15-20 points
void left_toggle() {
  start_at(START_LEFT);
  drive_forward(4);
  set_toggle(TOGGLE_LEFT);
  drive_to({-52.6, -8.5});
  place(GOAL_RED_LEFT, 4);
}

// Bottom wall start: bottom Toggle, Preload into the bottom red Goal. About 4 seconds, 15-20 points
void bottom_toggle() {
  start_at(START_BOTTOM);
  drive_forward(4);
  set_toggle(TOGGLE_BOTTOM);
  drive_to({-8.5, -52.6});
  place(GOAL_RED_BOTTOM, 4);
}

// Left wall start: left Toggle, Preload into the left red Goal, then the red-up Midfield Pin into
// the bottom red Goal. Uses both red Goals, so your partner shouldn't. About 11 seconds, 20-25 points
void left_two_goals() {
  start_at(START_LEFT);
  drive_forward(4);
  set_toggle(TOGGLE_LEFT);
  drive_to({-52.6, -8.5});
  place(GOAL_RED_LEFT, 4);
  drive_to({-38, -3.9});
  pick(MID_WEST, 4);
  drive_to({-36, -8});
  drive_to({-33.8, -34.8});
  place(GOAL_RED_BOTTOM, 4);
}

// BACKUPS, no claw needed: just set our Toggle and end off the wall. The yellow Pin already in
// our Quadrant's neutral Goal then counts for us (10 points). About 2 seconds
void left_toggle_only() {
  start_at(START_LEFT);
  drive_forward(4);
  set_toggle(TOGGLE_LEFT);
}

void bottom_toggle_only() {
  start_at(START_BOTTOM);
  drive_forward(4);
  set_toggle(TOGGLE_BOTTOM);
}

// Bottom wall start: bottom Toggle, Preload into the bottom red Goal, then the red-up Pin from
// the Midfield's south point into the left red Goal. Uses both red Goals, so your partner
// shouldn't. About 11 seconds, 20-25 points
void bottom_two_goals() {
  start_at(START_BOTTOM);
  drive_forward(4);
  set_toggle(TOGGLE_BOTTOM);
  drive_to({-8.5, -52.6});
  place(GOAL_RED_BOTTOM, 4);
  drive_to({-3.9, -38});
  pick(MID_SOUTH, 4);
  drive_to({-8, -36});
  drive_to({-34.8, -33.8});
  place(GOAL_RED_LEFT, 4);
}

}  // namespace

void red_bottom_two_goals() {
  blue = false;
  bottom_two_goals();
}
void blue_top_two_goals() {
  blue = true;
  bottom_two_goals();
}

void red_left_toggle_only() {
  blue = false;
  left_toggle_only();
}
void red_bottom_toggle_only() {
  blue = false;
  bottom_toggle_only();
}
void blue_right_toggle_only() {
  blue = true;
  left_toggle_only();
}
void blue_top_toggle_only() {
  blue = true;
  bottom_toggle_only();
}

void red_left_toggle() {
  blue = false;
  left_toggle();
}
void red_bottom_toggle() {
  blue = false;
  bottom_toggle();
}
void red_left_two_goals() {
  blue = false;
  left_two_goals();
}
void blue_right_toggle() {
  blue = true;
  left_toggle();
}
void blue_top_toggle() {
  blue = true;
  bottom_toggle();
}
void blue_right_two_goals() {
  blue = true;
  left_two_goals();
}

/////
// Skills autons (60 seconds, RSC2-RSC4): always start in the left Quadrant next to the red
// Alliance Station, and all Goals start empty. With our Toggles red, every yellow Pin half we
// Place in a red Quadrant scores 10, and a yellow half in the center Goal scores 10 if we end in
// the Midfield (8 more). Load the Preload yellow half up.
// Times are estimates from the simulation; the real robot will be slower, so they leave room.
/////

// Left Toggle, 3 Pins, park. 33 points, about 18 s
void skills_quick() {
  blue = false;
  start_at(START_LEFT);
  drive_forward(4);
  set_toggle(TOGGLE_LEFT);
  drive_to({-52.6, -8.5});
  place(GOAL_RED_LEFT, 4);
  drive_to({-34.2, -12.9});
  pick(STACK_RED_INNER, 4);
  drive_to({-41.6, 8.5});
  place(GOAL_NEUTRAL_LEFT, 4);
  drive_to({-37.1, 6.3});
  pick(MID_WEST, 4);
  drive_to({-34, -12});
  drive_to({-12.3, -10.3});
  place(GOAL_CENTER, 4);
  drive_to({-13, -8});  // Park in the Midfield
}

// Both red Toggles, yellow Pins in all 5 red-side Goals, park. 58 points, about 40 s
void skills_toggles() {
  blue = false;
  start_at(START_LEFT);
  drive_forward(4);
  set_toggle(TOGGLE_LEFT);
  drive_to({-52.6, -8.5});
  place(GOAL_RED_LEFT, 4);
  drive_to({-34.2, -12.9});
  pick(STACK_RED_INNER, 4);
  drive_to({-41.6, 8.5});
  place(GOAL_NEUTRAL_LEFT, 4);
  drive_to({-34, -10});
  drive_to({-14, -12});
  drive_to({-8, -36});
  set_toggle(TOGGLE_BOTTOM);
  drive_to({-10.5, -57.7});
  pick(WALL_BOTTOM_LEFT, 4);
  drive_to({-10.4, -56.3});
  place(GOAL_RED_BOTTOM, 4);
  drive_to({10.5, -57.7});
  pick(WALL_BOTTOM_RIGHT, 4);
  drive_to({10.4, -56.3});
  place(GOAL_NEUTRAL_BOTTOM, 4);
  drive_to({-8, -36});
  drive_to({-34, -34});
  drive_to({-36.5, -36.5});
  pick(STACK_RED_OUTER, 4);
  drive_to({-34, -12});
  drive_to({-12.3, -10.3});
  place(GOAL_CENTER, 4);
  drive_to({-13, -8});  // Park in the Midfield
}
