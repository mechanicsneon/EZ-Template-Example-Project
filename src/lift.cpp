#include "lift.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <mutex>

#include "main.h"

namespace lift {
namespace {

/////
// Settings, tune these on the robot
/////

// Lift PID in motor degrees, from tools/pid-tuner/lift-tune.mjs. The small I is what lets it
// finish against gravity instead of stalling short. Raise kD if it overshoots and bounces
ez::PID lift_pid{0.88, 0.0099, 7.5, 212.0, "Lift"};
constexpr double SETTLED_DEG = 15;     // Close enough to count as "there"
constexpr int SETTLED_MS = 100;        // ...for this long, then the motors lock in place
constexpr int MOVE_TIMEOUT_MS = 2500;  // Give up on a lift move after this long (something's stuck)
constexpr double MIN_HEIGHT_DEG = 20;  // Refuse to save a height this close to the bottom
constexpr double BACKUP_IN = 6;        // How far to back up after placing
constexpr int BACKUP_SPEED = 80;       // Out of 127
constexpr int BACKUP_TIMEOUT_MS = 1500;
constexpr int JOYSTICK_CANCEL = 20;  // Pushing a stick past this during the back-up cancels it
constexpr int MESSAGE_MS = 1500;     // How long the HUD shows a new message
const char *HEIGHTS_FILE = "/usd/lift_heights.txt";

/////
// State
/////

enum Mode { MANUAL, HOLD, TARGET };
std::atomic<int> mode{HOLD};
std::atomic<double> target{0};

double heights[HEIGHT_COUNT] = {};  // Motor degrees, 0 = not saved yet
std::atomic<int> next{0};           // Index into heights

std::atomic<bool> macro_running{false};
std::atomic<bool> cancel{false};
std::atomic<bool> busy_drive{false};
std::atomic<int> placing{0};

pros::Mutex message_mutex;
std::string message;
uint32_t message_ms = 0;

void show(std::string text) {
  std::lock_guard<pros::Mutex> lock(message_mutex);
  message = text;
  message_ms = pros::millis();
}

double read_position() {
  // Average the lift motors that are plugged in
  double sum = 0;
  int count = 0;
  for (double p : arm.get_position_all()) {
    if (std::isfinite(p)) {
      sum += p;
      count++;
    }
  }
  return count ? sum / count : 0;
}

/////
// Lift motion
/////

void go_to(double deg) {
  target = deg;
  mode = TARGET;
}

// Runs all the time: drives the lift to its target, then locks the motors there.
// This is the only place that powers the lift outside of manual control, so nothing
// else can leave the motors running.
void lift_task() {
  double active_target = NAN;
  uint32_t close_since = 0;
  uint32_t move_start = 0;

  while (true) {
    // Disabled by the field: drop any move or macro so nothing starts up on its own
    // when the robot is enabled again (PROS doesn't stop our tasks when disabled)
    if (pros::competition::is_disabled()) {
      if (macro_running) cancel = true;
      busy_drive = false;
      if (mode == TARGET) mode = HOLD;
    }

    if (mode == TARGET) {
      double t = target;
      double pos = read_position();
      if (t != active_target) {
        // Sync the PID's last position first so the lift moving by hand since the
        // last PID move doesn't look like a sudden jump (that would kick the D term)
        lift_pid.compute(pos);
        lift_pid.variables_reset();
        lift_pid.target_set(t);
        active_target = t;
        close_since = 0;
        move_start = pros::millis();
      }

      arm.move(std::clamp(lift_pid.compute(pos), -127.0, 127.0));

      bool settled = false;
      if (std::fabs(t - pos) < SETTLED_DEG) {
        if (!close_since) close_since = pros::millis();
        settled = pros::millis() - close_since >= SETTLED_MS;
      } else {
        close_since = 0;
      }
      bool stuck = pros::millis() - move_start > MOVE_TIMEOUT_MS;
      if (settled || stuck) {
        arm.brake();  // HOLD brake keeps it here
        mode = HOLD;
        active_target = NAN;
        if (stuck) show("LIFT STUCK?");
      }
    } else {
      active_target = NAN;
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}

// Wait for the lift to get there (or give up). Returns false if the driver cancelled
bool wait_for_lift() {
  while (mode == TARGET) {
    if (cancel) return false;
    pros::delay(ez::util::DELAY_TIME);
  }
  return !cancel;
}

/////
// Place macro
/////

// Back straight up. Returns false if the driver took over
bool back_up() {
  busy_drive = true;
  double start_l = chassis.drive_sensor_left();
  double start_r = chassis.drive_sensor_right();
  chassis.pid_drive_set(-BACKUP_IN, BACKUP_SPEED, false, false);  // No heading hold, the robot has turned since auton

  uint32_t start = pros::millis();
  while (!cancel) {
    double moved = (std::fabs(chassis.drive_sensor_left() - start_l) + std::fabs(chassis.drive_sensor_right() - start_r)) / 2;
    if (moved >= BACKUP_IN - 0.5 || pros::millis() - start > BACKUP_TIMEOUT_MS) break;
    pros::delay(ez::util::DELAY_TIME);
  }
  if (cancel) return false;  // opcontrol already has the drive back

  chassis.drive_set(0, 0);  // Stop the PID and hand the drive back to the joysticks
  busy_drive = false;
  return true;
}

void place() {
  int h = next;
  placing = h + 1;

  if (heights[h] <= 0) {
    show("H" + std::to_string(h + 1) + " NOT SAVED");
    macro_running = false;
    return;
  }

  // Raise -> release -> back up -> lower. Stops wherever the driver takes over
  logger::event("place macro H" + std::to_string(h + 1));
  go_to(heights[h]);
  if (wait_for_lift()) {
    claw::release();
    if (!cancel) {
      next = std::min(h + 1, HEIGHT_COUNT - 1);  // Placed it, so the next press goes one higher
      if (back_up() && !cancel) go_to(0);
    }
  }

  busy_drive = false;
  macro_running = false;
}

void stop_macro() {
  if (!macro_running) return;
  logger::event("place macro cancelled");
  cancel = true;
  busy_drive = false;  // opcontrol drives again starting this loop
}

/////
// Saved heights
/////

void save_heights() {
  if (!pros::usd::is_installed()) return;
  FILE *f = fopen(HEIGHTS_FILE, "w");
  if (!f) return;
  for (double h : heights) fprintf(f, "%.1f\n", h);
  fclose(f);
}

void load_heights() {
  if (!pros::usd::is_installed()) return;
  FILE *f = fopen(HEIGHTS_FILE, "r");
  if (!f) return;
  for (double &h : heights)
    if (fscanf(f, "%lf", &h) != 1) break;
  fclose(f);
}

void teach() {
  int h = next;
  double pos = read_position();
  if (pos < MIN_HEIGHT_DEG) {
    show("RAISE LIFT 1ST");
    return;
  }
  heights[h] = pos;
  save_heights();
  show("SAVED H" + std::to_string(h + 1) + (pros::usd::is_installed() ? " " + std::to_string((int)pos) : " NO SD"));
  next = std::min(h + 1, HEIGHT_COUNT - 1);
}

void pick_next(int index) {
  next = std::clamp(index, 0, HEIGHT_COUNT - 1);
  show("NEXT H" + std::to_string(next + 1) + (heights[next] > 0 ? "" : " UNSET"));
}

}  // namespace

void initialize() {
  arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);  // Lock in place instead of falling under its own weight
  arm.tare_position_all();                           // Wherever the lift is now counts as the bottom
  load_heights();
  pros::Task task(lift_task, "Lift");
}

void opcontrol() {
  // Manual always wins
  bool up = master.get_digital(DIGITAL_L1);
  bool down = master.get_digital(DIGITAL_L2);
  if (up || down) {
    stop_macro();
    mode = MANUAL;
    arm.move(up ? 127 : -127);
  } else if (mode == MANUAL) {
    arm.brake();
    mode = HOLD;
  }

  // Driving during the back-up cancels it
  if (busy_drive && (std::abs(master.get_analog(ANALOG_LEFT_Y)) > JOYSTICK_CANCEL ||
                     std::abs(master.get_analog(ANALOG_RIGHT_Y)) > JOYSTICK_CANCEL))
    stop_macro();

  // The PID tuner uses the arrows, A and Y
  if (chassis.pid_tuner_enabled()) return;

  if (master.get_digital_new_press(DIGITAL_R1)) {
    if (master.get_digital(DIGITAL_A)) {
      teach();
    } else if (!macro_running) {
      cancel = false;
      macro_running = true;
      pros::Task place_task(place, "Place");
    }
  }

  if (master.get_digital_new_press(DIGITAL_R2)) {
    stop_macro();
    if (master.get_digital(DIGITAL_A)) {
      // The lift is sitting at the bottom: count this as 0 again (fixes saved heights if the
      // program was started with the lift up)
      mode = HOLD;
      arm.brake();
      arm.tare_position_all();
      show("BOTTOM RESET");
    } else {
      go_to(0);
      show("LIFT DOWN");
    }
  }

  if (master.get_digital_new_press(DIGITAL_UP)) pick_next(next + 1);
  if (master.get_digital_new_press(DIGITAL_DOWN) && !master.get_digital(DIGITAL_B)) pick_next(next - 1);  // B+DOWN runs auton
  if (master.get_digital_new_press(DIGITAL_Y)) {
    if (master.get_digital(DIGITAL_A)) {
      move_to_height(cfg::PICK_HEIGHT);  // A+Y: the height for taking a Pin off a Cup
      show("PICK HEIGHT");
    } else {
      pick_next(0);
    }
  }
}

bool drive_busy() { return busy_drive; }

void move_to_height(int height) {
  if (height <= 0)
    go_to(0);
  else if (height <= HEIGHT_COUNT && heights[height - 1] > 0)
    go_to(heights[height - 1]);
}

void wait() {
  while (mode == TARGET) pros::delay(ez::util::DELAY_TIME);  // lift_task gives up after MOVE_TIMEOUT_MS
}

std::string status_text() {
  {
    std::lock_guard<pros::Mutex> lock(message_mutex);
    if (!message.empty() && pros::millis() - message_ms < MESSAGE_MS) return message;
  }
  if (macro_running) return "PLACING H" + std::to_string(placing);
  return "NEXT H" + std::to_string(next + 1) + (heights[next] > 0 ? "" : " UNSET");
}

bool status_is_new() {
  std::lock_guard<pros::Mutex> lock(message_mutex);
  return !message.empty() && pros::millis() - message_ms < MESSAGE_MS;
}

int next_height() { return next + 1; }

int heights_saved() {
  int n = 0;
  for (double h : heights) n += h > 0;
  return n;
}
double position() { return read_position(); }

}  // namespace lift
