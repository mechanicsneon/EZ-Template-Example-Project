#include "setup.hpp"

#include <atomic>
#include <cstdlib>

#include "main.h"

namespace setup {
namespace {

constexpr int STICK_CANCEL = 30;  // Pushing a stick past this cancels setup

std::atomic<int> current{NOT_RUN};
std::atomic<bool> cancel{false};
std::atomic<const char *> detail{""};  // Why it failed, for the pre-match check

bool cancelled() { return cancel || pros::competition::is_disabled(); }

// Wait for a lift move, but stop right away if cancelled
bool lift_wait() {
  while (lift::moving()) {
    if (cancelled()) {
      lift::stop();
      return false;
    }
    pros::delay(ez::util::DELAY_TIME);
  }
  return !cancelled();
}

bool fail(const char *why) {
  detail = why;
  lift::stop();
  lift::show_message(why);
  master.rumble("---");
  logger::event(std::string("setup failed: ") + why);
  current = FAILED;
  return false;
}

void run() {
  logger::event("setup start");

  // 1. Top of the lift
  lift::show_message("SETUP: LIFT");
  if (!lift::find_top(cancelled)) {
    if (cancelled()) return;
    fail("LIFT TOP NOT FOUND");
    return;
  }
  if (!lift_wait()) return;

  // 2. Claw, while it's up off the field
  lift::show_message("SETUP: CLAW");
  if (!claw::calibrate_rotate(cancelled)) {
    if (cancelled()) return;
    fail("CLAW ROTATE STUCK");
    return;
  }
  claw::cycle_pistons(cancelled);
  if (cancelled()) return;

  // 3. Down to the hard stop
  lift::move_to_height(0);
  if (!lift_wait()) return;

  // 4. Preload: open at the loading spot, count down, close, lift to the holding height
  lift::move_to_height(cfg::PRELOAD_LOAD_HEIGHT);
  if (!lift_wait()) return;
  claw::open();
  master.rumble(".");
  uint32_t start = pros::millis();
  while (pros::millis() - start < (uint32_t)cfg::PRELOAD_CLOSE_MS) {
    if (cancelled()) return;
    int left = (cfg::PRELOAD_CLOSE_MS - (pros::millis() - start) + 999) / 1000;
    lift::show_message("PRELOAD IN " + std::to_string(left));
    pros::delay(100);
  }
  master.rumble("-");  // Closing
  claw::grab();
  lift::move_to_height(cfg::PRELOAD_HOLD_HEIGHT);
  if (!lift_wait()) return;
  if (cfg::PRELOAD_HOLD_HEIGHT > 0 && lift::heights_saved() < cfg::PRELOAD_HOLD_HEIGHT)
    lift::show_message("H" + std::to_string(cfg::PRELOAD_HOLD_HEIGHT) + " NOT SAVED");
  else
    lift::show_message("SETUP DONE");

  logger::event("setup done");
  current = DONE;
}

void task() {
  run();
  if (current == RUNNING) {  // Stopped early
    lift::stop();
    lift::show_message("SETUP CANCELLED");
    logger::event("setup cancelled");
    current = CANCELLED;
  }
}

}  // namespace

void start() {
  if (current == RUNNING || pros::competition::is_disabled()) return;
  cancel = false;
  current = RUNNING;
  pros::Task t(task, "Setup");
}

void initialize() {
  if (cfg::SETUP_AT_START) start();  // Plugged into a disabled field: nothing can move, so it waits for the button
}

void opcontrol() {
  if (current != RUNNING) return;
  bool any_button = false;
  for (int b = pros::E_CONTROLLER_DIGITAL_L1; b <= pros::E_CONTROLLER_DIGITAL_A; b++)
    any_button |= master.get_digital((pros::controller_digital_e_t)b);
  bool stick = std::abs(master.get_analog(ANALOG_LEFT_Y)) > STICK_CANCEL || std::abs(master.get_analog(ANALOG_RIGHT_Y)) > STICK_CANCEL ||
               std::abs(master.get_analog(ANALOG_LEFT_X)) > STICK_CANCEL || std::abs(master.get_analog(ANALOG_RIGHT_X)) > STICK_CANCEL;
  if (any_button || stick) cancel = true;
}

bool running() { return current == RUNNING && !cancel; }
State state() { return (State)current.load(); }

std::string status_text() {
  switch (current.load()) {
    case RUNNING: return "Setup running...";
    case DONE: return "Setup done, Preload in";
    case FAILED: return std::string("Setup failed: ") + detail.load();
    case CANCELLED: return "Setup cancelled: tap SETUP";
    default: return "Setup not run: tap SETUP";
  }
}

}  // namespace setup
