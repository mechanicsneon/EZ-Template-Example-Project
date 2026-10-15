#include "assist.hpp"

#include <atomic>
#include <cstdlib>

#include "main.h"

namespace assist {
namespace {

std::atomic<bool> busy{false};
std::atomic<int> current{0};  // Bumps an assist's id; an older assist stops when it changes
constexpr int JOYSTICK_CANCEL = 20;

// Drive at `power` for `ms`. Stops early if a newer assist started, the driver took over, or the
// robot was disabled; returns false then (and the drive belongs to the driver again: hands off)
bool drive_for(int id, int power, int ms) {
  uint32_t start = pros::millis();
  while (pros::millis() - start < (uint32_t)ms) {
    if (current != id || pros::competition::is_disabled()) return false;
    chassis.drive_set(power, power);
    pros::delay(ez::util::DELAY_TIME);
  }
  return true;
}

}  // namespace

void bump_toggle() {
  if (busy) return;
  int id = ++current;
  busy = true;
  logger::event("driver: Toggle bump");
  pros::Task task([id] {
    if (drive_for(id, -cfg::BUMP_POWER, cfg::BUMP_MS) && drive_for(id, cfg::TOGGLE_PULL_POWER, cfg::TOGGLE_PULL_MS))
      chassis.drive_set(0, 0);
    if (current == id) busy = false;
  });
}

void opcontrol() {
  if (busy && (std::abs(master.get_analog(ANALOG_LEFT_Y)) > JOYSTICK_CANCEL ||
               std::abs(master.get_analog(ANALOG_RIGHT_Y)) > JOYSTICK_CANCEL)) {
    ++current;     // Stops the running assist
    busy = false;  // Joystick drive takes over this loop
  }
}

bool drive_busy() { return busy; }

}  // namespace assist
