#include "logger.hpp"

// Logs need the real brain's SD card. VEX Sim builds this project for the PC, where there is
// none, so it gets empty stand-ins (same as the HUD).
#ifndef __arm__
namespace logger {
void initialize() {}
void event(const std::string &) {}
}  // namespace logger
#else

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <vector>

#include "main.h"

namespace logger {
namespace {

FILE *file = nullptr;
pros::Mutex events_mutex;
std::vector<std::string> pending;  // Events waiting for the next row

const char *mode_name() {
  if (pros::competition::is_disabled()) return "disabled";
  if (pros::competition::is_autonomous()) return "auton";
  return "driver";
}

// Runs in the background so SD card writes never slow down driving or autons
void log_task() {
  uint32_t last_flush = pros::millis();
  while (true) {
    std::vector<std::string> events;
    {
      std::lock_guard<pros::Mutex> lock(events_mutex);
      events.swap(pending);
    }
    if (!pros::competition::is_disabled() || !events.empty()) {
      double hottest = 0;
      for (auto &m : chassis.left_motors) hottest = std::max(hottest, std::isfinite(m.get_temperature()) ? m.get_temperature() : 0.0);
      for (auto &m : chassis.right_motors) hottest = std::max(hottest, std::isfinite(m.get_temperature()) ? m.get_temperature() : 0.0);
      std::string event;
      for (auto &e : events) event += (event.empty() ? "" : " | ") + e;
      fprintf(file, "%u,%s,%.1f,%.1f,%.1f,%.0f,%s,%.0f,%.0f,\"%s\"\n", (unsigned)pros::millis(), mode_name(),
              chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get(), lift::position(),
              claw::holding() ? "closed" : "open", pros::battery::get_capacity(), hottest, event.c_str());
    }
    if (pros::millis() - last_flush > 1000) {
      fflush(file);  // At most a second lost if the power cuts
      last_flush = pros::millis();
    }
    pros::delay(100);
  }
}

}  // namespace

void initialize() {
  if (!pros::usd::is_installed()) return;
  // Every run gets its own log_NNN.csv; the number is kept in log_index.txt so startup stays fast
  int index = 0;
  if (FILE *f = fopen("/usd/log_index.txt", "r")) {
    if (fscanf(f, "%d", &index) != 1) index = 0;
    fclose(f);
  }
  if (FILE *f = fopen("/usd/log_index.txt", "w")) {
    fprintf(f, "%d\n", (index + 1) % 1000);
    fclose(f);
  }
  char path[40];
  snprintf(path, sizeof(path), "/usd/log_%03d.csv", index % 1000);
  file = fopen(path, "w");
  if (!file) return;
  fprintf(file, "ms,mode,x_in,y_in,heading_deg,lift_deg,claw,battery_pct,hottest_drive_c,event\n");
  pros::Task task(log_task, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_DEFAULT, "Logger");
}

void event(const std::string &text) {
  laptop::event(text);  // Also to the laptop, if one is listening
  if (!file) return;
  std::lock_guard<pros::Mutex> lock(events_mutex);
  if (pending.size() < 50) pending.push_back(text);
}

}  // namespace logger

#endif  // __arm__
