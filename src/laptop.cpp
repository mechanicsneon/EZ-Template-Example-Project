#include "laptop.hpp"

// Only on the real brain: VEX Sim builds this project for the PC, where there's no USB link
#ifndef __arm__
namespace laptop {
void initialize() {}
void opcontrol() {}
bool drive_busy() { return false; }
void event(const std::string &) {}
}  // namespace laptop
#else

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <sstream>
#include <string>

#include "main.h"
#include "pros/apix.h"

namespace laptop {
namespace {

constexpr int STATUS_MS = 100;     // Status 10 times a second
constexpr int STICK_CANCEL = 30;   // Pushing a stick past this stops whatever the laptop started

std::atomic<bool> busy{false};     // A laptop job is moving the robot
std::atomic<bool> started{false};  // initialize() ran (events before that are dropped)
pros::Mutex out_mutex;
pros::Task *job = nullptr;

std::string esc(const std::string &s) {
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') o += '\\', o += c;
    else if (c == '\n') o += "\\n";
    else if ((unsigned char)c >= 32) o += c;
  }
  return o;
}

void send(const std::string &json) {
  std::lock_guard<pros::Mutex> lock(out_mutex);
  printf("@%s\n", json.c_str());
  fflush(stdout);
}
void reply(const char *type, const std::string &msg) { send(std::string("{\"t\":\"") + type + "\",\"msg\":\"" + esc(msg) + "\"}"); }

// Moving the robot from the laptop: only off the field
bool pits() { return !pros::competition::is_connected(); }

void send_autons() {
  auto &as = ez::as::auton_selector;
  std::string s = "{\"t\":\"autons\",\"sel\":" + std::to_string(as.last_auton_page_current) + ",\"list\":[";
  for (size_t i = 0; i < as.Autons.size(); i++) {
    const std::string &n = as.Autons[i].Name;
    s += (i ? ",\"" : "\"") + esc(n.substr(0, n.find('\n'))) + "\"";
  }
  send(s + "]}");
}

void select(int i) {
  auto &as = ez::as::auton_selector;
  if (i < 0 || i >= (int)as.Autons.size()) return reply("err", "no auton " + std::to_string(i));
  as.auton_page_current = i;
  as.last_auton_page_current = i;
  ez::as::auto_sd_update();
  send_autons();
  logger::event("laptop picked auton " + std::to_string(i));
}

void stop() {
  if (job && busy) job->remove();  // Ends the laptop's task right away
  chassis.pid_targets_reset();
  chassis.drive_set(0, 0);  // Drive PID off, motors to 0
  lift::stop();
  busy = false;
  reply("ok", "stopped");
}

// Run something that drives the robot in its own task, set up like autonomous()
void start_job(const std::string &name, std::function<void()> fn) {
  if (busy) return reply("err", "already running: send stop first");
  busy = true;
  delete job;  // The old Task object (its task already ended)
  job = new pros::Task(
      [name, fn] {
        auto brake = chassis.drive_brake_get();
        chassis.pid_targets_reset();
        chassis.drive_sensor_reset();
        chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
        logger::event("laptop " + name + " start");
        fn();
        chassis.drive_set(0, 0);
        chassis.drive_brake_set(brake);
        busy = false;
        reply("done", name);
      },
      "Laptop job");
  reply("ok", name + " started");
}

void handle(std::string cmd) {
  while (!cmd.empty() && (cmd.back() == '\r' || cmd.back() == ' ')) cmd.pop_back();
  if (cmd.empty()) return;
  std::istringstream in(cmd);
  std::string word;
  in >> word;

  if (word == "hello") return send_autons();
  if (word == "stop") return stop();
  if (word == "select") {
    int i = -1;
    in >> i;
    return select(i);
  }

  // Everything below moves the robot
  if (!pits()) return reply("err", "plugged into a field: the laptop can't move the robot");
  if (busy && word != "zero") return reply("err", "busy: send stop first");

  if (word == "setup") {
    setup::start();
    return reply("ok", "setup started");
  }
  if (word == "lift") {
    int h = 0;
    in >> h;
    lift::move_to_height(h);
    return reply("ok", "lift to " + std::to_string(h));
  }
  if (word == "claw") {
    std::string how;
    in >> how;
    if (how == "open") claw::release();
    else claw::grab();
    return reply("ok", "claw " + how);
  }
  if (word == "bump") {
    assist::bump_toggle();
    return reply("ok", "Toggle bump");
  }
  if (word == "zero") {
    double x = 0, y = 0, h = 0;
    in >> x >> y >> h;
    chassis.odom_enable(true);
    chassis.odom_xyt_set(x, y, h);
    return reply("ok", "position set");
  }
  if (word == "auton") {
    return start_job("auton", [] { ez::as::auton_selector.selected_auton_call(); });
  }
  if (word == "route") {
    std::string rest;
    std::getline(in, rest);
    std::vector<std::string> steps;
    std::stringstream ss(rest);
    for (std::string step; std::getline(ss, step, ';');)
      if (step.find_first_not_of(' ') != std::string::npos) steps.push_back(step);
    for (auto &s : steps)
      if (!route_step(s, true)) return reply("err", "bad step: " + s);  // Check them all before moving
    return start_job("route (" + std::to_string(steps.size()) + " steps)", [steps] {
      for (auto &s : steps) route_step(s, false);
    });
  }
  reply("err", "unknown command: " + word);
}

void reader_task() {
  static char line[2048];
  while (true) {
    if (fgets(line, sizeof(line), stdin)) {
      handle(line);
    } else {
      clearerr(stdin);
      pros::delay(20);
    }
  }
}

void status_task() {
  while (true) {
    double hottest = 0;
    for (auto &m : chassis.left_motors) hottest = std::max(hottest, m.get_temperature());
    for (auto &m : chassis.right_motors) hottest = std::max(hottest, m.get_temperature());
    const char *mode = pros::competition::is_disabled() ? "disabled" : pros::competition::is_autonomous() ? "auton" : "driver";
    char buf[512];
    snprintf(buf, sizeof(buf),
             "{\"t\":\"s\",\"ms\":%lu,\"mode\":\"%s\",\"pits\":%d,\"busy\":%d,\"x\":%.2f,\"y\":%.2f,\"h\":%.1f,"
             "\"lift\":%.0f,\"claw\":%d,\"bat\":%d,\"drv\":%.0f,\"auton\":%d,\"setup\":\"%s\",\"lifttext\":\"%s\"}",
             (unsigned long)pros::millis(), mode, pits() ? 1 : 0, busy ? 1 : 0, chassis.odom_x_get(), chassis.odom_y_get(),
             chassis.odom_theta_get(), lift::position(), claw::holding() ? 1 : 0, (int)std::lround(pros::battery::get_capacity()),
             hottest, ez::as::auton_selector.last_auton_page_current, esc(setup::status_text()).c_str(), esc(lift::status_text()).c_str());
    send(buf);
    pros::delay(STATUS_MS);
  }
}

}  // namespace

void initialize() {
  pros::c::serctl(SERCTL_DISABLE_COBS, nullptr);  // Plain text lines, so a browser can read them
  started = true;
  pros::Task status(status_task, "Laptop status");
  pros::Task reader(reader_task, "Laptop commands");
}

void opcontrol() {
  if (!busy) return;
  bool stick = std::abs(master.get_analog(ANALOG_LEFT_Y)) > STICK_CANCEL || std::abs(master.get_analog(ANALOG_RIGHT_Y)) > STICK_CANCEL;
  if (stick) stop();
}

bool drive_busy() { return busy; }

void event(const std::string &text) {
  if (started) send("{\"t\":\"ev\",\"msg\":\"" + esc(text) + "\"}");
}

}  // namespace laptop

#endif  // __arm__
