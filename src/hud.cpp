#include "hud.hpp"

// The HUD only runs on the real brain (ARM). VEX Sim compiles this project for the PC,
// where there's no LVGL screen or real controller, so it gets empty stand-ins instead.
#ifndef __arm__
namespace hud {
void initialize() {}
void driver_timer_start() {}
}  // namespace hud
#else

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "auton_previews.hpp"
#include "liblvgl/lvgl.h"
#include "main.h"

namespace hud {
namespace {

/////
// Settings
/////

constexpr int DRIVER_MS = 105000;  // V5RC driver control is 1:45
constexpr int AUTON_MS = 15000;    // V5RC autonomous is 0:15
constexpr double TEMP_WARM = 45;   // C, yellow on the brain
constexpr double TEMP_HOT = 55;    // C, red + warning. V5 motors start cutting power here
constexpr int LOW_BATTERY = 30;    // %, warning below this
constexpr int CONTROLLER_MS = 60;  // The controller ignores screen updates sent faster than every 50ms

/////
// Robot status, shared by the controller and the brain
/////

enum Mode { DISABLED, AUTON, DRIVER };

struct MotorInfo {
  std::string name;  // "L10", "R1", "ARM18"
  double temp;       // C, NAN when unplugged
};

struct Warning {
  std::string key;   // Stays the same while the problem lasts (used to rumble once per new problem)
  std::string text;  // Fits the 15 character controller screen
};

struct Status {
  Mode mode;
  int ms_left;
  int battery;
  int controller_battery;
  bool imu_ok;
  double heading;
  std::vector<MotorInfo> motors;
  std::vector<Warning> warnings;
  std::string auton;
};

std::atomic<int> current_mode{-1};
std::atomic<uint32_t> mode_start_ms{0};

Mode read_mode() {
  if (pros::competition::is_disabled()) return DISABLED;
  if (pros::competition::is_autonomous()) return AUTON;
  return DRIVER;
}

// Restart the match timer whenever the field switches modes
void track_mode() {
  int mode = read_mode();
  if (mode != current_mode.load()) {
    current_mode = mode;
    mode_start_ms = pros::millis();
  }
}

std::string selected_auton() {
  auto &selector = ez::as::auton_selector;
  if (selector.Autons.empty()) return "NONE";
  // last_auton_page_current is the one autonomous() will actually run
  int i = selector.last_auton_page_current;
  if (i < 0 || i >= (int)selector.Autons.size()) i = 0;
  std::string name = selector.Autons[i].Name;
  return name.substr(0, name.find('\n'));  // Just the title, not the description
}

void add_motor(std::vector<MotorInfo> &motors, std::string side, int port, double temp) {
  motors.push_back({side + std::to_string(std::abs(port)), std::isfinite(temp) ? temp : NAN});
}

Status read_status() {
  Status s;
  s.mode = (Mode)std::max(0, current_mode.load());
  int total = s.mode == AUTON ? AUTON_MS : s.mode == DRIVER ? DRIVER_MS : 0;
  s.ms_left = std::max(0, total - (int)(pros::millis() - mode_start_ms.load()));
  s.battery = (int)std::lround(pros::battery::get_capacity());
  s.controller_battery = master.get_battery_level();
  s.imu_ok = chassis.imu.is_installed();
  double h = chassis.drive_imu_get();
  s.heading = std::fmod(std::fmod(h, 360.0) + 360.0, 360.0);
  s.auton = selected_auton();

  for (auto &m : chassis.left_motors) add_motor(s.motors, "L", m.get_port(), m.get_temperature());
  for (auto &m : chassis.right_motors) add_motor(s.motors, "R", m.get_port(), m.get_temperature());
  auto arm_ports = arm.get_port_all();
  auto arm_temps = arm.get_temperature_all();
  for (size_t i = 0; i < arm_ports.size() && i < arm_temps.size(); i++)
    add_motor(s.motors, "ARM", arm_ports[i], arm_temps[i]);

  // Warnings, most important first
  std::map<int, int> port_uses;
  for (auto &m : chassis.left_motors) port_uses[std::abs(m.get_port())]++;
  for (auto &m : chassis.right_motors) port_uses[std::abs(m.get_port())]++;
  for (int p : arm_ports) port_uses[std::abs(p)]++;
  for (auto &[port, uses] : port_uses)
    if (uses > 1) s.warnings.push_back({"dup" + std::to_string(port), "PORT " + std::to_string(port) + " TWICE"});

  if (!s.imu_ok)
    s.warnings.push_back({"imu", "IMU UNPLUGGED"});
  else if (!chassis.drive_imu_calibrated())
    s.warnings.push_back({"imucal", "IMU NOT READY"});

  for (auto &m : s.motors) {
    if (std::isnan(m.temp))
      s.warnings.push_back({"unplug" + m.name, "UNPLUGGED " + m.name});
    else if (m.temp >= TEMP_HOT)
      s.warnings.push_back({"hot" + m.name, "HOT " + m.name + " " + std::to_string((int)m.temp) + "C"});
  }

  if (s.battery < LOW_BATTERY)
    s.warnings.push_back({"bat", "LOW BATTERY " + std::to_string(s.battery) + "%"});

  return s;
}

std::string time_text(int ms) {
  int sec = (ms + 999) / 1000;  // Round up so it reads 0:00 only when time is actually up
  char buf[8];
  snprintf(buf, sizeof(buf), "%d:%02d", sec / 60, sec % 60);
  return buf;
}

// Cycle through warnings every 2 seconds so they all get seen
const Warning *current_warning(const Status &s) {
  if (s.warnings.empty()) return nullptr;
  return &s.warnings[(pros::millis() / 2000) % s.warnings.size()];
}

/////
// Controller screen (3 lines x 15 characters)
//   1:23   BAT  87%
//   AUTO:Drive        <- or a warning, like PORT 10 TWICE
//   DRV 42C ARM 38C   <- hottest drive motor and hottest arm motor
/////

std::string fit(std::string text) {
  text.resize(15, ' ');  // Pad so old characters get overwritten
  return text;
}

std::string hottest(const Status &s, bool drive) {
  double best = NAN;
  for (auto &m : s.motors) {
    bool is_drive = m.name[0] == 'L' || m.name[0] == 'R';
    if (is_drive == drive && !std::isnan(m.temp) && !(m.temp <= best)) best = m.temp;
  }
  if (std::isnan(best)) return " --";
  char buf[8];
  snprintf(buf, sizeof(buf), "%2dC", (int)best);
  return buf;
}

std::string controller_line(const Status &s, int line) {
  char buf[32];
  if (line == 0) {
    std::string clock = s.mode == DISABLED ? "READY" : time_text(s.ms_left);
    snprintf(buf, sizeof(buf), "%-5s  BAT %3d%%", clock.c_str(), s.battery);
    return buf;
  }
  if (line == 1) {
    if (s.mode != DRIVER) {
      if (auto *w = current_warning(s)) return w->text;
      return "AUTO:" + s.auton;
    }
    // Driver control: lift status (NEXT H2, SAVED H2 640...), taking turns with any warnings
    std::string lift_text = lift::status_text();
    if (lift::status_is_new() || s.warnings.empty()) return lift_text;
    size_t i = (pros::millis() / 2000) % (s.warnings.size() + 1);
    return i < s.warnings.size() ? s.warnings[i].text : lift_text;
  }
  return "DRV " + hottest(s, true) + " ARM " + hottest(s, false);
}

void controller_task() {
  pros::delay(250);  // Let initialize()'s calibration rumble finish first

  int line = 0;
  int last_mode = -1;
  int last_sec = -1;
  std::set<std::string> seen_warnings;

  while (true) {
    track_mode();
    Status s = read_status();

    // Rumbles in driver control: "-" at 0:30, "- -" at 0:10 (Endgame), "..." at 0:05, "." for each new problem
    std::string rumble;
    int sec = (s.ms_left + 999) / 1000;
    if (s.mode == DRIVER && s.mode == last_mode && sec != last_sec) {
      if (sec == 30) rumble = "-";
      if (sec == 10) rumble = "- -";  // Endgame (SG12): head for the Midfield, check our Toggles
      if (sec == 5) rumble = "...";
    }
    // Each problem rumbles once per match period, even if it flickers (like a motor right at 55C)
    if (s.mode != last_mode) seen_warnings.clear();
    for (auto &w : s.warnings) {
      if (seen_warnings.insert(w.key).second && s.mode == DRIVER && rumble.empty()) rumble = ".";
    }
    last_mode = s.mode;
    last_sec = sec;

    // Each controller message needs its own slot, so a rumble skips one text update
    if (!rumble.empty()) {
      master.rumble(rumble.c_str());
    } else {
      master.set_text(line, 0, fit(controller_line(s, line)));
      line = (line + 1) % 3;
    }
    pros::delay(CONTROLLER_MS);
  }
}

/////
// Brain dashboard (480 x 240)
/////

const lv_color_t BG = lv_color_hex(0x0B0F17);
const lv_color_t PANEL = lv_color_hex(0x161D2A);
const lv_color_t TEXT = lv_color_hex(0xE8EEF5);
const lv_color_t MUTED = lv_color_hex(0x8794A8);
const lv_color_t CYAN = lv_color_hex(0x19E6FF);
const lv_color_t MAGENTA = lv_color_hex(0xFF2BD6);
const lv_color_t GOOD = lv_color_hex(0x22C55E);
const lv_color_t WARM = lv_color_hex(0xF5B014);
const lv_color_t HOT = lv_color_hex(0xEF4444);
const lv_color_t OFF = lv_color_hex(0x3A4456);
const lv_color_t DARK_TEXT = lv_color_hex(0x0B0F17);

constexpr int MAX_TILES = 8;

// Which screen the driver picked with the touch buttons. Resets whenever the mode changes.
// AUTOMATIC: the pre-match check while disabled, the HUD while enabled
enum ScreenChoice { AUTOMATIC, SHOW_HUD, SHOW_SELECTOR, SHOW_CHECKS, SHOW_PICKER };
std::atomic<int> screen_choice{AUTOMATIC};

lv_obj_t *hud_screen = nullptr;
lv_obj_t *selector_screen = nullptr;  // EZ-Template's auton selector (LLEMU)
lv_obj_t *checks_screen = nullptr;    // Pre-match check
lv_obj_t *picker_screen = nullptr;    // Our auton picker (tiles + field map)

constexpr int CHECK_COUNT = 8;
lv_obj_t *check_status, *check_status_label, *check_auton, *check_start;
lv_obj_t *check_dots[CHECK_COUNT], *check_labels[CHECK_COUNT];

lv_obj_t *mode_pill, *mode_label, *timer_label, *subtitle, *warning_pill, *warning_label;
lv_obj_t *battery_label, *battery_bar, *controller_label, *imu_label;
lv_obj_t *tiles[MAX_TILES], *tile_names[MAX_TILES], *tile_temps[MAX_TILES];
lv_obj_t *auton_label, *lift_label;

lv_obj_t *box(lv_obj_t *parent, int x, int y, int w, int h, lv_color_t color, int radius = 8) {
  lv_obj_t *o = lv_obj_create(parent);
  lv_obj_remove_style_all(o);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_size(o, w, h);
  lv_obj_set_style_bg_color(o, color, 0);
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(o, radius, 0);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  return o;
}

lv_obj_t *text(lv_obj_t *parent, int x, int y, const lv_font_t *font, lv_color_t color, const char *str) {
  lv_obj_t *l = lv_label_create(parent);
  lv_obj_set_pos(l, x, y);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, color, 0);
  lv_label_set_text(l, str);
  return l;
}

lv_obj_t *button(lv_obj_t *parent, const char *str, lv_event_cb_t on_click, void *user_data = nullptr) {
  lv_obj_t *b = lv_btn_create(parent);
  lv_obj_set_style_bg_color(b, PANEL, 0);
  lv_obj_set_style_bg_color(b, OFF, LV_STATE_PRESSED);
  lv_obj_set_style_border_color(b, CYAN, 0);
  lv_obj_set_style_border_width(b, 2, 0);
  lv_obj_set_style_radius(b, 8, 0);
  lv_obj_set_style_shadow_width(b, 0, 0);
  lv_obj_add_event_cb(b, on_click, LV_EVENT_CLICKED, user_data);
  lv_obj_t *l = lv_label_create(b);
  lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(l, CYAN, 0);
  lv_label_set_text(l, str);
  lv_obj_center(l);
  return b;
}

lv_color_t temp_color(double temp) {
  if (std::isnan(temp)) return OFF;
  if (temp >= TEMP_HOT) return HOT;
  if (temp >= TEMP_WARM) return WARM;
  return GOOD;
}

void build_hud_screen() {
  hud_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(hud_screen, BG, 0);
  lv_obj_clear_flag(hud_screen, LV_OBJ_FLAG_SCROLLABLE);

  // Header: team, warnings, mode, match timer
  text(hud_screen, 10, 9, &lv_font_montserrat_20, CYAN, "17021V");
  subtitle = text(hud_screen, 94, 14, &lv_font_montserrat_12, MUTED, "NEON MECHANICS");
  warning_pill = box(hud_screen, 88, 8, 196, 26, HOT, 13);
  warning_label = text(warning_pill, 0, 0, &lv_font_montserrat_14, TEXT, "");
  lv_obj_center(warning_label);
  mode_pill = box(hud_screen, 292, 8, 86, 26, MAGENTA, 13);
  mode_label = text(mode_pill, 0, 0, &lv_font_montserrat_14, DARK_TEXT, "");
  lv_obj_center(mode_label);
  timer_label = text(hud_screen, 0, 0, &lv_font_montserrat_30, TEXT, "1:45");
  lv_obj_align(timer_label, LV_ALIGN_TOP_RIGHT, -10, 4);

  // Power + IMU
  lv_obj_t *power = box(hud_screen, 8, 44, 150, 128, PANEL);
  text(power, 10, 8, &lv_font_montserrat_12, MUTED, "BATTERY");
  battery_label = text(power, 10, 22, &lv_font_montserrat_36, TEXT, "--%");
  battery_bar = lv_bar_create(power);
  lv_obj_set_pos(battery_bar, 10, 66);
  lv_obj_set_size(battery_bar, 130, 10);
  lv_bar_set_range(battery_bar, 0, 100);
  lv_obj_set_style_bg_color(battery_bar, OFF, LV_PART_MAIN);
  lv_obj_set_style_bg_color(battery_bar, GOOD, LV_PART_INDICATOR);
  controller_label = text(power, 10, 84, &lv_font_montserrat_14, TEXT, "");
  imu_label = text(power, 10, 104, &lv_font_montserrat_14, TEXT, "");

  // Motor temps, one tile per motor
  lv_obj_t *motors = box(hud_screen, 166, 44, 306, 128, PANEL);
  text(motors, 10, 6, &lv_font_montserrat_12, MUTED, "MOTOR TEMPS");
  for (int i = 0; i < MAX_TILES; i++) {
    tiles[i] = box(motors, 0, 0, 10, 10, OFF, 6);
    tile_names[i] = text(tiles[i], 6, 4, &lv_font_montserrat_12, DARK_TEXT, "");
    tile_temps[i] = text(tiles[i], 6, 20, &lv_font_montserrat_20, DARK_TEXT, "");
    lv_obj_add_flag(tiles[i], LV_OBJ_FLAG_HIDDEN);
  }

  // Selected auton + button to change it
  lv_obj_t *auton = box(hud_screen, 8, 180, 464, 52, PANEL);
  text(auton, 10, 6, &lv_font_montserrat_12, MUTED, "AUTON");
  auton_label = text(auton, 10, 22, &lv_font_montserrat_20, TEXT, "");
  lv_obj_set_width(auton_label, 200);
  lv_label_set_long_mode(auton_label, LV_LABEL_LONG_DOT);
  lift_label = text(auton, 120, 6, &lv_font_montserrat_12, CYAN, "");
  lv_obj_t *pick = button(auton, "PICK AUTON", [](lv_event_t *) { screen_choice = SHOW_PICKER; });
  lv_obj_set_size(pick, 130, 38);
  lv_obj_align(pick, LV_ALIGN_RIGHT_MID, -7, 0);
  lv_obj_t *checks = button(auton, "CHECKS", [](lv_event_t *) { screen_choice = SHOW_CHECKS; });
  lv_obj_set_size(checks, 96, 38);
  lv_obj_align(checks, LV_ALIGN_RIGHT_MID, -145, 0);
}

/////
// Pre-match check: what's selected, where it starts, and anything that would cost the match
/////

void build_checks_screen() {
  checks_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(checks_screen, BG, 0);
  lv_obj_clear_flag(checks_screen, LV_OBJ_FLAG_SCROLLABLE);

  text(checks_screen, 10, 9, &lv_font_montserrat_20, CYAN, "PRE-MATCH");
  check_status = box(checks_screen, 136, 8, 150, 26, OFF, 13);
  check_status_label = text(check_status, 0, 0, &lv_font_montserrat_14, DARK_TEXT, "");
  lv_obj_t *auton = button(checks_screen, "AUTON", [](lv_event_t *) { screen_choice = SHOW_PICKER; });
  lv_obj_set_size(auton, 90, 32);
  lv_obj_align(auton, LV_ALIGN_TOP_RIGHT, -84, 4);
  lv_obj_t *hud = button(checks_screen, "HUD", [](lv_event_t *) { screen_choice = SHOW_HUD; });
  lv_obj_set_size(hud, 70, 32);
  lv_obj_align(hud, LV_ALIGN_TOP_RIGHT, -6, 4);

  lv_obj_t *auton_box = box(checks_screen, 8, 42, 464, 66, PANEL);
  check_auton = text(auton_box, 10, 6, &lv_font_montserrat_20, TEXT, "");
  lv_obj_set_width(check_auton, 340);
  lv_label_set_long_mode(check_auton, LV_LABEL_LONG_DOT);
  check_start = text(auton_box, 10, 32, &lv_font_montserrat_12, MUTED, "");
  lv_obj_set_width(check_start, 340);
  // Calibrate + load the Preload (only moves while the robot is enabled, so before plugging into the field)
  lv_obj_t *setup_btn = button(auton_box, "SETUP", [](lv_event_t *) { setup::start(); });
  lv_obj_set_size(setup_btn, 96, 44);
  lv_obj_align(setup_btn, LV_ALIGN_RIGHT_MID, -8, 0);
  lv_label_set_long_mode(check_start, LV_LABEL_LONG_WRAP);

  lv_obj_t *list = box(checks_screen, 8, 114, 464, 118, PANEL);
  for (int i = 0; i < CHECK_COUNT; i++) {
    int x = i < 4 ? 10 : 238, y = 10 + (i % 4) * 26;
    check_dots[i] = box(list, x, y + 3, 12, 12, OFF, 6);
    check_labels[i] = text(list, x + 20, y, &lv_font_montserrat_14, TEXT, "");
    lv_obj_set_width(check_labels[i], 200);
    lv_label_set_long_mode(check_labels[i], LV_LABEL_LONG_DOT);
  }
}

enum Level { OK, WARN, BAD };

void update_checks(const Status &s) {
  // Selected auton: its title + description (where it starts, what it does)
  auto &selector = ez::as::auton_selector;
  std::string title = "No auton", start = "";
  if (!selector.Autons.empty()) {
    int i = std::clamp(selector.last_auton_page_current, 0, (int)selector.Autons.size() - 1);
    const std::string &name = selector.Autons[i].Name;
    size_t br = name.find("\n\n");
    title = name.substr(0, name.find('\n'));
    if (br != std::string::npos) start = name.substr(br + 2);
    for (auto &p : previews::ALL)
      if (title == p.title) {
        char est[48];
        snprintf(est, sizeof(est), "~%d pts, %.0f s.  ", p.points, p.seconds);
        start = est + start;
      }
  }
  lv_label_set_text(check_auton, title.c_str());
  lv_label_set_text(check_start, start.c_str());

  int unplugged = 0;
  std::string first_unplugged;
  for (auto &m : s.motors)
    if (std::isnan(m.temp) && unplugged++ == 0) first_unplugged = m.name;
  bool dup = std::any_of(s.warnings.begin(), s.warnings.end(), [](const Warning &w) { return w.key.rfind("dup", 0) == 0; });
  int saved = lift::heights_saved();
  bool sd = pros::usd::is_installed();
  auto setup_state = setup::state();
  bool imu_ready = s.imu_ok && chassis.drive_imu_calibrated();
  bool field = pros::competition::is_connected();

  const std::pair<Level, std::string> checks[CHECK_COUNT] = {
      {imu_ready ? OK : BAD, !s.imu_ok ? "IMU unplugged" : imu_ready ? "IMU ready" : "IMU not ready: restart, hold still"},
      {unplugged || dup ? BAD : OK,
       dup ? "A port is used twice"
           : unplugged ? "Unplugged: " + first_unplugged + (unplugged > 1 ? " +" + std::to_string(unplugged - 1) : "")
                       : "All motors plugged in"},
      {s.battery >= 60 ? OK : s.battery >= LOW_BATTERY ? WARN : BAD, "Battery " + std::to_string(s.battery) + "%"},
      {cfg::CLAW_CONFIGURED ? OK : BAD, cfg::CLAW_CONFIGURED ? "Claw set up" : "Claw not set: robot_config.hpp"},
      {saved == lift::HEIGHT_COUNT ? OK : WARN,
       saved == lift::HEIGHT_COUNT ? "Lift heights saved" : std::to_string(saved) + "/3 lift heights saved (A+R1)"},
      {sd ? OK : WARN, sd ? "SD card in (logs on)" : "No SD card: no logs/heights"},
      {setup_state == setup::DONE ? OK : setup_state == setup::FAILED ? BAD : WARN, setup::status_text()},
      {field ? OK : WARN, field ? "Field connected" : "Not on a field/switch"},
  };

  int bad = 0, warn = 0;
  for (int i = 0; i < CHECK_COUNT; i++) {
    lv_color_t c = checks[i].first == OK ? GOOD : checks[i].first == WARN ? WARM : HOT;
    lv_obj_set_style_bg_color(check_dots[i], c, 0);
    lv_label_set_text(check_labels[i], checks[i].second.c_str());
    bad += checks[i].first == BAD;
    warn += checks[i].first == WARN;
  }
  std::string status = bad ? std::to_string(bad) + (bad == 1 ? " PROBLEM" : " PROBLEMS") : warn ? "CHECK " + std::to_string(warn) : "READY";
  lv_obj_set_style_bg_color(check_status, bad ? HOT : warn ? WARM : GOOD, 0);
  lv_obj_set_style_text_color(check_status_label, bad ? TEXT : DARK_TEXT, 0);
  lv_label_set_text(check_status_label, status.c_str());
  lv_obj_center(check_status_label);
}

void layout_tiles(int count) {
  count = std::min(count, MAX_TILES);
  int cols = std::max(1, (count + 1) / 2);
  int w = (306 - 20 - (cols - 1) * 6) / cols;
  for (int i = 0; i < MAX_TILES; i++) {
    if (i >= count) {
      lv_obj_add_flag(tiles[i], LV_OBJ_FLAG_HIDDEN);
      continue;
    }
    lv_obj_clear_flag(tiles[i], LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(tiles[i], 10 + (i % cols) * (w + 6), 26 + (i / cols) * 50);
    lv_obj_set_size(tiles[i], w, 46);
  }
}

void update_hud(const Status &s) {
  static const char *MODE_NAMES[] = {"DISABLED", "AUTON", "DRIVER"};
  lv_label_set_text(mode_label, MODE_NAMES[s.mode]);
  lv_obj_set_style_bg_color(mode_pill, s.mode == DRIVER ? MAGENTA : s.mode == AUTON ? CYAN : OFF, 0);
  lv_obj_set_style_text_color(mode_label, s.mode == DISABLED ? TEXT : DARK_TEXT, 0);

  lv_label_set_text(timer_label, s.mode == DISABLED ? "--:--" : time_text(s.ms_left).c_str());
  bool urgent = s.mode == DRIVER && s.ms_left <= 30000;
  lv_obj_set_style_text_color(timer_label, urgent ? HOT : TEXT, 0);

  if (auto *w = current_warning(s)) {
    lv_label_set_text(warning_label, w->text.c_str());
    lv_obj_center(warning_label);
    lv_obj_clear_flag(warning_pill, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(subtitle, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(warning_pill, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(subtitle, LV_OBJ_FLAG_HIDDEN);
  }

  lv_label_set_text_fmt(battery_label, "%d%%", s.battery);
  lv_bar_set_value(battery_bar, std::clamp(s.battery, 0, 100), LV_ANIM_OFF);
  lv_color_t bat = s.battery < LOW_BATTERY ? HOT : s.battery < 50 ? WARM : GOOD;
  lv_obj_set_style_bg_color(battery_bar, bat, LV_PART_INDICATOR);
  lv_obj_set_style_text_color(battery_label, s.battery < LOW_BATTERY ? HOT : TEXT, 0);
  lv_label_set_text_fmt(controller_label, "Controller %d%%", s.controller_battery);
  if (s.imu_ok)
    lv_label_set_text_fmt(imu_label, "IMU %d\xC2\xB0", (int)std::lround(s.heading) % 360);
  else
    lv_label_set_text(imu_label, "IMU unplugged");
  lv_obj_set_style_text_color(imu_label, s.imu_ok ? TEXT : HOT, 0);

  layout_tiles(s.motors.size());
  for (int i = 0; i < (int)s.motors.size() && i < MAX_TILES; i++) {
    auto &m = s.motors[i];
    lv_color_t c = temp_color(m.temp);
    lv_color_t ink = std::isnan(m.temp) ? TEXT : DARK_TEXT;
    lv_obj_set_style_bg_color(tiles[i], c, 0);
    lv_label_set_text(tile_names[i], m.name.c_str());
    lv_obj_set_style_text_color(tile_names[i], ink, 0);
    if (std::isnan(m.temp))
      lv_label_set_text(tile_temps[i], "OFF");
    else
      lv_label_set_text_fmt(tile_temps[i], "%d\xC2\xB0" "C", (int)m.temp);
    lv_obj_set_style_text_color(tile_temps[i], ink, 0);
  }

  lv_label_set_text(auton_label, s.auton.c_str());
  lv_label_set_text_fmt(lift_label, "LIFT %d\xC2\xB0   %s", (int)lift::position(), lift::status_text().c_str());
}

/////
// Auton picker: tap a category, tap an auton (that's it, it's picked and saved to the SD card).
// The map shows where it starts and the path it drives, with its estimated points + time
/////

enum Category { CAT_RED, CAT_BLUE, CAT_SKILLS, CAT_BACKUP, CAT_TEST, CAT_COUNT };
const char *CAT_NAMES[CAT_COUNT] = {"RED", "BLUE", "SKILLS", "BACKUP", "TEST"};
constexpr int MAP = 112;                // Map size in pixels
constexpr double MAP_S = MAP / 140.4;  // Pixels per field inch
constexpr int MAX_PATH = 64;

int picker_cat = CAT_RED;
lv_obj_t *cat_buttons[CAT_COUNT], *cat_labels[CAT_COUNT];
lv_obj_t *tile_list, *map_box, *path_line, *start_box, *end_dot;
lv_obj_t *info_points, *info_time, *info_note, *info_title, *info_desc;
std::vector<std::pair<lv_obj_t *, int>> tiles_shown;  // Tile + its index in the auton selector
lv_point_t path_pts[MAX_PATH];

std::string auton_title(int i) {
  const std::string &n = ez::as::auton_selector.Autons[i].Name;
  return n.substr(0, n.find('\n'));
}
std::string auton_desc(int i) {
  const std::string &n = ez::as::auton_selector.Autons[i].Name;
  size_t br = n.find("\n\n");
  return br == std::string::npos ? "" : n.substr(br + 2);
}
Category category_of(int i) {
  std::string t = auton_title(i);
  if (t.rfind("Test", 0) == 0) return CAT_TEST;
  if (t.rfind("Skills", 0) == 0) return CAT_SKILLS;
  if (auton_desc(i).rfind("BACKUP", 0) == 0) return CAT_BACKUP;
  return t.rfind("Blue", 0) == 0 ? CAT_BLUE : CAT_RED;
}
const previews::Preview *preview_of(int i) {
  std::string t = auton_title(i);
  for (auto &p : previews::ALL)
    if (t == p.title) return &p;
  return nullptr;
}
int selected_index() {
  auto &as = ez::as::auton_selector;
  return std::clamp(as.last_auton_page_current, 0, std::max(0, (int)as.Autons.size() - 1));
}
// Pick an auton: what autonomous() runs, saved so it survives a restart
void select_auton(int i) {
  auto &as = ez::as::auton_selector;
  as.auton_page_current = i;
  as.last_auton_page_current = i;
  ez::as::auto_sd_update();
}

// Field inches -> map pixels. The map is drawn from the audience like the field diagrams
// (red Alliance Station on the left); Blue autons are red's path turned 180 degrees
lv_point_t map_px(double x, double y) {
  return {(lv_coord_t)std::lround((x + 70.2) * MAP_S), (lv_coord_t)std::lround((70.2 - y) * MAP_S)};
}

lv_obj_t *dot(lv_obj_t *parent, double x, double y, int size, lv_color_t color) {
  lv_point_t p = map_px(x, y);
  return box(parent, p.x - size / 2, p.y - size / 2, size, size, color, size / 2);
}

void update_preview(int i) {
  auto *p = preview_of(i);
  bool blue = auton_title(i).rfind("Blue", 0) == 0;
  lv_color_t alliance = blue ? lv_color_hex(0x2F7DF6) : lv_color_hex(0xE23B3B);
  if (p && !p->path.empty()) {
    int n = std::min((int)p->path.size(), MAX_PATH);
    for (int k = 0; k < n; k++) {
      double x = p->path[k].x, y = p->path[k].y;
      path_pts[k] = blue ? map_px(-x, -y) : map_px(x, y);
    }
    lv_line_set_points(path_line, path_pts, n);
    lv_obj_clear_flag(path_line, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(start_box, path_pts[0].x - 7, path_pts[0].y - 7);
    lv_obj_set_style_border_color(start_box, alliance, 0);
    lv_obj_clear_flag(start_box, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(end_dot, path_pts[n - 1].x - 4, path_pts[n - 1].y - 4);
    lv_obj_clear_flag(end_dot, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text_fmt(info_points, "%d", p->points);
    lv_label_set_text_fmt(info_time, "%.0f s", p->seconds);
    lv_label_set_text(info_note, p->note);
  } else {
    lv_obj_add_flag(path_line, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(start_box, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(end_dot, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(info_points, "-");
    lv_label_set_text(info_time, "");
    lv_label_set_text(info_note, category_of(i) == CAT_TEST ? "PID test, not for matches" : "");
  }
  lv_label_set_text(info_title, auton_title(i).c_str());
  lv_label_set_text(info_desc, auton_desc(i).c_str());
}

void style_tiles() {
  int sel = selected_index();
  for (auto &[t, i] : tiles_shown) {
    bool on = i == sel;
    lv_obj_set_style_border_color(t, on ? CYAN : OFF, 0);
    lv_obj_set_style_bg_color(t, on ? lv_color_hex(0x123040) : PANEL, 0);
  }
}

void build_tiles() {
  lv_obj_clean(tile_list);
  tiles_shown.clear();
  auto &as = ez::as::auton_selector;
  int count = (int)as.Autons.size();
  for (int i = picker_cat == CAT_TEST ? -1 : 0; i < count; i++) {
    if (i >= 0 && category_of(i) != picker_cat) continue;
    lv_obj_t *t = lv_btn_create(tile_list);
    lv_obj_set_size(t, 226, 48);
    lv_obj_set_style_radius(t, 8, 0);
    lv_obj_set_style_border_width(t, 2, 0);
    lv_obj_set_style_shadow_width(t, 0, 0);
    lv_obj_set_style_pad_all(t, 0, 0);
    lv_obj_t *name = text(t, 10, 4, &lv_font_montserrat_20, TEXT, i < 0 ? "EZ screen" : auton_title(i).c_str());
    lv_obj_set_width(name, 206);
    lv_label_set_long_mode(name, LV_LABEL_LONG_DOT);
    auto *p = i < 0 ? nullptr : preview_of(i);
    char sub[64];
    if (i < 0)
      snprintf(sub, sizeof(sub), "Odometry readout + PID tuner page");
    else if (p)
      snprintf(sub, sizeof(sub), "~%d pts  %.0f s%s%s", p->points, p->seconds, *p->note ? "  " : "", p->note);
    else
      snprintf(sub, sizeof(sub), "%s", auton_desc(i).substr(0, 40).c_str());
    lv_obj_t *meta = text(t, 10, 28, &lv_font_montserrat_12, MUTED, sub);
    lv_obj_set_width(meta, 206);
    lv_label_set_long_mode(meta, LV_LABEL_LONG_DOT);
    lv_obj_add_event_cb(
        t,
        [](lv_event_t *e) {
          int i = (int)(intptr_t)lv_event_get_user_data(e);
          if (i < 0) {  // EZ-Template's own screen (odometry readout)
            screen_choice = SHOW_SELECTOR;
            return;
          }
          select_auton(i);
          style_tiles();
          update_preview(i);
          master.rumble(".");
        },
        LV_EVENT_CLICKED, (void *)(intptr_t)i);
    tiles_shown.push_back({t, i});
  }
  style_tiles();
  for (int c = 0; c < CAT_COUNT; c++) {
    lv_obj_set_style_bg_color(cat_buttons[c], c == picker_cat ? CYAN : PANEL, 0);
    lv_obj_set_style_text_color(cat_labels[c], c == picker_cat ? DARK_TEXT : CYAN, 0);
  }
}

void open_picker() {
  int sel = selected_index();
  if (!ez::as::auton_selector.Autons.empty()) picker_cat = category_of(sel);
  build_tiles();
  if (!ez::as::auton_selector.Autons.empty()) update_preview(sel);
}

void build_picker_screen() {
  picker_screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(picker_screen, BG, 0);
  lv_obj_clear_flag(picker_screen, LV_OBJ_FLAG_SCROLLABLE);

  // Top bar: category tabs + DONE
  text(picker_screen, 8, 11, &lv_font_montserrat_14, CYAN, "AUTON");
  for (int c = 0; c < CAT_COUNT; c++) {
    cat_buttons[c] = button(
        picker_screen, CAT_NAMES[c],
        [](lv_event_t *e) {
          picker_cat = (int)(intptr_t)lv_event_get_user_data(e);
          build_tiles();
        },
        (void *)(intptr_t)c);
    lv_obj_set_size(cat_buttons[c], 64, 30);
    lv_obj_set_pos(cat_buttons[c], 66 + c * 68, 4);
    cat_labels[c] = lv_obj_get_child(cat_buttons[c], 0);
  }
  lv_obj_t *done = button(picker_screen, "DONE", [](lv_event_t *) { screen_choice = AUTOMATIC; });
  lv_obj_set_size(done, 66, 30);
  lv_obj_set_pos(done, 408, 4);

  // Left: tiles for the category (scrolls if there are more than fit)
  tile_list = lv_obj_create(picker_screen);
  lv_obj_remove_style_all(tile_list);
  lv_obj_set_pos(tile_list, 6, 40);
  lv_obj_set_size(tile_list, 232, 196);
  lv_obj_set_flex_flow(tile_list, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(tile_list, 6, 0);
  lv_obj_add_flag(tile_list, LV_OBJ_FLAG_SCROLLABLE);

  // Right: field map + estimate + description
  map_box = box(picker_screen, 246, 40, MAP, MAP, lv_color_hex(0x2F343C), 4);
  static lv_point_t auton_line[2], midfield[5];
  auton_line[0] = map_px(-58.5, 58.5);
  auton_line[1] = map_px(58.5, -58.5);
  const double m = 23.55;
  midfield[0] = map_px(-m, 0); midfield[1] = map_px(0, m); midfield[2] = map_px(m, 0); midfield[3] = map_px(0, -m); midfield[4] = map_px(-m, 0);
  for (int k = 0; k < 2; k++) {
    lv_obj_t *l = lv_line_create(map_box);
    lv_line_set_points(l, k ? midfield : auton_line, k ? 5 : 2);
    lv_obj_set_style_line_width(l, 1, 0);
    lv_obj_set_style_line_color(l, lv_color_hex(0xD0D4DA), 0);
  }
  const lv_color_t RED_C = lv_color_hex(0xE23B3B), BLUE_C = lv_color_hex(0x2F7DF6), BLACK_C = lv_color_hex(0x15171B);
  const double T1 = 23.55, T2 = 47.1;
  dot(map_box, 0, 0, 8, BLACK_C);
  dot(map_box, -T2, -T1, 7, RED_C); dot(map_box, -T1, -T2, 7, RED_C);
  dot(map_box, T2, T1, 7, BLUE_C); dot(map_box, T1, T2, 7, BLUE_C);
  dot(map_box, -T2, T1, 7, BLACK_C); dot(map_box, T1, -T2, 7, BLACK_C); dot(map_box, -T1, T2, 7, BLACK_C); dot(map_box, T2, -T1, 7, BLACK_C);
  path_line = lv_line_create(map_box);
  lv_obj_set_style_line_width(path_line, 3, 0);
  lv_obj_set_style_line_color(path_line, CYAN, 0);
  lv_obj_set_style_line_rounded(path_line, true, 0);
  start_box = box(map_box, 0, 0, 14, 14, BG, 3);
  lv_obj_set_style_border_width(start_box, 3, 0);
  end_dot = box(map_box, 0, 0, 8, 8, MAGENTA, 4);

  text(picker_screen, 366, 40, &lv_font_montserrat_12, MUTED, "EST. POINTS");
  info_points = text(picker_screen, 366, 52, &lv_font_montserrat_36, TEXT, "");
  info_time = text(picker_screen, 366, 94, &lv_font_montserrat_20, TEXT, "");
  info_note = text(picker_screen, 366, 118, &lv_font_montserrat_12, WARM, "");
  lv_obj_set_width(info_note, 108);
  lv_label_set_long_mode(info_note, LV_LABEL_LONG_WRAP);
  info_title = text(picker_screen, 246, 158, &lv_font_montserrat_20, CYAN, "");
  lv_obj_set_width(info_title, 228);
  lv_label_set_long_mode(info_title, LV_LABEL_LONG_DOT);
  info_desc = text(picker_screen, 246, 182, &lv_font_montserrat_12, TEXT, "");
  lv_obj_set_size(info_desc, 228, 56);
  lv_label_set_long_mode(info_desc, LV_LABEL_LONG_WRAP);
}

// Runs inside LVGL's own task, so it's safe to touch the screen here
void brain_tick(lv_timer_t *) {
  // Build everything on the first tick, inside LVGL's task (LVGL isn't thread safe)
  if (!hud_screen) {
    // EZ-Template's auton selector is on the active screen, keep it so we can switch back
    selector_screen = lv_scr_act();
    lv_obj_t *back = button(selector_screen, "BACK", [](lv_event_t *) { screen_choice = AUTOMATIC; });
    lv_obj_set_size(back, 72, 34);
    lv_obj_align(back, LV_ALIGN_TOP_RIGHT, -6, 6);
    build_hud_screen();
    build_checks_screen();
    build_picker_screen();
  }

  static int last_mode = -1;
  Status s = read_status();

  if (s.mode != last_mode) {
    screen_choice = AUTOMATIC;  // New match period, go back to the default screen
    last_mode = s.mode;
  }

  lv_obj_t *want;
  if (chassis.pid_tuner_enabled())
    want = selector_screen;  // The PID tuner draws on EZ-Template's screen
  else if (screen_choice == SHOW_HUD)
    want = hud_screen;
  else if (screen_choice == SHOW_SELECTOR)
    want = selector_screen;
  else if (screen_choice == SHOW_CHECKS)
    want = checks_screen;
  else if (screen_choice == SHOW_PICKER)
    want = picker_screen;
  else
    want = s.mode == DISABLED ? checks_screen : hud_screen;  // Before the match: the pre-match check

  if (lv_scr_act() != want) {
    if (want == picker_screen) open_picker();  // Fresh tiles, opened to the selected auton's category
    lv_scr_load(want);
  }
  if (want == hud_screen) update_hud(s);
  if (want == checks_screen) update_checks(s);
}

}  // namespace

void driver_timer_start() {
  current_mode = DRIVER;
  mode_start_ms = pros::millis();
}

void initialize() {
  track_mode();
  lv_timer_create(brain_tick, 100, nullptr);  // Builds and updates the brain screen

  pros::Task controller(controller_task, "HUD controller");
}

}  // namespace hud

#endif  // __arm__
