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
    if (auto *w = current_warning(s)) return w->text;
    return "AUTO:" + s.auton;
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

    // Rumbles in driver control: "-" at 0:30, "- -" at 0:15, "..." at 0:05, "." for each new problem
    std::string rumble;
    int sec = (s.ms_left + 999) / 1000;
    if (s.mode == DRIVER && s.mode == last_mode && sec != last_sec) {
      if (sec == 30) rumble = "-";
      if (sec == 15) rumble = "- -";
      if (sec == 5) rumble = "...";
    }
    std::set<std::string> now;
    for (auto &w : s.warnings) {
      now.insert(w.key);
      if (!seen_warnings.count(w.key) && s.mode == DRIVER && rumble.empty()) rumble = ".";
    }
    seen_warnings = now;
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
enum ScreenChoice { AUTOMATIC, SHOW_HUD, SHOW_SELECTOR };
std::atomic<int> screen_choice{AUTOMATIC};

lv_obj_t *hud_screen = nullptr;
lv_obj_t *selector_screen = nullptr;  // EZ-Template's auton selector (LLEMU)

lv_obj_t *mode_pill, *mode_label, *timer_label, *subtitle, *warning_pill, *warning_label;
lv_obj_t *battery_label, *battery_bar, *controller_label, *imu_label;
lv_obj_t *tiles[MAX_TILES], *tile_names[MAX_TILES], *tile_temps[MAX_TILES];
lv_obj_t *auton_label;

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

lv_obj_t *button(lv_obj_t *parent, const char *str, lv_event_cb_t on_click) {
  lv_obj_t *b = lv_btn_create(parent);
  lv_obj_set_style_bg_color(b, PANEL, 0);
  lv_obj_set_style_bg_color(b, OFF, LV_STATE_PRESSED);
  lv_obj_set_style_border_color(b, CYAN, 0);
  lv_obj_set_style_border_width(b, 2, 0);
  lv_obj_set_style_radius(b, 8, 0);
  lv_obj_set_style_shadow_width(b, 0, 0);
  lv_obj_add_event_cb(b, on_click, LV_EVENT_CLICKED, nullptr);
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
  lv_obj_set_width(auton_label, 300);
  lv_label_set_long_mode(auton_label, LV_LABEL_LONG_DOT);
  lv_obj_t *pick = button(auton, "PICK AUTON", [](lv_event_t *) { screen_choice = SHOW_SELECTOR; });
  lv_obj_set_size(pick, 130, 38);
  lv_obj_align(pick, LV_ALIGN_RIGHT_MID, -7, 0);
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
}

// Runs inside LVGL's own task, so it's safe to touch the screen here
void brain_tick(lv_timer_t *) {
  static int last_mode = -1;
  Status s = read_status();

  if (s.mode != last_mode) {
    screen_choice = AUTOMATIC;  // New match period, go back to the default screen
    last_mode = s.mode;
  }

  bool want_hud;
  if (chassis.pid_tuner_enabled())
    want_hud = false;  // The PID tuner draws on EZ-Template's screen
  else if (screen_choice == SHOW_HUD)
    want_hud = true;
  else if (screen_choice == SHOW_SELECTOR)
    want_hud = false;
  else
    want_hud = s.mode != DISABLED;  // Before the match, show the auton selector

  lv_obj_t *want = want_hud ? hud_screen : selector_screen;
  if (lv_scr_act() != want) lv_scr_load(want);
  if (want_hud) update_hud(s);
}

}  // namespace

void driver_timer_start() {
  current_mode = DRIVER;
  mode_start_ms = pros::millis();
}

void initialize() {
  // EZ-Template's auton selector is on the active screen, keep it so we can switch back
  selector_screen = lv_scr_act();
  lv_obj_t *back = button(selector_screen, "HUD", [](lv_event_t *) { screen_choice = SHOW_HUD; });
  lv_obj_set_size(back, 72, 34);
  lv_obj_align(back, LV_ALIGN_TOP_RIGHT, -6, 6);

  build_hud_screen();
  track_mode();
  lv_timer_create(brain_tick, 100, nullptr);

  pros::Task controller(controller_task, "HUD controller");
}

}  // namespace hud

#endif  // __arm__
