#include "claw.hpp"

#include <atomic>

#include "main.h"
#include "robot_config.hpp"

namespace claw {
namespace {

std::atomic<bool> closed{false};
std::atomic<uint32_t> changed_ms{0};
bool tilted = false, flipped = false;

// Hardware is created on first use, so a claw type that isn't on the robot never touches its ports
pros::Motor &motor() {
  static pros::Motor m(cfg::CLAW_MOTOR_PORT);
  return m;
}
pros::adi::DigitalOut &piston() {
  static pros::adi::DigitalOut p(cfg::CLAW_PISTON_PORT);
  return p;
}
pros::adi::DigitalOut &piston2() {
  static pros::adi::DigitalOut p(cfg::CLAW_PISTON_2_PORT);
  return p;
}
pros::adi::DigitalOut &tilt_piston() {
  static pros::adi::DigitalOut p(cfg::TILT_PISTON_PORT);
  return p;
}
pros::adi::DigitalOut &flip_piston() {
  static pros::adi::DigitalOut p(cfg::FLIP_PISTON_PORT);
  return p;
}

// Start opening/closing. Pistons move right away; a motor claw is driven by claw_task
void set(bool close) {
  closed = close;
  changed_ms = pros::millis();
  if (cfg::CLAW_TYPE != cfg::ClawType::MOTOR) {
    bool on = close == cfg::CLAW_CLOSED_WHEN_ON;
    piston().set_value(on);
    if (cfg::CLAW_TYPE == cfg::ClawType::TWO_PISTONS) piston2().set_value(on);
  }
}

// Motor claw: full power while moving, then a light squeeze to keep holding
void claw_task() {
  while (true) {
    bool moving = pros::millis() - changed_ms < (uint32_t)cfg::CLAW_MOVE_MS;
    if (closed)
      motor().move(moving ? cfg::CLAW_POWER : cfg::CLAW_HOLD);
    else
      motor().move(moving ? -cfg::CLAW_POWER : 0);
    pros::delay(ez::util::DELAY_TIME);
  }
}

}  // namespace

void initialize() {
  if (cfg::CLAW_TYPE == cfg::ClawType::MOTOR) {
    motor().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::Task task(claw_task, "Claw");
  } else {
    set(false);  // Start open
  }
  if (cfg::TILT_PISTON_PORT) tilt_piston().set_value(false);
  if (cfg::FLIP_PISTON_PORT) flip_piston().set_value(false);
}

void opcontrol() {
  // B+DOWN runs the selected auton when not on a competition switch, so B alone only
  if (master.get_digital_new_press(DIGITAL_B) && !master.get_digital(DIGITAL_DOWN)) set(!closed);

  if (chassis.pid_tuner_enabled()) return;  // The PID tuner uses the arrows
  if (cfg::TILT_PISTON_PORT && master.get_digital_new_press(DIGITAL_LEFT)) {
    tilted = !tilted;
    tilt_piston().set_value(tilted);
  }
  if (cfg::FLIP_PISTON_PORT && master.get_digital_new_press(DIGITAL_RIGHT)) {
    flipped = !flipped;
    flip_piston().set_value(flipped);
  }
}

void grab() {
  set(true);
  pros::delay(cfg::CLAW_MOVE_MS);
}

void release() {
  set(false);
  pros::delay(cfg::CLAW_MOVE_MS);
}

bool holding() { return closed; }

}  // namespace claw
