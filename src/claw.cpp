#include "claw.hpp"

#include "main.h"

namespace claw {

// TODO claw: replace the delays with the real motor / piston moves, for example
//   inline pros::adi::DigitalOut claw_piston('A');   (in subsystems.hpp)
//   claw_piston.set_value(true);
// The delays stand in for how long the real move takes, so auton timing stays close.

void grab() {
  pros::delay(250);
}

void release() {
  pros::delay(300);
}

}  // namespace claw
