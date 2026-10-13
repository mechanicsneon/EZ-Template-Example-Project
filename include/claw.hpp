#pragma once

// Claw that grabs, tilts and flips Pins / Cups.
// NOT WIRED YET: these are stand-ins so autons and the lift macro can already call them.
// Fill in claw.cpp once we know which motors / pistons it uses.
namespace claw {

void grab();     // Close on a Pin or Cup
void release();  // Let go of it (tilt forward / open, whatever places it)

}  // namespace claw
