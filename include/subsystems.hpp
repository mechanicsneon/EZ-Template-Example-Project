#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::MotorGroup arm({-18, 6});
// inline pros::adi::DigitalIn limit_switch('A');