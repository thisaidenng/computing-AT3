#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

// Claw — 5.5W motor, spins left to open, right to close.
// Reversed (-14) so that positive = open.
inline pros::Motor claw(-14);

// Defined in main.cpp, shared with autons
void clawOpen();
void clawClose();
void armMoveTo(int firstTarget, int secondTarget, int timeoutMs);
extern const int FIRST_DROP;
extern const int S2_RETRACT;