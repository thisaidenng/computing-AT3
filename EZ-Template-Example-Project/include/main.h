/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"
#include "EZ-Template/api.hpp"

// Project headers
#include "autons.hpp"
#include "subsystems.hpp"

// ---- Globals defined in main.cpp, shared with all files ----
extern ez::Drive          chassis;
extern ez::tracking_wheel vert_tracker;

using namespace okapi::literals;

#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#endif  // _PROS_MAIN_H_