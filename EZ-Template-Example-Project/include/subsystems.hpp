#pragma once

// Arm and claw hardware + helpers live in autons.cpp.
// These are the pieces teleop (main.cpp) uses.
void armClawSetup();        // sensor/brake setup — zeroes the arm where it is
void armClawTeleop();       // L1/L2 arm positions, R1/R2 claw — call every opcontrol loop
void armClawTeleopReset();  // call after running an auton from opcontrol
