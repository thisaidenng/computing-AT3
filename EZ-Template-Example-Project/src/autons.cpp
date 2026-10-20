#include "main.h"

/////
// EZ-Template docs: https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED  = 90;
const int SWING_SPEED = 110;

// ##########################################################################
// ______________________________HARDWARE____________________________________
// ##########################################################################

// Arm
pros::Rotation   armRot(7);
pros::MotorGroup armMotors({-10, 9});

// Claw — MOVED OFF PORT 8 (was conflicting with the right drive motor)
pros::Motor      clawMotor(14);

// Positive output = close. Flip if your claw is wired the other way.
const bool CLAW_POSITIVE_IS_CLOSE = true;

// ##########################################################################
// ______________________________CONSTANTS___________________________________
// ##########################################################################

// Arm targets (centidegrees; 100 = 1°)
const int ARM_DOWN  = 0;
const int ARM_POS_1 = 4000;
const int ARM_POS_2 = 9000;

// Arm PID
double ARM_KP = 1.5;
double ARM_KI = 0.0;
double ARM_KD = 0.3;
double ARM_TOL = 200.0;       // 2.0°
double ARM_FF       = 12.0;   // gravity assist while moving
double ARM_HOLD_PWR = 15.0;   // constant hold torque when parked
const int ARM_MIN_PWR = 15;

// Claw
const int CLAW_STALL_MA      = 2000;   // mA — tune to your mechanism
const int CLAW_STALL_TIME_MS = 100;    // how long current must stay high
const int CLAW_TIMEOUT_MS    = 1500;   // safety max runtime
const int CLAW_HOLD_PWR      = 20;     // hold torque after gripping
const int CLAW_DRIVE_MS      = 600;    // how long open/close drives at full power — tune this

// ##########################################################################
// ______________________________HELPERS_____________________________________
// ##########################################################################

// -------- Arm: move to a target and block until settled or timeout --------
void armMoveTo(int targetDeg, int timeout_ms = 3000) {
    double integral = 0;
    double prev_raw = armRot.get_position();
    uint32_t start  = pros::millis();

    while (true) {
        int raw = armRot.get_position();
        if (raw == 2147483647) break;   // sensor not connected

        double error = targetDeg - raw;

        if (std::fabs(error) < ARM_TOL) break;
        if ((int)(pros::millis() - start) > timeout_ms) break;

        integral += error;
        double derivative = -(raw - prev_raw);
        prev_raw = raw;

        double out = ARM_KP * error + ARM_KI * integral + ARM_KD * derivative;

        if (error > 0) out += ARM_FF;
        else           out -= ARM_FF;

        if (out > 0 && out < ARM_MIN_PWR) out = ARM_MIN_PWR;
        if (out < 0 && out > -ARM_MIN_PWR) out = -ARM_MIN_PWR;
        if (out >  127) out =  127;
        if (out < -127) out = -127;

        armMotors.move((int)out);
        pros::delay(10);
    }

    // Settled — hold with constant torque. ARM_DOWN releases.
    int hold = (targetDeg == ARM_DOWN) ? 0 : (int)ARM_HOLD_PWR;
    armMotors.move(hold);
}

// -------- Claw: drive until stalled or timed out, then hold --------
void clawClose() {
    int dir = CLAW_POSITIVE_IS_CLOSE ? 1 : -1;
    clawMotor.move(127 * dir);
    pros::delay(CLAW_DRIVE_MS);
    clawMotor.move(CLAW_HOLD_PWR * dir);
}

void clawOpen() {
    int dir = CLAW_POSITIVE_IS_CLOSE ? -1 : 1;
    clawMotor.move(127 * dir);
    pros::delay(CLAW_DRIVE_MS);
    clawMotor.move(CLAW_HOLD_PWR * dir);
}

// -------- Arm + claw setup (called from initialize and autoSkills) --------
void armClawSetup() {
    armRot.set_reversed(true);
    armRot.reset_position();
    armMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clawMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

// ##########################################################################
// ______________________________TELEOP CONTROLS_____________________________
// ##########################################################################
//
// Same arm PID and claw timing as the auton helpers above, but non-blocking:
// each call does one step and returns, so the driver can keep driving.
//
//   L1 = arm up one position    (ARM_DOWN -> ARM_POS_1 -> ARM_POS_2)
//   L2 = arm down one position
//   R1 = open claw
//   R2 = close claw

const int ARM_POSITIONS[]   = {ARM_DOWN, ARM_POS_1, ARM_POS_2};
const int ARM_NUM_POSITIONS = 3;

int      teleArmIndex    = 0;
bool     teleArmActive   = false;   // arm is left alone until the driver presses L1/L2
double   teleArmIntegral = 0;
double   teleArmPrevRaw  = 0;

bool     teleClawDriving = false;
int      teleClawDir     = 0;
uint32_t teleClawStart   = 0;

// Index of the arm position closest to where the arm is now
int armNearestIndex() {
    int raw = armRot.get_position();
    int best = 0;
    for (int i = 1; i < ARM_NUM_POSITIONS; i++) {
        if (std::abs(raw - ARM_POSITIONS[i]) < std::abs(raw - ARM_POSITIONS[best])) best = i;
    }
    return best;
}

// One step of the arm PID toward targetDeg
void armTeleopStep(int targetDeg) {
    int raw = armRot.get_position();
    if (raw == 2147483647) {   // sensor not connected
        armMotors.move(0);
        return;
    }

    double error = targetDeg - raw;

    // Settled — hold with constant torque. ARM_DOWN releases.
    if (std::fabs(error) < ARM_TOL) {
        armMotors.move((targetDeg == ARM_DOWN) ? 0 : (int)ARM_HOLD_PWR);
        teleArmPrevRaw = raw;
        return;
    }

    teleArmIntegral += error;
    double derivative = -(raw - teleArmPrevRaw);
    teleArmPrevRaw = raw;

    double out = ARM_KP * error + ARM_KI * teleArmIntegral + ARM_KD * derivative;

    if (error > 0) out += ARM_FF;
    else           out -= ARM_FF;

    if (out > 0 && out < ARM_MIN_PWR) out = ARM_MIN_PWR;
    if (out < 0 && out > -ARM_MIN_PWR) out = -ARM_MIN_PWR;
    if (out >  127) out =  127;
    if (out < -127) out = -127;

    armMotors.move((int)out);
}

void clawTeleopStart(int dir) {
    clawMotor.move(127 * dir);
    teleClawDir     = dir;
    teleClawStart   = pros::millis();
    teleClawDriving = true;
}

// Call every opcontrol loop
void armClawTeleop() {
    bool up   = master.get_digital_new_press(DIGITAL_L1);
    bool down = master.get_digital_new_press(DIGITAL_L2);

    if (up || down) {
        if (!teleArmActive) {
            teleArmIndex  = armNearestIndex();
            teleArmActive = true;
        }
        if (up   && teleArmIndex < ARM_NUM_POSITIONS - 1) teleArmIndex++;
        if (down && teleArmIndex > 0)                     teleArmIndex--;
        teleArmIntegral = 0;
        teleArmPrevRaw  = armRot.get_position();
    }

    if (teleArmActive) armTeleopStep(ARM_POSITIONS[teleArmIndex]);

    int closeDir = CLAW_POSITIVE_IS_CLOSE ? 1 : -1;
    if (master.get_digital_new_press(DIGITAL_R1)) clawTeleopStart(-closeDir);  // open
    if (master.get_digital_new_press(DIGITAL_R2)) clawTeleopStart(closeDir);   // close

    // After CLAW_DRIVE_MS at full power, drop to hold torque (same as clawOpen/clawClose)
    if (teleClawDriving && pros::millis() - teleClawStart > (uint32_t)CLAW_DRIVE_MS) {
        clawMotor.move(CLAW_HOLD_PWR * teleClawDir);
        teleClawDriving = false;
    }
}

// Call after running an auton from opcontrol, so teleop doesn't yank the arm back
void armClawTeleopReset() {
    teleArmActive   = false;
    teleClawDriving = false;
}


// ##########################################################################
// ______________________________CONSTANTS___________________________________
// ##########################################################################

void default_constants() {
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);

  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  chassis.odom_turn_bias_set(0.9);
  chassis.odom_look_ahead_set(7_in);
  chassis.odom_boomerang_distance_set(16_in);
  chassis.odom_boomerang_dlead_set(0.625);

  chassis.pid_angle_behavior_set(ez::shortest);
}

// ##########################################################################
// ______________________________AUTO SKILLS________________________________
// ##########################################################################

void autoSkills() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  armClawSetup();

  // ---- Sequence ----
  armMoveTo(ARM_DOWN); 
  pros::delay(1000);      // 1. arm down
  clawClose();               // 2. close claw
  pros::delay(1000);
  armMoveTo(ARM_POS_1);  
  pros::delay(2000);    // 3. RAISE ARM

  chassis.pid_drive_set(0.5_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  pros::delay(2000);   
  clawOpen();  
  pros::delay(1000);   // drop the object
  chassis.pid_drive_set(-2_in, DRIVE_SPEED, false); // drive backwards
  chassis.pid_wait();

  chassis.pid_turn_set(100_deg, TURN_SPEED); // turn towards next pin

  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true); // drive to next pin
  clawClose();  // grab pin
  pros::delay(1000);    // make sure the arm doesn't get dragged back

  armMoveTo(ARM_POS_2);      // raise arm more

  clawOpen();   // release pin



  // chassis.pid_turn_set(90_deg, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_drive_set(12_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // chassis.pid_drive_set(-6_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // clawOpen();                // release

  // armMoveTo(ARM_DOWN);       // back down
}

// ##########################################################################
// ______________________________EXAMPLES____________________________________
// ##########################################################################

void drive_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void turn_example() {
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

void wait_until_change_speed() {
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();
}

void swing_example() {
  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

void motion_chaining() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    } else {
      return;
    }
  }
}

void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  if (chassis.interfered) {
    tug(3);
    return;
  }
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

void odom_drive_example() {
  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void odom_pure_pursuit_example() {
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait();
}

void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);
  chassis.pid_wait();
}

void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait();
}

void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();
  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait();
}

void measure_offsets() {
  int iterations = 10;
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  if (chassis.odom_tracker_left  != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back  != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();

  for (int i = 0; i < iterations; i++) {
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;

    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));
    double l_delta = chassis.odom_tracker_left  != nullptr ? chassis.odom_tracker_left->get()  : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back  != nullptr ? chassis.odom_tracker_back->get()  : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  if (chassis.odom_tracker_left  != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back  != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

void clawTest() {
  clawMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // Drive positive for 2 seconds
  clawMotor.move(60);
  pros::delay(2000);
  clawMotor.move(0);
  pros::delay(500);

  // Drive negative for 2 seconds
  clawMotor.move(-60);
  pros::delay(2000);
  clawMotor.move(0);
}