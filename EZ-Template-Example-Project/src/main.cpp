#include "main.h"
#include "string"
#include "pros/motor_group.hpp"
#include "autons.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    {11, -4, -3},
    {-8, 5, 20},
    6,             // ← Add this line with your IMU port
    4.125,
    450.0);  // Wheel RPM = cartridge * (motor gear / wheel gear)
  ez::tracking_wheel vert_tracker(16, 2.75, 4.0);
  //ez::tracking_wheel horiz_tracker(2, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels



// ##########################################################################
// ______________________________ARM CONSTANTS_______________________________
// ##########################################################################

// ---- First stage PID (rotation sensor on port 7) ----
double ARM_KP = 2;
double ARM_KI = 0.0;
double ARM_KD = 3.0;
double ARM_TOL = 30.0;   // centidegrees — 30 = 0.3 real deg

// ---- Second stage PID (built-in encoder on port 12) ----
double S2_KP = 1.5;
double S2_KI = 0.0;
double S2_KD = 0.5;
double S2_TOL = 8.0;     // motor degrees

// ---- PID state ----
double arm_integral = 0;
double arm_prev_error = 0;
double arm_last_target = 0;

double s2_integral = 0;
double s2_prev_error = 0;
double s2_last_target = 0;

// ---- Last computed outputs (for debug) ----
double arm_out = 0;
double s2_out  = 0;

// ---- First-stage targets (in ROTATION SENSOR centidegrees) ----
const int FIRST_UP   = 296;
const int FIRST_DROP = 248;

// ---- Second-stage targets (built-in motor encoder degrees) ----
const int S2_DEPLOY   = 300;   // state 1
const int S2_RETRACT  = 0;     // state 2
const int S2_STATE4   = 200;   // state 3

// #########################################################################
// ______________________________ARM CONTROL______________________________
// #########################################################################

// Stage 2 state (L1 cycles): 0 = idle, 1, 2, 3
int      armState     = 0;
bool     armActive    = false;

// Stage 1 manual speed (out of 127, full speed) — UP / DOWN arrows
const int ARM_MANUAL_SPEED = 127;

pros::Rotation   armRot(7);
pros::MotorGroup armMotors({10});
pros::Motor      secondStageMotor(12);

void updateArm() {
  if (master.get_digital_new_press(DIGITAL_L1)) {
    armState = (armState + 1) % 4;
    arm_last_target = -99999;
    s2_last_target  = -99999;
    armActive = true;
  }
}

// -------- First stage PID --------
void moveFirstStage(int targetDeg) {
    int raw = armRot.get_position();
    if (raw == 2147483647) {
        armMotors.move(0);
        arm_out = 0;
        return;
    }

    double error = targetDeg - raw;

    if (std::fabs(targetDeg - arm_last_target) > 0.5) {
        arm_integral = 0;
        arm_prev_error = 0;
        arm_last_target = targetDeg;
    }

    if (std::fabs(error) < ARM_TOL) {
        armMotors.move(0);
        arm_out = 0;
        return;
    }

    arm_integral += error;
    double derivative = error - arm_prev_error;
    arm_prev_error = error;

    double out = ARM_KP * error + ARM_KI * arm_integral + ARM_KD * derivative;

    if (out >  127) out =  127;
    if (out < -127) out = -127;

    arm_out = out;
    armMotors.move((int)out);
}

bool firstStageAt(int targetDeg) {
    return std::fabs(armRot.get_position() - targetDeg) < ARM_TOL;
}

// -------- Second stage PID --------
void moveSecondStage(int targetDeg) {
    double currentDeg = secondStageMotor.get_position();
    double error = targetDeg - currentDeg;

    if (std::fabs(targetDeg - s2_last_target) > 0.5) {
        s2_integral = 0;
        s2_prev_error = 0;
        s2_last_target = targetDeg;
    }

    if (std::fabs(error) < S2_TOL) {
        secondStageMotor.move(0);
        s2_out = 0;
        return;
    }

    s2_integral += error;
    double derivative = error - s2_prev_error;
    s2_prev_error = error;

    double out = S2_KP * error + S2_KI * s2_integral + S2_KD * derivative;

    if (out >  127) out =  127;
    if (out < -127) out = -127;

    s2_out = out;
    secondStageMotor.move((int)out);
}

bool secondStageAt(int targetDeg) {
    return std::fabs(secondStageMotor.get_position() - targetDeg) < S2_TOL;
}

// Runs every loop. Stage 2 only — stage 1 is manual (see runFirstStageManual).
//   state 0: second → 0
//   state 1: second → S2_DEPLOY
//   state 2: second → S2_RETRACT
//   state 3: second → S2_STATE4
void runArm() {
    switch (armState) {
        case 0: moveSecondStage(0);          break;
        case 1: moveSecondStage(S2_DEPLOY);  break;
        case 2: moveSecondStage(S2_RETRACT); break;
        case 3: moveSecondStage(S2_STATE4);  break;
    }
}

// Stage 1 manual: hold UP to raise, DOWN to lower, release to hold position (brake HOLD)
void runFirstStageManual() {
    if (master.get_digital(DIGITAL_UP)) {
        arm_out = ARM_MANUAL_SPEED;
    } else if (master.get_digital(DIGITAL_DOWN)) {
        arm_out = -ARM_MANUAL_SPEED;
    } else {
        arm_out = 0;
    }
    armMotors.move((int)arm_out);
}

// #########################################################################
// ______________________________CLAW CONTROL______________________________
// #########################################################################

// Claw encoder is zeroed at startup, so the claw must be CLOSED when the robot boots.
const int CLAW_CLOSED = 0;     // motor degrees
const int CLAW_OPEN   = 180;   // motor degrees (spinning left opens; motor is reversed)
const int CLAW_SPEED  = 150;   // rpm (5.5W motor maxes out at 200)

void clawOpen()  { claw.move_absolute(CLAW_OPEN,   CLAW_SPEED); }
void clawClose() { claw.move_absolute(CLAW_CLOSED, CLAW_SPEED); }

// R1 = open, R2 = close
void updateClaw() {
  if (master.get_digital_new_press(DIGITAL_R1)) clawOpen();
  if (master.get_digital_new_press(DIGITAL_R2)) clawClose();
}

// Blocking arm move for autons — runs both stage PIDs until both are on target or timeoutMs passes
void armMoveTo(int firstTarget, int secondTarget, int timeoutMs) {
  int elapsed = 0;
  while (elapsed < timeoutMs) {
    moveFirstStage(firstTarget);
    moveSecondStage(secondTarget);
    if (firstStageAt(firstTarget) && secondStageAt(secondTarget)) break;
    pros::delay(ez::util::DELAY_TIME);
    elapsed += ez::util::DELAY_TIME;
  }
  armMotors.brake();
  secondStageMotor.brake();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void moveFirstStage(int targetPosition);

void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);
  chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable. 
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // reversing arm rotation sensor so that it reads positive when the arm is going up and negative when the arm is going down
  armRot.set_reversed(false);
  armRot.reset_position();
  armMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  secondStageMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  claw.tare_position();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Place and Back\n\nDrive 30cm, lower arm, open claw, back 30cm", place_and_back},
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  // Always run Place and Back (selector bypassed). To use the brain-screen selector again,
  // swap this back to: ez::as::auton_selector.selected_auton_call();
  place_and_back();
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the VEX Competition Switch in the operator control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  armState = 0;
  armActive = false;

  while (true) {
    //ez_template_extras();
    chassis.opcontrol_tank();

    // ---- Debug: everything on the brain screen ----
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "arm:%d s2:%d",
        armRot.get_position(), (int)secondStageMotor.get_position());
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "st:%d",
        armState);
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "1out:%d  2out:%d",
        (int)arm_out, (int)s2_out);

    // Panic stop — resets everything to state 0
    if (master.get_digital(DIGITAL_B)) {
      armMotors.move(0);
      armMotors.brake();
      secondStageMotor.move(0);
      secondStageMotor.brake();
      claw.brake();
      armState = 0;
      armActive = false;
      continue;
    }

    updateArm();
    runArm();
    runFirstStageManual();
    updateClaw();

    pros::delay(ez::util::DELAY_TIME);
  }
}