#include "main.h"
#include "string"

/////
// EZ-Template docs: https://ez-robotics.github.io/EZ-Template/
/////

// ---- Ports ----
//  Left drive  : 11, -4, -3
//  Right drive : -8, 5, 20
//  IMU         : 1
//  Tracker     : 16
//
//  Arm rotation: 7
//  Arm motors  : -10, 9
//  Claw motor  : 14    <-- MOVED OFF PORT 20 (was conflicting with right drive)

ez::Drive chassis(
    {11, -4, -3},
    {-8, 5, 20},
    1,             // IMU port
    2.75,
    450.0);

ez::tracking_wheel vert_tracker(16, 2.75, 4.0);

void initialize() {
  ez::ez_template_print();
  pros::delay(500);

  chassis.odom_tracker_left_set(&vert_tracker);

  chassis.opcontrol_curve_buttons_toggle(true);
  chassis.opcontrol_drive_activebrake_set(0.0);
  chassis.opcontrol_curve_default_set(0.0, 0.0);

  default_constants();
  armClawSetup();

  ez::as::auton_selector.autons_add({
      {"Auto Skills\n\nFull skills routine", autoSkills},
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

  chassis.initialize();
  ez::as::initialize();
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  ez::as::auton_selector.selected_auton_call();
}

void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());
  }
  ez::screen_print(tracker_value + tracker_width, line);
}

void ez_screen_task() {
  while (true) {
    if (!pros::competition::is_connected()) {
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        if (ez::as::page_blank_is_on(0)) {
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);
          screen_print_tracker(chassis.odom_tracker_left,  "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back,  "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    } else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    chassis.opcontrol_tank();

    // L1/L2 = arm positions, R1/R2 = claw open/close
    armClawTeleop();

    // DOWN = run whichever auton is selected in the LLEMU menu
    if (master.get_digital_new_press(DIGITAL_DOWN)) {
      autonomous();
      chassis.drive_brake_set(MOTOR_BRAKE_COAST);
      armClawTeleopReset();
    }

    // LEFT = run autoSkills directly (ignores the LLEMU selection)
    if (master.get_digital_new_press(DIGITAL_LEFT)) {
      autoSkills();
      chassis.drive_brake_set(MOTOR_BRAKE_COAST);
      armClawTeleopReset();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}