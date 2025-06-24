#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor - keeping original for autonomous usage
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {1, 2 },     // Left Chassis Ports (negative port will reverse it!)
    {-3, -4},    // Right Chassis Ports (negative port will reverse it!)

    7,      // IMU Port
    4.125,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    343);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Mecanum drive motors - individual control for each wheel
pros::Motor front_left(1);     // Front left motor port
pros::Motor front_right(2);    // Front right motor port  
pros::Motor back_left(3);      // Back left motor port
pros::Motor back_right(4);     // Back right motor port

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

/**
 * Helper function to initialize mecanum drive motors
 */
void mecanum_drive_init() {
  // Set brake modes
  front_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  front_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  back_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  back_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  // Configure motor directions for mecanum drive
  // Adjust these based on your robot's motor orientation
  front_right.set_reversed(true);  // Typically reverse right side
  back_right.set_reversed(true);   // Typically reverse right side
  
  // Optional: Set motor gear ratio if using different cartridges
  // front_left.set_gearing(pros::E_MOTOR_GEARSET_18);  // Example: 200 RPM cartridge
}

/**
 * Helper function to set mecanum drive powers with normalization
 * 
 * @param drive Forward/backward movement (-127 to 127)
 * @param strafe Left/right movement (-127 to 127) 
 * @param turn Rotational movement (-127 to 127)
 */
void mecanum_drive_set(double drive, double strafe, double turn) {
  // Calculate mecanum wheel powers using standard mecanum math
  double front_left_power = drive + strafe + turn;
  double front_right_power = drive - strafe - turn;
  double back_left_power = drive - strafe + turn;
  double back_right_power = drive + strafe - turn;

  // Find the maximum absolute value to normalize if needed
  double max_power = std::max({
      std::abs(front_left_power),
      std::abs(front_right_power),
      std::abs(back_left_power),
      std::abs(back_right_power)
  });

  // Normalize powers if any exceed 127 (VEX motor range)
  if (max_power > 127) {
      front_left_power = (front_left_power / max_power) * 127;
      front_right_power = (front_right_power / max_power) * 127;
      back_left_power = (back_left_power / max_power) * 127;
      back_right_power = (back_right_power / max_power) * 127;
  }

  // Set motor velocities
  front_left.move(front_left_power);
  front_right.move(front_right_power);
  back_left.move(back_left_power);
  back_right.move(back_right_power);
}

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
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

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
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
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
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

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
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
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
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
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // Initialize mecanum drive motors
  mecanum_drive_init();
  
  // Optional: Initialize additional sensors for enhanced functionality
  // pros::Imu field_imu(7);  // Use IMU for field-centric drive
  // bool field_centric = false;  // Toggle for field-centric vs robot-centric

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    // Mecanum Drive Implementation
    // Get controller inputs (-127 to 127 range)
    double drive = -master.get_analog(ANALOG_LEFT_Y);    // Forward/backward (negative for correct direction)
    double strafe = master.get_analog(ANALOG_LEFT_X);    // Left/right strafe
    double turn = master.get_analog(ANALOG_RIGHT_X);     // Rotation

    // Apply deadzone to prevent drift
    int deadzone = 10;
    if (abs(drive) < deadzone) drive = 0;
    if (abs(strafe) < deadzone) strafe = 0;
    if (abs(turn) < deadzone) turn = 0;

    // Scale inputs for better control (adjust these values as needed)
    double drive_scale = 0.8;   // Reduce max speed if needed
    double strafe_scale = 0.8;  // Strafe scaling
    double turn_scale = 0.6;    // Typically want turning to be slower
    
    drive *= drive_scale;
    strafe *= strafe_scale;
    turn *= turn_scale;

    // Optional: Field-centric drive using IMU
    // if (field_centric && chassis.drive_imu_calibrated()) {
    //   double robot_angle = chassis.drive_imu_get() * M_PI / 180.0;  // Convert to radians
    //   double temp = drive * cos(robot_angle) + strafe * sin(robot_angle);
    //   strafe = -drive * sin(robot_angle) + strafe * cos(robot_angle);
    //   drive = temp;
    // }

    // Use the helper function to set mecanum drive powers
    mecanum_drive_set(drive, strafe, turn);

    // Control options with controller buttons
    // Toggle between tank drive and mecanum drive
    if (master.get_digital_new_press(DIGITAL_X)) {
        // Switch to EZ-Template tank drive for precise autonomous-like movements
        chassis.opcontrol_tank();
        pros::delay(20);  // Brief delay to prevent conflicts
        continue;  // Skip the rest of this loop iteration
    }

    // Optional: Toggle field-centric mode
    // if (master.get_digital_new_press(DIGITAL_Y)) {
    //     field_centric = !field_centric;
    //     master.rumble(".");  // Short rumble to confirm toggle
    // }

    // Emergency stop
    if (master.get_digital(DIGITAL_DOWN) && master.get_digital(DIGITAL_B)) {
        mecanum_drive_set(0, 0, 0);  // Stop all movement
    }

    // Optional: Display debug information on brain screen
    if (master.get_digital_new_press(DIGITAL_A)) {
        pros::lcd::print(0, "Mecanum Drive Active");
        pros::lcd::print(1, "Drive: %.0f Strafe: %.0f Turn: %.0f", drive, strafe, turn);
        pros::lcd::print(2, "Press X for Tank Drive");
    }

    // Motor temperature monitoring (safety feature)
    if (front_left.get_temperature() > 55 || front_right.get_temperature() > 55 ||
        back_left.get_temperature() > 55 || back_right.get_temperature() > 55) {
        master.rumble("---");  // Warning rumble if motors are getting hot
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
