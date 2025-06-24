#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern ez::Drive chassis;

// Mecanum drive motors - declare individual motors for precise control
extern pros::Motor front_left;
extern pros::Motor front_right;
extern pros::Motor back_left;
extern pros::Motor back_right;

// Controller for mecanum drive
extern pros::Controller master;

// Mecanum drive helper functions
void mecanum_drive_set(double drive, double strafe, double turn);
void mecanum_drive_init();

// Enhanced sensor integration for mecanum drive
// Uncomment and configure these based on your robot setup:
// extern pros::Imu mecanum_imu;  // For field-centric drive and drift correction
// extern pros::Distance front_distance;  // For collision avoidance
// extern pros::Distance back_distance;   // For collision avoidance
// extern pros::Optical color_sensor;     // For game element detection

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');