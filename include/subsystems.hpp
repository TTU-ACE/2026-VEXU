#ifndef SUBSYSTEMS_HPP
#define SYBSYSTEMS_HPP
#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern ez::Drive chassis;

// Mecanum drive motors - declare individual motors for precise control
extern pros::MotorGroup front_left;
extern pros::MotorGroup front_right;
extern pros::MotorGroup back_left;
extern pros::MotorGroup back_right;

// Controller for mecanum drive
extern pros::Controller master;

// Mecanum drive helper functions
void mecanum_drive_set(double drive, double strafe, double turn);
void mecanum_drive_init();

// Enhanced sensor integration for mecanum drive
// Uncomment and configure these based on your robot setup:
extern pros::Imu mecanum_imu;  // For field-centric drive and drift correction
// extern pros::Distance front_distance;  // For collision avoidance
// extern pros::Distance back_distance;   // For collision avoidance
// extern pros::Optical color_sensor;     // For game element detection

// Your motors, sensors, etc. should go here.  Below are examples

// extern pros::Motor intake_motor;
// extern pros::Motor intake_center_motor;
// extern pros::Motor intake_score_motor; //score indicates score side
// extern pros::Motor intake_center_score_motor;

// pros::Motor top_score_motor;

// pros::adi::Pneumatics hood;

// inline pros::Motor intake(1);S
// inline pros::adi::DigitalIn limit_switch('A');

#endif