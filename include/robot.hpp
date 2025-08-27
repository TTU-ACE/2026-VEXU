#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "api.h"
#include "EZ-Template/api.hpp"
#include <iostream>


class Robot {
    public:
        Robot();

        // mecanum drive groups and controller
        pros::MotorGroup front_left;   // {17, -15}
        pros::MotorGroup front_right;  // {13, -16}
        pros::MotorGroup back_left;    // {-3, 7}
        pros::MotorGroup back_right;   // {5, -6}
        pros::Controller master{pros::E_CONTROLLER_MASTER};

        // EZ-Template chassis (kept as a member for composition)
        ez::Drive chassis;

        // mechanum helpers
        void drive_mecanum_init();
        void drive_mecanum_set(double drive, double strafe, double turn);

        //motors
        pros::Motor intake_motor;
        pros::Motor intake_center_motor;
        pros::Motor intake_score_motor; //score indicates score side
        pros::Motor intake_center_score_motor;

        pros::Motor top_roller;

        //pneumatics
        pros::adi::Pneumatics hood;

        //sensors
        pros::Optical color_sensor_front;
        pros::Optical color_sensor_back;
        pros::Imu imu; // main IMU used for field-centric etc.

        //init tasks
        void pop_hood();

        //sensing
        pros::Color get_color(pros::Optical sensor);

        //intaking
        void intake_alliance(double speed);
        void intake_opposing(double speed);

        //scoring
        void extake(double speed);
        void score_teir2 (double speed); 
        void score_teir3(double speed);

        //future
        //function to grab from pipe thing

    private:

        bool hoodPopped; //is the hood up
};


#endif
