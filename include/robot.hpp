//#ifndef ROBOT_HPP
//#define ROBOT_HPP

#include "api.h"
#include <iostream>


class Robot {
    public:
        Robot();

        //motors
        pros::Motor intake_motor;
        pros::Motor intake_center_motor;
        pros::Motor intake_score_motor; //score indicates score side
        pros::Motor intake_center_score_motor;

        pros::Motor top_roller;

        //pneumatics
        pros::adi::Pneumatics hood;

        //sensors

        //init tasks
        void pop_hood();

        //sensing
        void color_sense();

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


//#endif
