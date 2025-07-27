#include "main.h"
#include "robot.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

Robot::Robot() 
    :
    intake_motor(2),
    intake_center_motor(11),
    intake_score_motor(1), //score indicates score side
    intake_center_score_motor(12),

    top_roller(5),

    hood('a', false, false)

{}

void color_sense() {

}

void Robot::intake_alliance(double speed) {
    //all move towards the center basket
    //TODO match directions
    intake_motor.move_velocity(speed);
    intake_center_motor.move_velocity(speed);
    intake_score_motor.move_velocity(speed);
    intake_center_score_motor.move_velocity(speed);
}

void Robot::intake_opposing(double speed) {
    //all move towards the back basket
    //TODO match directions
    intake_motor.move_velocity(speed);
    intake_center_motor.move_velocity(speed);
    intake_score_motor.move_velocity(speed);
    intake_center_score_motor.move_velocity(speed);
}

void Robot::extake(double speed) {
    //draw from center basket to score side
    //TODO match directions & determine speed differential between the center rollers
    intake_motor.move_velocity(speed);
    intake_center_motor.move_velocity(speed);
    intake_score_motor.move_velocity(speed);
    intake_center_score_motor.move_velocity(speed);
}

void Robot::score_teir2 (double speed) {
    //draw from the center basket and feed up to the second level
    //TODO match directions & determine speed differential & add last roller
    intake_motor.move_velocity(speed);
    intake_center_motor.move_velocity(speed);
    intake_score_motor.move_velocity(speed);
    intake_center_score_motor.move_velocity(speed);
}

void Robot::score_teir3(double speed) {
    //draw from center basket and feed up to the third level
    //TODO match directions & determine speed differential & add last roller
    intake_motor.move_velocity(speed);
    intake_center_motor.move_velocity(speed);
    intake_score_motor.move_velocity(speed);
    intake_center_score_motor.move_velocity(speed);
}

void Robot::pop_hood() {

}