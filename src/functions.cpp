#include "main.h"
#include "robot.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

Robot::Robot() 
    :
    // drive groups (match existing ports from main.cpp)
    front_left({17, -15}),
    front_right({13, -16}),
    back_left({-3, 7}),
    back_right({5, -6}),

    // EZ-Template chassis (match existing ports/imu/wheel spec)
    chassis({15, -17, -3, 7}, {-13, 16, 5, -6}, 14, 4.125, 343),

    intake_motor(2),
    intake_center_motor(11),
    intake_score_motor(1), //score indicates score side
    intake_center_score_motor(12),

    top_roller(5),

    hood('a', false, false),
    scraper('b', false, false),

    // Sensors
    color_sensor_front(13),
    color_sensor_back(15),
    imu(14)
{

    // Initialize color sensors
    color_sensor_front.set_led_pwm(100);
    color_sensor_back.set_led_pwm(100);
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
    //extend
    switch(hood.is_extended()) {
        case false:
            hood.extend();
            break;

        case true:
            break;
    }
}

void Robot::scraper_control(bool extended) {
    // Extends the scraper on a toggle
    switch(extended){
        case true:
            scraper.extend();
            break;
            
        case false:
            scraper.retract(); 
            break;
    }
}

int Robot::change_tier(int tier){
    // Changes what tier the score system is using
    tier++;

    if(tier > 2){
        tier = 0;
    }

    pros::lcd::print(3, "Tier Selected: %d", tier);

    return tier;
}


// Sensing

pros::Color Robot::get_color(pros::Optical sensor) {
    //TODO
    double hue = sensor.get_hue();
    double saturation = sensor.get_saturation();
    double brightness = sensor.get_brightness();

    double red_saturation_threshold = 0.4;
    double blue_saturation_threshold = 0.3;
    
    // Color reference: https://kb.vex.com/hc/en-us/articles/360051005291-Using-the-Optical-Sensor-with-VEX-V5
    // Red
    if ((hue > 330 || hue < 30) && saturation > red_saturation_threshold) { // 330-30 is the range of red
        return pros::Color::red;
    }
    // Blue
    else if ((hue > 150 && hue < 270) && saturation > blue_saturation_threshold) { // 150-270 is the range of blue
        return pros::Color::blue;
    }
    // Undefined
    else {
        return pros::Color::black;
    }
}

// Mecanum helpers
void Robot::drive_mecanum_init() {
    front_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    front_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void Robot::drive_mecanum_set(double drive, double strafe, double turn) {
    double fl = drive + strafe + turn;
    double fr = drive - strafe - turn;
    double bl = drive - strafe + turn;
    double br = drive + strafe - turn;

    double max_power = std::max({std::abs(fl), std::abs(fr), std::abs(bl), std::abs(br)});
    if (max_power > 127) {
        fl = fl / max_power * 127;
        fr = fr / max_power * 127;
        bl = bl / max_power * 127;
        br = br / max_power * 127;
    }

    front_left.move(fl);
    front_right.move(fr);
    back_left.move(bl);
    back_right.move(br);
}