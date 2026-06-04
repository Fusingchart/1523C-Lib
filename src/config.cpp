//
// Created by Hanyu Zhang on 10/24/25.
//

#include "config.h"

/*
 *blocker up auto
 *l1 intake blocker down
 *r1 score long block up
 *l2 mid
 *r2 low
 *y blocker down toggle
 *down arrow matchloader
 *up arrow hook
 *idle
 **/

const int PROCESS_DELAY = 10;

const pros::controller_digital_e_t INTAKE_BUTTON = pros::E_CONTROLLER_DIGITAL_L1;
const pros::controller_digital_e_t SCORE_LONG_BUTTON = pros::E_CONTROLLER_DIGITAL_R1;
const pros::controller_digital_e_t SCORE_MID_BUTTON = pros::E_CONTROLLER_DIGITAL_L2;
const pros::controller_digital_e_t SCORE_LOW_BUTTON = pros::E_CONTROLLER_DIGITAL_R2;
const pros::controller_digital_e_t BLOCKER_BUTTON = pros::E_CONTROLLER_DIGITAL_Y;
const pros::controller_digital_e_t MATCHLOAD_BUTTON = pros::E_CONTROLLER_DIGITAL_DOWN;
const pros::controller_digital_e_t HOOK_BUTTONS[3] = {
    pros::E_CONTROLLER_DIGITAL_UP,
    pros::E_CONTROLLER_DIGITAL_LEFT,
    pros::E_CONTROLLER_DIGITAL_RIGHT
};

pros::IMU imu(1);

pros::Motor front_left_motor(5);
pros::Motor mid_left_motor(6);
pros::Motor back_left_motor(-7);

pros::Motor front_right_motor(-2);
pros::Motor mid_right_motor(3);
pros::Motor back_right_motor(4);

pros::MotorGroup left_motor_group({5, -7, 6});
pros::MotorGroup right_motor_group({-2, 3, -4});

pros::Distance front_dist_sensor(19);
pros::Distance left_dist_sensor(20);
pros::Distance right_dist_sensor(17);

pros::Motor first_stage_motor(-15);
pros::Motor second_stage_motor(18);
pros::Motor third_stage_motor(-11);

pros::adi::DigitalOut blocker_piston('b');
pros::adi::DigitalOut matchload_piston('a');
pros::adi::DigitalOut hook_piston('c');

extern int deadzone = 5;
extern float drive_expo = 1.019;
extern float turn_expo = 1;
