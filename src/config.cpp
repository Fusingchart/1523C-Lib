//
// Created by Hanyu Zhang on 10/24/25.
//

/*
* =============================================================================
*                                   IMPORTANT
* =============================================================================
*
* Please check "IMPORTANT.txt" in the root of the project for config
* requirements and other information.
*
*/

#include "config.h"

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

const pros::controller_digital_e_t AUTO_SCORE_LOW_BUTTON = pros::E_CONTROLLER_DIGITAL_X;
const pros::controller_digital_e_t AUTO_SCORE_MID_BUTTON = pros::E_CONTROLLER_DIGITAL_B;

pros::IMU imu(1);

pros::Motor front_right_motor(5);
pros::Motor mid_right_motor(6);
pros::Motor back_right_motor(-7);

pros::Motor front_left_motor(-2);
pros::Motor mid_left_motor(3);
pros::Motor back_left_motor(4);

pros::MotorGroup right_motor_group({5, -7, 6});
pros::MotorGroup left_motor_group({-2, 3, -4});

pros::Distance front_dist1_sensor(10);
pros::Distance front_dist2_sensor(19);
pros::Distance left_dist_sensor(20);
pros::Distance right_dist_sensor(17);

Point front_dist1_sensor_offset(-3.85, 6.5799212598);
Point front_dist2_sensor_offset(4.5, -1.525);
Point left_dist_sensor_offset(-4.25, 3.25);
Point right_dist_sensor_offset(4.25, 3.25);

pros::Motor first_stage_motor(-15);
pros::Motor second_stage_motor(18);
pros::Motor third_stage_motor(-11);

pros::adi::DigitalOut blocker_piston('b');
pros::adi::DigitalOut matchload_piston('a');
pros::adi::DigitalOut hook_piston('c');

lemlib::Drivetrain drivetrain(
    &left_motor_group, // left motor group
    &right_motor_group, // right motor group
    10, // 10 inch track width
    lemlib::Omniwheel::NEW_4, // using new 4" omnis
    343, // drivetrain rpm is 360
    2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1, set to nullptr
    nullptr, // vertical tracking wheel 2, set to nullptr
    nullptr, // horizontal tracking wheel 1, set to nullptr
    nullptr, // horizontal tracking wheel 2, set to nullptr
    &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    5, // proportional gain (kP)
    0, // integral gain (kI)
    0, // derivative gain (kD)
    3, // anti windup
    0.5, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
    0.9, // proportional gain (kP)
    0.03, // integral gain (kI)
    1, // derivative gain (kD)
    1, // anti windup
    0.5, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(
    5, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(
    5, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(
    drivetrain, // drivetrain settings
    lateral_controller, // lateral PID settings
    angular_controller, // angular PID settings
    sensors, // odometry sensors
    &throttle_curve,
    &steer_curve
);