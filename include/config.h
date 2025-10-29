//
// Created by Hanyu Zhang on 10/24/25.
//

#ifndef CONFIG_H
#define CONFIG_H

#include "main.h"
#include "lemlib/api.hpp"

extern const int PROCESS_DELAY;

extern const pros::controller_digital_e_t INTAKE_BUTTON;
extern const pros::controller_digital_e_t SCORE_LONG_BUTTON;
extern const pros::controller_digital_e_t SCORE_MID_BUTTON;
extern const pros::controller_digital_e_t SCORE_LOW_BUTTON;
extern const pros::controller_digital_e_t BLOCKER_BUTTON;
extern const pros::controller_digital_e_t MATCHLOAD_BUTTON;
extern const pros::controller_digital_e_t HOOK_BUTTONS[3];

extern const pros::controller_digital_e_t AUTO_SCORE_LOW_BUTTON;
extern const pros::controller_digital_e_t AUTO_SCORE_MID_BUTTON;

extern pros::IMU imu;

extern pros::Motor front_left_motor;
extern pros::Motor mid_left_motor;
extern pros::Motor back_left_motor;

extern pros::Motor front_right_motor;
extern pros::Motor mid_right_motor;
extern pros::Motor back_right_motor;

extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

extern pros::Distance front_dist_sensor;
extern pros::Distance left_dist_sensor;
extern pros::Distance right_dist_sensor;

extern pros::Motor first_stage_motor;
extern pros::Motor second_stage_motor;
extern pros::Motor third_stage_motor;

extern pros::adi::DigitalOut blocker_piston;
extern pros::adi::DigitalOut matchload_piston;
extern pros::adi::DigitalOut hook_piston;

extern lemlib::Drivetrain drivetrain;

extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;

extern lemlib::Chassis chassis;

#endif //CONFIG_H
