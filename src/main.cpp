#include "main.h"

#include "config.h"

#include "syscontrol.h"

#include "lib/utils.hpp"
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    }
    else {
        pros::lcd::clear_line(2);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");

    init_syscontrol();

    pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
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
}

bool block_driver_movement = false;
bool block_driver_intake = false;
bool auto_scoring_low = false;

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
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true) {
        if (block_driver_intake) goto no_intake;
        if (master.get_digital(INTAKE_BUTTON)) {
            intake_state = IntakeState::Intake;
        }
        else if (master.get_digital(SCORE_LONG_BUTTON)) {
            intake_state = IntakeState::ScoreLong;
        }
        else if (master.get_digital(SCORE_MID_BUTTON)) {
            intake_state = IntakeState::ScoreMid;
        }
        else if (master.get_digital(SCORE_LOW_BUTTON)) {
            intake_state = IntakeState::ScoreLow;
        }
        else {
            intake_state = IntakeState::Idle;
        }
    no_intake:

        if (master.get_digital_new_press(BLOCKER_BUTTON)) {
            blocker_value = !blocker_value;
        }

        if (master.get_digital_new_press(MATCHLOAD_BUTTON)) {
            matchload_value = !matchload_value;
        }

        for (const auto button : HOOK_BUTTONS) {
            if (master.get_digital_new_press(button)) {
                hook_value = !hook_value;
                break;
            }
        }

        if (!auto_scoring_low && master.get_digital_new_press(AUTO_SCORE_LOW_BUTTON)) {
            block_driver_movement = true;
            block_driver_intake = true;
            auto_scoring_low = true;
            pros::Task([&master] {
                chassis.cancelAllMotions();
                chassis.arcade(-28, 0);
                pros::delay(250);
                chassis.arcade(0, 0);

                intake_state = IntakeState::ScoreLowSlow1;
                while (!master.get_digital_new_press(AUTO_SCORE_LOW_BUTTON)) {
                    pros::delay(PROCESS_DELAY);
                }

                intake_state = IntakeState::ScoreLowSlow2;
                while (!master.get_digital_new_press(AUTO_SCORE_LOW_BUTTON)) {
                    pros::delay(PROCESS_DELAY);
                }

                chassis.arcade(10, 0);
                pros::delay(250);
                chassis.arcade(0, 0);

                auto_scoring_low = false;
                block_driver_movement = false;
                block_driver_intake = false;
            });
        }

        update_syscontrol();

        const int drive = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int turn = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (!block_driver_movement) chassis.arcade(drive, turn);

        pros::delay(PROCESS_DELAY);
    }
}
