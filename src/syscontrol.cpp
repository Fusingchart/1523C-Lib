//
// Created by Hanyu Zhang on 10/24/25.
//

#include "syscontrol.h"

IntakeState intake_state = IntakeState::Idle;

bool blocker_value = false;
bool auto_blocker_value = false;
bool matchload_value = false;
bool hook_value = false;

bool p_blocker_final_value = false;
bool blocker_final_value = false;
int blocker_toggle_cooldown = 0;
const int blocker_toggle_cooldown_duration = 200;

int first_stage_velocity = 0;
int second_stage_velocity = 0;
int third_stage_velocity = 0;

void init_syscontrol() {
}

JamState::JamState(pros::Motor* target, const uint32_t jam_tolerance, const uint32_t unjam_duration)
    : target(target), jam_tolerance(jam_tolerance), unjam_duration(unjam_duration) {
    jam_start = pros::millis();
}

void JamState::update() {
    if (target->get_target_velocity() == 0 || target->get_actual_velocity() != 0) {
        jam_start = pros::millis();
        std::cout << "not jamming\n";
    }

    if (pros::millis() - jam_start > jam_tolerance) {
        std::cout << "starting unjam\n";

        unjam_end = pros::millis() + unjam_duration;
    }
}

JamState second_state_jam_state(&second_stage_motor, 150, 50);

void set_intake_velocity_frac(const float first, const float second, const float third) {
    first_stage_velocity = static_cast<int>(first * 600);
    second_stage_velocity = static_cast<int>(second * 200);
    third_stage_velocity = static_cast<int>(third * 200);
}

void set_intake_velocity(const int first, const int second, const int third) {
    first_stage_velocity = first;
    second_stage_velocity = second;
    third_stage_velocity = third;
}

void update_syscontrol() {
    switch (intake_state) {
    case IntakeState::Intake:
        set_intake_velocity_frac(1, 1, 1);
        auto_blocker_value = true;
        break;
    case IntakeState::ScoreLong:
        set_intake_velocity_frac(1, 1, 1);
        auto_blocker_value = false;
        blocker_value = false;
        break;
    case IntakeState::ScoreMid:
        set_intake_velocity_frac(1, 0.6, -0.4);
        break;
    case IntakeState::ScoreLow:
        set_intake_velocity_frac(-0.5, -0.7, -0.4);
        break;
    case IntakeState::Idle:
        set_intake_velocity_frac(0, 0, 0);
        break;
    case IntakeState::Custom:
        break;
    }

    second_state_jam_state.update();

    if (second_state_jam_state.unjam_end > pros::millis()) {
        second_stage_velocity *= -127;
    }

    (void)first_stage_motor.move_velocity(first_stage_velocity);
    (void)second_stage_motor.move_velocity(second_stage_velocity);
    (void)third_stage_motor.move_velocity(third_stage_velocity);

    if (blocker_final_value != p_blocker_final_value) {
        blocker_toggle_cooldown = blocker_toggle_cooldown_duration;
    }

    const bool blocker_candidate_value = auto_blocker_value || blocker_value;
    if (blocker_toggle_cooldown <= 0) {
        blocker_final_value = blocker_candidate_value;
    }
    else {
        blocker_toggle_cooldown -= PROCESS_DELAY;
    }

    p_blocker_final_value = blocker_final_value;

    (void)blocker_piston.set_value(blocker_final_value);
    (void)matchload_piston.set_value(matchload_value);
    (void)hook_piston.set_value(hook_value);
}
