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

void update_syscontrol() {
    switch (intake_state) {
    case IntakeState::Intake:
        first_stage_velocity = 600;
        second_stage_velocity = 200;
        third_stage_velocity = 200;
        auto_blocker_value = true;
        break;
    case IntakeState::ScoreLong:
        first_stage_velocity = 600;
        second_stage_velocity = 200;
        third_stage_velocity = 200;
        auto_blocker_value = false;
        blocker_value = false;
        break;
    case IntakeState::ScoreMid:
        first_stage_velocity = 600;
        second_stage_velocity = 200;
        third_stage_velocity = -200;
        break;
    case IntakeState::ScoreLow:
        first_stage_velocity = -0.5 * 600;
        second_stage_velocity = -0.7 * 200;
        third_stage_velocity = -0.5 * 200;
        break;
    case IntakeState::Idle:
        first_stage_velocity = 0;
        second_stage_velocity = 0;
        third_stage_velocity = 0;
        break;
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
    } else {
        blocker_toggle_cooldown -= PROCESS_DELAY;
    }

    p_blocker_final_value = blocker_final_value;

    (void)blocker_piston.set_value(blocker_final_value);
    (void)matchload_piston.set_value(matchload_value);
    (void)hook_piston.set_value(hook_value);
}