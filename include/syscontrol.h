//
// Created by Hanyu Zhang on 10/24/25.
//

#ifndef SYSCONTROL_H
#define SYSCONTROL_H

#include "config.h"

enum class IntakeState {
    Intake, ScoreLong, ScoreMid, ScoreLow, Idle,
    ScoreLowSlow1, ScoreLowSlow2
};

extern IntakeState intake_state;

extern bool blocker_value;
extern bool auto_blocker_value;
extern bool matchload_value;
extern bool hook_value;

extern bool p_blocker_final_value;
extern bool blocker_final_value;
extern int blocker_toggle_cooldown;
extern const int blocker_toggle_cooldown_duration;

extern int first_stage_velocity;
extern int second_stage_velocity;
extern int third_stage_velocity;

void init_syscontrol();

void update_syscontrol();

#endif //SYSCONTROL_H
