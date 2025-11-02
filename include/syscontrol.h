//
// Created by Hanyu Zhang on 10/24/25.
//

#ifndef SYSCONTROL_H
#define SYSCONTROL_H

#include "config.h"

enum class IntakeState {
    Intake, ScoreLong, ScoreMid, ScoreLow, Idle, Custom,
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

struct JamState {
    pros::Motor* target;

    const uint32_t jam_tolerance;
    const uint32_t unjam_duration;

    uint32_t jam_start = 0;
    uint32_t unjam_end = 0;

    JamState(pros::Motor* target, uint32_t jam_tolerance, uint32_t unjam_duration);

    void update();
};

extern JamState second_state_jam_state;

void set_intake_velocity_frac(float first, float second, float third);
void set_intake_velocity(int first, int second, int third);

void init_syscontrol();

void update_syscontrol();

#endif //SYSCONTROL_H
