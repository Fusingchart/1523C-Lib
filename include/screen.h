#pragma once

#include "config.h"
#include "main.h"
#include "syscontrol.h"

inline constexpr int autons_per_page = 6;
inline constexpr int max_pages = (auton_count + autons_per_page - 1) / autons_per_page;
inline int page = 0;

enum class ScreenState {
    MAIN,
    AUTON_SELECTOR,
    CONSOLE,
};

inline std::atomic screen_state = ScreenState::MAIN;

inline bool in_bound(const int16_t px, const int16_t py, const int16_t x1, const int16_t y1, const int16_t x2,
                     const int16_t y2) {
    return px >= x1 && px <= x2 && py >= y1 && py <= y2;
}

inline void change_screen_state(const ScreenState& new_screen_state) {
    pros::screen::erase();
    screen_state = new_screen_state;
}

inline void screen_touch_callback() {
    const pros::screen_touch_status_s_t status = pros::screen::touch_status();

    if (screen_state == ScreenState::MAIN) {
        if (in_bound(status.x, status.y, 400, 219, 480, 235)) {
            change_screen_state(ScreenState::AUTON_SELECTOR);
        }
        else if (in_bound(status.x, status.y, 400, 219 - 30, 480, 235 - 30)) {
            change_screen_state(ScreenState::CONSOLE);
        }
        else if (in_bound(status.x, status.y, 400, 219 - 60, 480, 235 - 60)) {
            // change_screen_state(ScreenState::FISH);
            // start_fish_screen_state();
        }
    }
    else if (screen_state == ScreenState::AUTON_SELECTOR) {
        if (in_bound(status.x, status.y, 400, 219, 480, 235)) {
            change_screen_state(ScreenState::MAIN);
        }
        else if (status.x >= 10 - 1 && status.x <= 10 + 20 + 240 + 1) {
            const int idx = (status.y - 30) / 25;
            if (idx >= 0 && idx < (page + 1) * autons_per_page) {
                int unbound_selected_auton = idx + page * autons_per_page;
                if (unbound_selected_auton >= auton_count) unbound_selected_auton = auton_count - 1;
                selected_auton = unbound_selected_auton;
                pros::screen::erase();
            }

        }
        else if (in_bound(status.x, status.y, 10 + 20 + 240 + 1 + 20, 30 + 20, 10 + 20 + 240 + 1 + 20 + 60,
                          30 + 20 + 20)) {
            if (page > 0) {
                --page;
                pros::screen::erase();
            }
        }
        else if (in_bound(status.x, status.y, 10 + 20 + 240 + 1 + 20, 30 + 20 + 25, 10 + 20 + 240 + 1 + 20 + 60,
                          30 + 20 + 20 + 25)) {
            if (page < max_pages - 1) {
                ++page;
                pros::screen::erase();
            }
        }
        else if (in_bound(status.x, status.y, 10 + 20 + 240 + 1 + 20, 30 + 20 + 25 + 40, 10 + 20 + 240 + 1 + 20 + 60,
                          30 + 20 + 30 + 25 + 40)) {
            if (auton_color != RED) {
                auton_color = RED;
                pros::screen::erase();
            }
        }
        else if (in_bound(status.x, status.y, 10 + 20 + 240 + 1 + 20, 30 + 20 + 25 + 40 + 40,
                          10 + 20 + 240 + 1 + 20 + 60,
                          30 + 20 + 30 + 25 + 40 + 40)) {
            if (auton_color != BLUE) {
                auton_color = BLUE;
                pros::screen::erase();
            }
        }
    }
    else if (screen_state == ScreenState::CONSOLE) {
        if (in_bound(status.x, status.y, 400, 219, 480, 235)) {
            change_screen_state(ScreenState::MAIN);
        }
    }
    else if (screen_state == ScreenState::FISH) {
        // fish_touch(status);
    }
}

inline int flash_counter = 8;

inline void main_screen_state() {
    static constexpr int16_t pad = 10;

    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_SMALL, pad, pad, "X: %f          ", chassis.getPose().x); // x
    pros::screen::print(pros::E_TEXT_SMALL, pad, pad + 15, "Y: %f          ", chassis.getPose().y); // x
    pros::screen::print(pros::E_TEXT_SMALL, pad, pad + 30, "H: %f          ", chassis.getPose().theta); // x

    pros::screen::print(pros::E_TEXT_SMALL, pad, pad + 60, "fds: %f          ",
                        front_dist_sensor.get_distance() / 25.4);
    pros::screen::print(pros::E_TEXT_SMALL, pad + 80, pad + 60, "c: %d          ", front_dist_sensor.get_confidence());
    pros::screen::print(pros::E_TEXT_SMALL, pad, pad + 75, "lds: %f          ", left_dist_sensor.get_distance() / 25.4);
    pros::screen::print(pros::E_TEXT_SMALL, pad + 80, pad + 75, "c: %d          ", left_dist_sensor.get_confidence());
    pros::screen::print(pros::E_TEXT_SMALL, pad, pad + 90, "rds: %f          ",
                        right_dist_sensor.get_distance() / 25.4);
    pros::screen::print(pros::E_TEXT_SMALL, pad + 80, pad + 90, "c: %d          ", right_dist_sensor.get_confidence());

    pros::screen::print(pros::E_TEXT_SMALL, pad, pad + 120, "hue: %f          ", optical.get_hue());
    pros::screen::print(pros::E_TEXT_SMALL, pad, pad + 135, "dist: %d          ", optical.get_proximity());
    pros::screen::print(pros::E_TEXT_SMALL, pad, pad + 150, "col: %s          ",
                        color_to_string(get_ball_color(optical)));

    pros::screen::print(pros::E_TEXT_SMALL, pad, pad + 165, "2imu: %c", using_dual_imu ? 'Y' : 'N');

    pros::screen::print(pros::E_TEXT_SMALL, pad + 120, pad, "ai-bmc: v=%c p=%d r=%c en=%c", blockerManualCommand.value ? 'T' : 'F', blockerManualCommand.get_priority(), blockerManualCommand.required ? 'Y' : 'N', blockerManualCommand.is_enabled() ? 'Y' : 'N');
    pros::screen::print(pros::E_TEXT_SMALL, pad + 120, pad + 15, "ai-bcc: v=%c p=%d r=%c en=%c", blockerColorsortCommand.value ? 'T' : 'F', blockerColorsortCommand.get_priority(), blockerColorsortCommand.required ? 'Y' : 'N', blockerColorsortCommand.is_enabled() ? 'Y' : 'N');

    pros::screen::print(pros::E_TEXT_SMALL, pad + 140, pad + 45, "time: %d        ", pros::millis());
    pros::screen::print(pros::E_TEXT_SMALL, pad + 140, pad + 60, "fidx: eff=%f cjt=%d uje=%d        ", frontIndex.get_efficiency(), front_index_profiler.current_jam_time, front_index_profiler.unjam_end);
    pros::screen::print(pros::E_TEXT_SMALL, pad + 140, pad + 75, "back: eff=%f cjt=%d uje=%d        ", backIndex.get_efficiency(), back_index_profiler.current_jam_time, back_index_profiler.unjam_end);
    pros::screen::print(pros::E_TEXT_SMALL, pad + 140, pad + 90, "botm: eff=%f cjt=%d uje=%d        ", bottomIndex.get_efficiency(), bottom_index_profiler.current_jam_time, bottom_index_profiler.unjam_end);

    const double battery_capacity = pros::battery::get_capacity();
    if (battery_capacity <= 20.0) {
        pros::screen::set_eraser(pros::Color::red);
        pros::screen::set_pen(pros::Color::red);
        if (battery_capacity <= 10.0) {
            if (flash_counter-- <= 0) {
                pros::screen::set_eraser(pros::Color::black);
                pros::screen::set_pen(pros::Color::black);
            }
            if (flash_counter <= 8) flash_counter = 8;
        }
        pros::screen::fill_rect(400 - 2, 10 - 2, 400 + 64 + 2, 10 + 12 + 2);
        pros::screen::set_pen(pros::Color::white);
    }
    else if (battery_capacity <= 35.0) pros::screen::set_pen(pros::Color::red);
    else if (battery_capacity <= 50.0) pros::screen::set_pen(pros::Color::orange);
    else if (battery_capacity <= 65.0) pros::screen::set_pen(pros::Color::yellow);
    pros::screen::print(pros::E_TEXT_SMALL, 400, 10, "batt. %.0f", battery_capacity); // x
    pros::screen::set_eraser(pros::Color::black);

    pros::screen::set_pen(pros::Color::gray);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 247 - 15 - 10, "        -----------------------------------------");
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 247 - 15 - 10, "Auton:");
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_MEDIUM, 60 + 5 + 15 + 5, 247 - 15 - 10 - 3,
                        autons[selected_auton].name.c_str());

    pros::screen::set_pen(auton_color == BLUE ? pros::Color::blue : pros::Color::red);
    pros::screen::fill_rect(60 + 5, 247 - 10 - 15 - 3, 60 + 5 + 15, 247 - 10 - 3);

    // pros::screen::set_pen(pros::Color::dark_blue);
    // pros::screen::fill_rect(160, 247 - 15 - 10 - 3, 160 + 65, 247 - 10 - 3);
    pros::screen::set_pen(pros::Color::light_blue);
    pros::screen::draw_rect(470 - 65 - 1, 247 - 15 - 10 - 3 - 1, 470 + 1, 247 - 10 - 3 + 1);
    pros::screen::print(pros::E_TEXT_SMALL, 470 - 65 + 8, 247 - 15 - 10 - 3 + 2, "change");

    pros::screen::set_pen(pros::Color::light_blue);
    pros::screen::draw_rect(470 - 65 - 1, 247 - 15 - 10 - 3 - 1 - 30, 470 + 1, 247 - 10 - 3 + 1 - 30);
    pros::screen::print(pros::E_TEXT_SMALL, 470 - 65 + 4, 247 - 15 - 10 - 3 + 2 - 30, "console");

    // pros::screen::set_pen(pros::Color::light_blue);
    // pros::screen::draw_rect(470 - 65 - 1, 247 - 15 - 10 - 3 - 1 - 60, 470 + 1, 247 - 10 - 3 + 1 - 60);
    // pros::screen::print(pros::E_TEXT_SMALL, 470 - 65 + 15, 247 - 15 - 10 - 3 + 2 - 60, "fish");
}

inline void auton_selector_screen_state() {
    pros::screen::set_pen(pros::Color::light_gray);
    pros::screen::fill_rect(0, 0, 480, 20);
    pros::screen::set_pen(pros::Color::black);
    pros::screen::set_eraser(pros::Color::light_gray);
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, 3, "AUTON SELECTOR");

    pros::screen::set_eraser(pros::Color::black);
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 247 - 15 - 10 - 25 - 1 + 5, "Auton:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 60 + 5 + 15 + 5, 247 - 15 - 10 - 3 - 25 + 5,
                        autons[selected_auton].name.c_str());

    pros::screen::set_pen(pros::Color::gray);
    pros::screen::print(pros::E_TEXT_SMALL, 10 - 3, 247 - 15 - 10 - 10 + 3,
                        "desc: -------------------------------------------");
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_SMALL, 10 + 3, 247 - 15 - 10 + 3, autons[selected_auton].desc.c_str());

    pros::screen::set_pen(auton_color == BLUE ? pros::Color::blue : pros::Color::red);
    pros::screen::fill_rect(60 + 5, 247 - 10 - 15 - 3 - 25 + 5, 60 + 5 + 15, 247 - 10 - 3 - 25 + 5);

    pros::screen::set_pen(pros::Color::light_blue);
    pros::screen::draw_rect(470 - 65 - 1, 247 - 15 - 10 - 3 - 1, 470 + 1, 247 - 10 - 3 + 1);
    pros::screen::print(pros::E_TEXT_SMALL, 470 - 65 + 15, 247 - 15 - 10 - 3 + 2, "done");

    pros::screen::set_pen(pros::Color::light_gray);
    pros::screen::print(pros::E_TEXT_SMALL, 10 + 20 + 240 + 1 + 20, 30, "pg.");
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_MEDIUM, 10 + 20 + 240 + 1 + 20 + 30, 30, "%d/%d", page + 1, max_pages);

    pros::screen::set_pen(page > 0 ? pros::Color::white : pros::Color::gray);
    pros::screen::draw_rect(10 + 20 + 240 + 1 + 20, 30 + 20, 10 + 20 + 240 + 1 + 20 + 60, 30 + 20 + 20);
    pros::screen::print(pros::E_TEXT_SMALL, 10 + 20 + 240 + 1 + 20 + 13, 30 + 20 + 4, "prev");

    pros::screen::set_pen(page < max_pages - 1 ? pros::Color::white : pros::Color::gray);
    pros::screen::draw_rect(10 + 20 + 240 + 1 + 20, 30 + 20 + 25, 10 + 20 + 240 + 1 + 20 + 60, 30 + 20 + 20 + 25);
    pros::screen::print(pros::E_TEXT_SMALL, 10 + 20 + 240 + 1 + 20 + 13, 30 + 20 + 25 + 4, "next");

    pros::screen::set_pen(auton_color == RED ? pros::Color::red : pros::Color::dark_red);
    pros::screen::fill_rect(10 + 20 + 240 + 1 + 20, 30 + 20 + 25 + 40, 10 + 20 + 240 + 1 + 20 + 60,
                            30 + 20 + 30 + 25 + 40);
    if (auton_color == BLUE) {
        pros::screen::set_pen(pros::Color::black);
        pros::screen::fill_rect(10 + 20 + 240 + 1 + 20 + 5, 30 + 20 + 25 + 40 + 5, 10 + 20 + 240 + 1 + 20 + 60 - 5,
                                30 + 20 + 30 + 25 + 40 - 5);
    }
    pros::screen::set_pen(auton_color == BLUE ? pros::Color::blue : pros::Color::dark_blue);
    pros::screen::fill_rect(10 + 20 + 240 + 1 + 20, 30 + 20 + 25 + 40 + 40, 10 + 20 + 240 + 1 + 20 + 60,
                            30 + 20 + 30 + 25 + 40 + 40);
    if (auton_color == RED) {
        pros::screen::set_pen(pros::Color::black);
        pros::screen::fill_rect(10 + 20 + 240 + 1 + 20 + 5, 30 + 20 + 25 + 40 + 5 + 40, 10 + 20 + 240 + 1 + 20 + 60 - 5,
                                30 + 20 + 30 + 25 + 40 - 5 + 40);
    }

    int16_t sy = 20 + 30;

    for (int i = page * autons_per_page; i < (page + 1) * autons_per_page; ++i) {
        if (i >= auton_count) break;

        const int up_shift = page * autons_per_page * 25;

        Auton& this_auton = autons[i];

        const int bonus = (i == selected_auton) * 10;

        pros::screen::set_pen(pros::Color::light_blue);

        pros::screen::set_pen(i == selected_auton ? pros::Color::light_blue : pros::Color::dark_gray);
        if (i == selected_auton) {
            pros::screen::set_eraser(pros::Color::light_blue);
            pros::screen::fill_rect(10 - 1 + bonus, 30 + i * 25 - 1 - up_shift, 10 + 20 + 240 + 1 + bonus,
                                    30 + i * 25 + 20 + 1 - up_shift);
        }
        else {
            pros::screen::draw_rect(10 - 1 + bonus, 30 + i * 25 - 1 - up_shift, 10 + 20 + 240 + 1 + bonus,
                                    30 + i * 25 + 20 + 1 - up_shift);
        }
        pros::screen::set_pen(i == selected_auton ? pros::Color::black : pros::Color::light_gray);
        pros::screen::print(pros::E_TEXT_SMALL, 10 + 3 + bonus, 30 + i * 25 + 5 - up_shift,
                            this_auton.name.c_str());

        pros::screen::set_eraser(pros::Color::black);

        sy += 20 + 5;
    }
}

inline uint32_t cached_timestamp = -1;


[[noreturn]] void inline screen_update_function() {
    pros::delay(50);
    while (true) {
        if (screen_state == ScreenState::MAIN) main_screen_state();
        else if (screen_state == ScreenState::AUTON_SELECTOR) auton_selector_screen_state();
        else if (screen_state == ScreenState::CONSOLE) console_screen_state();
        // else if (screen_state == ScreenState::FISH) fish_screen_state();

        // controller.print(0, 0, "qa: %d     ", quadrant);
        if (sort_target == BLUE) {
            controller.print(0, 0, "alliance: %s     ", color_to_string(RED));
        } else if (sort_target == RED) {
            controller.print(0, 0, "alliance: %s     ", color_to_string(BLUE));
        } else {
            controller.print(0, 0, "alliance: %s     ", color_to_string(NEUTRAL));
        }

        pros::delay(50);
    }
}