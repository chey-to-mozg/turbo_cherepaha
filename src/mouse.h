#ifndef MOUSE_H
#define MOUSE_H

#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "reporter.h"
#include "maze.h"

#define SPEEDMAX_EXPLORE 300
#define SPEEDMAX_FAST 500
#define SPEEDMAX_SPIN_TURN 200

class Mouse {
    public:
        Mouse();
        // make this public for tests
        uint8_t wait_to_start();
        void show_nominal_value();
        void error_ping();
        void finish_ping(int counts = 0);
        void stop();
        void move(float distance, float max_speed, int check_wall_distance = 0);
        void move_from_wall();
        void move_half_cell(bool untill_wall = false);
        void move_cell(bool untill_wall = false);
        void turn_90_left();
        void turn_90_right();
        void turn_90_left_smooth();
        void turn_90_right_smooth();
        void turn_around();
        void move_backward();
        void update_walls();
        bool explore(bool to_finish = true);
        bool run_short();
        void reset_mouse();
        int get_angle();
        float get_position();
        void set_config(int config_id);
        void switch_start_direction();
    private:
        void move_angle(float turn_angle, float speed);
        void turn_smooth(float turn_angle);
        void move_cell_unit(float target, bool untill_wall);
        void turn(float angle);

        bool is_start = true;
        bool is_center = false;
        uint8_t start_direction = UP;

        int max_speed = 0;
        int angle_offset = 0;
        int pre_turn_ofset = 0;
        int after_turn_offset = 0;
        int pre_turn_reference = 0;
        int front_reference = 0;
        int turn_speed = 0;
        int turn_inner_speed = 0;
        int angle = 0;
        float distance = 0;
};

extern Mouse mouse;

#endif