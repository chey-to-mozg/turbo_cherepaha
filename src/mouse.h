#ifndef MOUSE_H
#define MOUSE_H

#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "reporter.h"
#include "maze.h"

#define SPEEDMAX_EXPLORE 300
#define SPEEDMAX_SMOOTH_TURN 300
#define SPEEDMAX_SPIN_TURN 150

class Mouse {
    public:
        Mouse();
        // make this public for tests
        uint8_t wait_to_start();
        void maze_debug();
        void error_ping();
        void finish_ping(int counts = 0);
        void print_info();
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
        void run_short();
        void reset_mouse();
        float get_angle();
    private:
        void move_angle(float turn_angle, float speed);
        void turn(float angle);

        bool is_start = true;
        bool is_center = false;

        float angle = 0;
};

extern Mouse mouse;

#endif