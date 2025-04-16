#ifndef MOUSE_H
#define MOUSE_H

#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "reporter.h"
#include "maze.h"

#define SPEEDMAX_EXPLORE 200
#define SPEEDMAX_PRETURN 200
#define SPEEDMAX_SMOOTH_TURN 90
#define SPEEDMAX_SPIN_TURN 150

class Mouse {
    public:
        Mouse();
        // make this public for tests
        uint8_t wait_to_start(bool print_debug = true);
        void wait_to_start_front();
        void error_ping();
        void finish_ping(int counts = 0);
        void print_info();
        uint8_t stop();
        void move(float distance, float max_speed, bool check_wall = false);
        void move_from_wall();
        void move_to_center();
        void wait_until_position(float position);
        void move_cell();
        void turn_90_left();
        void turn_90_right();
        void turn_90_left_smooth();
        void turn_90_right_smooth();
        void turn_around();
        void move_backward();
        void update_walls();
        bool run_smooth(bool to_finish = true, bool check_walls = true); // front sensor should be calibrated properly
        bool run_normal(bool to_finish = true);
        void reset_mouse();
        float get_angle();
    private:
        void move_angle(float turn_angle, float speed);
        void turn(float angle);
        void turn_after_move(float angle);
        void calibrate_with_front_wall();

        bool is_start = true;
        bool is_center = false;

        bool left_wall;
        bool front_wall;
        bool right_wall;

        float angle = 0;
};

extern Mouse mouse;

#endif