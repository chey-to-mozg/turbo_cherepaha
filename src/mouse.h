#ifndef MOUSE_H
#define MOUSE_H

#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "reporter.h"
#include "profile.h"
#include "maze.h"

#define SEARCH_ACCELERATION 1000
#define SPIN_TURN_ACCELERATION 1000
#define SPIN_TURN_SMOOTH_ACCELERATION 1500
#define SPEEDMAX_EXPLORE 150
#define SPEEDMAX_PRETURN 100
#define SPEEDMAX_SMOOTH_TURN 90
#define SPEEDMAX_SPIN_TURN 300

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
        void move(float distance, float max_speed, bool check_wall = true);
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
    private:
        void move_angle(float angle);

        bool is_start = true;
        bool is_center = false;

        bool left_wall;
        bool front_wall;
        bool right_wall;
};

extern Mouse mouse;

#endif