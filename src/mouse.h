#ifndef MOUSE_H
#define MOUSE_H

#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "reporter.h"
#include "profile.h"
#include "maze.h"

#define SEARCH_ACCELERATION 700
#define SPIN_TURN_ACCELERATION 1000
#define SPEEDMAX_EXPLORE 300
#define SPEEDMAX_PRETURN 200
#define SPEEDMAX_STRAIGHT 500
#define SPEEDMAX_SMOOTH_TURN 300
#define SPEEDMAX_SPIN_TURN 300

class Mouse {
    public:
        Mouse();
        // make this public for tests
        void wait_to_start();
        void wait_to_start_front();
        void error_ping();
        void finish_ping(int counts = 0);
        void move(float distance, float max_speed, bool check_wall = true);
        void move_from_wall();
        void move_to_center();
        void wait_until_position(float position);
        void move_cell();
        void turn_90_left();
        void turn_90_right();
        void turn_90_left_smooth();
        void turn_90_right_smooth();
        void turn_around(bool enter_cell = true);
        void move_backward();
        void update_walls();
        void run_smooth(); // front sensor should be calibrated properly
        void run_normal(bool to_finish = true);
    private:
        void move_angle(float angle);
        void init_leds();

        bool is_start = false;

        bool left_wall;
        bool front_wall;
        bool right_wall;
};

extern Mouse mouse;

#endif