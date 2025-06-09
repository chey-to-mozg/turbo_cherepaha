#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <limits.h>
#include "config.h"
#include "encoders.h"
#include "sensors.h"
#include "mouse.h"

class Motor {
    public:
        Motor(int dir_pin, int pwm_pin, int encoder_polarity);
        void reset_motor();
        void set_speed(float speed);
        void set_pwm(int pwm);
        void update_pwm(float distance_change, float angle_error, float pos_error);
        float get_speed();
        int get_pwm();
        void set_enable(bool enabled);
    private:
        void set_direction(int direction); // 1 or -1
        void accelerate();
        int dir_pin;
        int pwm_pin;
        int polarity;
        bool accelerating = true;
        float speed = 0;
        float actual_speed = 0;
        float acceleration_speed = 0;
        float last_speed_error = 0;
        float last_angle_error = 0;
        float cum_speed_error = 0;
        int pwm = 0;
        uint32_t last_update = 0;
        bool enabled = false;
};

extern Motor motor_left;
extern Motor motor_right;

void stop_motors();
void update_motor_controllers();
void disable_motors();
void enable_motors();
void test_mototrs();
#endif