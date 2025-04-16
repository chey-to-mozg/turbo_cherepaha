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
        void set_speed(float speed);
        void set_pwm(int pwm);
        void update_pwm(float distance_change, float angle_error);
        float get_speed();
        int get_direction();
        int get_pwm();
    private:
        void set_direction(int direction); // 1 or -1
        void accelerate();
        int direction = 1;
        int dir_pin;
        int pwm_pin;
        int polarity;
        bool accelerating = true;
        float speed = 0;
        float actual_speed = 0;
        float acceleration_speed = 0;
        float cum_speed_error = 0;
        int pwm = 0;
        uint32_t last_update = 0;
};

extern Motor motor_left;
extern Motor motor_right;

void stop_motors();
void update_motor_controllers();
#endif