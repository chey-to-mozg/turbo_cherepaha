#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "config.h"
#include "encoders.h"
#include "sensors.h"
#include "profile.h"

extern float s_err_fwd;
extern float s_err_rot;

extern float s_pwm_left;
extern float s_pwm_right;

void init_motors();
void reset_motor_controllers();

float position_controller();
float angle_controller(float steering_adjustment);

void update_motor_controllers(float target);

void set_direction_left(int forward);
void set_direction_right(int forward);

void set_left_motor_pwm(int pwm);
void set_right_motor_pwm(int pwm);

void stop_motors();
void disable_mototrs();
void enable_mototrs();
extern float var1;
extern float var2;
#endif