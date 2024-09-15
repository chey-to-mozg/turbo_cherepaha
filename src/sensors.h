#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <config.h>
#include "leds.h"
#include <util/atomic.h>


extern volatile int g_left_sensor_raw;
extern volatile int g_right_sensor_raw;
extern volatile int g_front_sensor_raw_left;
extern volatile int g_front_sensor_raw_right;

extern volatile int g_left_sensor;
extern volatile int g_right_sensor;
extern volatile int g_front_sensor;

extern volatile bool g_is_left_wall;
extern volatile bool g_is_right_wall;
extern volatile bool g_is_front_wall;

extern volatile bool g_left_button;
extern volatile bool g_right_button;

extern volatile float g_gyro_angle;
extern volatile float gyro_error;
extern float gyro_delta;


/*** steering variables ***/
extern bool g_steering_enabled;
extern volatile float g_cross_track_error;
extern volatile float g_steering_adjustment;

void read_sensors();

float calculate_steering_adjustment();

int read_row(uint8_t sensor);
void init_sesnors();
bool button_pressed();

void reset_steering();
void enable_steering();
void disable_steering();
void update_gyro_reference(float diff);

void reset_gyro();
void calibrate_gyro();

#endif