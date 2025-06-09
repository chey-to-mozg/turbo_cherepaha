#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <config.h>
#include "leds.h"
#include <iarduino_VCC.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


extern int g_left_sensor_raw;
extern int g_right_sensor_raw;
extern int g_front_sensor_raw_left;
extern int g_front_sensor_raw_right;

extern int g_left_sensor;
extern int g_right_sensor;
extern int g_front_sensor;

extern bool g_is_left_wall;
extern bool g_is_right_wall;
extern bool g_is_front_wall;

extern bool g_left_button;
extern bool g_right_button;

extern float g_gyro_angle;


/*** steering variables ***/
extern bool g_steering_enabled;
extern float g_cross_track_error;
extern float g_steering_adjustment;

void update_sensors();

float calculate_steering_adjustment();

int read_row(uint8_t sensor);
void init_sesnors();
bool button_pressed();

void reset_steering();
void enable_steering();
void disable_steering();

int get_front_sensor();

#endif