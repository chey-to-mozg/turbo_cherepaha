#ifndef ENCODERS_H
#define ENCODERS_H

#include "config.h"
#include <util/atomic.h>

extern int g_left_dir;
extern int g_right_dir;

extern volatile int g_count_left;
extern volatile int g_count_right;

extern float s_total_left;
extern float s_total_right;

extern float s_distance_left;
extern float s_distance_right;

extern float g_distance;
extern float g_angle;
extern float g_increment;
extern float g_angle_increment;

void init_encoders();
void reset_encoders();
float robot_position();
float robot_angle();
float robot_fwd_increment();
float robot_rot_increment();

void update_encoders();

#endif