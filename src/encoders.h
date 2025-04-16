#ifndef ENCODERS_H
#define ENCODERS_H

#include "config.h"
#include <util/atomic.h>

extern int dir_left;
extern int dir_right;

extern int total_conut_left;
extern int total_count_right;

extern float distance_left;
extern float distance_right;

extern float distance;
extern float angle;

void init_encoders();
void reset_encoders();
float get_robot_position();
float get_robot_angle();
float get_increment_left();
float get_increment_right();

void update_encoders();

#endif