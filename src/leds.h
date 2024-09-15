#ifndef LEDS_H
#define LEDS_H

#include "config.h"

/*
0b00000001 | | 0b00001000
0b00100000 | | 0b00000100
0b00010000 | | 0b00000010
*/


void init_leds();
void turn_leds(uint8_t controls);
void turn_all_leds();
void reset_leds();
void init_loading_leds();
void step_loading_leds();
void turn_wall_leds(bool left_wall, bool front_wall, bool right_wall);

#endif