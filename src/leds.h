#ifndef LEDS_H
#define LEDS_H

#include "config.h"

/*
0b00000001 | | 0b00001000
0b00100000 | | 0b00000100
0b00010000 | | 0b00000010
*/
#define RED_LEFT_LED 0b00000001
#define RED_RIGHT_LED 0b00001000
#define GREEN_LEFT_LED 0b00100000
#define GREEN_RIGHT_LED 0b00000100
#define BLUE_LEFT_LED 0b00010000
#define BLUE_RIGHT_LED 0b00000010


void init_leds();
void turn_leds(uint8_t controls);
void turn_all_leds();
void reset_leds();
void init_loading_leds();
void step_loading_leds();
void turn_wall_leds(bool left_wall, bool front_wall, bool right_wall);
void turn_mode_leds(uint8_t mode, bool signal_led);

#endif