#include <leds.h>

    /*
    Leds controls:

    0b00000001 red   | |   red 0b00001000
    0b00100000 green | | green 0b00000100
    0b00010000 blue  | |  blue 0b00000010

    */

uint8_t step = 0;
const uint8_t front_wall_leds = RED_LEFT_LED | RED_RIGHT_LED;
const uint8_t left_wall_leds = BLUE_LEFT_LED;
const uint8_t right_wall_leds = BLUE_RIGHT_LED;


void init_leds() {
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
}

void turn_leds(uint8_t controls) {
}

void turn_all_leds() {
}

void reset_leds() {
}

void init_loading_leds() {
}

void step_loading_leds() {
}

void turn_wall_leds(bool left_wall, bool front_wall, bool right_wall) {
}

void turn_mode_leds(uint8_t mode, uint8_t signal_led) {
    bool s = signal_led > 0 ? true : false;
    digitalWrite(LED2, s);
}