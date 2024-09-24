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
    pinMode(DATA_PIN, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);
}

void turn_leds(uint8_t controls) {
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, controls);
    digitalWrite(LATCH_PIN, HIGH);
}

void turn_all_leds() {
    turn_leds(0b00111111);
}

void reset_leds() {
    turn_leds(0);
}

void init_loading_leds() {
    step = 0;
    reset_leds();
}

void step_loading_leds() {
    uint8_t leds = 0;
    switch (step)
    {
        case 0:
            leds = 0b00001001;
            break;
        case 1:
            leds = 0b00001100;
            break;
        case 2:
            leds = 0b00000110;
            break;
        case 3:
            leds = 0b00010010;
            break;
        case 4:
            leds = 0b00110000;
            break;
        case 5:
            leds = 0b00100001;
            break;
    }
    step = (step + 1) % 6;
    turn_leds(leds);
}

void turn_wall_leds(bool left_wall, bool front_wall, bool right_wall) {
    uint8_t leds = 0;
    if (left_wall) 
        leds |= left_wall_leds;
    if (front_wall)
        leds |= front_wall_leds;
    if (right_wall)
        leds |= right_wall_leds;
    
    turn_leds(leds);
}

void turn_mode_leds(uint8_t mode, bool signal_led) {
    uint8_t mode_leds[3] = {BLUE_RIGHT_LED, GREEN_RIGHT_LED, RED_RIGHT_LED};
    uint8_t leds = 0;
    for (int i = 0; i < 3; i++) {
        if (mode >> i & 1) {
            leds |= mode_leds[i];
        }
    }
    if (signal_led) {
        leds |= BLUE_LEFT_LED;
    }
    turn_leds(leds);
}