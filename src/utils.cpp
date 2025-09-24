#include "utils.h"

uint8_t wait_to_start() {
    /*
    This function will return code of execution
    0 -- normal run from start to finish and back
    1 -- normal run from start to finish and back + save map
    2 -- normal run from start to finish with loaded map
    3 -- smooth run from start to finish and back
    4 -- smooth run from start to finish and back + save map
    5 -- smooth run from start to finish with loaded map
    6 -- ...
    7 -- Print maze info
    */
    uint8_t mode = 0;
    uint8_t signal = 0;
    int angle = mouse.get_angle();
    while(true) {
        update_motor_controllers(angle);
        if (g_front_sensor > 200) {
            break;
        }
        print_sensors();
        print_motors();
        if (g_right_button) {
            mode = (mode + 1) % 8;
            turn_mode_leds(mode, signal);
            delay(500);
        }
        turn_mode_leds(mode, signal);
        // turn_wall_leds(g_is_left_wall, g_is_front_wall, g_is_right_wall);
        if (signal == 0) {
            signal = (maze.get_direction() == UP) ? 1 : 2;
        }
        else {
            signal = 0;
        }
        delay(200);
    }
    turn_all_leds();
    delay(2000);
    update_motor_controllers(angle);
    reset_leds();

    return mode;
}