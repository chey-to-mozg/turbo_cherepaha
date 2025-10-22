#include <Arduino.h>
#include "reporter.h"
#include "leds.h"
#include "mouse.h"
#include "maze.h"
#include "motors.h"
#include "utils.h"

uint8_t mode = 0;

void setup() {
  init_serial();
  // init_bluetooth();
  init_sesnors();
  init_encoders();
  init_leds();
}

void test_run() {  
  mouse.reset_mouse();
  mouse.set_config(0);  
  enable_motors();
  // mouse.move_from_wall();
  // mouse.move_half_cell();
  // while(true) {
  //   mouse.move_cell();
  // }
  mouse.move_cell();
  mouse.move_cell();
  mouse.move_cell();
  mouse.move_cell();
  // mouse.turn_90_left_smooth();
  // mouse.turn_90_left_smooth();
  // mouse.turn_90_left_smooth();
  // mouse.turn_90_right_smooth();
  // mouse.turn_90_left_smooth();
  // mouse.turn_90_right_smooth();
  // mouse.turn_90_left_smooth();
  // mouse.turn_90_right_smooth();
  // mouse.turn_90_left_smooth();
  // mouse.turn_90_right_smooth();
}

void test_loop() {
  while (1) {
    mode = mouse.wait_to_start();
    if (mode == 0)
    {
      test_run();
    }
    else if (mode == 1) {
      while(true) {
        print_sensors();
        delay(1000);
      }
    }
    else if (mode == 1) {
      check_pwm_control();
    }
    mouse.stop();
  }
  
}

void main_loop() {
  while (1) {
    mode = mouse.wait_to_start();

    if (mode == 0){
      //** NORMAL RUN **//
      mouse.reset_mouse();
      mouse.set_config(0);
      
      bool finished = mouse.explore();
      if (finished) {
        mouse.finish_ping();
        finished = mouse.explore(false);
      }

      if (!finished) {
        mouse.error_ping();
      }
    }
    else if (mode == 1)
    {
      //** NORMAL RUN WITH MAP SAVING **/

      mouse.reset_mouse();
      mouse.set_config(0);

      bool finished = mouse.explore();
      if (finished) {
        mouse.finish_ping();
        finished = mouse.explore(false);
      }

      if (!finished) {
        mouse.error_ping();
      }
      
      maze.save_maze();
    }
    else if (mode == 2)
    {
      //** FAST RUN WITH MAP LOADING **/

      mouse.reset_mouse();
      mouse.set_config(1);
      maze.reset_maze();

      maze.load_maze();
      maze.lock_maze();

      bool finished = mouse.run_short();
      if (finished) {
        mouse.finish_ping();
        mouse.set_config(0);
        mouse.explore(false);
      } else {
        mouse.error_ping();
      }
    }
    // else if (mode == 3)
    // {
      
    // }
    else if (mode == 4)
    {
      while(!button_pressed()) {
        uint8_t leds = 0;
        if (g_front_sensor > 30)
          leds |= RED_LEFT_LED;
        if (g_front_sensor > 60)
          leds |= RED_RIGHT_LED;
        if (g_front_sensor > 90)
          leds |= GREEN_LEFT_LED;
        if (g_front_sensor > 120)
          leds |= GREEN_RIGHT_LED;
        if (g_front_sensor > 150)
          leds |= BLUE_LEFT_LED;
        if (g_front_sensor > 180)
          leds |= BLUE_RIGHT_LED;
        turn_leds(leds);
        delay(2);
      }
    }
    else if (mode == 5)
    {
      test_mototrs();
    }
    else if (mode == 6)
    {
      // reset maze
      maze.reset_maze();
    }
    else if (mode == 7)
    {
      // change starting direction of mouse
      mouse.switch_start_direction();
      mouse.reset_mouse();
    }
    mouse.stop();
  }

  
}

void loop() {
  test_loop();
  // main_loop();
}