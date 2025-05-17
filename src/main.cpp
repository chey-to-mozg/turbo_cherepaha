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
  delay(3000); // wait for sensor data dump
  mouse.update_walls();
}

void test_run() {
  // enable_steering();
  // motor_left.set_speed(300);
  // motor_right.set_speed(300);
  // while(true) {
  //   update_motor_controllers();
  // }
  // check_pwm_control();
  // while(true) {
  //   print_sensors();
  //   delay(1000);
  // }
  mouse.set_config(1);
  enable_motors();
  mouse.move_from_wall();
  mouse.move_half_cell();
  // mouse.turn_90_right_smooth();
  mouse.turn_90_left_smooth();
  // mouse.turn_90_right_smooth();
  // mouse.turn_90_right_smooth();
  // mouse.turn_90_right_smooth();
  // mouse.move_cell();
  // mouse.turn_90_left_smooth();
  // mouse.turn_90_left_smooth();
  // mouse.turn_90_left_smooth();
  // mouse.move_cell();
  // mouse.turn_90_left_smooth();
  // mouse.move_cell();
  // mouse.turn_90_left_smooth();
  // mouse.move_cell();
  // mouse.move_cell(true);
}

void test_loop() {
  while (1) {
    mode = mouse.wait_to_start();
    if (mode == 0)
    {
      test_run();
    }
    else if (mode == 1) {
      check_speed();
    }
    
    mouse.stop();
  }
  
}

void main_loop() {
  while (1) {
    mode = mouse.wait_to_start();
    // while(true) {
    //   print_sensors();
    //   delay(1000);
    // }

    if (mode == 0){
      //** NORMAL RUN **//
      mouse.reset_mouse();
      mouse.set_config(0);
      maze.reset_maze();
      
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
      maze.reset_maze();

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
      //** NORMAL RUN WITH MAP LOADING **/

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
    //   // smouth run and back

    //   mouse.reset_mouse();
    //   maze.reset_maze();
      
    //   maze.load_maze();
    //   maze.lock_maze();

    //   bool finished = mouse.run_smooth();
    //   if (finished) {
    //     mouse.finish_ping();
    //     finished = mouse.run_smooth(false);
    //   }

    //   if (!finished) {
    //     mouse.error_ping();
    //     mouse.print_info();
    //   }
    // }
    // else if (mode == 4)
    // {
    //   // smouth run and back

    //   mouse.reset_mouse();
    //   maze.reset_maze();

    //   maze.set_direction(RIGHT);
      
    //   maze.load_maze();
    //   maze.lock_maze();

    //   bool finished = mouse.run_smooth();
    //   if (finished) {
    //     mouse.finish_ping();
    //     finished = mouse.run_smooth(false);
    //   }

    //   if (!finished) {
    //     mouse.error_ping();
    //     mouse.print_info();
    //   }
    // }
    // else if (mode == 5)
    // {
    //   if (maze.get_direction() == UP) {
    //     maze.set_direction(RIGHT);
    //   }
    //   else {
    //     maze.set_direction(UP);
    //   }
    // }
    else if (mode == 6)
    {
      
    }
    // else if (mode == 7)
    // {
    //   // print maze related info
    //     Serial.println("Maze before load");
    //     maze.print_maze();
    //     maze.load_maze();
    //     maze.floodfill(maze.get_finish());
    //     Serial.println("Maze after load");
    //     mouse.print_info();
    //     while (!button_pressed())
    //     {
    //       delay(100);
    //     }
        
    // }
    mouse.stop();
  }

  
}

void loop() {
  // test_loop();
  main_loop();
}