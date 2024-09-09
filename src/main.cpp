#include <Arduino.h>
#include "reporter.h"
#include "motors.h"
#include "mouse.h"
#include "maze.h"
#include "systick.h"
#include "profile.h"

void setup() {
  init_serial();
  // init_bluetooth();
  init_sesnors();
  init_encoders();
  init_motors();
  init_systick();
  forward.reset();
  rotation.reset();
  mouse.wait_to_start();
  calibrate_gyro();
  mouse.update_walls();
}


void loop() {
  reset_encoders();
  reset_motor_controllers();
  enable_mototrs();
  enable_steering();
  mouse.move(1000, 300);
  while (!button_pressed())
  {
    print_gyro();
    print_debug();
    delay(100);
  }
  
  // mouse.move_from_wall();
  // // mouse.turn_90_left_smooth();
  // mouse.turn_90_right_smooth();
  // // mouse.turn_90_left_smooth();
  // // mouse.turn_90_left_smooth();
  // // mouse.move_cell();
  // forward.stop();
  // disable_mototrs();
  // stop_motors();
  // mouse.wait_to_start();
  
  // print_sensors();
  // maze.print_maze();
  // mouse.run();
  // // while (true)
  // // {
  // //   delay(1000);
  // // }
  
    // mouse.wait_to_start();
    // bool signal = true;
    // while(!button_pressed()) {
    //     if (g_left_button) {
    //       maze.find_path({15, 0});
    //       maze.set_walls(1, 0, 1);
    //       maze.update_position();
    //       maze.set_walls(1, 1, 0);
    //       maze.floodfill(maze.get_finish());
    //       maze.find_path(maze.get_position());
    //       maze.print_maze();
    //       maze.update_direction(1);
    //       maze.update_position();
    //       maze.set_walls(1, 1, 0);
    //       maze.floodfill(maze.get_finish());
    //       maze.find_path(maze.get_position());
    //       maze.print_maze();
    //       maze.update_direction(1);
    //       maze.update_position();
    //       maze.set_walls(0, 1, 1);
    //       maze.floodfill(maze.get_finish());
    //       maze.find_path(maze.get_position());
    //       maze.print_maze();
    //       maze.update_direction(3);
    //       for (int i = 0; i < 10; i++) {
    //         maze.update_position();
    //         maze.set_walls(1, 0, 1);
    //         maze.floodfill(maze.get_finish());
    //         maze.find_path(maze.get_position());
    //       }
    //       maze.print_maze();
    //       maze.save_maze();
    //     }
    //     else if (g_right_button) {
    //       maze.print_maze();
    //       maze.load_maze();
    //       maze.floodfill(maze.get_finish());
    //       maze.print_maze();
    //     }
    //     digitalWrite(LED_GREEN, signal);
    //     signal = !signal;
    //     delay(300);
    // }
    
}