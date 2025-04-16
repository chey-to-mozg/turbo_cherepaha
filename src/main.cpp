#include <Arduino.h>
#include "reporter.h"
#include "leds.h"
#include "mouse.h"
#include "maze.h"
// #include "systick.h"
// #include "profile.h"

uint8_t mode = 0;

void setup() {
  init_serial();
  // init_bluetooth();
  init_sesnors();
  init_encoders();
  init_leds();
  delay(3000); // wait for sensor data dump
  
  // calibrate_gyro();
  mouse.update_walls();
}

void test_run() {
  mouse.move(106.7, 200);
  // mouse.move_cell();
  // mouse.turn_90_right();
  mouse.stop();
  while(!button_pressed()) {
    update_encoders();
    print_motors();
  }
  // mouse.move_from_wall();
  // mouse.move_cell();
  // mouse.turn_90_right();
  // mouse.move_cell();
  // mouse.turn_90_right();
  // mouse.move_cell();
  // mouse.turn_90_left();
  // mouse.move_cell();
  // mouse.turn_90_left();
  // mouse.move_cell();
  // mouse.move_cell();
  // mouse.turn_90_left();
  // mouse.move_cell();
  // mouse.move_cell();
  // mouse.turn_90_left();
  // mouse.move_cell();
  // mouse.move_cell();
}
void loop() {
  mode = mouse.wait_to_start();
  if (mode == 0){
    //** NORMAL RUN **//
    // mouse.reset_mouse();
    // maze.reset_maze();
    test_run();
    // bool finished = mouse.run_normal();
    // if (finished) {
    //   mouse.finish_ping();
    //   finished = mouse.run_normal(false);
    // }

    // if (!finished) {
    //   mouse.error_ping();
    //   mouse.print_info();
    // }
  }
  // else if (mode == 1)
  // {
  //   //** NORMAL RUN WITH MAP SAVING **/

  //   mouse.reset_mouse();
  //   maze.reset_maze();

  //   bool finished = mouse.run_normal();
  //   if (finished) {
  //     mouse.finish_ping();
  //     finished = mouse.run_normal(false);
  //   }

  //   if (!finished) {
  //     mouse.error_ping();
  //     mouse.print_info();
  //   }

  //   maze.save_maze();
  // }
  // else if (mode == 2)
  // {
  //   //** NORMAL RUN WITH MAP LOADING **/

  //   mouse.reset_mouse();
  //   maze.reset_maze();

  //   maze.load_maze();
  //   maze.lock_maze();

  //   bool finished = mouse.run_normal();
  //   if (finished) {
  //     mouse.finish_ping();
  //     finished = mouse.run_normal(false);
  //   }

  //   if (!finished) {
  //     mouse.error_ping();
  //     mouse.print_info();
  //   }
  // }
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
    test_run();
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