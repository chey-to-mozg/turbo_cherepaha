#include <Arduino.h>
#include "reporter.h"
#include "motors.h"
#include "leds.h"
#include "mouse.h"
#include "maze.h"
#include "systick.h"
#include "profile.h"

uint8_t mode = 0;

void setup() {
  init_serial();
  // init_bluetooth();
  init_sesnors();
  init_encoders();
  init_motors();
  init_leds();
  init_systick();
  forward.reset();
  rotation.reset();
  delay(3000); // wait for sensor data dump
  mode = mouse.wait_to_start();
  // calibrate_gyro();
  mouse.update_walls();
}

void test_run() {
  // enable_steering();
  // reset_encoders();
  // reset_motor_controllers();
  // enable_mototrs();

  // mouse.move(100, SPEEDMAX_EXPLORE);
  // for (int i = 0; i < 9; i++) {
  //   forward.adjust_position(-100);
  //   mouse.wait_until_position(100);
  // }
  maze.load_maze();
  maze.print_maze();
  maze.lock_maze();
  maze.floodfill(maze.get_finish());
  maze.print_maze();
  maze.find_path(maze.get_start());
  maze.print_path();
  // mouse.move_to_center();
  // forward.adjust_position(-CELL);
  // mouse.wait_until_position(CELL - SENSING_OFFSET);
  // float remaining = CELL - forward.position();
  // forward.start(remaining, SPEEDMAX_PRETURN_NORMAL, SPEEDMAX_PRETURN_NORMAL, SEARCH_ACCELERATION);
  // while(!forward.is_finished()) {
  //     delay(2);
  //     if (get_front_sensor() > FRONT_REFERENCE) {
  //         break;
  //     }
  // }
  // forward.stop();
  // mouse.turn_90_right_smooth();
  // mouse.turn_around();

  // mouse.move_from_wall();
  // mouse.turn_90_right_smooth();
  // mouse.turn_90_right_smooth();

  // // forward.adjust_position(-CELL);
  // // mouse.wait_until_position(CELL - SENSING_OFFSET);
  // mouse.turn_90_right_smooth();
  // mouse.turn_90_right_smooth();

  // mouse.move_to_center();

  // forward.adjust_position(-CELL);
  // mouse.wait_until_position(CELL);

  // // forward.adjust_position(-CELL);
  // // mouse.wait_until_position(CELL);

  // mouse.turn_90_right();

  // // mouse.move_cell();


  // // mouse.move_from_wall();
  // // forward.adjust_position(-CELL);
  // // mouse.wait_until_position(CELL - SENSING_OFFSET);
  // // // mouse.turn_90_right_smooth();
  // int l = g_left_sensor;
  // int f = g_front_sensor;
  // int r = g_right_sensor;
  mouse.stop();
  // Serial.println(l);
  // Serial.println(f);
  // Serial.println(r);
  // delay(2000);
}

void loop() {
  if (mode == 0){
    //** NORMAL RUN **/

    mouse.reset_mouse();
    maze.reset_maze();

    bool finished = mouse.run_normal();
    if (finished) {
      mouse.finish_ping();
      finished = mouse.run_normal(false);
    }

    if (!finished) {
      mouse.error_ping();
      mouse.print_info();
    }
  }
  else if (mode == 1)
  {
    //** NORMAL RUN WITH MAP SAVING **/

    mouse.reset_mouse();
    maze.reset_maze();

    bool finished = mouse.run_normal();
    if (finished) {
      mouse.finish_ping();
      finished = mouse.run_normal(false);
    }

    if (!finished) {
      mouse.error_ping();
      mouse.print_info();
    }

    maze.save_maze();
  }
  else if (mode == 2)
  {
    //** NORMAL RUN WITH MAP LOADING **/

    mouse.reset_mouse();
    maze.reset_maze();

    maze.load_maze();
    maze.lock_maze();

    bool finished = mouse.run_normal();
    if (finished) {
      mouse.finish_ping();
      finished = mouse.run_normal(false);
    }

    if (!finished) {
      mouse.error_ping();
      mouse.print_info();
    }
  }
  else if (mode == 3)
  {
    // smouth run and back
    bool finished = mouse.run_smooth();
    if (finished) {
      mouse.finish_ping();
      finished = mouse.run_smooth(false);
    }

    if (!finished) {
      mouse.error_ping();
      mouse.print_info();
    }
    
    mouse.reset_mouse();
    maze.reset_maze();
  }
  else if (mode == 4)
  {
    /* code */
  }
  else if (mode == 5)
  {
    if (maze.get_direction() == UP) {
      maze.set_direction(RIGHT);
    }
    else {
      maze.set_direction(UP);
    }
  }
  else if (mode == 6)
  {
    test_run();
  }
  else if (mode == 7)
  {
    // print maze related info
      Serial.println("Maze before load");
      maze.print_maze();
      maze.load_maze();
      maze.floodfill(maze.get_finish());
      Serial.println("Maze after load");
      mouse.print_info();
      while (!button_pressed())
      {
        delay(100);
      }
      
  }

  mode = mouse.stop();
}