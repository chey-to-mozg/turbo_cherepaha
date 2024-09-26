#include <Arduino.h>
#include "reporter.h"
#include "motors.h"
#include "leds.h"
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
  init_leds();
  init_systick();
  forward.reset();
  rotation.reset();
  delay(3000); // wait for sensor data dump
  mouse.wait_to_start();
  // calibrate_gyro();
  mouse.update_walls();
}


void loop() {
  // enable_steering();
  // reset_encoders();
  // reset_motor_controllers();
  // enable_mototrs();

  // mouse.move_from_wall();
  // forward.adjust_position(-CELL);
  // mouse.wait_until_position(CELL - SENSING_OFFSET);
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
  // mouse.stop();
  // Serial.println(l);
  // Serial.println(f);
  // Serial.println(r);
  // delay(2000);

  // // TODO make normal run for search

  // // if (DEBUG_LOGGING) {
  // //   // maze.load_maze();
  // //   maze.print_maze();
  // // }
  // // mouse.run_smooth();
  
  
  bool finished = mouse.run_smooth();
  // bool finished = mouse.run_normal();

  if (finished) {
    mouse.finish_ping();
  } 
  else {
    mouse.error_ping();
  }
  
  finished = mouse.run_smooth(false);

  if (!finished) {
    mouse.error_ping();
  }

  mouse.wait_to_start();

  finished = mouse.run_smooth(true, false);

  mouse.wait_to_start();


  // // // maze.save_maze();
  // mouse.error_ping();

    
}