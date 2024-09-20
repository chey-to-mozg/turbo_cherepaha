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
  enable_steering();
}


void loop() {
  // enable_steering();
  // reset_encoders();
  // reset_motor_controllers();
  // enable_mototrs();

  // mouse.move_from_wall();
  // forward.adjust_position(-CELL);
  // mouse.wait_until_position(CELL - SENSING_OFFSET);
  // // mouse.turn_90_right_smooth();

  // forward.stop();
  // // disable_steering();
  // disable_mototrs();
  // set_left_motor_pwm(0);
  // set_right_motor_pwm(0);
  // mouse.wait_to_start();

  // TODO make normal run for search

  // if (DEBUG_LOGGING) {
  //   // maze.load_maze();
  //   maze.print_maze();
  // }
  // mouse.run_smooth();
  mouse.run_normal();

  mouse.finish_ping();

  mouse.run_normal(false);

  mouse.wait_to_start();


  // // // maze.save_maze();
  // mouse.error_ping();

    
}