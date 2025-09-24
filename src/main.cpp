#include <Arduino.h>
// #include "reporter.h"
// #include "leds.h"
#include "mouse.h"
// #include "maze.h"
#include "motors.h"
#include "motor_tests.h"
#include "utils.h"

void setup() {
  init_encoders();
  init_sesnors();
  enable_steering();
}

void loop() {
  wait_to_start();
  mouse.reset_mouse();
  enable_motors();
  mouse.set_config(0);
  mouse.move_from_wall();
  mouse.move_half_cell();
  // mouse.turn_90_left_smooth();
  // mouse.turn_90_left_smooth();
  mouse.turn_90_right_smooth();
  mouse.turn_90_right_smooth();
  mouse.move_cell(true);
  stop_motors();
  disable_motors();
  delay(2000);
}