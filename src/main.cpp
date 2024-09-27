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
}

void loop() {
  Serial.println(mode);
  if (mode == 0){
    // TODO add normal run and back here
  }
  else if (mode == 1)
  {
    /* code */
  }
  else if (mode == 2)
  {
    // smouth run and back
    bool finished = mouse.run_smooth();
    if (finished) {
      mouse.finish_ping();
      finished = mouse.run_smooth(false);
    }

    if (!finished) {
      mouse.error_ping();
    }
    
    mouse.reset_mouse();
    maze.reset_maze();
  }
  else if (mode == 3)
  {
    
  }
  else if (mode == 4)
  {
    /* code */
  }
  else if (mode == 5)
  {
    /* code */
  }
  else if (mode == 6)
  {
    /* code */
  }
  else if (mode == 7)
  {
    // print maze related info
      mouse.print_info();
      while (!button_pressed())
      {
        delay(100);
      }
      
  }

  mode = mouse.stop();
}