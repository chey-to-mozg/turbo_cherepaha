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
  mouse.wait_to_start();
  // calibrate_gyro();
  mouse.update_walls();
}


void loop() {
  digitalWrite(RIGHT_DIR, 0);
  analogWrite(RIGHT_PWM, 100);
  delay(1000);
  digitalWrite(RIGHT_DIR, 1);
  analogWrite(RIGHT_PWM, 100);
  delay(1000);
  // set_left_motor_pwm(50);
  // delay(1000);
  // set_left_motor_pwm(-50);
  // delay(1000);
  // set_left_motor_pwm(0);
  // set_right_motor_pwm(50);
  // delay(1000);
  // set_right_motor_pwm(-50);
  // delay(1000);
  // set_right_motor_pwm(0);
  // enable_mototrs();
  // reset_encoders();
  // reset_motor_controllers();

  // mouse.move_from_wall();

  // forward.stop();
  // disable_mototrs();
  // set_left_motor_pwm(0);
  // set_right_motor_pwm(0);
  // mouse.wait_to_start();
  // if (DEBUG_LOGGING) {
  //   // maze.load_maze();
  //   maze.print_maze();
  // }
  // mouse.run();

  // maze.save_maze();
  // mouse.error_ping();


  
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