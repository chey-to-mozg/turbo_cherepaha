#include "mouse.h"

Mouse mouse;

Mouse::Mouse() {
    init_leds();
    disable_steering();
    reset_encoders();
}

float Mouse::get_angle() {
    return angle;
}

uint8_t Mouse::stop() {
    stop_motors();
    disable_steering();
}

void Mouse::reset_mouse() {
    is_start = true;
    is_center = false;
    reset_encoders();
    maze.set_direction(UP);
    maze.set_position(maze.get_start());
}

void Mouse::print_info() {
    maze.print_maze();
    Serial.print("Current position: ");
    Serial.print(maze.get_position().y);
    Serial.print(" ");
    Serial.println(maze.get_position().x);
    Serial.print("Finish position: ");
    Serial.print(maze.get_finish().y);
    Serial.print(" ");
    Serial.println(maze.get_finish().x);
    Serial.print("On finish");
    Serial.print(" ");
    Serial.println(maze.get_finish() == maze.get_position());
    maze.print_path();
}

void Mouse::move(float distance, float speed, bool check_wall) {
    if (distance < 0) {
        speed *= -1;
    }
    motor_left.set_speed(speed);
    motor_right.set_speed(speed);
    float start_position = get_robot_position();
    while(abs(get_robot_position() - start_position) < abs(distance)) {
        read_sensors();
        update_motor_controllers();
        if (check_wall && g_front_sensor > FRONT_REFERENCE) {
            break;
        }
    }
}

// move until reference
// void Mouse::wait_until_position(float position) {
//   while (forward.position() < position) {
//     print_profile();
//     if (g_front_sensor > FRONT_REFERENCE) {
//         break;
//     }
//   }
// }


void Mouse::move_angle(float turn_angle, float speed) {
    // get ready to turn
    disable_steering();
    float left_speed = speed;
    float right_speed = speed;
    if (turn_angle < 0) {
        left_speed *= -1;
    }
    else {
        right_speed *= -1;
    }
    motor_left.set_speed(left_speed);
    motor_right.set_speed(right_speed);
    // while (abs(get_robot_angle() - this->angle) < abs(turn_angle)) {
    //     update_motor_controllers();
    //     print_motors();
    // }
    float cur_angle = 0;
    start_gyro_read();
    while (abs(cur_angle) < abs(turn_angle)) {
        update_motor_controllers();
        cur_angle += read_gyro();
        print_motors();
    }
    this->angle += turn_angle;
    stop_motors();
}

void Mouse::turn(float angle) {
    stop_motors();
    move_angle(angle, SPEEDMAX_SPIN_TURN);
}

uint8_t Mouse::wait_to_start(bool print_debug) {
    /*
    This function will return code of execution
    0 -- normal run from start to finish and back
    1 -- normal run from start to finish and back + save map
    2 -- normal run from start to finish with loaded map
    3 -- smooth run from start to finish and back
    4 -- smooth run from start to finish and back + save map
    5 -- smooth run from start to finish with loaded map
    6 -- ...
    7 -- Print maze info
    */
    uint8_t mode = 0;
    uint8_t signal = 0;
    read_sensors();
    while(!g_left_button) {
        read_sensors();
        update_encoders();
        if (print_debug) {
            print_sensors();
            print_motors();
            // print_profile();
        }
        if (g_right_button) {
            mode = (mode + 1) % 8;
            turn_mode_leds(mode, signal);
            delay(500);
        }
        turn_mode_leds(mode, signal);
        if (signal == 0) {
            signal = (maze.get_direction() == UP) ? 1 : 2;
        }
        else {
            signal = 0;
        }
        delay(200);
    }
    turn_all_leds();
    delay(2000);
    reset_leds();

    return mode;
}

void Mouse::error_ping() {
    disable_steering();
    bool signal = false;
    while (!button_pressed()) {
        read_sensors();
        if (signal) {
            turn_all_leds();
        }
        else {
            reset_leds();
        }
        delay(500);
        signal = !signal;
    }
    turn_all_leds();
    delay(1000);
}

void Mouse::finish_ping(int counts) {
    uint8_t leds = RED_LEFT_LED | RED_RIGHT_LED;
    turn_leds(leds);
    delay(500);
    leds |= GREEN_LEFT_LED | GREEN_RIGHT_LED;
    turn_leds(leds);
    delay(500);
    leds |= BLUE_LEFT_LED | BLUE_RIGHT_LED;
    turn_leds(leds);
    delay(500);
}

void Mouse::move_from_wall() {
    move(HALF_CELL - ROBOT_OFFSET, SPEEDMAX_EXPLORE);
}

// void Mouse::move_to_center() {
//     int speed;
//     if (front_wall) {
//         speed = SPEEDMAX_PRETURN_NORMAL;
//     }
//     else {
//         speed = SPEEDMAX_EXPLORE_NORMAL;
//     }
//     forward.start(HALF_CELL, speed, speed, SEARCH_ACCELERATION);
//     forward.set_position(ROBOT_OFFSET);
//     while(!forward.is_finished()) {
//         print_profile();
//         // delay(2); // wait for 1 update loop
//     }
//     forward.set_position(CELL);
// }

void Mouse::move_cell() {
    move(CELL, SPEEDMAX_EXPLORE);
}

void Mouse::move_backward() {
    move(-BACK_WALL_TO_CENTER, SPEEDMAX_EXPLORE);
}

// void Mouse::calibrate_with_front_wall() {
//     float remaining = CELL - forward.position();
//     forward.start(remaining, SPEEDMAX_PRETURN_NORMAL, SPEEDMAX_PRETURN_NORMAL, SEARCH_ACCELERATION);
//     if (front_wall) {
//         while (get_front_sensor() < FRONT_REFERENCE) {
//             delay(2);
//         }
//     }
//     else {
//         while(!forward.is_finished()) {
//             delay(2);
//         }
//     }
//     forward.stop();
//     forward.set_position(CELL);
// }

// void Mouse::turn_after_move(float angle) {
//     calibrate_with_front_wall();
//     turn(angle, SPEEDMAX_SPIN_TURN, SPIN_TURN_ACCELERATION);
// }

void Mouse::turn_90_left() {
    turn(-90);
}

void Mouse::turn_90_right() {
    turn(90);
}

// void Mouse::turn_90_left_smooth() {
//     float angle = 90;
//     float offset = 25;
//     float run_out = 20;
//     disable_steering();

//     // check when we reset position
//     float distance = CELL - forward.position() + offset;
//     forward.start(distance, forward.speed(), SPEEDMAX_PRETURN, SEARCH_ACCELERATION);

//     while (!forward.is_finished()) {
//         delay(2); // wait for 1 update loop
//         if (g_front_sensor > PRE_TURN_REFERENCE) {
//             forward.set_state(CS_FINISHED);
//             turn_wall_leds(false, true, false); // enable front wall leds
//         }
//     }

//     rotation.start(angle, SPEEDMAX_SMOOTH_TURN, 0, SPIN_TURN_ACCELERATION);
//     while (!rotation.is_finished()) {
//         delay(2); // wait for 1 update loop
//     }

//     forward.start(run_out, SPEEDMAX_EXPLORE, SPEEDMAX_EXPLORE, SEARCH_ACCELERATION);
//     while (not forward.is_finished()) {
//         delay(2);
//     }
//     forward.set_position(CELL - SENSING_OFFSET);
// }

// void Mouse::turn_90_right_smooth() {
//     float angle = -90;
//     float offset = 30;
//     float run_out = 15;
//     disable_steering();

//     float distance = CELL - forward.position() + offset;
//     forward.start(distance, forward.speed(), SPEEDMAX_PRETURN, SEARCH_ACCELERATION);

//     while (!forward.is_finished()) {
//         delay(2); // wait for 1 update loop
//         if (g_front_sensor > PRE_TURN_REFERENCE) {
//             forward.set_state(CS_FINISHED);
//             turn_wall_leds(false, true, false); // enable front wall leds
//         }
//     }

//     rotation.start(angle, SPEEDMAX_SMOOTH_TURN, 0, SPIN_TURN_SMOOTH_ACCELERATION);
//     while (!rotation.is_finished()) {
//         delay(2); // wait for 1 update loop
//     }

//     forward.start(run_out, SPEEDMAX_EXPLORE, SPEEDMAX_EXPLORE, SEARCH_ACCELERATION);
//     while (not forward.is_finished()) {
//         delay(2);
//     }
//     forward.set_position(CELL - SENSING_OFFSET);
// }

void Mouse::turn_around() {
    float angle = 180;
    turn(angle);
    if (front_wall) {
        move_backward();
    }
    stop_motors();
}

void Mouse::update_walls() {
    read_sensors();
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      left_wall = g_is_left_wall;
      front_wall = g_is_front_wall;
      right_wall = g_is_right_wall;
    }
    maze.set_walls(left_wall, front_wall, right_wall);
}

void print_maze_info(Pair target) {
    Serial.print("Current position: ");
    Serial.print(maze.get_position().y);
    Serial.print(" ");
    Serial.println(maze.get_position().x);
    Serial.print("Finish position: ");
    Serial.print(target.y);
    Serial.print(" ");
    Serial.println(target.x);
    Serial.print("On finish");
    Serial.print(" ");
    Serial.println(target == maze.get_position());
    maze.print_path();
}

// bool Mouse::run_smooth(bool to_finish, bool check_walls) {
//     // init wall before start
//     update_walls();
//     Pair target;
//     if (to_finish) {
//         target = maze.get_finish();
//     }
//     else {
//         target = maze.get_start();
//     }
//     maze.floodfill(target);

//     bool path_exists = maze.find_path(maze.get_position());
//     char next_path;
//     bool recalculate = false;

//     if (path_exists) {
//         reset_encoders();
//         reset_motor_controllers();
//         enable_mototrs();

//         while(path_exists && maze.get_position() != target) {
//             for (int i = 0; i < maze.get_path_len(); i++) {
//                 if (button_pressed()) {
//                     return false;
//                 }
//                 enable_steering();
//                 update_walls();

//                 next_path = maze.get_next_move();

//                 if (DEBUG_LOGGING) {
//                     print_info();
//                     Serial.print("Next move: ");
//                     Serial.println(next_path);
//                     wait_to_start(false);
//                 }

//                 if (DEBUG_LOGGING_WITH_MOTOTRS) {
//                     print_info();
//                     Serial.print("Next move: ");
//                     Serial.println(next_path);
//                 }

//                 if (is_start) {
//                     if (next_path == 'F') {
//                         if (!DEBUG_LOGGING) {
//                             move_from_wall();
//                         }
//                         maze.update_position();
//                         is_start = false;
//                         is_center = false;
//                     }
//                     else {
//                         recalculate = true;
//                     }
                    
//                 }
//                 else {
//                     switch (next_path)
//                     {
//                         case 'F':
//                             if (check_walls && front_wall) {
//                                 recalculate = true;
//                             }
//                             else {
//                                 if (!DEBUG_LOGGING) {
//                                     forward.adjust_position(-CELL);
//                                     wait_until_position(CELL - SENSING_OFFSET);
//                                 }
//                                 maze.update_position();
//                             }
//                             break;
//                         case 'R':
//                             if (check_walls && right_wall) {
//                                 recalculate = true;
//                             }
//                             else {
//                                 if (!DEBUG_LOGGING) {
//                                     turn_90_right_smooth();
//                                 }
//                                 maze.get_next_move(true); // after turn command it is forward command, so we should pop it
//                                 i++;
//                                 maze.update_direction(RIGHT);
//                                 maze.update_position();
//                             }
//                             break;
//                         case 'A':
//                             if (!DEBUG_LOGGING) {
//                                 turn_around();
//                             }
//                             maze.update_direction(DOWN);
//                             // set gyro error to zero
//                             break;
//                         case 'L':
//                             if (check_walls && left_wall) {
//                                 recalculate = true;
//                             }
//                             else {
//                                 if (!DEBUG_LOGGING) {
//                                     turn_90_left_smooth();
//                                 }
//                                 maze.get_next_move(true); // after turn command it is forward command, so we should pop it
//                                 i++;
//                                 maze.update_direction(LEFT);
//                                 maze.update_position();
//                             }
//                             break;
//                         default:
//                             // shouldnt exist
//                             forward.stop();
//                             rotation.stop();
//                             error_ping();
//                             break;
//                     }
                    
//                 }
//                 // check if mouse can move next step, otherwise floodfill
//                 if (recalculate) {
//                     maze.floodfill(maze.get_finish());
//                     path_exists = maze.find_path(maze.get_position());
//                     recalculate = false;
//                     if (DEBUG_LOGGING) {
//                         Serial.println("Recalculated!");
//                     }
//                     break;
//                 }
//                 delay(2);
//             }
//         }

//         if (path_exists) {
//             if (!DEBUG_LOGGING) {
//                 forward.start(HALF_CELL + SENSING_OFFSET, forward.speed(), 0, SEARCH_ACCELERATION);
//                 while (!forward.is_finished()) {
//                     delay(2);
//                     if (g_front_sensor > FRONT_REFERENCE) {
//                         break;
//                     }
//                 }
//                 is_center = true;
//             }
//         }
           
//     }

//     forward.stop();
//     disable_mototrs();
//     disable_steering();
//     stop_motors();

//     return path_exists;
// }

// bool Mouse::run_normal(bool to_finish) {
//     Pair target;
//     if (to_finish) {
//         target = maze.get_finish();
//     }
//     else {
//         target = maze.get_start();
//     }
//     maze.floodfill(target);
//     bool path_exists = maze.find_path(maze.get_position());
    
//     maze.print_maze();

//     char next_path;
//     bool recalculate = false;

//     if (path_exists) {
//         if (!DEBUG_LOGGING) {
//             reset_encoders();
//             reset_motor_controllers();
//             enable_mototrs();
//         }
        

//         while(path_exists && maze.get_position() != target) {
//             for (int i = 0; i < maze.get_path_len(); i++) {
//                 if (button_pressed()) {
//                     return false;
//                 }
//                 update_walls();
//                 maze.print_maze();

//                 enable_steering();

//                 next_path = maze.get_next_move();

//                 if (DEBUG_LOGGING) {
//                     print_maze_info(target);
//                     Serial.print("Next move: ");
//                     Serial.println(next_path);
//                     wait_to_start(false);                
//                 }

//                 if (is_start) {
//                     Serial.println("starting");
//                     if (!DEBUG_LOGGING) {
//                         move_to_center();
//                     }
//                     is_start = false;
//                     is_center = true;
//                 }
                
//                 // check if mouse can move next step, otherwise floodfill
//                 switch (next_path)
//                 {
//                     case 'F':
//                         if (front_wall) {
//                             recalculate = true;
//                         }
//                         else {
//                             if (!DEBUG_LOGGING) {
//                                 if (forward.speed() == 0 || forward.speed() == SPEEDMAX_PRETURN_NORMAL) {
//                                     forward.start(CELL, SPEEDMAX_EXPLORE_NORMAL, SPEEDMAX_EXPLORE_NORMAL, SEARCH_ACCELERATION);
//                                 }
//                                 else {
//                                     forward.adjust_position(-CELL);
//                                 }
//                                 wait_until_position(CELL - SENSING_OFFSET);
//                             }
//                             maze.update_position();
//                         }
//                         break;
//                     case 'R':
//                         if (right_wall) {
//                             recalculate = true;
//                         }
//                         else {
//                             if (!DEBUG_LOGGING) {
//                                 turn_90_right();
//                             }
//                             maze.update_direction(RIGHT);
//                         }
//                         break;
//                     case 'A':
//                         if (!DEBUG_LOGGING) {
//                             turn_around();
//                         }
//                         maze.update_direction(DOWN);
//                         // set gyro error to zero
//                         break;
//                     case 'L':
//                         if (left_wall) {
//                             recalculate = true;
//                         }
//                         else {
//                             if (!DEBUG_LOGGING) {
//                                 turn_90_left();
//                             }
//                             maze.update_direction(LEFT);
//                         }
//                         break;
//                     default:
//                         // shouldnt exist
//                         forward.stop();
//                         rotation.stop();
//                         error_ping();
//                         break;
//                 }
//                 if (recalculate) {
//                     maze.floodfill(target);
//                     path_exists = maze.find_path(maze.get_position());
//                     recalculate = false;
//                     maze.print_maze();
//                     if (DEBUG_LOGGING) {
//                         Serial.println("Recalculated!");
//                     }
//                     break;
//                 }
//             }
//         }
//     }
//     calibrate_with_front_wall();

//     forward.stop();
//     disable_mototrs();
//     disable_steering();
//     stop_motors();

//     return path_exists;
// }