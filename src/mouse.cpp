#include "mouse.h"

Mouse mouse;

Mouse::Mouse() {
    init_leds();
    disable_steering();
}

void Mouse::stop() {
    forward.stop();
    disable_steering();
    disable_mototrs();
    stop_motors();
    mouse.wait_to_start();
}

void Mouse::move(float distance, float max_speed, bool check_wall) {

    forward.start(distance, max_speed, SPEEDMAX_EXPLORE, SEARCH_ACCELERATION);
    while(!forward.is_finished()) {
        if (check_wall && g_front_sensor > FRONT_REFERENCE) {
            break;
        }
        delay(2); // wait for 1 update loop
        // print_profile();
    }
}

// move until reference
void Mouse::wait_until_position(float position) {
  while (forward.position() < position) {
    delay(2);
    if (g_front_sensor > FRONT_REFERENCE) {
        break;
    }
  }
}


void Mouse::move_angle(float angle) {
    // get ready to turn
    disable_steering();
    rotation.reset();
    rotation.start(angle, SPEEDMAX_SPIN_TURN, 0, SPIN_TURN_ACCELERATION);
    while (!rotation.is_finished()) {
        print_profile();
    }
    enable_steering();
}

uint8_t Mouse::wait_to_start(bool print_debug) {
    /*
    This function will return code of execution
    0 -- normal run from start to finish and back
    1 -- normal run from start to finish and back + save map
    2 -- smooth run from start to finish and back
    3 -- smooth run from start to finish and back + save map
    4 -- normal run from start to finish with loaded map
    5 -- smooth run from start to finish with loaded map
    */
    uint8_t mode = 0;
    bool signal = false;

    while(!g_left_button) {
        if (print_debug) {
            print_sensors();
            print_motors();
            print_profile();
        }
        if (g_right_button) {
            mode = (mode + 1) % 6;
            turn_mode_leds(mode, signal);
            delay(500);
        }
        turn_mode_leds(mode, signal);
        signal = !signal;
        delay(200);
    }
    turn_all_leds();
    delay(2000);
    reset_leds();
}

void Mouse::error_ping() {
    disable_steering();
    bool signal = false;
    while (true) {
        if (signal) {
            turn_all_leds();
        }
        else {
            reset_leds();
        }
        delay(500);
        signal = !signal;
    } 
    
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
    forward.start(CELL - SENSING_OFFSET, SPEEDMAX_EXPLORE, SPEEDMAX_EXPLORE, SEARCH_ACCELERATION);
    forward.set_position(ROBOT_OFFSET);
    while(!forward.is_finished()) {
        print_profile();
        // delay(2); // wait for 1 update loop
    }
    
}

void Mouse::move_to_center() {
    forward.start(HALF_CELL, SPEEDMAX_EXPLORE, SPEEDMAX_EXPLORE, SEARCH_ACCELERATION);
    forward.set_position(ROBOT_OFFSET);
    while(!forward.is_finished()) {
        print_profile();
        // delay(2); // wait for 1 update loop
    }
    forward.set_position(CELL);
    
}

void Mouse::move_cell() {
    move(CELL, SPEEDMAX_EXPLORE);
}

void Mouse::move_backward() {
    move(-BACK_WALL_TO_CENTER, SPEEDMAX_EXPLORE);
}

void Mouse::turn_90_left() {
    forward.set_target_speed(SPEEDMAX_PRETURN);
    wait_until_position(CELL);
    forward.stop();
    stop_motors();
    
    float angle = 90;
    move_angle(angle);
    
}

void Mouse::turn_90_right() {
    forward.set_target_speed(SPEEDMAX_PRETURN);
    wait_until_position(CELL);
    forward.stop();
    stop_motors();
    
    float angle = -90;
    move_angle(angle);
}

void Mouse::turn_90_left_smooth() {
    float angle = 90;
    float offset = 20;
    float run_out = 20;
    disable_steering();

    // check when we reset position
    float distance = CELL + SENSING_OFFSET + offset - forward.position();
    forward.start(distance, forward.speed(), SPEEDMAX_PRETURN, SEARCH_ACCELERATION);

    while (!forward.is_finished()) {
        delay(2); // wait for 1 update loop
        if (g_front_sensor > PRE_TURN_REFERENCE) {
            forward.set_state(CS_FINISHED);
            turn_wall_leds(false, true, false); // enable front wall leds
        }
    }

    rotation.start(angle, SPEEDMAX_SMOOTH_TURN, 0, SPIN_TURN_ACCELERATION);
    while (!rotation.is_finished()) {
        delay(2); // wait for 1 update loop
    }

    forward.start(run_out, forward.speed(), SPEEDMAX_EXPLORE, SEARCH_ACCELERATION);
    while (not forward.is_finished()) {
        delay(2);
    }
    forward.set_position(CELL - SENSING_OFFSET);
}

void Mouse::turn_90_right_smooth() {
    float angle = -90;
    float offset = 30;
    float run_out = 15;
    disable_steering();

    float distance = CELL + SENSING_OFFSET + offset - forward.position();
    forward.start(distance, forward.speed(), SPEEDMAX_PRETURN, SEARCH_ACCELERATION);

    while (!forward.is_finished()) {
        delay(2); // wait for 1 update loop
        if (g_front_sensor > PRE_TURN_REFERENCE) {
            forward.set_state(CS_FINISHED);
            turn_wall_leds(false, true, false); // enable front wall leds
        }
    }

    rotation.start(angle, SPEEDMAX_SMOOTH_TURN, 0, SPIN_TURN_SMOOTH_ACCELERATION);
    while (!rotation.is_finished()) {
        delay(2); // wait for 1 update loop
    }

    forward.start(run_out, forward.speed(), SPEEDMAX_EXPLORE, SEARCH_ACCELERATION);
    while (not forward.is_finished()) {
        delay(2);
    }
    forward.set_position(CELL - SENSING_OFFSET);
}

void Mouse::turn_around() {
    
    reset_steering();
    if (!is_center) {
        forward.start(HALF_CELL + SENSING_OFFSET, forward.speed(), 30, forward.acceleration());
        while (!forward.is_finished()) {
            delay(2);
            if (g_front_sensor < FRONT_REFERENCE) {
                break;
            }
        }
        is_center = true;
    }
    forward.stop();
    float angle = 180;
    move_angle(angle);
    if (front_wall) {
        forward.start(-HALF_CELL, SPEEDMAX_EXPLORE, 30, forward.acceleration());
        while (!forward.is_finished()) {
            delay(2);
        }
        is_start = true;
        is_center = false;
    }
}

void Mouse::update_walls() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      left_wall = g_is_left_wall;
      front_wall = g_is_front_wall;
      right_wall = g_is_right_wall;
    }
    maze.set_walls(left_wall, front_wall, right_wall);
}

bool Mouse::run_smooth(bool to_finish, bool check_walls) {
    Pair target;
    if (to_finish) {
        target = maze.get_finish();
    }
    else {
        target = maze.get_start();
    }
    maze.floodfill(target);

    bool path_exists = maze.find_path(maze.get_position());
    char next_path;
    bool recalculate = false;

    if (path_exists) {
        reset_encoders();
        reset_motor_controllers();
        enable_mototrs();

        while(path_exists && maze.get_position() != target) {
            for (int i = 0; i < maze.get_path_len(); i++) {

                enable_steering();
                update_walls();
                maze.print_maze();

                next_path = maze.get_next_move();

                if (DEBUG_LOGGING) {
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
                    Serial.print("Next move: ");
                    Serial.println(next_path);
                    maze.print_path();
                    wait_to_start(false);
                }

                if (is_start) {
                    if (!DEBUG_LOGGING) {
                        move_from_wall();
                    }
                    maze.update_position();
                    is_start = false;
                    is_center = false;
                }
                else {
                    switch (next_path)
                    {
                        case 'F':
                            if (check_walls && front_wall) {
                                recalculate = true;
                            }
                            else {
                                if (!DEBUG_LOGGING) {
                                    forward.adjust_position(-CELL);
                                    wait_until_position(CELL - SENSING_OFFSET);
                                }
                                maze.update_position();
                            }
                            break;
                        case 'R':
                            if (check_walls && right_wall) {
                                recalculate = true;
                            }
                            else {
                                if (!DEBUG_LOGGING) {
                                    turn_90_right_smooth();
                                }
                                maze.get_next_move(true); // after turn command it is forward command, so we should pop it
                                i++;
                                maze.update_direction(RIGHT);
                                maze.update_position();
                                float f = g_front_sensor;
                                float g_w = g_is_front_wall;
                            }
                            break;
                        case 'A':
                            if (!DEBUG_LOGGING) {
                                turn_around();
                            }
                            maze.update_direction(DOWN);
                            // set gyro error to zero
                            break;
                        case 'L':
                            if (check_walls && left_wall) {
                                recalculate = true;
                            }
                            else {
                                if (!DEBUG_LOGGING) {
                                    turn_90_left_smooth();
                                }
                                maze.get_next_move(true); // after turn command it is forward command, so we should pop it
                                i++;
                                maze.update_direction(LEFT);
                                maze.update_position();
                            }
                            break;
                        default:
                            // shouldnt exist
                            forward.stop();
                            rotation.stop();
                            error_ping();
                            break;
                    }
                    
                }
                // check if mouse can move next step, otherwise floodfill
                if (recalculate) {
                    maze.floodfill(maze.get_finish());
                    path_exists = maze.find_path(maze.get_position());
                    recalculate = false;
                    if (DEBUG_LOGGING) {
                        Serial.println("Recalculated!");
                    }
                    break;
                }
                delay(2);
            }
        }

        if (path_exists) {
            if (!DEBUG_LOGGING) {
                forward.start(HALF_CELL + SENSING_OFFSET, forward.speed(), 0, SEARCH_ACCELERATION);
                while (!forward.is_finished()) {
                    delay(2);
                    if (g_front_sensor > FRONT_REFERENCE) {
                        break;
                    }
                }
                is_center = true;
            }
        }
           
    }

    if (!to_finish) {
        if (!DEBUG_LOGGING) {
            turn_around();
        }
        maze.update_direction(DOWN);
    }

    forward.stop();
    disable_mototrs();
    disable_steering();
    stop_motors();

    return path_exists;
}

bool Mouse::run_normal(bool to_finish) {
    Pair target;
    if (to_finish) {
        target = maze.get_finish();
    }
    else {
        target = maze.get_start();
    }
    maze.floodfill(target);
    bool path_exists = maze.find_path(maze.get_position());
    
    maze.print_maze();

    char next_path;
    bool recalculate = false;

    if (path_exists) {
        reset_encoders();
        reset_motor_controllers();
        enable_mototrs();

        while(path_exists && maze.get_position() != target) {
            for (int i = 0; i < maze.get_path_len(); i++) {
                enable_steering();
                next_path = maze.get_next_move();
                if (DEBUG_LOGGING) {
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
                    Serial.print("Next move: ");
                    Serial.println(next_path);
                    maze.print_path();
                    wait_to_start(false);                
                }
                if (is_start) {
                    Serial.println("starting");
                    if (!DEBUG_LOGGING) {
                        move_to_center();
                    }
                    is_start = false;
                    is_center = true;
                }
                
                // check if mouse can move next step, otherwise floodfill
                Serial.println("try move");
                switch (next_path)
                {
                    case 'F':
                        if (front_wall) {
                            recalculate = true;
                        }
                        else {
                            if (!DEBUG_LOGGING) {
                                if (is_center) {
                                    float remaining = forward.position() - CELL;
                                    forward.start(CELL - remaining, SPEEDMAX_EXPLORE, SPEEDMAX_EXPLORE, SEARCH_ACCELERATION);
                                    wait_until_position(HALF_CELL);
                                    is_center = false;
                                }
                                else {
                                    forward.adjust_position(-HALF_CELL);
                                    wait_until_position(CELL);
                                }
                                // char after_path = maze.get_next_move(false);
                                // if (after_path == 'R' || after_path == 'L') {
                                //     forward.set_target_speed(SPEEDMAX_PRETURN);
                                // }
                                // else if (forward.speed() != SPEEDMAX_EXPLORE) {
                                //     forward.set_target_speed(SPEEDMAX_EXPLORE);
                                // }
                                
                                
                            }
                            maze.update_position();
                        }
                        break;
                    case 'R':
                        if (right_wall) {
                            recalculate = true;
                        }
                        else {
                            if (!DEBUG_LOGGING) {
                                turn_90_right();
                                is_center = true;
                            }
                            maze.update_direction(RIGHT);
                        }
                        break;
                    case 'A':
                        if (!DEBUG_LOGGING) {
                            turn_around();
                        }
                        maze.update_direction(DOWN);
                        // set gyro error to zero
                        break;
                    case 'L':
                        if (left_wall) {
                            recalculate = true;
                        }
                        else {
                            if (!DEBUG_LOGGING) {
                                turn_90_left();
                                is_center = true;
                            }
                            maze.update_direction(LEFT);
                        }
                        break;
                    default:
                        // shouldnt exist
                        forward.stop();
                        rotation.stop();
                        error_ping();
                        break;
                }
                if (recalculate) {
                    maze.floodfill(target);
                    path_exists = maze.find_path(maze.get_position());
                    recalculate = false;
                    maze.print_maze();
                    if (DEBUG_LOGGING) {
                        Serial.println("Recalculated!");
                    }
                    stop();
                    break;
                    
                }
                else {
                    update_walls();
                    maze.print_maze();
                }
            }
        }
    }

    if (!to_finish) {
        if (!DEBUG_LOGGING) {
            turn_around();
        }
        maze.update_direction(DOWN);
    }

    forward.stop();
    disable_mototrs();
    disable_steering();
    stop_motors();

    return path_exists;
}