#include "mouse.h"

Mouse mouse;

Mouse::Mouse() {
    init_leds();
    disable_steering();
}

void Mouse::init_leds() {
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
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
        delay(2); // wait for 1 update loop
    }
    enable_steering();
}

void Mouse::wait_to_start() {
    init_loading_leds();
    while(!button_pressed()) {
        step_loading_leds();
        delay(300);
    }
    turn_all_leds();
    delay(2000);
    reset_leds();
}

void Mouse::wait_to_start_front() {
    // bool signal = true;
    // while(g_front_sensor < FRONT_ACTIVATE_THRESHOLD) {
    //     digitalWrite(LED_GREEN, signal);
    //     signal = !signal;
    //     delay(300);
    // }
    // digitalWrite(LED_GREEN, true);
    // delay(2000);
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
    for (int i = 0; i < 3; i++) {
        turn_all_leds();
        delay(500);
        reset_leds();
        delay(500);
    }
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
    forward.stop();
    delay(5);
    stop_motors();
    
    float angle = 90;
    move_angle(angle);
    
}

void Mouse::turn_90_right() {
    forward.stop();
    delay(5);
    stop_motors();
    
    float angle = -90;
    move_angle(angle);
    
}

void Mouse::turn_90_left_smooth() {
    float angle = 90;
    float offset = 10;
    float run_out = 20;
    disable_steering();

    // check when we reset position
    float distance = CELL + SENSING_OFFSET + offset - forward.position();
    forward.start(distance, forward.speed(), SPEEDMAX_PRETURN, SEARCH_ACCELERATION);

    while (!forward.is_finished()) {
        delay(2); // wait for 1 update loop
        if (g_front_sensor > PRE_TURN_REFERENCE) {
            forward.set_state(CS_FINISHED);
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
    float offset = 0;
    float run_out = 10;
    disable_steering();

    float distance = CELL + SENSING_OFFSET + offset - forward.position();
    forward.start(distance, forward.speed(), SPEEDMAX_PRETURN, SEARCH_ACCELERATION);

    if (g_is_front_wall) {
        while (g_front_sensor < PRE_TURN_REFERENCE) {
            delay(2);
        }
        forward.set_state(CS_FINISHED);
    }
    else {
        while (!forward.is_finished()) {
            delay(2); // wait for 1 update loop
        // if (g_front_sensor > 70) {
        //     forward.set_state(CS_FINISHED);
        //     digitalWrite(LED_GREEN, true);
        // }
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

void Mouse::turn_around(bool enter_cell) {
    
    reset_steering();
    if (enter_cell) {
        forward.start(HALF_CELL, forward.speed(), 30, forward.acceleration());
        if (g_is_front_wall) {
            while (g_front_sensor < FRONT_REFERENCE) {
                delay(2);
            }
            
        } else {
            while (!forward.is_finished()) {
                delay(2);
            }
        }
        forward.stop();
    }
    float angle = 180;
    move_angle(angle);
    forward.start(-HALF_CELL, SPEEDMAX_EXPLORE, 30, forward.acceleration());
    while (!forward.is_finished()) {
        delay(2);
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

void Mouse::run_smooth() {
    maze.floodfill(maze.get_finish());
    maze.find_path(maze.get_position());
    char next_path;
    bool recalculate = false;

    reset_encoders();
    reset_motor_controllers();
    enable_mototrs();

    while(maze.get_finish() != maze.get_position()) {
        for (int i = 0; i < maze.get_path_len(); i++) {
            next_path = maze.get_next_move();
            enable_steering();
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
                wait_to_start_front();
            }
            if (is_start) {
                if (!DEBUG_LOGGING) {
                    move_from_wall();
                }
                maze.update_position();
                is_start = false;
            }
            
            // check if mouse can move next step, otherwise floodfill
            switch (next_path)
            {
                case 'F':
                    if (front_wall) {
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
                    if (right_wall) {
                        recalculate = true;
                    }
                    else {
                        if (!DEBUG_LOGGING) {
                            turn_90_right_smooth();
                        }
                        maze.update_direction(RIGHT);
                        maze.update_position();
                    }
                    break;
                case 'A':
                    if (!DEBUG_LOGGING) {
                        turn_around();
                    }
                    is_start = true;
                    maze.update_direction(DOWN);
                    // set gyro error to zero
                    break;
                case 'L':
                    if (left_wall) {
                        recalculate = true;
                    }
                    else {
                        if (!DEBUG_LOGGING) {
                            turn_90_left_smooth();
                        }
                        maze.update_direction(LEFT);
                        maze.update_position();
                    }
                    break;
                default:
                    // shouldnt exist
                    error_ping();
                    forward.stop();
                    rotation.stop();
                    break;
            }
            if (recalculate) {
                maze.floodfill(maze.get_finish());
                maze.find_path(maze.get_position());
                recalculate = false;
                maze.print_maze();
                if (DEBUG_LOGGING) {
                    Serial.println("Recalculated!");
                }
                
            }
            else {
                update_walls();
                maze.print_maze();
            }
        }
    }
    forward.start(HALF_CELL + SENSING_OFFSET, forward.speed(), 0, SEARCH_ACCELERATION);
    while (!forward.is_finished()) {
        delay(2);
    }

    disable_mototrs();
    disable_steering();
}

void Mouse::run_normal(bool to_finish) {
    Pair target;
    if (to_finish) {
        target = maze.get_finish();
    }
    else {
        target = maze.get_start();
    }
    maze.floodfill(target);
    maze.find_path(maze.get_position());
    
    char next_path;
    bool recalculate = false;

    forward.reset();
    rotation.reset();
    reset_encoders();
    reset_motor_controllers();
    enable_mototrs();

    
    while(maze.get_position() != target) {
        for (int i = 0; i < maze.get_path_len(); i++) {
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
                wait_to_start_front();
                
                
            }
            if (is_start) {
                Serial.println("starting");
                if (!DEBUG_LOGGING) {
                    move_to_center();
                }
                is_start = false;
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
                            enable_steering();
                            forward.adjust_position(-CELL);
                            wait_until_position(CELL);
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
                            move_cell();
                        }
                        maze.update_direction(RIGHT);
                        maze.update_position();
                    }
                    break;
                case 'A':
                    // Serial.println("taround");
                    // while (!button_pressed())
                    // {
                    //     print_debug();
                    //     delay(100);
                    // }
                    // delay(2000);
                    // turn_around(false);
                    if (!DEBUG_LOGGING) {
                        turn_around(false);
                    }
                    is_start = true;
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
                            move_cell();
                        }
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
            if (recalculate) {
                maze.floodfill(target);
                maze.find_path(maze.get_position());
                recalculate = false;
                maze.print_maze();
                if (DEBUG_LOGGING) {
                    Serial.println("Recalculated!");
                }
                break;
                
            }
            else {
                update_walls();
                maze.print_maze();
            }
        }
    }

    if (!to_finish) {
        if (!DEBUG_LOGGING) {
            turn_around(false);
        }
        is_start = true;
        maze.update_direction(DOWN);
    }
    forward.stop();
    disable_mototrs();
    disable_steering();
    stop_motors();
}