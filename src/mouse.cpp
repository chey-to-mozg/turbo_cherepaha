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
        // delay(2); // wait for 1 update loop
        print_profile();
    }
}

// move until reference


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
    bool signal = true;
    while(!button_pressed()) {
        digitalWrite(LED_GREEN, signal);
        signal = !signal;
        delay(300);
    }
    digitalWrite(LED_GREEN, true);
    delay(2000);
}

void Mouse::error_ping() {
    digitalWrite(LED_GREEN, 1);
    digitalWrite(LED_BLUE, 1);
    digitalWrite(LED_RED, 1);
}

void Mouse::move_from_wall() {
    forward.start(CELL, SPEEDMAX_EXPLORE, SPEEDMAX_EXPLORE, SEARCH_ACCELERATION);
    forward.set_position(ROBOT_OFFSET);
    while(!forward.is_finished()) {
        delay(2); // wait for 1 update loop
    }
    
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
    float ofset = 0;
    disable_steering();

    // check when we reset position
    float distance = CELL + ofset - forward.position();
    forward.start(distance, forward.speed(), SPEEDMAX_PRETURN, SEARCH_ACCELERATION);
    while(!forward.is_finished()) {
        delay(2); // wait for 1 update loop
    }

    rotation.start(angle, SPEEDMAX_SMOOTH_TURN, 0, SPIN_TURN_ACCELERATION);
    while (!rotation.is_finished()) {
        delay(2); // wait for 1 update loop
    }
}

void Mouse::turn_90_right_smooth() {
    float angle = -90;
    float ofset = 0;
    disable_steering();

    float distance = CELL + ofset - forward.position();
    forward.start(distance, forward.speed(), SPEEDMAX_PRETURN, SEARCH_ACCELERATION);
    while(!forward.is_finished()) {
        delay(2); // wait for 1 update loop
    }

    rotation.start(angle, SPEEDMAX_SMOOTH_TURN, 0, SPIN_TURN_ACCELERATION);
    while (!rotation.is_finished()) {
        delay(2); // wait for 1 update loop
    }
}

void Mouse::turn_around() {
    disable_steering();
    forward.start(HALF_CELL, forward.speed(), 30, forward.acceleration());
    if (true/*check front wall*/) {
        while (!forward.is_finished()) {
            delay(2);
        }
    } else {
        while (g_front_sensor < FRONT_REFERENCE) {
            delay(2);
        }
    }
    forward.stop();
    float angle = 180;
    move_angle(angle);
    forward.start(-HALF_CELL, SPEEDMAX_STRAIGHT, 30, forward.acceleration());
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

void Mouse::run() {
    maze.floodfill(maze.get_finish());
    maze.find_path(maze.get_position());
    char next_path;
    bool recalculate = false;
    while(maze.get_finish() != maze.get_position()) {
        for (int i = 0; i < maze.get_path_len(); i++) {
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
                wait_to_start();
            }
            if (is_start) {
                move_from_wall();
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
                        move_cell();
                        maze.update_position();
                    }
                    break;
                case 'R':
                    if (right_wall) {
                        recalculate = true;
                    }
                    else {
                        turn_90_right_smooth();
                        maze.update_direction(RIGHT);
                        maze.update_position();
                    }
                    break;
                case 'A':
                    turn_around();
                    is_start = true;
                    maze.update_direction(DOWN);
                    // set gyro error to zero
                    break;
                case 'L':
                    if (left_wall) {
                        recalculate = true;
                    }
                    else {
                        turn_90_left_smooth();
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
            }
            else {
                update_walls();
                maze.print_maze();
            }
        }
    }
}
