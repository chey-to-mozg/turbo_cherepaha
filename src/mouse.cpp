#include "mouse.h"

Mouse mouse;

float MOUSE_CONFIG[2][11] = {
// max_speed |  angle_offset_left | pre_turn_ofset_left |   after_turn_offset_left | pre_turn_reference_left |      angle_offset_right |    pre_turn_ofset_right |  after_turn_offset_right |   pre_turn_reference_right |  front_reference |   turn_ratio
    {300.0,     0,                  15.0,                   20.0,                    80.0,                          0,                      5.0,                    30.0,                       80.0,                       130.0,              0.33},
    {500.0,     0,                  10.0,                   10.0,                    87.0,                          0,                      5.0,                    20.0,                       90.0,                       130.0,              0.3},
};

Mouse::Mouse() {
    init_leds();
    disable_steering();
    reset_encoders();
    max_speed = SPEEDMAX_EXPLORE;
}

float Mouse::get_angle() {
    return angle;
}

void Mouse::set_config(int config_id) {
    this->max_speed = MOUSE_CONFIG[config_id][0];
    this->angle_offset_left = MOUSE_CONFIG[config_id][1];
    this->pre_turn_ofset_left = MOUSE_CONFIG[config_id][2];
    this->after_turn_offset_left = MOUSE_CONFIG[config_id][3];
    this->pre_turn_reference_left = MOUSE_CONFIG[config_id][4];
    this->angle_offset_right = MOUSE_CONFIG[config_id][5];
    this->pre_turn_ofset_right = MOUSE_CONFIG[config_id][6];
    this->after_turn_offset_right = MOUSE_CONFIG[config_id][7];
    this->pre_turn_reference_right = MOUSE_CONFIG[config_id][8];
    this->front_reference = MOUSE_CONFIG[config_id][9];
    this->turn_ratio = MOUSE_CONFIG[config_id][10];
}

void Mouse::stop() {
    disable_steering();
    stop_motors();
    disable_motors();
}

void Mouse::reset_mouse() {
    is_start = true;
    is_center = false;
    angle = 0;
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

void Mouse::move(float distance, float speed, int check_wall_distance) {
    if (distance < 0) {
        speed *= -1;
    }
    motor_left.set_speed(speed);
    motor_right.set_speed(speed);
    float start_position = get_robot_position();
    while(abs(get_robot_position() - start_position) < abs(distance)) {
        update_motor_controllers();
        if (check_wall_distance > 0 && g_is_front_wall) {
            turn_wall_leds(false, true, false);
            while(g_front_sensor < check_wall_distance) {
                update_motor_controllers();
            }
            break;
        } 
    }  
}


void Mouse::move_angle(float turn_angle, float speed) {
    // get ready to turn
    disable_steering();
    float left_speed = speed;
    float right_speed = speed;
    if (turn_angle < 0) {
        right_speed *= -1;
    }
    else {
        left_speed *= -1;
    }
    motor_left.set_speed(left_speed);
    motor_right.set_speed(right_speed);
    while (abs(get_robot_angle() - this->angle) < abs(turn_angle)) {
        update_motor_controllers();
        print_motors();
    }
    // float cur_angle = 0;
    // start_gyro_read();
    // while (abs(cur_angle) < abs(turn_angle)) {
    //     update_motor_controllers();
    //     cur_angle += read_gyro();
    //     print_motors();
    // }
    this->angle += turn_angle;
    stop_motors();
}

void Mouse::turn(float angle) {
    stop_motors();
    move_angle(angle, SPEEDMAX_SPIN_TURN);
}

uint8_t Mouse::wait_to_start() {
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
    while(true) {
        update_motor_controllers();
        if (g_left_button) {
            break;
        }
        print_sensors();
        print_motors();
        if (g_right_button) {
            mode = (mode + 1) % 8;
            turn_mode_leds(mode, signal);
            delay(500);
        }
        turn_mode_leds(mode, signal);
        // turn_wall_leds(g_is_left_wall, g_is_front_wall, g_is_right_wall);
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
    update_motor_controllers();
    reset_leds();

    return mode;
}

void Mouse::maze_debug() {
    enable_steering();
    while(true) {
        update_sensors();
        print_sensors();
        if (g_left_button) {
            break;
        }
        delay(200);
    }
    turn_all_leds();
    delay(2000);
    reset_leds();
    disable_steering();
}

void Mouse::error_ping() {
    disable_steering();
    bool signal = false;
    while (!button_pressed()) {
        if (signal) {
            turn_all_leds();
        }
        else {
            reset_leds();
        }
        print_info();
        print_sensors();
        print_motors();
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
    disable_steering();
    motor_left.reset_motor();
    motor_right.reset_motor();
    move(HALF_CELL - ROBOT_OFFSET, max_speed);
}

void Mouse::move_half_cell(bool untill_wall) {
    enable_steering();
    int dist_to_wall = untill_wall ? this->front_reference : 0;
    move(HALF_CELL, max_speed, dist_to_wall);
}

void Mouse::move_cell(bool untill_wall) {
    enable_steering();
    int dist_to_wall = untill_wall ? this->front_reference : 0;
    move(CELL, max_speed, dist_to_wall);
}

void Mouse::move_backward() {
    move(-BACK_WALL_TO_CENTER, max_speed / 2);
}

void Mouse::turn_90_left() {
    turn(-90);
}

void Mouse::turn_90_right() {
    turn(90);
}

void Mouse::turn_90_left_smooth() {
    float turn_angle = 90;
    disable_steering();
    
    move(this->pre_turn_ofset_left, this->max_speed, this->pre_turn_reference_left);
    // int saved_sensor = g_front_sensor;
    // stop();
    // while (1) {
    //     Serial.println(saved_sensor);
    //     delay(1000);
    // }
    float left_speed = this->turn_ratio * this->max_speed;
    float right_speed = this->max_speed;
    motor_left.set_speed(left_speed);
    motor_right.set_speed(right_speed);
    // float cur_angle = this->angle;
    // start_gyro_read();
    while (get_robot_angle() < this->angle + turn_angle + this->angle_offset_left) {
        update_motor_controllers();
        // cur_angle += read_gyro();
    }
    this->angle += turn_angle;
    move(this->after_turn_offset_left, this->max_speed);
}

void Mouse::turn_90_right_smooth() {
    float turn_angle = -90;
    disable_steering();
    
    move(this->pre_turn_ofset_right, this->max_speed, this->pre_turn_reference_right);
    // int saved_sensor = g_front_sensor;
    // stop();
    // while (1) {
    //     Serial.println(saved_sensor);
    //     delay(1000);
    // }
    
    float left_speed = this->max_speed;
    float right_speed = this->turn_ratio * this->max_speed;
    motor_left.set_speed(left_speed);
    motor_right.set_speed(right_speed);
    // float cur_angle = this->angle;
    // start_gyro_read();
    while (get_robot_angle() > this->angle + turn_angle + this->angle_offset_right) {
        update_motor_controllers();
        // cur_angle += read_gyro();
        print_motors();
    }
    this->angle += turn_angle;
    
    move(this->after_turn_offset_right, SPEEDMAX_EXPLORE);
}

void Mouse::turn_around() {
    float angle = 180;
    turn(angle);
    // we check wall before direction update
    if (maze.is_wall(UP)) {
        move_backward();
        is_start = true;
        is_center = false;
        this->angle = 0;
        reset_encoders();
    }
    stop_motors();
}

void Mouse::update_walls() {
    update_sensors();
    maze.set_walls(g_is_left_wall, g_is_front_wall, g_is_right_wall);
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

bool Mouse::explore(bool to_finish) {
    // init wall before start
    
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
        enable_motors();

        while(path_exists && maze.get_position() != target) {
            update_walls();
            if (DEBUG_MAZE) {
                print_info();
                maze_debug();
            }
            for (int i = 0; i < maze.get_path_len(); i++) {
                if (button_pressed()) {
                    return false;
                }
                
                next_path = maze.get_next_move();

                if (is_start) {
                    if (!DEBUG_MAZE) {
                        move_from_wall();
                    }
                    is_start = false;
                    is_center = true;
                }

                switch (next_path)
                {
                    case 'F':
                        if (maze.is_wall(UP)) {
                            recalculate = true;
                        }
                        else {
                            if (DEBUG_MAZE) {
                                Serial.println("Forward");
                            }
                            else {
                                if (is_center) {
                                    move_half_cell();
                                    is_center = false;
                                }
                                else {
                                    move_cell();
                                }
                            }
                            
                            maze.update_position();
                        }
                        break;
                    case 'R':
                        if (maze.is_wall(RIGHT)) {
                            recalculate = true;
                        }
                        else {
                            if (DEBUG_MAZE) {
                                Serial.println("Right and forward");
                            }
                            else {
                                turn_90_right_smooth();
                            }
                            maze.get_next_move(true); // after turn command it is forward command, so we should pop it
                            i++;
                            maze.update_direction(RIGHT);
                            maze.update_position();
                        }
                        break;
                    case 'A':
                        if (DEBUG_MAZE) {
                            Serial.println("Around");
                        }
                        else {
                            if (!is_center) {
                                move_half_cell(true);
                                is_center = true;
                            }
                            turn_around();
                        }
                        maze.update_direction(DOWN);
                        
                        // set gyro error to zero
                        break;
                    case 'L':
                        if (maze.is_wall(LEFT)) {
                            recalculate = true;
                        }
                        else {
                            if (DEBUG_MAZE) {
                                Serial.println("Left and forward");
                            }
                            else {
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
                        stop();
                        return false;
                }

                // check if mouse can move next step, otherwise floodfill
                if (recalculate) {
                    maze.floodfill(target);
                    path_exists = maze.find_path(maze.get_position());
                    recalculate = false;
                    if (DEBUG_LOGGING) {
                        Serial.println("Recalculated!");
                    }
                    break;
                }

                update_walls();
                if (DEBUG_MAZE) {
                    print_info();
                    Serial.print("Current iteration: ");
                    Serial.println(i);
                    maze_debug();
                }
            }
        }

        if (path_exists) {
            if (!DEBUG_MAZE) {
                move_half_cell(true);
                is_center = true;
            }
        }
           
    }
    stop();
    return path_exists;
}

bool Mouse::run_short() {
    Pair target = maze.get_finish();
    maze.floodfill(target);
    bool path_exists = maze.find_path(maze.get_position());
    if (!path_exists) {
        return false;
    }
    maze.print_maze();
    maze.print_path();
    enable_motors();
    char next_path;
    for (int i = 0; i < maze.get_path_len(); i++) {
        if (button_pressed()) {
            return false;
        }
        
        next_path = maze.get_next_move();

        if (is_start) {
            if (!DEBUG_MAZE) {
                move_from_wall();
            }
            is_start = false;
            is_center = true;
        }

        switch (next_path)
        {
            case 'F':
                if (DEBUG_MAZE) {
                    Serial.println("Forward");
                }
                else {
                    if (is_center) {
                        move_half_cell();
                        is_center = false;
                    }
                    else {
                        move_cell();
                    }
                }  
                maze.update_position();
                break;
            case 'R':
                if (DEBUG_MAZE) {
                    Serial.println("Right and forward");
                }
                else {
                    turn_90_right_smooth();
                }
                maze.get_next_move(true); // after turn command it is forward command, so we should pop it
                i++;
                maze.update_direction(RIGHT);
                maze.update_position();
                break;
            case 'L':
                if (DEBUG_MAZE) {
                    Serial.println("Left and forward");
                }
                else {
                    turn_90_left_smooth();
                }
                maze.get_next_move(true); // after turn command it is forward command, so we should pop it
                i++;
                maze.update_direction(LEFT);
                maze.update_position();
                // float cur_angle = get_robot_angle();
                // stop();
                // while(1) {
                //     Serial.println(cur_angle);
                //     delay(1000);
                // }
                break;
            default:
                // shouldnt exist
                stop();
                return false;
        }
    }

    if (path_exists) {
        set_config(0);
        if (!DEBUG_MAZE) {
            move_half_cell(true);
            is_center = true;
        }
    }
    
    stop();
    return path_exists;
}
