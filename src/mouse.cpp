#include "mouse.h"

Mouse mouse;

float MOUSE_CONFIG[2][11] = {
// max_speed |  angle_offset_left | pre_turn_ofset_left |   after_turn_offset_left | pre_turn_reference_left |      angle_offset_right |    pre_turn_ofset_right |  after_turn_offset_right |   pre_turn_reference_right |  front_reference |   turn_ratio
    {300.0,    -5,                  60.0,                   15.0,                    95.0,                          5,                      60.0,                   15.0,                       95.0,                       140.0,              0.2},
    {500.0,    -15,                 5.0,                    35.0,                    80.0,                          10,                     5.0,                    40.0,                       82.0,                       120.0,              0.25},
};

Mouse::Mouse() {
    init_leds();
    disable_steering();
    reset_encoders();
    max_speed = SPEEDMAX_EXPLORE;
}

int Mouse::get_angle() {
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
    g_gyro_angle = 0;
    reset_encoders();
    maze.set_direction(this->start_direction);
    maze.set_position(maze.get_start());
}

// void Mouse::switch_start_direction() {
//     if (this->start_direction == UP) {
//         this->start_direction = RIGHT;
//     } else {
//         this->start_direction = UP;
//     }
// }

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
        update_motor_controllers(this->angle);
        if (check_wall_distance > 0 && g_is_front_wall) {
            turn_wall_leds(false, true, false);
            while(g_front_sensor < check_wall_distance) {
                update_motor_controllers(this->angle);
            }
            break;
        } 
    }  
}


void Mouse::move_angle(int turn_angle, float speed) {
    // get ready to turn
    disable_steering();
    stop_motors();

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
    if (USE_GYRO) {
        while (abs(g_gyro_angle - this->angle) < abs(turn_angle)) {
            update_motor_controllers(this->angle);
        }
    } else {
        while (abs(get_robot_angle() - this->angle) < abs(turn_angle)) {
            update_motor_controllers(this->angle);
        }
    }
    this->angle += turn_angle;

    stop_motors();
}

void Mouse::turn(int angle) {
    move_angle(angle, SPEEDMAX_SPIN_TURN);
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

void Mouse::move_diag() {
    enable_steering();
    move(DIAG, max_speed);
}

void Mouse::move_backward() {
    move(-HALF_CELL, max_speed / 2);
}

void Mouse::turn_90_left() {
    turn(90);
}

void Mouse::turn_90_right() {
    turn(-90);
}

void Mouse::turn_90_left_smooth() {
    int turn_angle = 90;
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
    if (USE_GYRO) {
        while (g_gyro_angle < this->angle + turn_angle + this->angle_offset_left) {
            update_motor_controllers();
        }
    } else {
        while (get_robot_angle() < this->angle + turn_angle + this->angle_offset_left) {
            update_motor_controllers();
        }
    }
    this->angle += turn_angle;

    enable_steering();
    move(this->after_turn_offset_left, this->max_speed);
}

void Mouse::turn_45_left_smooth() {
    int turn_angle = 45;
    disable_steering();
    float pre_turn_ref = 0;
    if (!is_diag) {
        pre_turn_ref = this->pre_turn_reference_left;
        is_diag = true;
    } else {
        is_diag = false;
    }
    
    move(this->pre_turn_ofset_left, this->max_speed, pre_turn_ref);
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
    if (USE_GYRO) {
        while (g_gyro_angle < this->angle + turn_angle + this->angle_offset_left) {
            update_motor_controllers();
        }
    } else {
        while (get_robot_angle() < this->angle + turn_angle + this->angle_offset_left) {
            update_motor_controllers();
        }
    }
    this->angle += turn_angle;

    enable_gyro();
    move(this->after_turn_offset_left, this->max_speed);
}

void Mouse::turn_90_right_smooth() {
    int turn_angle = -90;
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
    if (USE_GYRO) {
        while (g_gyro_angle > this->angle + turn_angle + this->angle_offset_right) {
            update_motor_controllers(this->angle);
        }
    } else {
        while (get_robot_angle() > this->angle + turn_angle + this->angle_offset_right) {
            update_motor_controllers(this->angle);
        }
    }
    this->angle += turn_angle;

    enable_steering();
    move(this->after_turn_offset_right, this->max_speed);
}

void Mouse::turn_45_right_smooth() {
    int turn_angle = -45;
    disable_steering();
    
    float pre_turn_ref = 0;
    if (!is_diag) {
        pre_turn_ref = this->pre_turn_reference_right;
        is_diag = true;
    } else {
        is_diag = false;
    }

    move(this->pre_turn_ofset_right, this->max_speed, pre_turn_ref);
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
    if (USE_GYRO) {
        while (g_gyro_angle > this->angle + turn_angle + this->angle_offset_right) {
            update_motor_controllers(this->angle);
        }
    } else {
        while (get_robot_angle() > this->angle + turn_angle + this->angle_offset_right) {
            update_motor_controllers(this->angle);
        }
    }
    this->angle += turn_angle;

    enable_gyro();
    move(this->after_turn_offset_right, this->max_speed);
}


void Mouse::turn_around() {
    int angle = 180;
    turn(angle);
    // we check wall before direction update
    if (maze.is_wall(UP)) {
        move_backward();
        is_start = true;
        is_center = false;
        this->angle = 0;
        g_gyro_angle = 0;
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

bool Mouse::run_short(bool diag) {
    Pair target = maze.get_finish();
    maze.floodfill(target);
    bool path_exists = maze.find_path(maze.get_position());
    if (!path_exists) {
        return false;
    }
    if (diag) {
        maze.gen_diag();
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
                break;
            case 'D':
                if (DEBUG_MAZE) {
                    Serial.println("Diagonal");
                }
                else {
                    move_diag();
                }
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
                break;
            case 'r':
                if (DEBUG_MAZE) {
                    Serial.println("Right 45");
                }
                else {
                    turn_45_right_smooth();
                }
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
                // float cur_angle = get_robot_angle();
                // stop();
                // while(1) {
                //     Serial.println(cur_angle);
                //     delay(1000);
                // }
                break;
            case 'l':
                if (DEBUG_MAZE) {
                    Serial.println("Left 45");
                }
                else {
                    turn_45_left_smooth();
                }
                break;
            default:
                // shouldnt exist
                stop();
                return false;
        }
    }

    set_config(0);
    if (!DEBUG_MAZE) {
        move_half_cell(true);
        is_center = true;
    }

    maze.set_position(maze.get_finish());
    uint8_t ang = UP;
    this->angle = this->angle % 360;
    switch (this->angle) {
        case 0:
            ang = UP;
            break;
        case 90:
            ang = RIGHT;
            break;
        case 180:
            ang = DOWN;
            break;
        case 270:
            ang = LEFT;
            break;
    }
    maze.set_direction(ang);
    
    stop();
    return path_exists;
}

