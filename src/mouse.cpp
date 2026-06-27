#include "mouse.h"

Mouse mouse;

int MOUSE_CONFIG[3][8] = {
// max_speed | angle_offset |   pre_turn_ofset |    after_turn_offset |     pre_turn_reference |    front_reference |   outer_turn_speed |      inner_turn_speed
    {300,       10,             0,                  20,                     45,                     140,                400,                    162},
    {500,       20,             0,                  30,                     45,                     140,                600,                    190},
    {800,       20,             20,                 30,                     45,                     140,                600,                    190},
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

float Mouse::get_position() {
    return this->distance;
}

void Mouse::set_config(int config_id) {
    this->max_speed = MOUSE_CONFIG[config_id][0];
    this->angle_offset = MOUSE_CONFIG[config_id][1];
    this->pre_turn_ofset = MOUSE_CONFIG[config_id][2];
    this->after_turn_offset = MOUSE_CONFIG[config_id][3];
    this->pre_turn_reference = MOUSE_CONFIG[config_id][4];
    this->front_reference = MOUSE_CONFIG[config_id][5];
    this->turn_speed = MOUSE_CONFIG[config_id][6];
    this->turn_inner_speed = MOUSE_CONFIG[config_id][7];
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
    distance = 0;
    g_gyro_angle = 0;
    reset_encoders();
    maze.set_direction(this->start_direction);
    maze.set_position(maze.get_start());
}

void Mouse::switch_start_direction() {
    if (this->start_direction == UP) {
        this->start_direction = Direction::RIGHT;
    } else {
        this->start_direction = UP;
    }
}

void Mouse::move(float distance, float speed, int check_wall_distance) {
    if (distance < 0) {
        speed *= -1;
    }

    motor_left.set_speed(speed);
    motor_right.set_speed(speed);

    if (check_wall_distance > 0 && g_is_front_wall) {
        turn_wall_leds(false, true, false);
        while(g_front_sensor < check_wall_distance) {
            update_motor_controllers();
        }
    } else {
        while(abs(get_robot_position() - this->distance) < abs(distance)) {
            update_motor_controllers();
        }  
    }
    
    this->distance += distance;
}


void Mouse::move_angle(float turn_angle, float speed) {
    // get ready to turn
    disable_steering();
    stop_motors();
    float angle_offset = 10;

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
    if (USE_GYRO) {
        while (abs(g_gyro_angle - this->angle) < (abs(turn_angle) - angle_offset)) {
            update_motor_controllers();
        }
    } else {
        while (abs(get_robot_angle() - this->angle) < (abs(turn_angle) - angle_offset)) {
            update_motor_controllers();
        }
    }
    this->angle += turn_angle;

    stop_motors();
}

void Mouse::turn(float angle) {
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

void Mouse::show_nominal_value() {
    uint8_t mod = 0;
    uint8_t leds = 0;
    while(true) {
        leds = 0;
        update_sensors();
        if (g_left_button) {
            break;
        }
        if (g_right_button) {
            mod++;
            mod = mod % 2;
            turn_all_leds();
            delay(1000);
        }

        switch (mod)
        {
            case 0:
                // front sensors
                if (g_front_sensor_left > 105) {
                    leds |= RED_LEFT_LED;
                } else if (g_front_sensor_left < 95) {
                    leds |= BLUE_LEFT_LED;
                }

                if (g_front_sensor_right > 105) {
                    leds |= RED_RIGHT_LED;
                } else if (g_front_sensor_right < 95) {
                    leds |= BLUE_RIGHT_LED;
                }
                break;
            case 1:
                // side sensors
                if (g_left_sensor > 105) {
                    leds |= RED_LEFT_LED;
                } else if (g_left_sensor < 95) {
                    leds |= BLUE_LEFT_LED;
                }

                if (g_right_sensor > 105) {
                    leds |= RED_RIGHT_LED;
                } else if (g_right_sensor < 95) {
                    leds |= BLUE_RIGHT_LED;
                }
                break;
            default:
                break;
        }
        print_sensors();
        turn_leds(leds);
        delay(50);
    }
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

void Mouse::move_cell_unit(float target, bool untill_wall) {
    enable_steering();
    int dist_to_wall = untill_wall ? this->front_reference : 0;
    move(target, max_speed, dist_to_wall);
}

void Mouse::move_half_cell(bool untill_wall) {
    move_cell_unit(HALF_CELL, untill_wall);
}

void Mouse::move_cell(bool untill_wall) {
    move_cell_unit(CELL, untill_wall);
}

void Mouse::move_backward() {
    move(-HALF_CELL, max_speed);
}

void Mouse::turn_90_left() {
    turn(-90);
}

void Mouse::turn_90_right() {
    turn(90);
}

void Mouse::turn_smooth(float turn_angle) {
    disable_steering();

    float calibration_speed = min(this->max_speed, this->turn_speed);

    move(this->pre_turn_ofset, calibration_speed, this->pre_turn_reference);

    float left_speed;
    float right_speed;
    if (turn_angle < 0) {
        left_speed = this->turn_inner_speed;
        right_speed = this->turn_speed;
    } else {
        left_speed = this->turn_speed;
        right_speed = this->turn_inner_speed;
    }
    motor_left.set_speed(left_speed);
    motor_right.set_speed(right_speed);
    if (USE_GYRO) {
        while (abs(g_gyro_angle - this->angle) < (abs(turn_angle) - this->angle_offset)) {
            update_motor_controllers();
        }
    } else {
        while (abs(get_robot_angle() - this->angle) < (abs(turn_angle) - this->angle_offset)) {
            update_motor_controllers();
        }
    }
    this->angle += turn_angle;

    enable_steering();
    this->distance = get_robot_position();
    move(this->after_turn_offset, calibration_speed);
}

void Mouse::turn_90_left_smooth() {
    turn_smooth(-90);
}

void Mouse::turn_90_right_smooth() {
    turn_smooth(90);
}

void Mouse::turn_diag(int angle, char dir) {
    int prev_inner_speed = this->turn_inner_speed;
    this->turn_inner_speed = 260;

    if (dir == 'L') {
        angle *= -1;
    }

    this->turn_smooth(angle);

    this->turn_inner_speed = prev_inner_speed;
}

void Mouse::turn_45_diag(char dir) {
    this->turn_diag(45, dir);
}

void Mouse::turn_90_diag(char dir) {
    this->turn_diag(90, dir);
}

void Mouse::turn_135_diag(char dir) {
    this->turn_diag(135, dir);
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
        this->distance = 0;
        g_gyro_angle = 0;
        reset_encoders();
    }
    stop_motors();
}

void Mouse::update_walls() {
    update_sensors();
    maze.set_walls(g_is_left_wall, g_is_front_wall, g_is_right_wall);
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
            for (int i = 0; i < maze.get_path_len(); i++) {
                if (button_pressed()) {
                    return false;
                }
                
                next_path = maze.get_next_move();

                if (is_start) {
                    move_from_wall();
                    is_start = false;
                    is_center = true;
                }

                switch (next_path)
                {
                    case Action::FORWARD:
                        if (maze.is_wall(Direction::UP)) {
                            recalculate = true;
                        }
                        else {
                            if (is_center) {
                                move_half_cell();
                                is_center = false;
                            }
                            else {
                                move_cell();
                            }
                            
                            maze.update_position();
                        }
                        break;
                    case Action::TURN_RIGHT:
                        if (maze.is_wall(Direction::RIGHT)) {
                            recalculate = true;
                        }
                        else {
                            turn_90_right_smooth();
                            maze.update_direction(Direction::RIGHT);
                            maze.update_position();
                        }
                        break;
                    case Action::AROUND:
                        if (!is_center) {
                            move_half_cell(true);
                            is_center = true;
                        }
                        turn_around();
                        maze.update_direction(Direction::DOWN);
                        break;
                    case Action::TURN_LEFT:
                        if (maze.is_wall(Direction::LEFT)) {
                            recalculate = true;
                        }
                        else {
                            turn_90_left_smooth();
                            maze.update_direction(Direction::LEFT);
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
            }
        }

        if (path_exists) {
            move_half_cell(true);
            is_center = true;
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

    uint8_t action;
    uint8_t path_len = maze.get_path_len();
    uint8_t new_len = 0;
    int units = 0;

    int path[MAZE_WIDTH * MAZE_WIDTH] = {};
    int new_path[MAZE_WIDTH * MAZE_WIDTH] = {};
    for (int i = 0; i < path_len; i++) {
        action = maze.get_next_move();
        path[i] = action;
    }
    

    for (int i = 0; i < path_len; i++) {
        if (i+1 >= path_len || i+2 >= path_len) {
            continue;
        }
        if (path[i] == Action::FORWARD && path[i+1] == Action::FORWARD) {
            new_path[new_len++] = Action::FORWARD;
            new_path[new_len++] = CELL;
        }

        else if (path[i] == Action::FORWARD && path[i+1] == Action::TURN_RIGHT && path[i+2] == Action::TURN_LEFT) {
            new_path[new_len++] = Action::FORWARD;
            new_path[new_len++] = HALF_CELL;
            new_path[new_len++] = Action::TURN_RIGHT_DIAG;
            new_path[new_len++] = 45;
            new_path[new_len++] = Action::FORWARD_DIAG;
            new_path[new_len++] = DIAG;
        }
        else if (path[i] == Action::FORWARD && path[i+1] == Action::TURN_LEFT && path[i+2] == Action::TURN_RIGHT) {
            new_path[new_len++] = Action::FORWARD;
            new_path[new_len++] = HALF_CELL;
            new_path[new_len++] = Action::TURN_LEFT_DIAG;
            new_path[new_len++] = 45;
            new_path[new_len++] = Action::FORWARD_DIAG;
            new_path[new_len++] = DIAG;
        }

        else if (path[i] == Action::TURN_RIGHT && path[i+1] == Action::TURN_LEFT && path[i+2] == Action::TURN_RIGHT) {
            new_path[new_len++] = Action::FORWARD_DIAG;
            new_path[new_len++] = DIAG;
        }
        else if (path[i] == Action::TURN_LEFT && path[i+1] == Action::TURN_RIGHT && path[i+2] == Action::TURN_LEFT) {
            new_path[new_len++] = Action::FORWARD_DIAG;
            new_path[new_len++] = DIAG;
        }

        else if (path[i] == Action::TURN_LEFT && path[i+1] == Action::TURN_RIGHT && path[i+2] == Action::TURN_RIGHT && path[i+3] == Action::FORWARD) {
            new_path[new_len++] = Action::FORWARD_DIAG;
            new_path[new_len++] = HALF_DIAG;
            new_path[new_len++] = Action::TURN_RIGHT_DIAG;
            new_path[new_len++] = 135;
            new_path[new_len++] = Action::FORWARD;
            new_path[new_len++] = HALF_CELL;
        }
        else if (path[i] == Action::TURN_RIGHT && path[i+1] == Action::TURN_LEFT && path[i+2] == Action::TURN_LEFT && path[i+3] == Action::FORWARD) {
            new_path[new_len++] = Action::FORWARD_DIAG;
            new_path[new_len++] = HALF_DIAG;
            new_path[new_len++] = Action::TURN_LEFT_DIAG;
            new_path[new_len++] = 135;
            new_path[new_len++] = Action::FORWARD;
            new_path[new_len++] = HALF_CELL;
        }

        else if (path[i] == Action::TURN_LEFT && path[i+1] == Action::TURN_RIGHT && path[i+2] == Action::TURN_RIGHT) {
            new_path[new_len++] = Action::FORWARD_DIAG;
            new_path[new_len++] = HALF_DIAG;
            new_path[new_len++] = Action::TURN_RIGHT_DIAG;
            new_path[new_len++] = 90;
            new_path[new_len++] = Action::FORWARD_DIAG;
            new_path[new_len++] = HALF_DIAG;
        }
        else if (path[i] == Action::TURN_RIGHT && path[i+1] == Action::TURN_LEFT && path[i+2] == Action::TURN_LEFT) {
            new_path[new_len++] = Action::FORWARD_DIAG;
            new_path[new_len++] = HALF_DIAG;
            new_path[new_len++] = Action::TURN_LEFT_DIAG;
            new_path[new_len++] = 90;
            new_path[new_len++] = Action::FORWARD_DIAG;
            new_path[new_len++] = HALF_DIAG;
        }

        else if (path[i] == Action::FORWARD && path[i+1] == Action::TURN_RIGHT && path[i+2] == Action::TURN_RIGHT && path[i+3] == Action::TURN_LEFT) {
            new_path[new_len++] = Action::FORWARD;
            new_path[new_len++] = HALF_CELL;
            new_path[new_len++] = Action::TURN_RIGHT_DIAG;
            new_path[new_len++] = 135;
            new_path[new_len++] = Action::FORWARD_DIAG;
            new_path[new_len++] = HALF_DIAG;
        }
        else if (path[i] == Action::FORWARD && path[i+1] == Action::TURN_LEFT && path[i+2] == Action::TURN_LEFT && path[i+3] == Action::TURN_RIGHT) {
            new_path[new_len++] = Action::FORWARD;
            new_path[new_len++] = HALF_CELL;
            new_path[new_len++] = Action::TURN_RIGHT_DIAG;
            new_path[new_len++] = 135;
            new_path[new_len++] = Action::FORWARD_DIAG;
            new_path[new_len++] = HALF_DIAG;
        }

        else if (path[i] == Action::TURN_RIGHT && path[i+1] == Action::TURN_LEFT && path[i+2] == Action::FORWARD) {
            new_path[new_len++] = Action::TURN_LEFT_DIAG;
            new_path[new_len++] = 45;
            new_path[new_len++] = Action::FORWARD;
            new_path[new_len++] = HALF_CELL;
        }
        else if (path[i] == Action::TURN_LEFT && path[i+1] == Action::TURN_RIGHT && path[i+2] == Action::FORWARD) {
            new_path[new_len++] = Action::TURN_RIGHT_DIAG;
            new_path[new_len++] = 45;
            new_path[new_len++] = Action::FORWARD;
            new_path[new_len++] = HALF_CELL;
        }
    }

    enable_motors();
    
    float stop_dist = 50;
    char symb;
    for (int i = 0; i < new_len; i+=2) {
        
        action = new_path[i];
        units = path[i+1];
        if (action == Action::FORWARD) {
            symb = 'F';
        } else if (action == Action::FORWARD_DIAG) {
            symb = 'f';
        } else if (action == Action::TURN_RIGHT) {
            symb = 'R';
        } else if (action == Action::TURN_RIGHT_DIAG) {
            symb = 'r';
        } else if (action == Action::TURN_LEFT) {
            symb = 'L';
        } else if (action == Action::TURN_LEFT_DIAG) {
            symb = 'l';
        }
        Serial.print(symb);
        Serial.print(units);
    }
    while (true)
    {
        /* code */
    }
    //     switch (action)
    //     {
    //         case Action::FORWARD:
    //             // move(units, max_speed);
    //             move(units - stop_dist - this->pre_turn_ofset, max_speed);
    //             move(stop_dist, this->turn_speed, pre_turn_reference);
    //             break;
    //         case Action::TURN_RIGHT:
    //             turn_smooth(units);
    //             break;
    //         case Action::TURN_LEFT:
    //             turn_smooth(-1 * units);
    //             break;
    //         default:
    //             // shouldnt exist
    //             stop();
    //             return false;
    //     }
    // }
    
    // set_config(0);
    // move_half_cell(true);
    // is_center = true;
    // is_start = false;
    // uint8_t norm_angle = ((this->angle % 360 + 360) / 90);

    // maze.set_position(maze.get_finish());
    // maze.set_direction(norm_angle % 4);

    // stop();
    return path_exists;
}