#include "maze.h"

Maze maze;

uint8_t WALLS[4] = {UP_WALL, RIGHT_WALL, DOWN_WALL, LEFT_WALL};
uint8_t WALLS_VISITED[4] = {UP_WALL_VISITED, RIGHT_WALL_VISITED, DOWN_WALL_VISITED, LEFT_WALL_VISITED};
Pair NEIGHBOURS[4] = {
    {MAZE_WIDTH - 1, 0},
    {0, 1},
    {1, 0},
    {0, MAZE_WIDTH - 1},
};
char DIRECTION_TO_CHAR[4] = {'^', '>', 'v', '<'};

Maze::Maze() {
    reset_maze();
    floodfill(target);
}

void Maze::reset_maze() {
    for (uint8_t y = 0; y < MAZE_WIDTH; y++) {
        for (uint8_t x = 0; x < MAZE_WIDTH; x++) {
            maze[y][x] = 0;
            walls[y][x] = 0;
        }
    }

    for (uint8_t i = 0; i < MAZE_WIDTH; i++) {
        walls[i][0] |= LEFT_WALL;
        walls[i][MAZE_WIDTH - 1] |= RIGHT_WALL;
        walls[0][i] |= UP_WALL;
        walls[MAZE_WIDTH - 1][i] |= DOWN_WALL;
    }
}

bool Maze::is_visited() {
    return (walls[mouse_position.y][mouse_position.x] & VISITED) == VISITED;
}

void Maze::set_visited() {
    walls[mouse_position.y][mouse_position.x] |= VISITED;
}

void Maze::set_walls(bool is_left_wall, bool is_front_wall, bool is_right_wall) {
    if (is_visited()) {
        return;
    }
    uint8_t left_wall_idx = (mouse_direction + 3) % 4;
    uint8_t front_wall_idx = mouse_direction;
    uint8_t right_wall_idx = (mouse_direction + 1) % 4;

    bool _walls[3] = {is_left_wall, is_front_wall, is_right_wall};
    uint8_t _walls_idx[3] = {left_wall_idx, front_wall_idx, right_wall_idx};

    for (uint8_t i = 0; i < 3; i++) {
        uint8_t wall_idx = _walls_idx[i];
        if (_walls[i] && !(walls[mouse_position.y][mouse_position.x] & WALLS_VISITED[wall_idx])) {
            walls[mouse_position.y][mouse_position.x] |= WALLS[wall_idx];
            uint8_t neigh_y = (mouse_position.y + NEIGHBOURS[wall_idx].y) % MAZE_WIDTH;
            uint8_t neigh_x = (mouse_position.x + NEIGHBOURS[wall_idx].x) % MAZE_WIDTH;
            uint8_t neigh_wall_idx = (wall_idx + 2) % 4;
            walls[neigh_y][neigh_x] |= WALLS[neigh_wall_idx];
            walls[neigh_y][neigh_x] |= WALLS_VISITED[neigh_wall_idx];
        }
    }
    
    set_visited();
}

void Maze::update_direction(uint8_t change) {
    mouse_direction = (mouse_direction + change) % 4;
}

void Maze::update_position() {
    uint8_t new_y = (mouse_position.y + NEIGHBOURS[mouse_direction].y) % MAZE_WIDTH;
    uint8_t new_x = (mouse_position.x + NEIGHBOURS[mouse_direction].x) % MAZE_WIDTH;
    mouse_position = {new_y, new_x};
}

Pair Maze::get_position() {
    return mouse_position;
}

Pair Maze::get_finish() {
    return target;
}

Pair Maze::get_start() {
    return start_position;
}

void Maze::set_position(Pair position) {
    mouse_position = position;
}

void Maze::set_direction(uint8_t dir) {
    mouse_direction = dir;
}

bool Maze::check_wall(Pair position, uint8_t wall) {
    return (walls[position.y][position.x] & wall) == wall;
}

void Maze::floodfill(Pair target) {
    for (uint8_t y = 0; y < MAZE_WIDTH; y++) {
        for (uint8_t x = 0; x < MAZE_WIDTH; x++) {
            maze[y][x] = 0;
        }
    }
    bool visited[MAZE_WIDTH][MAZE_WIDTH] = {0};

    visited[target.y][target.x] = 1;

    Queue<Pair> to_process;
    to_process.add(target);
    Pair current_pos;
    while (!to_process.empty()) {
        current_pos = to_process.pop();
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t neigh_y = (current_pos.y + NEIGHBOURS[i].y) % MAZE_WIDTH;
            uint8_t neigh_x = (current_pos.x + NEIGHBOURS[i].x) % MAZE_WIDTH;
            if (!check_wall(current_pos, WALLS[i]) && !visited[neigh_y][neigh_x]) {
                visited[neigh_y][neigh_x] = 1;
                to_process.add({neigh_y, neigh_x});
                maze[neigh_y][neigh_x] = maze[current_pos.y][current_pos.x] + 1;
            }
        }
    }
}

bool Maze::find_path(Pair start) {
    current_path_idx = 0;
    path_len = 0;
    uint8_t cur_val = maze[start.y][start.x];
    Pair current_pos = start;
    uint8_t direction = mouse_direction;

    while (cur_val != 0) {
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t neigh_y = (current_pos.y + NEIGHBOURS[i].y) % MAZE_WIDTH;
            uint8_t neigh_x = (current_pos.x + NEIGHBOURS[i].x) % MAZE_WIDTH;
            uint8_t neigh_val = maze[neigh_y][neigh_x];

            if (!check_wall(current_pos, WALLS[i]) && (cur_val - 1) == neigh_val) {
                cur_val = neigh_val;
                current_pos = {neigh_y, neigh_x};
                if (direction == i) {
                    path[path_len++] = 'F';
                }
                else if ((direction + 1) % 4 == i) {
                    path[path_len++] = 'R';
                    path[path_len++] = 'F';
                    direction = (direction + 1) % 4;
                }
                else if ((direction + 2) % 4 == i) {
                    path[path_len++] = 'A';
                    path[path_len++] = 'F';
                    direction = (direction + 2) % 4;
                }
                else if ((direction + 3) % 4 == i) {
                    path[path_len++] = 'L';
                    path[path_len++] = 'F';
                    direction = (direction + 3) % 4;
                }
                break;
            }
        }
    }
    return path_len != 0;
}

char Maze::get_next_move(bool update_counter) {
    char next_move = path[current_path_idx];
    if (update_counter) {
        current_path_idx++;
    }
    return next_move;
}

uint8_t Maze::get_path_len() {
    return path_len;
}

void Maze::save_maze() {
    if (DEBUG_AVAILABLE) {
       Serial.println("Saving maze...");
    }
    int address = 0;
    uint8_t wall_value;
    for (int y = 0; y < MAZE_WIDTH; y++) {
        for (int x = 0; x < MAZE_WIDTH; x++) {
            wall_value = walls[y][x];
            EEPROM.put(address, wall_value);
            address += sizeof(uint8_t);
        }
    }
    if (DEBUG_AVAILABLE) {
       Serial.println("Saved...");
    }
}

void Maze::load_maze() {
    if (DEBUG_AVAILABLE) {
        Serial.println("loading maze...");
    }
    int address = 0;
    uint8_t wall_value;
    for (int y = 0; y < MAZE_WIDTH; y++) {
        for (int x = 0; x < MAZE_WIDTH; x++) {
            EEPROM.get(address, wall_value);
            walls[y][x] = wall_value;
            address += sizeof(uint8_t);
        }
    }
    if (DEBUG_AVAILABLE) {
        Serial.println("Loaded...");
    }
}

void Maze::print_path() {
    if (!DEBUG_AVAILABLE) {
        return;
    }
    for (int i = current_path_idx; i < path_len; i++) {
        Serial.print(path[i]);
    }
    Serial.println();
}

void Maze::print_maze() {
    if (!DEBUG_AVAILABLE) {
        return;
    }
    
    uint8_t cell_shape = 3;
    // print header
    for (uint8_t x = 0; x < MAZE_WIDTH; x++) {
        Serial.print("+");
        for (uint8_t sub_x = 0; sub_x < cell_shape; sub_x++) {
            Serial.print("-");
        }
    }
    Serial.println("+");
    for (uint8_t y = 0; y < MAZE_WIDTH; y++) {
        for (uint8_t sub_y = 0; sub_y < cell_shape - 1; sub_y++) {
            for (uint8_t x = 0; x < MAZE_WIDTH; x++) {
                uint8_t maze_value = maze[y][x];
                uint8_t wall_value = walls[y][x];
                bool left_wall = (wall_value & LEFT_WALL);
                bool down_wall = (wall_value & DOWN_WALL);
                
                
                if (sub_y == 0) {
                    if (left_wall) {
                        Serial.print("|");
                    }
                    else {
                        Serial.print(" ");
                    }
                    if (y == mouse_position.y && x == mouse_position.x) {
                        Serial.print(" ");
                        Serial.print(DIRECTION_TO_CHAR[mouse_direction]);
                        Serial.print(" ");
                    }
                    else if (maze_value > 99) {
                        Serial.print(maze_value);
                    }
                    else if (maze_value > 9) {
                        Serial.print(" ");
                        Serial.print(maze_value);
                    }
                    else {
                        Serial.print(" ");
                        Serial.print(maze_value);
                        Serial.print(" ");
                    }

                }
                else {
                    Serial.print("+");
                    if (down_wall) {
                        Serial.print("---");
                    }
                    else {
                        Serial.print("   ");
                    }
                }
            }
            if (sub_y == 1) {
                Serial.println("+");
            }
            else {
                Serial.println("|");
            }
        }
    }
}