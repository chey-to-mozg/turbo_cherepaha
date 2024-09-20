#ifndef MAZE_H
#define MAZE_H

#include <Arduino.h>
#include "queue.h"
#include <EEPROM.h>

#define MAZE_WIDTH 16

#define VISITED 0xF0
#define UP_WALL_VISITED 0b10000000
#define RIGHT_WALL_VISITED 0b01000000
#define DOWN_WALL_VISITED 0b00100000
#define LEFT_WALL_VISITED 0b00010000
#define UP_WALL 0b00001000
#define RIGHT_WALL 0b00000100
#define DOWN_WALL 0b00000010
#define LEFT_WALL 0b00000001

struct Pair {
    uint8_t y;
    uint8_t x;

    bool operator==(const Pair& a) const
    {
        return (x == a.x && y == a.y);
    }

    bool operator!=(const Pair& a) const
    {
        return !(*this == a);
    }
};

extern uint8_t WALLS[4];
extern Pair NEIGHBOURS[4];
extern char DIRECTION_TO_CHAR[4];


class Maze {
    public:
        Maze();
        void floodfill(Pair target);
        void find_path(Pair start);
        char get_next_move();
        uint8_t get_path_len();
        void set_walls(bool is_left_wall, bool is_front_wall, bool is_right_wall);
        void set_visited();
        bool is_visited();
        void update_direction(uint8_t change);
        void update_position();
        Pair get_position();
        Pair get_finish();
        Pair get_start();
        void set_position(Pair position);
        void set_direction(uint8_t dir);
        void save_maze(); // save only walls and floodfill after load
        void load_maze();
        void print_maze(); // print walls with flooded values
        void print_path();

    private:
        uint8_t maze[MAZE_WIDTH][MAZE_WIDTH];
        uint8_t walls[MAZE_WIDTH][MAZE_WIDTH];
        Pair start_position = {15, 0};
        Pair mouse_position = start_position;
        uint8_t mouse_direction = UP;
        Pair target = {15, 2};
        char path[MAZE_WIDTH * MAZE_WIDTH];
        uint8_t path_len = 0;
        uint8_t current_path_idx = 0;

        bool check_wall(Pair position, uint8_t wall);
};

extern Maze maze;

#endif