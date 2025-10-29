#ifndef MAZE_H
#define MAZE_H

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"

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

class Queue {
  public:
  explicit Queue(int maxSize = 64) : MAX_ITEMS(maxSize) {
    mData = new Pair[MAX_ITEMS + 1];
    clear();
  }

  ~Queue() {
    delete[] mData;
  };

  int size() {
    return mItemCount;
  }

  void clear() {
    mHead = 0;
    mTail = 0;
    mItemCount = 0;
  }

  void add(Pair item) {
    mData[mTail] = item;
    ++mTail;
    ++mItemCount;
    if (mTail > MAX_ITEMS) {
      mTail -= MAX_ITEMS;
    }
  }

  Pair pop() {
    Pair result = mData[mHead];
    ++mHead;
    if (mHead > MAX_ITEMS) {
      mHead -= MAX_ITEMS;
    }
    --mItemCount;
    return result;
  }

  bool empty() {
    return mItemCount == 0;
  }

  protected:
  Pair *mData;
  const int MAX_ITEMS;
  int mHead;
  int mTail;
  int mItemCount;
};

extern uint8_t WALLS[4];
extern Pair NEIGHBOURS[4];
extern char DIRECTION_TO_CHAR[4];


class Maze {
    public:
        Maze();
        void reset_maze();
        void floodfill(Pair target);
        bool find_path(Pair start);
        uint8_t get_next_move(bool update_counter = true);
        uint8_t get_path_len();
        void set_walls(bool is_left_wall, bool is_front_wall, bool is_right_wall);
        void set_visited();
        bool is_visited();
        void update_direction(uint8_t change);
        void update_position();
        Pair get_position();
        uint8_t get_direction();
        Pair get_finish();
        Pair get_start();
        bool is_wall(uint8_t mouse_dir_wall);
        void set_position(Pair position);
        void set_direction(uint8_t dir);
        void save_maze(); // save only walls and floodfill after load
        void load_maze();
        void print_maze(); // print walls with flooded values
        void print_path();
        void lock_maze();

    private:
        uint8_t maze[MAZE_WIDTH][MAZE_WIDTH];
        uint8_t walls[MAZE_WIDTH][MAZE_WIDTH];
        Pair start_position = {15, 0};
        Pair mouse_position = start_position;
        uint8_t mouse_direction = UP;
        Pair target = {11, 3};
        uint8_t path[MAZE_WIDTH * MAZE_WIDTH];
        uint8_t path_len = 0;
        uint8_t current_path_idx = 0;

        bool check_wall(Pair position, uint8_t wall);
};

extern Maze maze;

#endif