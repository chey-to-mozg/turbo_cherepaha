#include "encoders.h"

const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const float DEG_PER_MM_DIFFERENCE = (180.0 / (2 * MOUSE_RADIUS * PI));

int dir_left;
int dir_right;

volatile int count_left;
volatile int count_right;

int total_conut_left;
int total_count_right;

float distance_left;
float distance_increment_left;
float distance_right;
float distance_increment_right;

static volatile int delta_left;
static volatile int delta_right;

float distance;
float angle;

void left_increment() {
  static bool oldA = false;
  static bool oldB = false;
  bool newB = digitalRead(ENCODER_LEFT_B);
  bool newA = digitalRead(ENCODER_LEFT_CLK) ^ newB;
  int delta = ENCODER_LEFT_POLARITY * ((oldA ^ newB) - (newA ^ oldB));
  count_left += delta;
  oldA = newA;
  oldB = newB;
}

void right_increment() {
  static bool oldA = false;
  static bool oldB = false;
  bool newB = digitalRead(ENCODER_RIGHT_B);
  bool newA = digitalRead(ENCODER_RIGHT_CLK) ^ newB;
  int delta = ENCODER_RIGHT_POLARITY * ((oldA ^ newB) - (newA ^ oldB));
  count_right += delta;
  oldA = newA;
  oldB = newB;
}

void reset_encoders() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      dir_left = 1;
      dir_right = 1;
      count_left = 0;
      count_right = 0;
      total_conut_left = 0;
      total_count_right = 0;
      distance_left = 0;
      distance_increment_left = 0;
      distance_right = 0;
      distance_increment_right = 0;
      delta_left = 0;
      delta_right = 0;
      distance = 0;
      angle = 0; 
    }
}

void init_encoders() {
    pinMode(ENCOUNTER_RIGHT, INPUT);
    pinMode(ENCOUNTER_LEFT, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCOUNTER_RIGHT), right_increment, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCOUNTER_LEFT), left_increment, FALLING);

    reset_encoders();
}

float get_robot_position() {
  float _distance;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { _distance = distance; }
  return _distance;
}

float get_robot_angle() {
  float _angle;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { _angle = angle; }
  return _angle;
}

float get_increment_left() {
  float _increment;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { _increment = distance_increment_left; }
  return _increment;
}

float get_increment_right() {
  float _increment;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { _increment = distance_increment_right; }
  return _increment;
}

void update_encoders() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      delta_left = count_left;
      delta_right = count_right;
      count_left = 0;
      count_right = 0;
    }
    total_conut_left += delta_left;
    total_count_right += delta_right;
    distance_increment_left = delta_left * MM_PER_COUNT_LEFT;
    distance_increment_right = delta_right * MM_PER_COUNT_RIGHT;
    distance_left += distance_increment_left;
    distance_right += distance_increment_right;
    float distance_increment = 0.5 * (distance_increment_left + distance_increment_right);
    float angle_increment = (distance_increment_right - distance_increment_left) * DEG_PER_MM_DIFFERENCE;
    angle += angle_increment;
    distance += distance_increment;
}

