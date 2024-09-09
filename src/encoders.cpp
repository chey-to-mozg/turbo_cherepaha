#include "encoders.h"

const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const float DEG_PER_MM_DIFFERENCE = (180.0 / (2 * MOUSE_RADIUS * PI));

int g_left_dir;
int g_right_dir;

volatile int g_count_left;
volatile int g_count_right;

float s_total_left;
float s_total_right;

float s_distance_left;
float s_distance_right;

static volatile int left_delta;
static volatile int right_delta;


int diff_left;
int diff_right;

float g_increment;
float g_distance;
float g_angle;
float g_angle_increment;

void left_increment() {
  g_count_left += g_left_dir;
}

void right_increment() {
  g_count_right += g_right_dir;
}

void reset_encoders() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      left_delta = 0;
      right_delta = 0;
      g_count_right = 0;
      g_count_left = 0;
      g_distance = 0;
      g_angle = 0;
      g_left_dir = 1;
      g_right_dir = 1;
      s_distance_left = 0;
      s_total_left = 0;
      s_distance_right = 0;
      s_total_right = 0;
      g_increment = 0;
      g_angle_increment = 0;
    }
}

void init_encoders() {
    pinMode(ENCOUNTER_RIGHT, INPUT);
    pinMode(ENCOUNTER_LEFT, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCOUNTER_RIGHT), right_increment, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCOUNTER_LEFT), left_increment, FALLING);

    reset_encoders();
}

float robot_position() {
  float distance;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { distance = g_distance; }
  return distance;
}

float robot_angle() {
  float angle;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { angle = g_angle; }
  return angle;
}

float robot_fwd_increment() {
    float distance;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { 
      distance = g_increment; 
    }
    return distance;
}

float robot_rot_increment() {
    float distance;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { 
      distance = g_angle_increment; 
    }
    return distance;
}

void update_encoders() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      left_delta = g_count_left;
      right_delta = g_count_right;
      g_count_left = 0;
      g_count_right = 0;
    }
    s_total_left += left_delta;
    s_total_right += right_delta;
    float left_change = left_delta * MM_PER_COUNT_LEFT;
    float right_change = right_delta * MM_PER_COUNT_RIGHT;
    s_distance_left += left_change;
    s_distance_right += right_change;
    g_increment = 0.5 * (right_change + left_change);
    g_angle_increment = (right_change - left_change) * DEG_PER_MM_DIFFERENCE;
    g_angle += g_angle_increment;
    g_distance += g_increment;
}

