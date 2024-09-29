#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

//**** ENABLERS ****/
const bool BLUETOOTH_ENABLE = false;
const bool SERIAL_ENABLE = true;

//**** CONSTANTS ****//

//** SERIAL PRINT **//
#define DEBUG_LOGGING 0
#define DEBUG_LOGGING_WITH_MOTOTRS 0

const bool DEBUG_AVAILABLE = DEBUG_LOGGING || DEBUG_LOGGING_WITH_MOTOTRS;

//** DRIVER **//

const float ENCODER_PULSES = 3.0; // 3 up pulses per one cycle (total 12 pulses for 2 channels for up and down events)
const float GEAR_RATIO = 100.0; // numbers from documentation. In real life need to correct it somehow
const float WHEEL_DIAMETER = 34.4; // numbers from documentation.

const float MOUSE_RADIUS = 40.0; // left turn R = 40.455; // reduce in case of pere-turn, increase in case of nedo-turn

const float SPEED_FF = (1.0 / 6.0); // tune to make average speed 
const float ROTATION_BIAS = 0.0018; // to make robot go forward

const int ENCODER_LEFT_POLARITY = 1;
const int ENCODER_RIGHT_POLARITY = 1;

const int MAX_PWM = 250;
const int MIN_PWM = -250;

//** PD  **/
const float KP_FWD = 3.2;
const float KD_FWD = 2.0;

const float KP_ROT = 2.2;
const float KD_ROT = 1.1;

// controller constants for the steering controller
const float KP_STEER = 0.5;
const float KD_STEER = 0.0;

//** MAZE **/

const float CELL = 180.0;
const float HALF_CELL = CELL / 2;
const int SHAPE = 3;

enum Direction: uint8_t {
  UP = 0,
  RIGHT = 1,
  DOWN = 2,
  LEFT = 3,
};

//** SENSORS **/
const int LEFT_CALIBRATION = 265; // test when robot centered and no front wall
const int RIGHT_CALIBRATION = 265; // test when robot centered and no front wall
const int FRONT_CALIBRATION_LEFT = 300; // test when robot with back walls
// const int FRONT_CALIBRATION_RIGHT = 195; // test when robot with back walls

const int NOMINAL_VALUE = 100; // sensors should give 100 in normal position

const float LEFT_SCALE = (float)NOMINAL_VALUE / LEFT_CALIBRATION;
const float RIGHT_SCALE = (float)NOMINAL_VALUE / RIGHT_CALIBRATION;
const float FRONT_SCALE = (float)NOMINAL_VALUE / FRONT_CALIBRATION_LEFT;

const int READS_PER_SENSOR = 1;

// values to detect walls
const int LEFT_THRESHOLD = 70;
const int RIGHT_THRESHOLD = 70;
const int FRONT_THRESHOLD = 70;

// value to decide if we in cell center
const int FRONT_REFERENCE = 110;
const int PRE_TURN_REFERENCE = 85;

// button treshold
const int LEFT_BUTTON_THRESHOLD = 200;
const int RIGHT_BUTTON_THRESHOLD = 100;

// sensor error limit
const float STEERING_ADJUST_LIMIT = 10.0; // deg/s

//**** DISTANCE RELATED ****//
const int ROBOT_OFFSET = 60;
const int BACK_WALL_TO_CENTER = HALF_CELL - ROBOT_OFFSET;
const int SENSING_OFFSET = 60;

//**** Control loop timing. Pre-calculate to save time in interrupts ****..
const float LOOP_FREQUENCY = 500.0;
const float LOOP_INTERVAL = (1.0 / LOOP_FREQUENCY);

//**** HARDWARE CONFIGURATION ****//

// ** SENSORS ** //
const uint8_t LEFT_WALL_SENSOR          = A0;
const uint8_t RIGHT_WALL_SENSOR         = A1;
const uint8_t FRONT_LEFT_WALL_SENSOR    = A2;
// const uint8_t FRONT_RIGHT_WALL_SENSOR   = A3;

const uint8_t BUTTON = 10;

// ** MOTORS ** //
const uint8_t ENCOUNTER_RIGHT = 0;
const uint8_t ENCOUNTER_LEFT = 1;

const uint8_t LEFT_DIR = 8;
const uint8_t RIGHT_DIR = 4;
const uint8_t LEFT_PWM = 9;
const uint8_t RIGHT_PWM = 6;

// ** LEDS **//
const uint8_t DATA_PIN = 5;
const uint8_t LATCH_PIN = 7;
const uint8_t CLOCK_PIN = 16;

// ** BLUETOOTH ** //
const uint8_t BT_RX = 15;
const uint8_t BT_TX = 14;


// ** common place for debugging
// extern float var1;
// extern float var2;
// float var1;
// float var2;

#endif