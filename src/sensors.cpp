#include "sensors.h"

int g_left_sensor_raw;
int g_right_sensor_raw;
int g_front_sensor_raw_left;
int g_front_sensor_raw_right;

int g_left_sensor;
int g_right_sensor;
int g_front_sensor;

bool g_is_left_wall;
bool g_is_right_wall;
bool g_is_front_wall;

bool g_left_button;
bool g_right_button;

static float last_steering_error = 0;

bool g_steering_enabled;
float g_cross_track_error = 0;
float g_steering_adjustment = 0;

uint32_t last_gyro_read_time = 0;

const int MPU_addr=0x68;

float gyro_error = 0;

float vcc_coef = 0.0;

int read_step = 0;

int get_front_sensor() {
    int value;
    value = g_front_sensor;
    return value;
}

int read_row(uint8_t sensor) {
    float rawData = 0;
    for (int i = 0; i < READS_PER_SENSOR; i++) {
        rawData += analogRead(sensor);
    }
    return vcc_coef * rawData / READS_PER_SENSOR;
}

void start_gyro_read() {
    last_gyro_read_time = millis();
}

float read_gyro() {
    unsigned long start_time = millis();
    float time_delta = float(start_time - last_gyro_read_time) / 1000; // sec
    last_gyro_read_time = start_time;
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x47); // gyro data at 0x43, to get acc set to 0x3b
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    int16_t gyro_z_raw = Wire.read()<<8 | Wire.read();
    float gyro_delta = gyro_z_raw / 131.0 + gyro_error;
    gyro_delta = gyro_delta * time_delta;
    return gyro_delta;
}

void update_sensors() {
    g_left_sensor_raw = read_row(LEFT_WALL_SENSOR);
    g_right_sensor_raw = read_row(RIGHT_WALL_SENSOR);
    g_front_sensor_raw_left = read_row(FRONT_LEFT_WALL_SENSOR);
    // g_front_sensor_raw_right = read_row(FRONT_RIGHT_WALL_SENSOR);

    g_left_sensor = (int)(g_left_sensor_raw * LEFT_SCALE);
    g_right_sensor = (int)(g_right_sensor_raw * RIGHT_SCALE);
    // g_front_sensor = (int)((g_front_sensor_raw_left + g_front_sensor_raw_right) * FRONT_SCALE);
    g_front_sensor = (int)((g_front_sensor_raw_left) * FRONT_SCALE);

    g_is_left_wall = g_left_sensor > LEFT_THRESHOLD;
    g_is_right_wall = g_right_sensor > RIGHT_THRESHOLD;
    g_is_front_wall = g_front_sensor > FRONT_THRESHOLD;

    int button = analogRead(BUTTON);
    g_left_button = false;
    g_right_button = false;
    if (button < RIGHT_BUTTON_THRESHOLD) {
        g_right_button = true;
    }
    else if (button < LEFT_BUTTON_THRESHOLD) {
        g_left_button = true;
    }
    if (g_steering_enabled) {
        turn_wall_leds(g_is_left_wall, g_is_front_wall, g_is_right_wall);
    }
    
}

float calculate_steering_adjustment() {
    float error = 0;
    float left_error = g_left_sensor - NOMINAL_VALUE;
    float right_error = g_right_sensor - NOMINAL_VALUE;

    if (g_is_left_wall && g_is_right_wall) {
        error = left_error - right_error;
    } else if (g_is_left_wall) {
        error = 2.0 * left_error;
    } else if (g_is_right_wall) {
        error = -2.0 * right_error;
    }
    // Check value 100
    // if (g_front_sensor > 100) {
    //     error = 0;
    // }

    g_cross_track_error = error;

    // always calculate the adjustment for testing. It may not get used.
    float pTerm = KP_STEER * error;
    float dTerm = KD_STEER * (error - last_steering_error);
    float adjustment = (pTerm + dTerm);
    // TODO: are these limits appropriate, or even needed?
    // adjustment = constrain(adjustment, -STEERING_ADJUST_LIMIT, STEERING_ADJUST_LIMIT);
    last_steering_error = error;
    return adjustment;
}

// wall calibration controls

void reset_steering() {
  last_steering_error = g_cross_track_error;
  g_steering_adjustment = 0;
}

void enable_steering() {
  reset_steering();
  g_steering_enabled = true;
};

void disable_steering() {
    g_steering_enabled = false;
    reset_leds();
}

bool button_pressed() {
    update_sensors();
    return g_left_button || g_right_button;
}

void calibrate_gyro() {
    int num_reads = 100;
    float gyro_val = 0;
    for (int i = 0; i < num_reads; i++) {
        gyro_val += read_gyro();
    }
    gyro_error = gyro_val / num_reads;
}

void init_sesnors() {
    pinMode(LEFT_WALL_SENSOR, INPUT);
    pinMode(RIGHT_WALL_SENSOR, INPUT);
    pinMode(FRONT_LEFT_WALL_SENSOR, INPUT);
    // pinMode(FRONT_RIGHT_WALL_SENSOR, INPUT);

    pinMode(BUTTON, INPUT_PULLUP);

    g_left_button = false;
    g_right_button = false;

    vcc_coef = analogRead_VCC() / REF_VCC;

    // Wire.begin();
    // Wire.beginTransmission(MPU_addr);
    // Wire.write(0x6B);
    // Wire.write(0);
    // Wire.endTransmission(true);
    // calibrate_gyro();
}
