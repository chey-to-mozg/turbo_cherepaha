#include "sensors.h"

volatile int g_left_sensor_raw;
volatile int g_right_sensor_raw;
volatile int g_front_sensor_raw_left;
volatile int g_front_sensor_raw_right;

volatile int g_left_sensor;
volatile int g_right_sensor;
volatile int g_front_sensor;

volatile bool g_is_left_wall;
volatile bool g_is_right_wall;
volatile bool g_is_front_wall;

volatile bool g_left_button;
volatile bool g_right_button;

static float last_steering_error = 0;

bool g_steering_enabled;
volatile float g_cross_track_error;
volatile float g_steering_adjustment;

const int MPU_addr=0x68;

int16_t gyro_z_raw;
float gyro_delta = 0;

volatile float gyro_reference = 0;
volatile float gyro_error = 0;
volatile float g_gyro_angle = 0;

int read_step = 0;


int read_row(uint8_t sensor) {
    int rawData = 0;
    for (int i = 0; i < READS_PER_SENSOR; i++) {
        rawData += analogRead(sensor);
    }
    return rawData / READS_PER_SENSOR;
}

void read_gyro() {
    switch (read_step)
    {
        case 0:
            Wire.beginTransmission(MPU_addr);
            break;
        case 1:
            Wire.write(0x47); // gyro data at 0x43, to get acc set to 0x3b
            break;
        case 2:
            Wire.endTransmission(false);
            break;
        case 3:
            Wire.requestFrom(MPU_addr,14,true);
            break;
        case 4:
            gyro_z_raw = Wire.read()<<8 | Wire.read();
            gyro_delta = gyro_z_raw / 131.0 - gyro_error;
            g_gyro_angle += gyro_delta * LOOP_INTERVAL * 5; // we get one value per 5 intervals
            break;
        default:
            break;
    }
    read_step = (read_step + 1) % 5;
}

void read_sensors() {
    g_left_sensor_raw = read_row(LEFT_WALL_SENSOR);
    g_right_sensor_raw = read_row(RIGHT_WALL_SENSOR);
    g_front_sensor_raw_left = read_row(FRONT_LEFT_WALL_SENSOR);
    g_front_sensor_raw_right = read_row(FRONT_RIGHT_WALL_SENSOR);
    
    read_gyro();

    g_left_sensor = (int)(g_left_sensor_raw * LEFT_SCALE);
    g_right_sensor = (int)(g_right_sensor_raw * RIGHT_SCALE);
    g_front_sensor = (int)((g_front_sensor_raw_left + g_front_sensor_raw_right) * FRONT_SCALE);

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

}

float calculate_steering_adjustment() {
    float error = 0;
    float left_error = NOMINAL_VALUE - g_left_sensor;
    float right_error = NOMINAL_VALUE - g_right_sensor;
    float gyro_error = g_gyro_angle - gyro_reference;

    if (g_is_left_wall && g_is_right_wall) {
        error = left_error - right_error;
    } else if (g_is_left_wall) {
        error = 2.0 * left_error;
    } else if (g_is_right_wall) {
        error = -2.0 * right_error;
    }
    // Check value 100
    if (g_front_sensor > 100) {
        error = 0;
    }

    error += gyro_error;

    g_cross_track_error = error;

    // always calculate the adjustment for testing. It may not get used.
    float pTerm = KP_STEER * error;
    float dTerm = KD_STEER * (error - last_steering_error) ;
    float adjustment = (pTerm + dTerm) * LOOP_INTERVAL;
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
}

bool button_pressed() {
    return analogRead(BUTTON) < 500;
}

void calibrate_gyro() {
    int num_reads = 100;
    float gyro_val = 0;
    for (int i = 0; i < num_reads; i++) {
        gyro_val += gyro_delta;
        delay(10); // wait for full read cycle
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        gyro_error = gyro_val / num_reads;
        g_gyro_angle = 0;
    }
}

void reset_gyro() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        g_gyro_angle = gyro_reference;
    }
}

void update_gyro_reference(float diff) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        gyro_reference += diff;
    }
}

void init_sesnors() {
    pinMode(LEFT_WALL_SENSOR, INPUT);
    pinMode(RIGHT_WALL_SENSOR, INPUT);
    pinMode(FRONT_LEFT_WALL_SENSOR, INPUT);
    pinMode(FRONT_RIGHT_WALL_SENSOR, INPUT);

    pinMode(BUTTON, INPUT_PULLUP);

    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    g_steering_enabled = false;
}
