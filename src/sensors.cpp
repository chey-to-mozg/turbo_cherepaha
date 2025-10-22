#include "sensors.h"

int g_left_sensor_raw;
int g_right_sensor_raw;
int g_front_sensor_raw_left;
int g_front_sensor_raw_right;

int sensor_data[4] = {0}; // contain data with turned on/off reads for every sensor
uint8_t read_step = 0;

int g_left_sensor;
int g_right_sensor;
int g_front_sensor_left;
int g_front_sensor_right;
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

MPU6050 mpu;
bool DMPReady = false;

uint8_t FIFOBuffer[64];
float g_gyro_angle;
float prev_gyro_angle = 0;

float vcc_coef = 1.0;


int read_row(uint8_t sensor) {
    float rawData = 0;
    for (int i = 0; i < READS_PER_SENSOR; i++) {
        rawData += analogRead(sensor);
    }
    return vcc_coef * rawData / READS_PER_SENSOR;
}

void read_gyro() {
    float angle = 0;
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
        Quaternion q;           // [w, x, y, z]         Quaternion container
        VectorFloat gravity;    // [x, y, z]            Gravity vector
        float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        angle = ypr[0] * 180/M_PI;
    } else {
        angle = prev_gyro_angle;
    }
    if (prev_gyro_angle < -90 && angle > 90) {
        prev_gyro_angle += 360;
    } else if (prev_gyro_angle > 90 && angle < -90) {
        prev_gyro_angle -= 360;
    }
    float angle_delta = prev_gyro_angle - angle;
    prev_gyro_angle = angle;
    g_gyro_angle += angle_delta;
}

void read_sensors() {
    // switch (read_step) {
    //     case 0:
    //         // enable 1 emitter
    //         turn_emmiters(FRONT_EMITTER);
    //         break;
    //     case 1:
    //         // wait for emitter to power up
    //         break;
    //     case 2:
    //         g_front_sensor_raw_left = analogRead(FRONT_LEFT_WALL_SENSOR);
    //         g_front_sensor_raw_right = analogRead(FRONT_RIGHT_WALL_SENSOR);
    //         break;
    //     case 3:
    //         // enable another emitter
    //         turn_emmiters(SIDE_EMITTER);
    //         break;
    //     case 4:
    //         // wait for emitter to power up
    //         break;
    //     case 5:
    //         // read all sensors right and front left is active
    //         g_left_sensor_raw = analogRead(LEFT_WALL_SENSOR);
    //         g_right_sensor_raw = analogRead(RIGHT_WALL_SENSOR);
    //         break;
    // }
    // read_step++;
    // read_step = read_step % 6;
    switch (read_step) {
        case 0:
            // enable 1 emitter
            turn_emmiters(FRONT_EMITTER);
            break;
        case 1:
            g_front_sensor_raw_left = analogRead(FRONT_LEFT_WALL_SENSOR);
            g_front_sensor_raw_right = analogRead(FRONT_RIGHT_WALL_SENSOR);
            break;
        case 2:
            // enable another emitter
            turn_emmiters(SIDE_EMITTER);
            break;
        case 3:
            // read all sensors right and front left is active
            g_left_sensor_raw = analogRead(LEFT_WALL_SENSOR);
            g_right_sensor_raw = analogRead(RIGHT_WALL_SENSOR);
            break;
    }
    read_step++;
    read_step = read_step % 4;
}

void update_sensors() {
    read_sensors();

    g_left_sensor = (int)(g_left_sensor_raw * LEFT_SCALE);
    g_right_sensor = (int)(g_right_sensor_raw * RIGHT_SCALE);
    g_front_sensor_left = (int)(g_front_sensor_raw_left * FRONT_SCALE_LEFT);
    g_front_sensor_right = (int)( g_front_sensor_raw_right * FRONT_SCALE_RIGHT);
    g_front_sensor = (int)((g_front_sensor_left + g_front_sensor_right) / 2);

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
    read_gyro();
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

void calibrate_gyro(uint8_t devStatus) {
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateGyro(6);
        Serial.println("These are the Active offsets: ");
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));   //Turning ON DMP
        mpu.setDMPEnabled(true);

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
    } 
    else {
        while(!button_pressed()) {
            Serial.print(F("DMP Initialization failed (code ")); //Print the error code
            Serial.print(devStatus);
            Serial.println(F(")"));
            turn_all_leds();
            delay(1000);
        }
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
    }
}

void init_gyro() {
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Initializing DMP..."));
    uint8_t devStatus = mpu.dmpInitialize();
    mpu.setXAccelOffset(-6568);
    mpu.setYAccelOffset(4124);
    mpu.setZAccelOffset(9834);
    mpu.setXGyroOffset(-97);
    mpu.setYGyroOffset(7);
    mpu.setZGyroOffset(25);
    
    calibrate_gyro(devStatus);
    g_gyro_angle = 0;
}

void init_sesnors() {
    pinMode(LEFT_WALL_SENSOR, INPUT);
    pinMode(RIGHT_WALL_SENSOR, INPUT);
    pinMode(FRONT_LEFT_WALL_SENSOR, INPUT);
    pinMode(FRONT_RIGHT_WALL_SENSOR, INPUT);

    pinMode(BUTTON, INPUT_PULLUP);

    g_left_button = false;
    g_right_button = false;

    // vcc_coef = analogRead_VCC() / REF_VCC;

    init_gyro();
}
