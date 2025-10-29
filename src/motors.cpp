#include "motors.h"

Motor motor_left(LEFT_DIR, LEFT_PWM, MOTOR_LEFT_POLARITY);
Motor motor_right(RIGHT_DIR, RIGHT_PWM, MOTOR_RIGHT_POLARITY);

uint32_t last_update = micros();
float last_angle_error = 0;

Motor::Motor(int dir_pin, int pwm_pin, int encoder_polarity) {
    this->dir_pin = dir_pin;
    this->pwm_pin = pwm_pin;
    this->polarity = encoder_polarity;
    pinMode(dir_pin, OUTPUT);
    pinMode(pwm_pin, OUTPUT);
    this->reset_motor();
}

void Motor::reset_motor() {
    this->accelerating = true;
    this->pwm = 0;
    this->acceleration_speed = 0;
    this->last_speed_error = 0;
    this->cum_speed_error = 0;
    set_pwm(0);
}

void Motor::set_speed(float speed) {
    this->speed = speed;
    if (this->acceleration_speed != speed) {
        this->cum_speed_error = 0;
        this->last_speed_error = 0;
    }
}

void Motor::set_direction(int direction) {
    int polarity_bit = (direction + 1) >> 1;
    digitalWrite(this->dir_pin, polarity_bit ^ this->polarity);
}

void Motor::set_pwm(int pwm) {
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);
    if (pwm < 0) {
        set_direction(-1);
        pwm *= -1;
    }
    else {
        set_direction(1);
    }
    this->pwm = pwm;
    analogWrite(this->pwm_pin, pwm);
}

void Motor::set_enable(bool enabled) {
    this->enabled = enabled;
}

void Motor::accelerate() {
    if (this->accelerating) {
        if (abs(this->acceleration_speed) >= abs(this->speed)) {
            this->accelerating = false;
            this->acceleration_speed = this->speed;
        }
        else {
            int speed_delta = 8;
            if (this->speed < 0) {
                speed_delta *= -1;
            }
            this->acceleration_speed += speed_delta;
        }
    } else if (this->acceleration_speed != this->speed) {
        this->acceleration_speed = this->speed;
    }
}

int pwm_converter(float speed) {
    float multiplier = 0.1;
    int pwm = 0;
    if (speed <= 50) {
        multiplier = 0.4;
    } else if (speed <= 150) {
        multiplier = 0.4;
    } else if (speed <= 250) {
        multiplier = 0.21;
    } else if (speed <= 350) {
        multiplier = 0.16;
    } else if (speed <= 450) {
        multiplier = 0.135;
    } else if (speed <= 550) {
        multiplier = 0.12;
    } else if (speed <= 650) {
        multiplier = 0.11;
    } else if (speed <= 750) {
        multiplier = 0.1;
    } else if (speed <= 850) {
        multiplier = 0.1;
    } else if (speed <= 950) {
        multiplier = 0.1;
    } 
    pwm = (int) (speed * multiplier);
    return pwm;
}

void Motor::update_pwm(float distance_change, float angle_error, float pos_error, uint32_t time_delta_micros) {
    if (!this->enabled) {
        return;
    }
    accelerate();
    if (time_delta_micros < 1) {
        return;
    }
    float time_delta = (float)time_delta_micros / 1000000; // secs
    float current_speed = distance_change / time_delta;
    // in same dt we can get different amount of encodfer ticks. need to avarage value
    this->actual_speed = (current_speed + this->last_actual_speed) / 2;
    this->last_actual_speed = current_speed;

    float target_speed = this->acceleration_speed + angle_error + pos_error;

    float e = target_speed - this->actual_speed;
    float de = e - this->last_speed_error;
    this->last_speed_error = e;
    float speed_error = e * KP_FWD + de * KD_FWD;

    this->cum_speed_error = speed_error;
    float pwm_new = target_speed + this->cum_speed_error;
    bool is_neg = pwm_new < 0;
    pwm_new = pwm_converter(abs(pwm_new));
    if (is_neg) {
        pwm_new *= -1;
    }
    set_pwm(pwm_new);
}

float Motor::get_speed() {
    return this->actual_speed;
}

float Motor::get_inner_speed() {
    return this->acceleration_speed;
}

int Motor::get_pwm() {
    return this->pwm;
}

void disable_motors() {
    motor_left.set_enable(false);
    motor_right.set_enable(false);
}

void enable_motors() {
    motor_left.set_enable(true);
    motor_right.set_enable(true);
}

void stop_motors() {
    motor_left.set_speed(0);
    motor_right.set_speed(0);
    do {
        update_motor_controllers();
    } while(motor_left.get_speed() != 0 || motor_right.get_speed() != 0);
    motor_left.reset_motor();
    motor_right.reset_motor();
}

float calculate_angle_error() {
    float angle_error = 0;
    if (USE_GYRO) {
            angle_error = g_gyro_angle - mouse.get_angle();
        } else {
            angle_error = get_robot_angle() - mouse.get_angle();
        }
        float de = angle_error - last_angle_error;
        last_angle_error = angle_error;
        angle_error = angle_error * KP_ROT + de * KD_ROT;
        return angle_error;
}

void update_motor_controllers() {
    update_encoders();
    update_sensors();
    float increment_left = get_increment_left();
    float increment_right = get_increment_right();
    float angle_error = 0;
    float pos_error = 0;
    if (g_steering_enabled) {
        pos_error = calculate_steering_adjustment();
        angle_error = calculate_angle_error();
    }
    uint32_t cur_time = micros();
    uint32_t time_delta = cur_time - last_update;
    last_update = cur_time;
    motor_left.update_pwm(increment_left, -angle_error, pos_error, time_delta);
    motor_right.update_pwm(increment_right, angle_error, -pos_error, time_delta);
    print_motors();
}

void test_mototrs() {
    for (int i = 0; i < 255; i++) {
        analogWrite(LEFT_PWM, i);
        analogWrite(RIGHT_PWM, i);
        delay(10);
    }
    while(!button_pressed()) {

    }
    analogWrite(LEFT_PWM, 0);
    analogWrite(RIGHT_PWM, 0);
    delay(2000);
}