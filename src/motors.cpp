#include "motors.h"

Motor motor_left(LEFT_DIR, LEFT_PWM, MOTOR_LEFT_POLARITY);
Motor motor_right(RIGHT_DIR, RIGHT_PWM, MOTOR_RIGHT_POLARITY);

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
    if (!this->accelerating) {
        this->acceleration_speed = this->speed;
    }
    update_encoders();
    this->last_update = millis();
    delay(2);
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
            int speed_delta = 4;
            if (this->speed < 0) {
                speed_delta *= -1;
            }
            this->acceleration_speed += speed_delta;
        }
    }
}

void Motor::update_pwm(float distance_change, float angle_error, float pos_error) {
    if (!this->enabled) {
        return;
    }
    accelerate();
    uint32_t cur_time = millis();
    uint32_t time_delta_millis = cur_time - this->last_update;
    float time_delta = (float)time_delta_millis / 1000; // secs
    if (time_delta == 0) {
        time_delta = ULONG_MAX / 1000;
    }
    this->actual_speed = distance_change / time_delta;
    this->last_update = cur_time;
    float pwm_new;
    float e = (this->acceleration_speed - this->actual_speed) * SPEED_FF;
    float de = this->last_speed_error - e;
    this->last_speed_error = e;
    float speed_error = e * KP_FWD + de * KD_FWD;
    angle_error = angle_error * KP_ROT;
    pwm_new = this->acceleration_speed * SPEED_FF;
    this->cum_speed_error = speed_error + angle_error;
    pwm_new += this->cum_speed_error + pos_error;
    set_pwm((int)pwm_new);
}

float Motor::get_speed() {
    return this->actual_speed;
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

void update_motor_controllers() {
    update_encoders();
    update_sensors();
    float increment_left = get_increment_left();
    float increment_right = get_increment_right();
    float angle_error = 0;
    float pos_error = 0;
    if (g_steering_enabled) {
        pos_error = calculate_steering_adjustment();
        angle_error = get_robot_angle() - mouse.get_angle();
    }
    motor_left.update_pwm(increment_left, angle_error, pos_error);
    motor_right.update_pwm(increment_right, -angle_error, -pos_error);
    print_motors();
}