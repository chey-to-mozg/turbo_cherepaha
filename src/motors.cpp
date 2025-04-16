#include "motors.h"

Motor motor_left(LEFT_DIR, LEFT_PWM, ENCODER_LEFT_POLARITY);
Motor motor_right(RIGHT_DIR, RIGHT_PWM, ENCODER_RIGHT_POLARITY);

Motor::Motor(int dir_pin, int pwm_pin, int encoder_polarity) {
    this->dir_pin = dir_pin;
    this->pwm_pin = pwm_pin;
    this->polarity = encoder_polarity;
    pinMode(dir_pin, OUTPUT);
    pinMode(pwm_pin, OUTPUT);
}

void Motor::set_speed(float speed) {
    this->accelerating = true;
    this->acceleration_speed = this->actual_speed;
    this->speed = speed;
    this->cum_speed_error = 0;
    update_encoders();
    this->last_update = millis();
}

void Motor::set_direction(int direction) {
    this->direction = direction;
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

void Motor::accelerate() {
    if (this->accelerating) {
        if (abs(this->acceleration_speed) > abs(this->speed)) {
            this->accelerating = false;
            this->acceleration_speed = this->speed;
        }
        else {
            int speed_delta = 3;
            if (this->speed < 0) {
                speed_delta *= -1;
            }
            this->acceleration_speed += speed_delta;
        }
    }
}

void Motor::update_pwm(float distance_change, float angle_error) {
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
    if (this->speed == 0) {
        pwm_new = 0;
        this->acceleration_speed = 0;
        this->cum_speed_error = 0;
    }
    else {
        pwm_new = this->acceleration_speed * SPEED_FF;
        float speed_error = this->acceleration_speed - this->actual_speed;
        speed_error = speed_error * KP_FWD;
        angle_error = angle_error * KP_ROT;
        this->cum_speed_error += speed_error + angle_error;
        pwm_new += this->cum_speed_error;
    }
    set_pwm((int)pwm_new);
}

float Motor::get_speed() {
    return this->actual_speed;
}

int Motor::get_direction() {
    return this->direction;
}

int Motor::get_pwm() {
    return this->pwm;
}

void stop_motors() {
    int l_impuls = motor_left.get_direction() * -1 * 30;
    int r_impuls = motor_right.get_direction() * -1 * 30;
    motor_left.set_speed(0);
    motor_right.set_speed(0);
    motor_left.set_pwm(l_impuls);
    motor_right.set_pwm(r_impuls);
    delay(50);
    update_motor_controllers();
}

void update_motor_controllers() {
    update_encoders();
    float increment_left = get_increment_left();
    float increment_right = get_increment_right();
    float angle_error = 0;
    // if (g_steering_enabled) {
    //     angle_error = mouse.get_angle() - get_robot_angle();
    // }
    motor_left.update_pwm(increment_left, angle_error);
    motor_right.update_pwm(increment_right, -angle_error);

    if (dir_left != motor_left.get_direction()) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { dir_left = motor_left.get_direction(); }
    }
    if (dir_right != motor_right.get_direction()) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { dir_right = motor_right.get_direction(); }
    }
    print_motors();
}