#include "motors.h"

float s_err_fwd;
float s_err_rot;

static float old_err_fwd = 0;
static float old_err_rot = 0;
float var1;
float var2;

float s_pwm_left;
float s_pwm_right;

bool l_motors_enabled;

void init_motors() {
    pinMode(LEFT_DIR, OUTPUT);
    pinMode(RIGHT_DIR, OUTPUT);
    pinMode(LEFT_PWM, OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);

    stop_motors();
    disable_mototrs();
    reset_motor_controllers();
}

void reset_motor_controllers() {
  s_err_fwd = 0;
  s_err_rot = 0;
  old_err_fwd = 0;
  old_err_rot = 0;
}

void disable_mototrs() {
    l_motors_enabled = false;
}

void enable_mototrs() {
    l_motors_enabled = true;
}

void set_direction_left(int forward) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        g_left_dir = forward;
    }
    int polarity_bit = (forward + 1) >> 1;
    digitalWrite(LEFT_DIR, polarity_bit ^ ENCODER_LEFT_POLARITY);
}

void set_direction_right(int forward) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        g_right_dir = forward;
    }
    int polarity_bit = (forward + 1) >> 1;
    digitalWrite(RIGHT_DIR, polarity_bit ^ ENCODER_RIGHT_POLARITY);
}

void set_left_motor_pwm(int pwm) {
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);
    if (pwm < 0) {
        set_direction_left(-1);
        pwm *= -1;
    }
    else {
        set_direction_left(1);
    }
    s_pwm_left = pwm;
    analogWrite(LEFT_PWM, pwm);
}

void set_right_motor_pwm(int pwm) {
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);
    if (pwm < 0) {
        set_direction_right(-1);
        pwm *= -1;
    }
    else {
        set_direction_right(1);
    }
    s_pwm_right = pwm;
    analogWrite(RIGHT_PWM, pwm);
}

void stop_motors() {
    set_left_motor_pwm(0);
    set_right_motor_pwm(0);
}

float position_controller() {
    s_err_fwd += forward.increment() - robot_fwd_increment();
    float diff = s_err_fwd - old_err_fwd;
    old_err_fwd = s_err_fwd;
    float output = s_err_fwd * KP_FWD + diff * KD_FWD;
    return output;
}

float angle_controller(float steering_adjustment) {
    s_err_rot += rotation.increment() - robot_rot_increment();
    if (g_steering_enabled) {
        s_err_rot += steering_adjustment;
    }
    float diff = s_err_rot - old_err_rot;
    old_err_rot = s_err_rot;
    float output = s_err_rot * KP_ROT + diff * KD_ROT;
    return output;
}

void update_motor_controllers(float steering_adjustment) {
    float pos_output = position_controller();
    float rot_output = angle_controller(steering_adjustment);

    float left_output = 0;
    float right_output = 0;

    left_output += pos_output;
    right_output += pos_output;

    left_output -= rot_output;
    right_output += rot_output;

    float v_fwd = forward.speed();
    float v_rot = rotation.speed();

    float v_left = v_fwd - (PI / 180.0) * MOUSE_RADIUS * v_rot;
    float v_right = v_fwd + (PI / 180.0) * MOUSE_RADIUS * v_rot;
    left_output += SPEED_FF * v_left;
    right_output += SPEED_FF * v_right;
    var1 = left_output;
    var2 = right_output;
    int left_pwm = (int)left_output;
    int right_pwm = (int)right_output;
    if (l_motors_enabled) {
        set_left_motor_pwm(left_pwm);
        set_right_motor_pwm(right_pwm);
    }
}