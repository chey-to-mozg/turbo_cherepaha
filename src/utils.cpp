#include "utils.h"

void check_speed() {
    int pwm = 40;
    while (!Serial.available()) {
        delay(100);
    }
    while (Serial.available()) {
        Serial.read();
    }
    motor_left.set_pwm(pwm);
    motor_right.set_pwm(pwm);
    uint32_t start_time = millis();
    int counter = 0;
    delay(100);
    while(pwm < 250) {
        if(counter >= 20) {
            pwm += 10;
            motor_left.set_pwm(pwm);
            motor_right.set_pwm(pwm);
            counter = 0;
            delay(1000);
        }
        update_encoders();
        uint32_t cur_time = millis();
        float time_delta = (float)(cur_time - start_time) / 1000; // sec
        start_time = cur_time;
        float increment_left = get_increment_left();
        float increment_right = get_increment_right();
        float left_speed = increment_left / time_delta;
        float right_speed = increment_right / time_delta;
        Serial.print(pwm);
        Serial.print(" ");
        Serial.print(left_speed);
        Serial.print(" ");
        Serial.println(right_speed);
        counter++;
        delay(10);
    }
    Serial.println("0");
}

void report_speed(int target_speed, uint32_t time_millis) {
    motor_left.set_speed(target_speed);
    motor_right.set_speed(target_speed);
    uint32_t start_time = millis();
    while(millis() - start_time < time_millis) {
        update_motor_controllers();
        Serial.print(motor_left.get_pwm());
        Serial.print(" ");
        Serial.print(motor_right.get_pwm());
        Serial.print(" ");
        Serial.print(motor_left.get_speed());
        Serial.print(" ");
        Serial.print(motor_right.get_speed());
        Serial.print(" ");
        Serial.println(target_speed);
        delay(10);
    }
}

void check_pwm_control() {
    motor_left.reset_motor();
    motor_right.reset_motor();
    while (!Serial.available()) {
        delay(100);
    }
    while (Serial.available()) {
        Serial.read();
    }
    report_speed(400, 5000);
    report_speed(200, 5000);
    report_speed(0, 1000);
    
    Serial.println("0");
}
