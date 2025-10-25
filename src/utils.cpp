#include "utils.h"

void check_speed() {
    int pwm = 0;
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

void report_speed(int left_speed, int right_speed, uint32_t time_millis) {
    motor_left.set_speed(left_speed);
    motor_right.set_speed(right_speed);
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
        Serial.print(left_speed);
        Serial.print(" ");
        Serial.println(right_speed);
    }
}

void check_pwm_control() {
    enable_motors();
    motor_left.reset_motor();
    motor_right.reset_motor();
    while (!Serial.available()) {
        delay(100);
    }
    while (Serial.available()) {
        Serial.read();
    }
    // report_speed(-800, -800, 3000);
    // report_speed(-700, -700, 3000);
    // report_speed(-600, -600, 3000);
    // report_speed(-500, -500, 3000);
    // report_speed(-400, -400, 3000);
    // report_speed(-300, -300, 3000);
    // report_speed(-200, -200, 3000);
    // report_speed(-100, -100, 3000);
    // report_speed(100, 100, 3000);
    // report_speed(200, 200, 3000);
    // report_speed(300, 300, 3000);
    // report_speed(400, 400, 3000);
    // report_speed(500, 500, 3000);
    // report_speed(600, 600, 3000);
    // report_speed(700, 700, 3000);
    // report_speed(800, 800, 3000);
    report_speed(300, 300, 1000);
    report_speed(300, 121, 2000);
    report_speed(300, 300, 1000);
    report_speed(121, 300, 2000);
    report_speed(300, 300, 1000);
    report_speed(0, 0, 1000);
    // report_speed(200, -200, 2000);
    // report_speed(0, 0, 1000);
    // report_speed(-200, 200, 2000);
    
    Serial.println("0");
}
