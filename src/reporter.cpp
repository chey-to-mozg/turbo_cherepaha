#include "reporter.h"

// SoftwareSerial BT(BT_TX, BT_RX);

void init_serial() {
    if (!SERIAL_ENABLE) {
        return;
    }
    int attempt = 0;
    Serial.begin(115200);
    Serial.setTimeout(1);
    while (!Serial) {
        if (attempt++ > 10) {
            return;
        }
        delay(100);
    }
    Serial.println("Enabled");
}

// void init_bluetooth() {
//     if (!BLUETOOTH_ENABLE) {
//         return;
//     }
//     if (SERIAL_ENABLE) {
//         Serial.println("Connecting to bluetooth...");
//     }
//     BT.begin(115200);
//     BT.setTimeout(1);
//     while (!BT) {
//         delay(1000);
//         if (SERIAL_ENABLE) {
//             Serial.println("Connecting...");
//         }
//     }
//     if (SERIAL_ENABLE) {
//         Serial.println("Connected");
//     }
// } 


void print_sensors() {
    if (DEBUG_AVAILABLE) {
        Serial.print("<");
        Serial.print(" ");
        Serial.print(g_left_sensor_raw);
        Serial.print(" ");
        Serial.print(g_left_sensor);
        Serial.print(" ");

        Serial.print(">");
        Serial.print(" ");
        Serial.print(g_right_sensor_raw);
        Serial.print(" ");
        Serial.print(g_right_sensor);
        Serial.print(" ");

        Serial.print("^");
        Serial.print(" ");
        Serial.print(g_front_sensor_raw_left);
        // Serial.print(" ");
        // Serial.print(front_raw_right);
        Serial.print(" ");
        Serial.print(g_front_sensor);

        Serial.print(" [ ");
        Serial.print(g_left_button);
        Serial.print(" | ");
        Serial.print(g_right_button);
        Serial.print(" ] |");

        Serial.print(" ");
        Serial.println(g_gyro_angle);
    }
    else
      delay(2);
}

void print_motors() {
    if (DEBUG_AVAILABLE) {
        Serial.print(" < dist ");
        Serial.print(distance_left);
        Serial.print(" enc ");
        Serial.print(total_conut_left);
        Serial.print(" speed ");
        Serial.print(motor_left.get_speed());
        Serial.print(" pwm ");
        Serial.print(motor_left.get_pwm());

        Serial.print(" > dist ");
        Serial.print(distance_right);
        Serial.print(" enc ");
        Serial.print(total_count_right);
        Serial.print(" speed ");
        Serial.print(motor_right.get_speed());
        Serial.print(" pwm ");
        Serial.print(motor_right.get_pwm());

        Serial.print(" ^ dist ");
        Serial.print(get_robot_position());
        // Serial.print(" position ");
        // Serial.print(mouse.get_position());
        Serial.print(" enc angle ");
        Serial.print(get_robot_angle());
        Serial.print(" mouse angle ");
        Serial.print(mouse.get_angle());

        // Serial.print(" | pos err ");
        // Serial.print(s_err_fwd);
        // Serial.print(" | rot err ");
        // Serial.println(s_err_rot);
        Serial.println("");
    }
    else {
        delay(2);
    }
}

void report_bluetooth() {
    print_sensors();
}

void report_serial() {
    print_sensors();
    print_motors();
    // print_profile();
    // print_debug();
}