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

void print_gyro() {
    #if DEBUG_LOGGING == 1
        Serial.print(" gyro delta ");
        Serial.print(gyro_delta);
        Serial.print(" gyro error ");
        Serial.print(gyro_error);
        Serial.print(" gyro angle ");
        Serial.println(g_gyro_angle);
    #else
      delay(2);
    #endif
}   


void print_sensors() {
    #if DEBUG_LOGGING == 1
        int left_raw;
        int front_raw_left;
        int front_raw_right;
        int right_raw;
        int left;
        int front;
        int right;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            left = g_left_sensor;
            front = g_front_sensor;
            right = g_right_sensor;
            left_raw = g_left_sensor_raw;
            front_raw_left = g_front_sensor_raw_left;
            // front_raw_right = g_front_sensor_raw_right;
            right_raw = g_right_sensor_raw;
        }

        Serial.print("<");
        Serial.print(" ");
        Serial.print(left_raw);
        Serial.print(" ");
        Serial.print(left);
        Serial.print(" ");

        Serial.print(">");
        Serial.print(" ");
        Serial.print(right_raw);
        Serial.print(" ");
        Serial.print(right);
        Serial.print(" ");

        Serial.print("^");
        Serial.print(" ");
        Serial.print(front_raw_left);
        // Serial.print(" ");
        // Serial.print(front_raw_right);
        Serial.print(" ");
        Serial.print(front);

        Serial.print(" |");
        Serial.print(" ");
        Serial.print(g_cross_track_error);
        Serial.print(" ");
        Serial.println(g_steering_adjustment);  
    #else
      delay(2);
    #endif
}

void print_motors() {
    #if DEBUG_LOGGING == 1
        Serial.print(" < dist ");
        Serial.print(s_distance_left);
        Serial.print(" enc ");
        Serial.print(s_total_left);

        Serial.print(" > dist ");
        Serial.print(s_distance_right);
        Serial.print(" enc ");
        Serial.print(s_total_right);

        Serial.print(" ^ dist ");
        Serial.print(robot_position());
        Serial.print(" angle ");
        Serial.print(robot_angle());

        Serial.print(" | pos err ");
        Serial.print(s_err_fwd);
        Serial.print(" | rot err ");
        Serial.println(s_err_rot);
    #else
      delay(2);
    #endif
}

void print_debug() {
    #if DEBUG_LOGGING == 1
        float debug1;
        float debug2;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            debug1 = var1;
            debug2 = var2;
        }
        Serial.print(debug1);
        Serial.print(" ");
        Serial.println(debug2);
    #else
      delay(2);
    #endif
}

void print_profile() {
    #if DEBUG_LOGGING == 1
        Serial.print("Enc pos ");
        Serial.print(robot_position());
        Serial.print(" Enc ang ");
        Serial.print(robot_angle());
        Serial.print(" f pos ");
        Serial.print(forward.position());
        Serial.print(" f speed ");
        Serial.print(forward.speed());
        Serial.print(" r pos ");
        Serial.print(rotation.position());
        Serial.print(" r speed ");
        Serial.print(rotation.speed());
        Serial.print(" < pwm ");
        Serial.print(s_pwm_left);
        Serial.print(" > pwm ");
        Serial.print(s_pwm_right);
        Serial.println();
    #else
      delay(2);
    #endif
}

void report_bluetooth() {
    print_sensors();
}

void report_serial() {
    print_sensors();
    print_motors();
    print_profile();
    print_debug();
}