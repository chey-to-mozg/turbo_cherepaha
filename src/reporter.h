#ifndef REPORTER_H
#define REPORTER_H

#include <Arduino.h>
// #include <SoftwareSerial.h>
#include "config.h"
#include "sensors.h"
#include "encoders.h"
#include "motors.h"
#include "mouse.h"

// extern SoftwareSerial BT;

void init_serial();
// void init_bluetooth();
void print_gyro();
void print_sensors();
void print_motors();
void report_bluetooth();
void report_serial();
// void print_debug();
// void print_profile();
#endif