#ifndef REPORTER_H
#define REPORTER_H

#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "encoders.h"
#include "motors.h"
#include "mouse.h"

void init_serial();
void print_sensors();
void print_motors();
void report_bluetooth();
void report_serial();
#endif