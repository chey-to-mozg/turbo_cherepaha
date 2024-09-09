#ifndef SYSTICK_H
#define SYSTICK_H

#include <Arduino.h>
#include "encoders.h"
#include "sensors.h"
#include "motors.h"
#include "profile.h"

extern volatile long counter;

void init_systick();

#endif