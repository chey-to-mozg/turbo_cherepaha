/*
 * File: profile.h
 * Project: mazerunner
 * -----
 * MIT License
 *
 * Copyright (c) 2021 Peter Harrison
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef PROFILE_H
#define PROFILE_H

#include "encoders.h"
#include <Arduino.h>
#include <util/atomic.h>
//***************************************************************************//
class Profile;

extern Profile forward;
extern Profile rotation;

enum ProfileState : uint8_t {
  CS_IDLE = 0,
  CS_ACCELERATING = 1,
  CS_BRAKING = 2,
  CS_FINISHED = 3,
};

class Profile {
  public:
    void reset();
    bool is_finished();
    void start(float distance, float top_speed, float final_speed, float acceleration);
    void stop();
    void set_state(ProfileState state);
    float position();
    float speed();
    float increment();
    float acceleration();
    void set_target_speed(float speed);
    // normally only used to alter position for forward error correction
    void adjust_position(float adjustment);
    void set_position(float position);
    // update is called from within systick and shoul dbe safe from interrupts
    void update();

  private:
    volatile uint8_t m_state = CS_IDLE;
    volatile float m_speed = 0;
    volatile float m_position = 0;
    int8_t m_sign = 1;
    float m_acceleration = 0;
    float m_one_over_acc = 1;
    float m_target_speed = 0;
    float m_final_speed = 0;
    float m_final_position = 0;

    void finish();
    float get_braking_distance();
};

#endif