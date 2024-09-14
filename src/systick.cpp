#include "systick.h"



void init_systick() {
    bitClear(TCCR2A, WGM20);
    bitClear(TCCR2A, WGM21);
    bitSet(TCCR2B, WGM22);
    // set divisor to 128 => 125kHz
    bitSet(TCCR2B, CS22);
    bitClear(TCCR2B, CS21);
    bitSet(TCCR2B, CS20);
    OCR2A = 249; // (16000000/128/500)-1 => 500Hz
    bitSet(TIMSK2, OCIE2A);
}


ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
    update_encoders();
    read_sensors();
    forward.update();
    rotation.update();
    g_steering_adjustment = calculate_steering_adjustment();
    update_motor_controllers(g_steering_adjustment);
}