#include "systick.h"



void init_systick() {
    bitClear(TCCR3A, WGM30);
    bitClear(TCCR3A, WGM31);
    bitSet(TCCR3B, WGM32);
    // set divisor to 1024 => 125kHz
    bitSet(TCCR3B, CS32);
    bitClear(TCCR3B, CS31);
    bitSet(TCCR3B, CS30);
    OCR3A = 31; // (16000000/1024/500)-1 = 31,  500Hz
    bitSet(TIMSK3, OCIE3A);
}


ISR(TIMER3_COMPA_vect, ISR_NOBLOCK) {
    update_encoders();
    read_sensors();
    forward.update();
    rotation.update();
    g_steering_adjustment = calculate_steering_adjustment();
    update_motor_controllers(g_steering_adjustment);
}