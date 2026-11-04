#include "stepper.h"
#define PI 3.14159265f

uint32_t speed_to_word(float v){
    uint32_t res = 1, N;
    if (v < 0) { res = 0; v = -v; } // dir bit (dir=1: counter-clockwise)
    if (v < VMIN) return 0;
    if (v > VMAX) v = VMAX;
    float cc = 2*PI*PIO_FREQ / (STEPS_REV * v); // period in SM cycles
    N = (uint32_t)lroundf(cc) - PIO_OVERHEAD;
    res |= N << 1;
    return res;
}


void set_speed(float v, PIO pio, uint sm){
    uint32_t N = speed_to_word(v);
    pio_sm_clear_fifos(pio, sm); 
    pio_sm_put(pio, sm, N);
}


float steps2rad(PIO pio, uint sm){
    float steps = (float)pio_sm_pull(pio, sm);
    // 1 step = 1.8/16 deg = 2pi/1600 rad
    float rad = 2*PI/1600
    return rad;
}