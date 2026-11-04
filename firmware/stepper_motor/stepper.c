#include "stepper.h"
#include <math.h>

#define PI 3.14159265f

#define MICROSTEP 16
#define STEPS_REV (200*MICROSTEP)
#define VMAX 20.0f
#define VMIN 0.01f      // below this the motor stops (dead band); also absorbs float round-off 
// low VMIN -> high speed resolution / low time resolution 
#define PIO_OVERHEAD 9      // cycles per period outside the wait loop: period = N + 9


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

int32_t read_steps(PIO pio, uint sm){
    return (int32_t)pio->rxf_putget[sm][0];
}

float steps2rad(PIO pio, uint sm){
    float steps = (float)read_steps(pio, sm);
    // 1 step = 1.8/16 deg = 2pi/3200 rad
    float rad = 2*PI*steps/STEPS_REV;
    return rad;
}