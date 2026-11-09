#include "stepper.h"
#include <math.h>
#include <hardware/clocks.h>
#include "move.pio.h"
#include "read.pio.h"


#define STEP_PIN 5
#define DIR_PIN 4
#define MS1 9
#define MS2 8


#define MICROSTEP 16
#define STEPS_REV (200*MICROSTEP)
#define PIO_OVERHEAD 9      // cycles per period outside the wait loop: period = N + 9


void set_MS(){
    gpio_init(MS1);
    gpio_init(MS2);
    gpio_set_dir(MS1, 1);
    gpio_set_dir(MS2, 1);
    gpio_put(MS1, true);
    gpio_put(MS2, true);
}


void init_move_pio(PIO *pio, uint *state_machine_id){
    set_MS();

    *pio = pio0;
    *state_machine_id = pio_claim_unused_sm(pio0, true);
    uint offset = pio_add_program(*pio, &move_program);
    float factor = clock_get_hz(clk_sys) / PIO_FREQ;
    move_program_init(*pio, *state_machine_id, offset, DIR_PIN, STEP_PIN, factor);
}

void init_read_pio(PIO *pio, uint *state_machine_id){
    *pio = pio0;
    *state_machine_id = pio_claim_unused_sm(pio0, true);
    uint offset = pio_add_program(*pio, &read_program);
    read_program_init(*pio, *state_machine_id, offset, DIR_PIN, STEP_PIN, 1.0f);
}


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

float steps2rad(PIO pio, uint sm, int32_t steps0){
    float steps = (float)(read_steps(pio, sm) - steps0);
    // 1 step = 1.8/16 deg = 2pi/3200 rad
    float rad = 2*PI*steps/STEPS_REV;
    return rad;
}