#include <stdio.h>
#include <math.h>
#include <pico/stdlib.h>
#include <hardware/pio.h>
#include "stepper.pio.h"

#define PI 3.14159265f

#define STEP_PIN 14
#define DIR_PIN 15
#define MICROSTEP 16
#define STEPS_REV (200*MICROSTEP)
#define VMAX 20.0f
#define VMIN 0.01f      // below this the motor stops (dead band); also absorbs float round-off
#define DT_US 10000
#define DT (DT_US * 1e-6f)
#define PIO_FREQ 1000000.0f // SM cycles per second: 150 MHz / clkdiv 150
#define PIO_OVERHEAD 8      // cycles per period outside the wait loop: period = N + 8

// Test profile (in control ticks of 10 ms)
#define V_TEST (2*PI)                 // 1 rev/s
#define A_TEST (V_TEST / 0.5f)        // reach V_TEST in 0.5 s
#define T_RAMP 50
#define T_HOLD 200
#define T_PAUSE 50                    // standstill before reversing

void init_stepper_pio(PIO *pio, uint *state_machine_id){
    *pio = pio0;
    *state_machine_id = pio_claim_unused_sm(pio0, true);
    uint offset = pio_add_program(*pio, &stepper_program);
    stepper_program_init(*pio, *state_machine_id, offset, DIR_PIN, STEP_PIN, 150.0f); // TODO: clock_get_hz(clk_sys) / PIO_FREQ
}

uint32_t speed_to_word(float v){
    uint32_t res = 0, N;
    if (v < 0) { res = 1; v = -v; } // dir bit
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

int main()
{
    stdio_init_all();
    sleep_ms(2000); // open serial monitor

    PIO pio; uint sm;
    init_stepper_pio(&pio, &sm);

    // Limits test. Each stage = (acceleration, top speed), run REPS times back and forth:
    // ramp to +v, hold, ramp to 0, ramp to -v, hold, ramp to 0. Net travel per stage is 0,
    // so the shaft mark must be at the same spot in every pause. A shift = lost steps.
    typedef struct { float a; float v; } stage_t;   // rad/s^2, rad/s
    const stage_t stages[] = {
        { 50,  5}, { 50, 10}, { 50, 15}, { 50, 20},  // speed sweep, gentle acceleration
        {100, 10}, {200, 10}, {300, 10}, {400, 10},  // acceleration sweep, mid speed
        {200, 20},                                   // the sim's limits together
    };
    const int n_stages = sizeof(stages) / sizeof(stages[0]);
    const int REPS = 3;           // back-and-forth moves per stage
    const int HOLD_TICKS = 50;    // 0.5 s at top speed
    const int PAUSE_TICKS = 300;  // 3 s standstill after each stage: check the mark

    float v = 0, s = 1;           // s = direction of the current move
    int stage = 0, half = 0;      // half = one-direction moves done in this stage
    int phase = 0, cnt = 0;       // phase: 0 ramp up, 1 hold, 2 ramp down, 3 pause
    uint32_t tick = 0;

    printf("stage %d: a=%.0f v=%.1f\n", stage, stages[stage].a, stages[stage].v);
    absolute_time_t next = get_absolute_time();
    while (true) {
        next = delayed_by_us(next, DT_US);

        if (stage >= n_stages) {  // all stages done: stay stopped
            set_speed(0, pio, sm);
            sleep_until(next);
            continue;
        }

        float a = stages[stage].a, v_top = stages[stage].v;
        switch (phase) {
        case 0: // ramp up (v changes once per tick, like the real controller)
            v += s * a * DT;
            if (fabsf(v) >= v_top) { v = s * v_top; phase = 1; cnt = 0; }
            break;
        case 1: // hold top speed
            if (++cnt >= HOLD_TICKS) phase = 2;
            break;
        case 2: // ramp down to 0, then reverse
            v -= s * a * DT;
            if (s * v <= 0) {
                v = 0; s = -s;
                if (++half >= 2 * REPS) {
                    half = 0; phase = 3; cnt = 0;
                    printf("stage %d done -> check the mark\n", stage);
                } else {
                    phase = 0;
                }
            }
            break;
        case 3: // pause between stages
            if (++cnt >= PAUSE_TICKS) {
                ++stage; phase = 0;
                if (stage < n_stages)
                    printf("stage %d: a=%.0f v=%.1f\n", stage, stages[stage].a, stages[stage].v);
                else
                    printf("all stages done\n");
            }
            break;
        }

        set_speed(v, pio, sm);

        if (tick % 50 == 0) {
            uint32_t w = speed_to_word(v);
            printf("  t=%.2f v=%.2f N=%lu dir=%lu\n", tick * DT, v, (unsigned long)(w >> 1), (unsigned long)(w & 1));
        }
        ++tick;
        sleep_until(next);
    }
}
