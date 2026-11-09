#ifndef STEPPER_H_   // guard
#define STEPPER_H_

#include <stdint.h>
#include <pico/stdlib.h>
#include <hardware/pio.h>

#define PI 3.14159265f

#define PIO_FREQ 10000000.0f // SM cycles per second: 150 MHz / clkdiv 15
#define VMAX 20.0f
#define VMIN 0.1 // low VMIN -> high speed resolution, low time resolution
#define MAX_ACCEL 200
#define TH_LIMIT = PI/2

void set_MS();

void init_move_pio(PIO *pio, uint *state_machine_id);

void init_read_pio(PIO *pio, uint *state_machine_id);

uint32_t speed_to_word(float v);

void set_speed(float v, PIO pio, uint sm);

int32_t read_steps(PIO pio, uint sm);

float steps2rad(PIO pio, uint sm, int32_t steps0);

#endif 