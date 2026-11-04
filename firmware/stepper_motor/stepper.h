#ifndef STEPPER_H_   // guard
#define STEPPER_H_

#include <stdint.h>
#include <hardware/pio.h>

#define PIO_FREQ 10000000.0f // SM cycles per second: 150 MHz / clkdiv 15

uint32_t speed_to_word(float v);

void set_speed(float v, PIO pio, uint sm);

int32_t read_steps(PIO pio, uint sm);

float steps2rad(PIO pio, uint sm);

#endif 