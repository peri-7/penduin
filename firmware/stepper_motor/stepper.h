#ifndef STEPPER_H_   // guard
#define STEPPER_H_

uint32_t speed_to_word(float v);

void set_speed(float v, PIO pio, uint sm);


#endif 