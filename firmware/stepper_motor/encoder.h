#ifndef ENCODER_H_   // guard
#define ENCODER_H_

#include <stdint.h>
#include <hardware/i2c.h>

void as5600_init(i2c_inst_t *i2c, uint baud, uint gpio_data, uint gpio_clock);

#endif