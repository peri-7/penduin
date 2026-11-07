#ifndef ENCODER_H_   // guard
#define ENCODER_H_
#define ANGLE_REG 0x0C
#define STATUS_REG 0x0B
#define AGC_REG 0x1A

#include <stdint.h>
#include <hardware/i2c.h>

void as5600_init(i2c_inst_t *i2c, uint baud, uint gpio_data, uint gpio_clock);

int read_reg(i2c_inst_t *i2c, uint8_t *reg, int len, uint8_t *buf);

int read_raw_angle(i2c_inst_t *i2c, uint16_t *angle);

float raw2deg(uint16_t angle);
float raw2rad(uint16_t angle);

#endif