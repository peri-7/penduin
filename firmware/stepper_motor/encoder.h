#ifndef ENCODER_H_   // guard
#define ENCODER_H_
#define CONF_REG 0x07       // 0x07 holds CONF bits 13:8 (WD, FTH, SF), 0x08 bits 7:0
#define STATUS_REG 0x0B
#define ANGLE_REG 0x0C      // RAW ANGLE, 0x0C/0x0D
#define AGC_REG 0x1A
#define MAGNITUDE_REG 0x1B  // 0x1B/0x1C

#define STATUS_MH 0x08      // magnet too strong
#define STATUS_ML 0x10      // magnet too weak
#define STATUS_MD 0x20      // magnet detected

#define ALPHA_SIGN 1        // set to -1 if the sign check against the sim says so

#define SDA_PIN 16
#define CLK_PIN 17


#include <stdint.h>
#include <hardware/i2c.h>

// Pendulum angle the way env.py sees it. Upright = 0, hanging = +-pi.
typedef struct {
    uint16_t raw_up;    // raw reading with the pendulum upright
    uint16_t raw_prev;  // last raw reading, for alpha_dot
    float alpha;        // rad, [-pi, pi)
    float alpha_dot;    // rad/s
} alpha_t;

void as5600_init(i2c_inst_t *i2c, uint baud, uint gpio_data, uint gpio_clock);

int read_reg(i2c_inst_t *i2c, uint8_t *reg, int len, uint8_t *buf);

// Writes one byte. Returns 2 (register + value) on success. Refuses the BURN register.
int write_reg(i2c_inst_t *i2c, uint8_t reg, uint8_t value);

int set_SF(i2c_inst_t *i2c, int factor);

int read_raw_angle(i2c_inst_t *i2c, uint16_t *angle);

float raw2deg(uint16_t angle);
float raw2rad(uint16_t angle);

// Shortest signed difference between two raw angles, in [-2048, 2048)
int wrap_counts(int d);

// Average of raw readings that may straddle the 4095 -> 0 wrap
uint16_t raw_average(const uint16_t *raw, int n);

// Only math, can't fail. alpha_start sets alpha from raw with alpha_dot = 0: call it on the first
// read and after a failed one, so alpha_dot never spans more than one dt. Set raw_up first.
void alpha_start(alpha_t *a, uint16_t raw);
void alpha_update(alpha_t *a, uint16_t raw, float dt);

#endif
