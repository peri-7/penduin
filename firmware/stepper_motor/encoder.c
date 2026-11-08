#include "encoder.h"
#include <math.h>
#include <hardware/gpio.h>
#define ADDR 0x36
#define AS5600_TIMEOUT_US 1000
#define BURN_REG 0xFF       // permanent OTP writes: never touched from this driver
#define PI 3.14159265f
#define RAD_PER_COUNT (2 * PI / 4096)

void as5600_init(i2c_inst_t *i2c, uint baud, uint gpio_data, uint gpio_clock){
    i2c_init(i2c, baud);

    gpio_set_function(gpio_data, GPIO_FUNC_I2C);
    gpio_set_function(gpio_clock, GPIO_FUNC_I2C);

    gpio_pull_up(gpio_data);
    gpio_pull_up(gpio_clock);
}

int read_reg(i2c_inst_t *i2c, uint8_t *reg, int len, uint8_t *buf){

    int ret = i2c_write_timeout_us(i2c, ADDR, reg, 1, true, AS5600_TIMEOUT_US); //send register address
    if(ret!=1) return ret;
    ret = i2c_read_timeout_us(i2c, ADDR, buf, len, false, AS5600_TIMEOUT_US); //read register
    return ret;
}

int write_reg(i2c_inst_t *i2c, uint8_t reg, uint8_t value){
    if(reg == BURN_REG) return PICO_ERROR_GENERIC;
    uint8_t buf[2] = {reg, value};
    return i2c_write_timeout_us(i2c, ADDR, buf, 2, false, AS5600_TIMEOUT_US);
}


int read_raw_angle(i2c_inst_t *i2c, uint16_t *angle){
    uint8_t reg = ANGLE_REG;
    uint8_t buf[2];
    int len = 2;

    int err_val = read_reg(i2c, &reg, len, buf);
    if(err_val!=len) return err_val;
    *angle = (buf[0]<<8) | (buf[1]);
    *angle &= 0x0FFF;
    return err_val;
}

float raw2deg(uint16_t angle){
    return 1.0f*angle*360/4096;
}
float raw2rad(uint16_t angle){
    return 1.0f*angle*2*PI/4096;
}

int wrap_counts(int d){
    if(d >= 2048) d -= 4096;
    if(d < -2048) d += 4096;
    return d;
}

int set_SF(i2c_inst_t *i2c, int factor){
    // choose filter
    uint8_t sf = 0;
    switch (factor){
        case 16: sf = 0; break;
        case 8: sf = 1; break;
        case 4: sf = 2; break;
        case 2: sf = 3; break;
        default: return -1; // -1: no valid choice
    }
    // read-modify-write CONF
    uint8_t reg = CONF_REG;
    uint8_t buf[2];
    int len = 2;
    int err = read_reg(i2c, &reg, len, buf);
    if (err != 2) return -2; // -2: couldnt read
    uint8_t prev = buf[0];
    uint8_t next = (prev&(0b11111100)) | sf;
    err = write_reg(i2c, reg, next);
    if (err != 2) return -3; // -3: couldnt write
    // check CONF
    err = read_reg(i2c, &reg, len, buf);
    if (err != 2) return -2;
    prev = buf[0];
    if (prev != next) return -4; // -4: didnt write correctly
    
    return 0;  
}

// Averages the signed differences from the first reading, not the raw values: near the wrap,
// 4095 and 1 would otherwise average to about 2048, half a turn off.
uint16_t raw_average(const uint16_t *raw, int n){
    int sum = 0;
    for(int i=0; i<n; i++) sum += wrap_counts((int)raw[i] - (int)raw[0]);
    int mean = (int)lroundf((float)sum / n);
    return (uint16_t)((raw[0] + mean + 4096) % 4096);
}

void alpha_start(alpha_t *a, uint16_t raw){
    a->alpha = ALPHA_SIGN * wrap_counts((int)raw - (int)a->raw_up) * RAD_PER_COUNT;
    a->alpha_dot = 0;
    a->raw_prev = raw;
}

void alpha_update(alpha_t *a, uint16_t raw, float dt){
    a->alpha = ALPHA_SIGN * wrap_counts((int)raw - (int)a->raw_up) * RAD_PER_COUNT;
    a->alpha_dot = ALPHA_SIGN * wrap_counts((int)raw - (int)a->raw_prev) * RAD_PER_COUNT / dt;
    a->raw_prev = raw;
}
