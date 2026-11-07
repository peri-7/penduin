#include "encoder.h"
#include <hardware/gpio.h>
#define ADDR 0x36
#define AS5600_TIMEOUT_US 1000
#define PI 3.14159265f

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