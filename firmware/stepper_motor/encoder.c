#include "encoder.h"
#include <hardware/gpio.h>

void as5600_init(i2c_inst_t *i2c, uint baud, uint gpio_data, uint gpio_clock){
    i2c_init(i2c, baud);
    
    gpio_set_function(gpio_data, GPIO_FUNC_I2C);
    gpio_set_function(gpio_clock, GPIO_FUNC_I2C);

    gpio_pull_up(gpio_data);
    gpio_pull_up(gpio_clock);
}