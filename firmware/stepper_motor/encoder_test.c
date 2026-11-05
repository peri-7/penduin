#include <stdio.h>
#include <pico/stdlib.h> 
#include "encoder.h"
#define SDA_PIN 4
#define CLK_PIN 5

int main(){
    stdio_init_all();

    as5600_init(i2c0, 100*1000, SDA_PIN, CLK_PIN);

    uint8_t byte;
    int timeouts;

    while(true){
        sleep_ms(2000); // open serial monitor
        timeouts = 0;
        
        for (int i=0x08; i<0x78 ;i++){
            int ret = i2c_read_timeout_us(i2c0, i, &byte, 1, false, 1000);
            if(ret==1) printf("0x%02x reached\n", i);
            if(ret==PICO_ERROR_TIMEOUT) timeouts++;
        }
        printf("scan done with %d timeouts!\n", timeouts);
    }
}