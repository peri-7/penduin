// Free-swing logger for system identification (experiment A).
// No motor code: the driver holds the arm by itself (EN is tied to GND).
// Each tick prints "time_us,raw", or "time_us,-1" for a failed read.
// Each time a serial monitor connects it first prints the CONF check (a line starting with #) and
// the column names, so every log file starts with them.
#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/stdio_usb.h>
#include "encoder.h"

#define DT_US 10000     // 100 Hz, same as the control loop
#define SF 2            // slow filter, the setting chosen for the control firmware


static void print_header(void){
    int err = set_SF(i2c0, SF);
    uint8_t reg = CONF_REG;
    uint8_t conf[2];
    if(read_reg(i2c0, &reg, 2, conf) == 2)
        printf("# set_SF(%d) returned %d, conf 0x%02x%02x\n", SF, err, conf[0], conf[1]);
    else
        printf("# set_SF(%d) returned %d, conf read failed\n", SF, err);
    printf("time_us,raw\n");
}


int main(){
    stdio_init_all();
    as5600_init(i2c0, 100*1000, SDA_PIN, CLK_PIN);

    bool was_connected = false;
    absolute_time_t next = get_absolute_time();
    while(true){
        next = delayed_by_us(next, DT_US);

        bool connected = stdio_usb_connected();
        if(connected && !was_connected) print_header();
        was_connected = connected;

        uint64_t t = time_us_64();
        uint16_t raw;
        if(read_raw_angle(i2c0, &raw) == 2) printf("%llu,%d\n", t, raw);
        else printf("%llu,-1\n", t);

        sleep_until(next);
    }
}
