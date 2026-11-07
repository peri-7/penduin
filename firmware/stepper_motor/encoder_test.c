#include <stdio.h>
#include <stdlib.h>
#include <pico/stdlib.h>
#include "encoder.h"
#define SDA_PIN 16
#define CLK_PIN 17
#define DT_US 10000     // 100 Hz, same as the control loop
#define M 60000         // ticks in the test: 10 minutes
#define PRINT_EVERY 10  // ticks between status lines

// A torn read has the high byte of one sample and the low byte of the next, so it is off by 256
// counts for one tick. That makes the changes before and after it differ by about 512 counts.
// Real motion can't do that: 256 counts/tick^2 would be about 3900 rad/s^2.
#define TEAR_LIMIT 256

// shortest signed difference between two raw angles, in [-2048, 2048)
static int wrap_counts(int d){
    if(d >= 2048) d -= 4096;
    if(d < -2048) d += 4096;
    return d;
}

int main(){
    stdio_init_all();

    as5600_init(i2c0, 100*1000, SDA_PIN, CLK_PIN);

    uint8_t byte;
    int timeouts = 0;

    uint8_t reg_agc = AGC_REG;
    uint8_t reg_status = STATUS_REG;
    uint8_t agc_val, status_val;
    uint16_t angle;

    int x;

    sleep_ms(3000); // wait to open serial monitor

    // start-up checks, once
    for (int j=0x08; j<0x78 ;j++){
        int ret = i2c_read_timeout_us(i2c0, j, &byte, 1, false, 1000);
        if(ret==1) printf("0x%02x reached\n", j);
        if(ret==PICO_ERROR_TIMEOUT) timeouts++;
    }
    printf("scan done with %d timeouts!\n", timeouts);

    x = read_reg(i2c0, &reg_agc, 1, &agc_val);
    if(x != 1) printf("agc error %d\n", x);
    else printf("agc value %d\n",  agc_val);

    x = read_reg(i2c0, &reg_status, 1, &status_val);
    if(x != 1) printf("status error %d\n", x);
    else printf("magnet too strong:%d\nmagnet too weak:%d\nmagnet detected:%d\n", (status_val&0x08)!=0, (status_val&0x10)!=0, (status_val&0x20)!=0);

    // 100 Hz loop
    int err_angle = 0;      // failed reads
    int late = 0;           // ticks that used up their whole 10 ms
    int crossings = 0;      // ticks where the high byte changed: the only chance for a torn read
    int tears = 0;          // one-tick spikes that look like a torn read
    uint32_t max_read_us = 0;

    int have = 0;           // valid samples in a row, up to 2 (r0 older, r1 newer)
    uint16_t r0 = 0, r1 = 0;

    absolute_time_t next = get_absolute_time();
    for(int i=0; i<M; i++){
        next = delayed_by_us(next, DT_US);

        uint32_t t0 = time_us_32();
        x = read_raw_angle(i2c0, &angle);
        uint32_t read_us = time_us_32() - t0;
        if(read_us > max_read_us) max_read_us = read_us;

        if(x != 2){
            err_angle++;
            have = 0;   // the next change would span more than one tick
            printf("angle error %d\n", x);
        }
        else{
            if(have >= 1 && (angle>>8) != (r1>>8)) crossings++;

            if(have == 2){
                int d_before = wrap_counts((int)r1 - (int)r0);
                int d_after = wrap_counts((int)angle - (int)r1);
                if(abs(d_before - d_after) > TEAR_LIMIT){
                    tears++;
                    printf("TEAR? raw %d %d %d\n", r0, r1, angle);
                }
            }

            r0 = r1;
            r1 = angle;
            if(have < 2) have++;

            if(i % PRINT_EVERY == 0)
                printf("angle: %f / bits: %d / read %lu us (max %lu) / err %d / cross %d / tears %d / late %d\n",
                    raw2deg(angle), angle, (unsigned long)read_us, (unsigned long)max_read_us,
                    err_angle, crossings, tears, late);
        }

        if(time_reached(next)) late++;
        sleep_until(next);
    }

    while (true) {
        sleep_ms(2000);

        printf("%d/%d angle errors\n", err_angle, M);
        printf("max read %lu us\n", (unsigned long)max_read_us);
        printf("%d crossings, %d tears\n", crossings, tears);
        printf("%d late ticks\n", late);
    }
}
