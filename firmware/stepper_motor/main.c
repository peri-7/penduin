// control loop
#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/stdio_usb.h>

#include "stepper.h"
#include "encoder.h"

#define DT_US 10000
#define DT (DT_US * 1e-6f)
#define SF 2
#define FAILED_CNTS 3
#define CAL_N 100
#define CAL_SPREAD 4

static PIO pio;
static uint sm0, sm1, baud = 100*1000;
static alpha_t a;
static float th, th_dot, th_ddot;
static int32_t steps0;
static const char *reason = "";

static int cnt = 0, ticks = 0, restart = 0;
static uint16_t raw = 0;

typedef enum STATES {
    WAIT, CALIBRATE, RUN, FAULT 
} state_t;


int calibrate(){
    // take 100 measurments of raw_down
    uint16_t cal[CAL_N]; 
    absolute_time_t next = get_absolute_time();
    for(int k=0; k<CAL_N; k++){
        next = delayed_by_us(next, DT_US);
        if(read_raw_angle(i2c0, &cal[k]) != 2){
            reason = "read failures";
            printf("state: calibrate -> read failed during calibration\n");
            return -1;
        }
        sleep_until(next);
    }
    // find the spread to check stillness
    int lo = 0, hi = 0;
    for(int k=0; k<CAL_N; k++){
        int d = wrap_counts((int)cal[k] - (int)cal[0]);
        if(d < lo) lo = d;
        if(d > hi) hi = d;
    }
    if(hi - lo > CAL_SPREAD){
        reason = "pend. not still";
        printf("state: calibrate -> pendulum not still: spread %d counts. Let it settle and run again\n", hi - lo);
        return 1;
    }
    // compute raw_up and start alpha_t a
    uint16_t raw_down = raw_average(cal, CAL_N);
    a.raw_up = (raw_down + 2048) % 4096;

    return 0;

}

int main()
{
    // init all hardware
    stdio_init_all();
    while(!stdio_usb_connected()) sleep_ms(100);
    as5600_init(i2c0, baud, SDA_PIN, CLK_PIN);
    init_move_pio(&pio, &sm0);
    init_read_pio(&pio, &sm1);
    state_t st = WAIT;


    absolute_time_t next = get_absolute_time();
    while (true) {

        
        if (st == WAIT){
            // while WAIT, move the arm at theta=0
            printf("state: wait -> center the arm, then press any key to calibrate and run\n");
            while(getchar_timeout_us(0) != PICO_ERROR_TIMEOUT); // clear left-over keys
            getchar();
            while(getchar_timeout_us(0) != PICO_ERROR_TIMEOUT); // clear left-over keys
            st = CALIBRATE;

        }
// ------------------------------------------------------------------------
        if (st == CALIBRATE){
            int err = set_SF(i2c0, SF);
            if (err != 0) { reason = "SF failure"; st = FAULT; continue; }
            int c = calibrate();
            if (c == 0) {
                cnt = 0; th_dot = 0; th_ddot = 0;
                steps0 = read_steps(pio, sm1);
                restart = 1; ticks = 0;
                next = get_absolute_time();
                st = RUN;
            }
            else if (c == 1) st = WAIT;
            else st = FAULT; 
        }
// ------------------------------------------------------------------------        

        if (st == RUN){
            next = delayed_by_us(next, DT_US);
            // stop RUN if key is pressed
            if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT){
                set_speed(0, pio, sm0);
                st = WAIT;
                continue;
            }
            // read angle 'alpha' and 'theta'
            if(read_raw_angle(i2c0, &raw) != 2){
                restart = 1;
                printf("state: run -> read failed during run\n");
                if(++cnt >= FAILED_CNTS){
                    reason = "read failures";
                    st = FAULT;
                    continue;
                }
            }
            else {
                cnt = 0; 
                if (restart == 1) {
                    alpha_start(&a, raw);
                    restart = 0;
                }
                else
                    alpha_update(&a, raw, DT);
            }
            th = steps2rad(pio, sm1, steps0);
            
            // control u = th_ddot -> command th_dot
            th_ddot = 0;
            th_dot = th_dot + th_ddot*DT;
            set_speed(th_dot, pio, sm0);
            
            // print to test 6a
            if(++ticks >= 10){
                printf("alpha: %f / alpha_dot: %f / theta: %f\n", a.alpha, a.alpha_dot, th);
                ticks = 0;
            }
            
            sleep_until(next);
        }
// ------------------------------------------------------------------------       
        if (st == FAULT){
            set_speed(0, pio, sm0);

            printf("state: fault -> reason (%s) -> press 'r' to restart\n", reason);
            int reset = getchar();
            while(getchar_timeout_us(0) != PICO_ERROR_TIMEOUT); // clear left-over keys
            if (reset == 'r') st = WAIT;
            
        }
        

    }
}
