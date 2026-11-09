// control loop
#include <stdio.h>
#include <pico/stdlib.h>
#include <math.h>
#include <pico/stdio_usb.h>
#include <pico/multicore.h>
#include <pico/util/queue.h>
#include <hardware/watchdog.h>

#include "stepper.h"
#include "encoder.h"
#include "excitation.h"

#define DT_US 10000
#define DT (DT_US * 1e-6f)
#define SF 2
#define FAILED_CNTS 3
#define CAL_N 100
#define CAL_SPREAD 4

static PIO pio;
static uint sm0, sm1, baud = 100*1000;
static alpha_t a;
static float th, th_dot, th_ddot, action;
static int32_t steps0;
static const char *reason = "";

static int cnt = 0, restart = 0, ticks = 0;
static uint16_t raw = 0;
static uint16_t late = 0, dropped = 0;  // running totals over a RUN, sent with every record

static queue_t write_q, read_q;

typedef enum STATES {
    WAIT, CALIBRATE, RUN, FAULT 
} state_t;

typedef struct {
    uint32_t time_us;
    int16_t raw;
    int32_t steps;
    float v;
    float action;
    float alpha;
    float alpha_dot;
    uint16_t loop_us;
    uint16_t late;
    uint16_t dropped;
} record_t;


static void core1_main(void){
    record_t r;
    while(true){
        while(queue_try_remove(&write_q, &r)) 
            printf("%lu,%d,%ld,%.3f,%.3f,%.4f,%.3f,%u,%u,%u\n", 
            r.time_us, r.raw, r.steps, r.v, r.action, r.alpha, r.alpha_dot, r.loop_us, r.late, r.dropped);
        int c = getchar_timeout_us(0);
        if(c != PICO_ERROR_TIMEOUT) {
            char k = (char)c;
            queue_try_add(&read_q, &k);
        }
        sleep_ms(1);
    }
}

static float controller(float a, float a_dot, float th, float th_dot){
    return 0.01; // placeholder for tests
}



int calibrate(){
    // take 100 measurments of raw_down
    uint16_t cal[CAL_N]; 
    absolute_time_t next = get_absolute_time();
    for(int k=0; k<CAL_N; k++){
        next = delayed_by_us(next, DT_US);
        if(read_raw_angle(i2c0, &cal[k]) != 2){
            reason = "read failures";
            printf("# state: calibrate -> read failed during calibration\n");
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
        printf("# state: calibrate -> pendulum not still: spread %d counts. Let it settle and run again\n", hi - lo);
        return 1;
    }
    // compute raw_up and start alpha_t a
    uint16_t raw_down = raw_average(cal, CAL_N);
    a.raw_up = (raw_down + 2048) % 4096;

    printf("# calibration ended -> raw_down: %d / raw_up: %d\n", raw_down, a.raw_up);
    return 0;

}

int main()
{
    // init all hardware
    record_t record; char key;
    // stdio_usb_init() must run on core 0 (it owns the default alarm pool), or it returns false and USB never enumerates
    stdio_init_all();
    queue_init(&write_q, sizeof(record_t), 64); 
    queue_init(&read_q, sizeof(char), 64);
    multicore_launch_core1(core1_main);
    while(!stdio_usb_connected()) sleep_ms(100);
    if(watchdog_enable_caused_reboot()) printf("# rebooted by watchdog\n");
    as5600_init(i2c0, baud, SDA_PIN, CLK_PIN);
    init_move_pio(&pio, &sm0);
    init_read_pio(&pio, &sm1);
    state_t st = WAIT;


    absolute_time_t next = get_absolute_time();
    while (true) {
// ------------------------------------------------------------------------        
        if (st == WAIT){
            watchdog_disable();
            // while WAIT, move the arm at theta=0
            absolute_time_t prompt = get_absolute_time();
            while(!queue_try_remove(&read_q, &key)){
                if(time_reached(prompt)){
                    printf("# state: wait -> center the arm, then press any key to calibrate and run\n");
                    prompt = make_timeout_time_ms(2000);
                }
                sleep_ms(10);
            }
            st = CALIBRATE;

        }
// ------------------------------------------------------------------------
        if (st == CALIBRATE){
            int err = set_SF(i2c0, SF);
            if (err != 0) { reason = "SF failure"; st = FAULT; continue; }
            int c = calibrate();
            if (c == 0) {
                while(queue_try_remove(&read_q, &key)); // empty queue
                cnt = 0; th_dot = 0; th_ddot = 0; ticks = 0;
                steps0 = read_steps(pio, sm1);
                restart = 1; late = 0; dropped = 0;
                printf("time_us,raw,steps,v,action,alpha,alpha_dot,loop_us,late,dropped\n");
                watchdog_enable(50, true);
                next = get_absolute_time();
                st = RUN;
            }
            else if (c == 1) st = WAIT;
            else st = FAULT; 
        }
// ------------------------------------------------------------------------        
        if (st == RUN){
            next = delayed_by_us(next, DT_US);
            watchdog_update();
            uint32_t t0 = time_us_32();     // start of this tick's work, for time_us and loop_us
            // stop RUN if key is pressed
            if(queue_try_remove(&read_q, &key)) {
                if(key == 'w') while(1) ; // watchdog test
                set_speed(0, pio, sm0);
                st = WAIT;
                continue;
            }
            // read angle 'alpha' and 'theta'
            bool ok = read_raw_angle(i2c0, &raw) == 2;
            if(!ok){
                restart = 1;
                // printf("state: run -> read failed during run\n");
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
            if(fabsf(th) > TH_LIMIT){
                reason = "theta passed the limit";
                st = FAULT;
                continue;                
            }
            
            // control u = th_ddot -> command th_dot
            action = controller(a.alpha, a.alpha_dot, th, th_dot);
            // action = ticks < EXCITATION_N ? EXCITATION[ticks++] : 0;
            if(action > 1) action = 1;
            if(action < -1) action = -1;
            th_ddot = action*MAX_ACCEL;
            th_dot = th_dot + th_ddot*DT;
            if(th_dot > VMAX) th_dot = VMAX;
            if(th_dot < -VMAX) th_dot = -VMAX;
            set_speed(th_dot, pio, sm0);

            // telemetry: one record per tick for core 1 to print. queue_try_add never blocks: if
            // the queue is full (nobody reading the USB), the record is dropped and counted.
            record.time_us = t0;
            record.raw = ok ? (int16_t)raw : -1;        // raw keeps the last good value for alpha
            record.steps = read_steps(pio, sm1) - steps0;
            record.v = th_dot;
            record.action = action;                    // 6c: the controller's action, in [-1, 1]
            record.alpha = a.alpha;
            record.alpha_dot = a.alpha_dot;
            if(time_reached(next)) late++;              // this tick's work used up the whole 10 ms
            record.late = late;
            record.dropped = dropped;
            record.loop_us = (uint16_t)(time_us_32() - t0);
            if(!queue_try_add(&write_q, &record)) dropped++;

            sleep_until(next);
        }
        
// ------------------------------------------------------------------------       
        if (st == FAULT){
            watchdog_disable();
            set_speed(0, pio, sm0);

            printf("# state: fault -> reason (%s) -> press 'r' to restart\n", reason);
            queue_remove_blocking(&read_q, &key);
            if (key == 'r') st = WAIT;
            
        }
        

    }
}
