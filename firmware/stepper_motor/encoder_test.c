// All the AS5600 tests in one program: pick one from the serial monitor.
// Only m and n move the motor: +-45 deg around where the arm is, so center the arm first.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <pico/stdlib.h>
#include <hardware/pio.h>
#include "encoder.h"
#include "stepper.h"


#define DT_US 10000     // 100 Hz, same as the control loop
#define DT (DT_US * 1e-6f)
#define PI 3.14159265f

#define M 60000         // ticks in the a and m tests: 10 minutes, or until a key
#define PRINT_EVERY 10  // ticks between status lines

// A torn read has the high byte of one sample and the low byte of the next, so it is off by 256
// counts for one tick. That makes the changes before and after it differ by about 512 counts.
// Real motion can't do that: 256 counts/tick^2 would be about 3900 rad/s^2.
#define TEAR_LIMIT 256

// m and n: arm motion x = +-M_X around the start, with a period in ticks.
// m, 2 Hz: peak 9.9 rad/s and about 124 rad/s^2, inside the sim's limits.
// n, 0.5 Hz: peak 2.5 rad/s and about 7.8 rad/s^2, to tell violent motion from motor noise.
#define M_PERIOD 50
#define N_PERIOD 200
#define M_X (PI / 4)

#define CAL_N 100       // d: calibration samples (1 s)
#define CAL_SPREAD 4    // d: largest spread in counts that still counts as "still"
#define D_PRINT_EVERY 2 // d: 50 lines/s, enough to see alpha_dot over a swing

#define F_N 500         // f: samples per filter setting (5 s)


static PIO pio;
static uint sm0, sm1;


static void drop_keys(void){
    while(getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
}

static bool key_pressed(void){
    return getchar_timeout_us(0) != PICO_ERROR_TIMEOUT;
}

static bool read8(uint8_t reg, uint8_t *v){
    return read_reg(i2c0, &reg, 1, v) == 1;
}

static bool read16(uint8_t reg, uint16_t *v){
    uint8_t b[2];
    if(read_reg(i2c0, &reg, 2, b) != 2) return false;
    *v = (b[0] << 8) | b[1];
    return true;
}


// i: I2C scan, then the magnet flags, AGC, MAGNITUDE and CONF once
static void test_info(void){
    uint8_t byte, v;
    uint16_t w;
    int found = 0, timeouts = 0;

    for(int addr=0x08; addr<0x78; addr++){
        int ret = i2c_read_timeout_us(i2c0, addr, &byte, 1, false, 1000);
        if(ret == 1){ printf("0x%02x answered\n", addr); found++; }
        if(ret == PICO_ERROR_TIMEOUT) timeouts++;
    }
    printf("scan done: %d found, %d timeouts (expect only 0x36, 0 timeouts)\n", found, timeouts);

    if(read8(STATUS_REG, &v))
        printf("magnet detected %d, too weak %d, too strong %d (expect 1 0 0)\n",
            (v & STATUS_MD) != 0, (v & STATUS_ML) != 0, (v & STATUS_MH) != 0);
    else printf("status read failed\n");

    if(read8(AGC_REG, &v)) printf("agc %d (0-128 at 3.3 V, centre about 64)\n", v);
    else printf("agc read failed\n");

    if(read16(MAGNITUDE_REG, &w)) printf("magnitude %d\n", w & 0x0FFF);
    else printf("magnitude read failed\n");

    if(read16(CONF_REG, &w))
        printf("conf 0x%04x: SF %d, FTH %d, HYST %d, PM %d (default: all 0)\n",
            w, (w >> 8) & 3, (w >> 10) & 7, (w >> 2) & 3, w & 3);
    else printf("conf read failed\n");
}


// a (period = 0): 100 Hz angle loop: errors, read time, late ticks, torn reads, range and wrap.
// m and n (period in ticks): the same while the arm moves +-45 deg. A key stops the motion at the
// end of the current cycle, and the arm goes back to its start.
static void angle_loop(int period){
    bool motor = period > 0;
    float amp = motor ? M_X * 2 * PI / (period * DT) : 0;    // peak speed, rad/s
    int err_angle = 0;      // failed reads
    int late = 0;           // ticks that used up their whole 10 ms
    int crossings = 0;      // ticks where the high byte changed: the only chance for a torn read
    int tears = 0;          // one-tick spikes that look like a torn read
    int wraps = 0;          // 4095 <-> 0 crossings
    uint32_t max_read_us = 0;
    uint16_t sectors = 0;   // bit k set: a reading with high byte k was seen (16 sectors)
    uint16_t raw_min = 4095, raw_max = 0;

    int have = 0;           // valid samples in a row, up to 2 (r0 older, r1 newer)
    uint16_t r0 = 0, r1 = 0, angle = 0;

    int phase = motor ? 0 : 1;  // motor: 0 out to -M_X, 1 cycles, 2 back to the start, 3 done
    int j = 0;
    bool stop = false;
    int32_t steps0 = read_steps(pio, sm1);

    absolute_time_t next = get_absolute_time();
    int i;
    for(i=0; ; i++){
        next = delayed_by_us(next, DT_US);

        uint32_t t0 = time_us_32();
        int x = read_raw_angle(i2c0, &angle);
        uint32_t read_us = time_us_32() - t0;
        if(read_us > max_read_us) max_read_us = read_us;

        if(x != 2){
            err_angle++;
            have = 0;   // the next change would span more than one tick
            printf("angle error %d\n", x);
        }
        else{
            if(angle < raw_min) raw_min = angle;
            if(angle > raw_max) raw_max = angle;
            sectors |= 1u << (angle >> 8);

            if(have >= 1){
                if((angle >> 8) != (r1 >> 8)) crossings++;
                if(abs((int)angle - (int)r1) > 2048) wraps++;
            }

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

        // motor command after the read, in the same order as the control loop will use
        if(motor){
            float v = 0;
            if(phase == 0){         // half cycle at half amplitude: out to -M_X
                v = -0.5f * amp * sinf(2 * PI * j / period);
                if(++j >= period / 2){ phase = 1; j = 0; }
            }
            else if(phase == 1){    // whole cycles between -M_X and +M_X
                v = amp * sinf(2 * PI * j / period);
                if(++j >= period){ j = 0; if(stop) phase = 2; }
            }
            else if(phase == 2){    // half cycle at half amplitude: back to the start
                v = 0.5f * amp * sinf(2 * PI * j / period);
                if(++j >= period / 2) phase = 3;
            }
            set_speed(v, pio, sm0);
        }

        if(!stop && (i + 1 >= M || key_pressed())) stop = true;

        if(time_reached(next)) late++;
        sleep_until(next);
        if(stop && (!motor || phase == 3)) break;
    }

    if(motor){
        set_speed(0, pio, sm0);
        sleep_ms(500);  // let the last, slow period finish before reading the count
    }

    printf("%s done after %d ticks: %d errors, max read %lu us, %d late ticks\n",
        motor ? "motor error test" : "angle test", i + 1, err_angle, (unsigned long)max_read_us, late);
    printf("  raw range %d..%d, %d/16 sectors seen, %d wraps, %d crossings, %d tears\n",
        raw_min, raw_max, __builtin_popcount(sectors), wraps, crossings, tears);
    if(motor)
        printf("  arm count at the end: %ld (expect about 0)\n", (long)(read_steps(pio, sm1) - steps0));
}


// i, after test_info: magnet strength while turning the pendulum by hand, until a key
static void test_magnet(void){
    printf("turn the pendulum slowly through a full turn, then push it along its axis. Any key stops\n");
    int errors = 0, n = 0, no_md = 0, ml = 0, mh = 0;
    int agc_min = 255, agc_max = 0, mag_min = 4095, mag_max = 0;
    uint16_t sectors = 0;

    absolute_time_t next = get_absolute_time();
    for(int i=0; !key_pressed(); i++){
        next = delayed_by_us(next, DT_US);

        uint16_t raw, mag;
        uint8_t status, agc;
        if(read_raw_angle(i2c0, &raw) != 2 || !read8(STATUS_REG, &status) ||
           !read8(AGC_REG, &agc) || !read16(MAGNITUDE_REG, &mag)){
            errors++;
        }
        else{
            n++;
            mag &= 0x0FFF;
            sectors |= 1u << (raw >> 8);
            if(!(status & STATUS_MD)) no_md++;
            if(status & STATUS_ML) ml++;
            if(status & STATUS_MH) mh++;
            if(agc < agc_min) agc_min = agc;
            if(agc > agc_max) agc_max = agc;
            if(mag < mag_min) mag_min = mag;
            if(mag > mag_max) mag_max = mag;

            if(i % PRINT_EVERY == 0)
                printf("raw %4d / agc %3d / magnitude %4d / MD %d ML %d MH %d\n", raw, agc, mag,
                    (status & STATUS_MD) != 0, (status & STATUS_ML) != 0, (status & STATUS_MH) != 0);
        }
        sleep_until(next);
    }

    printf("magnet test done: %d samples, %d errors, %d/16 sectors seen\n", n, errors, __builtin_popcount(sectors));
    printf("  agc %d..%d, magnitude %d..%d\n", agc_min, agc_max, mag_min, mag_max);
    printf("  samples with no magnet %d, too weak %d, too strong %d (expect 0 0 0)\n", no_md, ml, mh);
}


static int fails;

static void check(const char *name, float got, float want){
    bool ok = fabsf(got - want) < 1e-3f;
    if(!ok) fails++;
    printf("  %s %s: got %.4f, want %.4f\n", ok ? "pass" : "FAIL", name, got, want);
}

// d: math checks, calibration, then live alpha and alpha_dot until a key
static void test_alpha(void){
    alpha_t a;

    // 1. math checks with the test values from FirmPlan.md (no sensor needed)
    printf("math checks:\n");
    fails = 0;
    a.raw_up = 1000;
    alpha_start(&a, 1000);
    check("raw 1000 -> alpha", a.alpha, 0);
    alpha_start(&a, 3048);
    check("raw 3048 -> alpha", a.alpha, ALPHA_SIGN * -PI);
    alpha_start(&a, 4095);
    alpha_update(&a, 1, DT);
    check("4095 then 1 -> alpha_dot", a.alpha_dot, ALPHA_SIGN * 0.3068f);
    alpha_start(&a, 1);
    alpha_update(&a, 4095, DT);
    check("1 then 4095 -> alpha_dot", a.alpha_dot, ALPHA_SIGN * -0.3068f);
    uint16_t samples[2] = {4095, 1};
    uint16_t down = raw_average(samples, 2);
    check("samples 4095, 1 -> raw_down", down, 0);
    check("samples 4095, 1 -> raw_up", (down + 2048) % 4096, 2048);
    printf("%d failed\n", fails);

    // 2. calibration: the pendulum hangs still, upright is half a turn away
    uint16_t cal[CAL_N];
    absolute_time_t next = get_absolute_time();
    for(int k=0; k<CAL_N; k++){
        next = delayed_by_us(next, DT_US);
        if(read_raw_angle(i2c0, &cal[k]) != 2){
            printf("read failed during calibration\n");
            return;
        }
        sleep_until(next);
    }
    int lo = 0, hi = 0;
    for(int k=0; k<CAL_N; k++){
        int d = wrap_counts((int)cal[k] - (int)cal[0]);
        if(d < lo) lo = d;
        if(d > hi) hi = d;
    }
    if(hi - lo > CAL_SPREAD){
        printf("pendulum not still: spread %d counts. Let it settle and run d again\n", hi - lo);
        return;
    }
    uint16_t raw_down = raw_average(cal, CAL_N);
    a.raw_up = (raw_down + 2048) % 4096;
    printf("raw_down = %d, raw_up = %d (spread %d counts)\n", raw_down, a.raw_up, hi - lo);

    // 3. live: swing the pendulum by hand
    int errors = 0, late = 0;
    bool restart = true;
    next = get_absolute_time();
    for(int i=0; !key_pressed(); i++){
        next = delayed_by_us(next, DT_US);

        uint16_t raw;
        if(read_raw_angle(i2c0, &raw) != 2){
            errors++;
            restart = true;     // hold alpha; alpha_dot restarts at 0 after the next good read
        }
        else{
            if(restart){ alpha_start(&a, raw); restart = false; }
            else alpha_update(&a, raw, DT);

            if(i % D_PRINT_EVERY == 0)
                printf("alpha %7.3f rad (%7.1f deg) / alpha_dot %7.2f rad/s / sin %6.3f / cos %6.3f / raw %4d\n",
                    a.alpha, a.alpha * 180 / PI, a.alpha_dot, sinf(a.alpha), cosf(a.alpha), raw);
        }
        if(time_reached(next)) late++;
        sleep_until(next);
    }
    printf("alpha test done: %d errors, %d late ticks\n", errors, late);
}


// f: noise at rest for each slow-filter setting (CONF SF). Written at runtime only and restored
// at the end. The chip keeps CONF until it loses power, even if the Pico resets.
static void test_filter(void){
    const char *names[4] = {"16x (default)", "8x", "4x", "2x"};
    uint8_t hi0, back;

    if(!read8(CONF_REG, &hi0)){
        printf("conf read failed\n");
        return;
    }

    for(int sf=0; sf<4; sf++){
        uint8_t hi = (hi0 & ~0x03) | sf;    // SF is CONF bits 9:8 = bits 1:0 of register 0x07
        if(write_reg(i2c0, CONF_REG, hi) != 2 || !read8(CONF_REG, &back) || back != hi){
            printf("conf write failed\n");
            break;
        }
        sleep_ms(100);

        int errors = 0, n = 0, lo = 0, hi_d = 0;
        float sum = 0, sum2 = 0;
        uint16_t first = 0, raw;
        absolute_time_t next = get_absolute_time();
        for(int k=0; k<F_N; k++){
            next = delayed_by_us(next, DT_US);
            if(read_raw_angle(i2c0, &raw) != 2) errors++;
            else{
                if(n == 0) first = raw;
                int d = wrap_counts((int)raw - (int)first);
                if(d < lo) lo = d;
                if(d > hi_d) hi_d = d;
                sum += d;
                sum2 += (float)d * d;
                n++;
            }
            sleep_until(next);
        }
        float mean = n ? sum / n : 0;
        float sd = n ? sqrtf(fmaxf(sum2 / n - mean * mean, 0)) : 0;
        printf("SF %-13s: std %.2f counts (%.3f deg), range %d counts, %d errors\n",
            names[sf], sd, sd * 360 / 4096, hi_d - lo, errors);
    }

    if(write_reg(i2c0, CONF_REG, hi0) == 2 && read8(CONF_REG, &back) && back == hi0)
        printf("conf restored\n");
    else
        printf("conf NOT restored: power-cycle the sensor\n");
}


// Shows the choices every 2 s until one key arrives, so a late serial monitor still sees them
static int ask_test(void){
    drop_keys();
    while(true){
        printf("i: info + magnet, a: angle + errors, m: errors with motor, n: same but gentle, d: alpha, f: filter\n");
        int key = getchar_timeout_us(2000000);
        if(key != PICO_ERROR_TIMEOUT){
            sleep_ms(50);
            drop_keys();    // a line ending sent with the key would stop the test at once
            return key;
        }
    }
}


int main(){   
    stdio_init_all();

    as5600_init(i2c0, 100*1000, SDA_PIN, CLK_PIN);
    init_move_pio(&pio, &sm0);
    init_read_pio(&pio, &sm1);

    while(true){
        int key = ask_test();
        if(key == 'i'){ test_info(); test_magnet(); }
        else if(key == 'a') angle_loop(0);
        else if(key == 'm') angle_loop(M_PERIOD);
        else if(key == 'n') angle_loop(N_PERIOD);
        else if(key == 'd') test_alpha();
        else if(key == 'f') test_filter();
        else printf("unknown choice\n");
    }
}
