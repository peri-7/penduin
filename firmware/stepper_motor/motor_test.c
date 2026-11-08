// All the motor tests in one program: pick one from the serial monitor.
// c and l make many full turns, so the encoder cable must be off. s stays within about 40 deg.
#include <stdio.h>
#include <math.h>
#include <pico/stdlib.h>
#include <hardware/pio.h>
#include "stepper.h"


#define DT_US 10000
#define DT (DT_US * 1e-6f)

#define PI 3.14159265f
#define STEPS_PER_RAD (3200 / (2 * PI))
#define V_TOP VMAX          // the period rounds to 982 cycles: 10183 steps/s

// One move is about 22404 steps = 7.001 turns, so the mark is within about 0.5 deg of its start
#define RAMP 50             // ticks to reach V_TOP (40 rad/s^2)
#define HOLD 170            // further ticks at V_TOP
#define HOLD_CHECK 100      // ticks between c0 and c1: expect 10183 steps
#define PAUSE 100           // ticks stopped after each move: check the mark
#define CYCLES 10           // forward + backward moves

#define SINE_TICKS 100      // low-speed test: 1 s per sine cycle
#define SINE_CYCLES 10      // cycles at each amplitude


static PIO pio;
static uint sm0, sm1;
static absolute_time_t next;
static int32_t start;       // count when the current test began: the count isn't reset between tests
static float expected;      // steps from integrating the commanded speed on the CPU
static float max_diff;      // largest |count - expected|


// Steps since the current test began
static long test_steps(void){
    return (long)(read_steps(pio, sm1) - start);
}

// Drops keys sent earlier, such as during a test
static void drop_keys(void){
    while(getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
}

static int wait_key(void){
    int key;
    do {
        key = getchar_timeout_us(1000000);
    } while(key == PICO_ERROR_TIMEOUT);
    return key;
}

// One 10 ms tick at speed v. Returns the count read at the start of the tick.
static long tick(float v){
    long count = test_steps();

    float diff = fabsf(count - expected);
    if(diff > max_diff) max_diff = diff;

    set_speed(v, pio, sm0);
    expected += v * DT * STEPS_PER_RAD;

    next = delayed_by_us(next, DT_US);
    sleep_until(next);
    return count;
}

// Ramp up, hold, ramp down and pause, in direction s (+1 or -1). Prints only in the pause,
// so the USB output can't disturb the moving part.
static void counter_move(float s){
    long c0 = 0, c1 = 0;

    for(int k=1; k<=RAMP; k++) tick(s * V_TOP * k / RAMP);

    for(int i=0; i<HOLD; i++){
        long count = tick(s * V_TOP);
        if(i == 0) c0 = count;              // V_TOP has been running for a full tick by now
        if(i == HOLD_CHECK) c1 = count;
    }

    for(int k=RAMP-1; k>=0; k--) tick(s * V_TOP * k / RAMP);

    for(int i=0; i<PAUSE; i++) tick(0);

    printf("dir %+.0f: count = %ld, expected = %.1f, max diff = %.2f, hold change = %ld\n",
        s, test_steps(), expected, max_diff, c1 - c0);
}

// Step counter at top speed. Pass: count alternates 22404 / 0, hold change 10183 +-1,
// the offset doesn't grow and the mark stays put.
static void test_counter(void){
    start = read_steps(pio, sm1);
    expected = 0;
    max_diff = 0;

    next = get_absolute_time();
    for(int c=1; c<=CYCLES; c++){
        printf("cycle %d\n", c);
        counter_move(1);
        counter_move(-1);
    }
    printf("counter test done: count = %ld, max diff = %.2f\n", test_steps(), max_diff);
}

// Low-speed test: the speed follows a sine, so it keeps crossing zero and spends time below
// 0.2 rad/s, where a step period is longer than the 10 ms tick. A new speed waits for the current
// period to end, so count - expected gets larger here. Pass: the cycle counts stay in a narrow
// range with no trend, the final count is within about 6 of 0 and the mark agrees with it.
static void test_low_speed(void){
    const float amps[] = {0.1f, 0.2f, 0.5f, 2.0f};  // rad/s
    const int n_amps = sizeof(amps) / sizeof(amps[0]);

    start = read_steps(pio, sm1);
    expected = 0;

    next = get_absolute_time();
    for(int a=0; a<n_amps; a++){
        max_diff = 0;
        for(int c=1; c<=SINE_CYCLES; c++){
            for(int i=0; i<SINE_TICKS; i++) tick(amps[a] * sinf(2 * PI * i / SINE_TICKS));
            printf("A = %.1f, cycle %d: count = %ld\n", amps[a], c, test_steps());
        }

        for(int i=0; i<PAUSE; i++) tick(0);

        printf("A = %.1f done: count = %ld, expected = %.1f, max diff = %.2f\n",
            amps[a], test_steps(), expected, max_diff);
    }
    printf("low-speed test done\n");
}

// Limits test. Each stage = (acceleration, top speed), run REPS times back and forth:
// ramp to +v, hold, ramp to 0, ramp to -v, hold, ramp to 0. Net travel per stage is 0, so the
// shaft mark must only move by what the printed count says. Any more than that = lost steps.
// It waits for a key after each stage, so a shift can be pinned to one stage.
static void test_limits(void){
    typedef struct { float a; float v; } stage_t;   // rad/s^2, rad/s
    const stage_t stages[] = {
        { 50,  5}, { 50, 10}, { 50, 15}, { 50, 20},  // speed sweep, gentle acceleration
        {100, 10}, {200, 10}, {300, 10}, {400, 10},  // acceleration sweep, mid speed
        {200, 20},                                   // the sim's limits together
    };
    const int n_stages = sizeof(stages) / sizeof(stages[0]);
    const int REPS = 3;           // back-and-forth moves per stage
    const int HOLD_TICKS = 50;    // 0.5 s at top speed
    const int PAUSE_TICKS = 100;  // 1 s to settle after each stage, then wait for a key

    float v = 0, s = 1;           // s = direction of the current move
    int stage = 0, half = 0;      // half = one-direction moves done in this stage
    int phase = 0, cnt = 0;       // phase: 0 ramp up, 1 hold, 2 ramp down, 3 pause
    uint32_t t = 0;

    start = read_steps(pio, sm1);

    printf("stage %d: a=%.0f v=%.1f\n", stage, stages[stage].a, stages[stage].v);
    next = get_absolute_time();
    while (stage < n_stages) {
        next = delayed_by_us(next, DT_US);

        float a = stages[stage].a, v_top = stages[stage].v;
        switch (phase) {
        case 0: // ramp up (v changes once per tick, like the real controller)
            v += s * a * DT;
            if (fabsf(v) >= v_top) { v = s * v_top; phase = 1; cnt = 0; }
            break;
        case 1: // hold top speed
            if (++cnt >= HOLD_TICKS) phase = 2;
            break;
        case 2: // ramp down to 0, then reverse
            v -= s * a * DT;
            if (s * v <= 0) {
                v = 0; s = -s;
                if (++half >= 2 * REPS) {
                    half = 0; phase = 3; cnt = 0;
                } else {
                    phase = 0;
                }
            }
            break;
        case 3: // pause between stages
            if (++cnt >= PAUSE_TICKS) {
                printf("stage %d done: count = %ld. Check the mark, then press a key (q: back to menu)\n",
                    stage, test_steps());
                drop_keys();
                if (wait_key() == 'q') return;
                next = get_absolute_time();
                ++stage; phase = 0;
                if (stage < n_stages)
                    printf("stage %d: a=%.0f v=%.1f\n", stage, stages[stage].a, stages[stage].v);
                else
                    printf("limits test done\n");
            }
            break;
        }

        set_speed(v, pio, sm0);

        if (t % 50 == 0) {
            uint32_t w = speed_to_word(v);
            printf("  t=%.2f v=%.2f N=%lu dir=%lu\n", t * DT, v, (unsigned long)(w >> 1), (unsigned long)(w & 1));
        }
        ++t;
        sleep_until(next);
    }
}

// Shows the choices every 2 s until one key arrives, so a late serial monitor still sees them.
static int ask_test(void){
    drop_keys();

    while(true){
        printf("steps = %ld | c: counter test, l: limits test, s: low-speed test\n", (long)read_steps(pio, sm1));
        int key = getchar_timeout_us(2000000);
        if(key != PICO_ERROR_TIMEOUT) return key;
    }
}


int main()
{
    stdio_init_all();

    init_move_pio(&pio, &sm0);
    init_read_pio(&pio, &sm1);

    while(true){
        int key = ask_test();
        if(key == 'c') test_counter();
        else if(key == 'l') test_limits();
        else if(key == 's') test_low_speed();
        else printf("unknown choice\n");    }
}
