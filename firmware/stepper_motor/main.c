// The control loop goes here (FirmPlan.md, step 6). The motor tests are in motor_test.c.
#include <pico/stdlib.h>
#include <hardware/pio.h>
#include "stepper.h"


int main()
{
    stdio_init_all();
 
    PIO pio; uint sm0, sm1;
    init_move_pio(&pio, &sm0);
    init_read_pio(&pio, &sm1);

    while (true) {
        sleep_ms(1000);
    }
}
