#include <stdio.h>
#include "pico/stdlib.h"

int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    printf("usbmsc bench: skeleton OK\n");
    for (;;) tight_loop_contents();
}
