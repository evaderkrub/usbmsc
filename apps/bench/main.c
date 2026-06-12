#include <stdio.h>
#include "pico/stdlib.h"
#include "usb_hcd.h"

int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    printf("usbmsc bench: stage 1 — connection detect\n");
    hcd_init();

    hcd_speed_t last = HCD_SPEED_NONE;
    for (;;) {
        hcd_speed_t s = hcd_port_speed();
        if (s != last) {
            const char *names[] = {"disconnected", "low-speed", "full-speed"};
            printf("port: %s\n", names[s <= 2 ? s : 0]);
            last = s;
        }
        sleep_ms(10);
    }
}
