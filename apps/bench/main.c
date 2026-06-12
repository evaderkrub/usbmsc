#include <stdio.h>
#include "pico/stdlib.h"
#include "usb_msc.h"

int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    printf("usbmsc bench: stage 4 — drive bring-up\n");
    usb_msc_init();

    bool was_ready = false;
    for (;;) {
        usb_msc_task();
        if (usb_msc_ready() && !was_ready) {
            printf("drive ready: [%s]\n", usb_msc_drive_name());
            printf("capacity: %lu blocks x %lu bytes = %llu MiB\n",
                   (unsigned long)usb_msc_block_count(),
                   (unsigned long)usb_msc_block_size(),
                   (unsigned long long)usb_msc_block_count() *
                       usb_msc_block_size() / (1024 * 1024));
        }
        if (!usb_msc_ready() && was_ready) printf("drive removed\n");
        was_ready = usb_msc_ready();
        sleep_ms(10);
    }
}
