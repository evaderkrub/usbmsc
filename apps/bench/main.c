#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "usb_msc.h"

int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    printf("usbmsc bench: stage 5 — integrity\n");
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

            // Integrity check on the last 64 sectors (scratch area —
            // WARNING: overwrites drive contents there)
            static uint8_t wr[64 * 512], rd[64 * 512];
            uint32_t scratch = usb_msc_block_count() - 64;
            for (uint32_t i = 0; i < sizeof wr; i++)
                wr[i] = (uint8_t)(i * 7 + (i >> 8));
            msc_result_t mr = usb_msc_write(scratch, 64, wr);
            printf("write 64 sectors @%lu: %d\n", (unsigned long)scratch, (int)mr);
            mr = usb_msc_read(scratch, 64, rd);
            printf("read back: %d, compare: %s\n", (int)mr,
                   memcmp(wr, rd, sizeof wr) == 0 ? "MATCH" : "MISMATCH");
        }
        if (!usb_msc_ready() && was_ready) printf("drive removed\n");
        was_ready = usb_msc_ready();
        sleep_ms(10);
    }
}
