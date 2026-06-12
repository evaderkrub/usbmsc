#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "usb_msc.h"

int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    printf("usbmsc bench: stage 6 — benchmark\n");
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

            // Throughput: sequential read, 8 MiB in 32 KiB chunks
            const uint32_t TOTAL_SECTORS = 16384;          // 8 MiB
            const uint32_t CHUNK = 64;
            static uint8_t tbuf[64 * 512];
            uint64_t t0 = time_us_64();
            bool ok = true;
            for (uint32_t s = 0; s < TOTAL_SECTORS && ok; s += CHUNK)
                ok = usb_msc_read(s, CHUNK, tbuf) == MSC_OK;
            uint64_t dt_us = time_us_64() - t0;
            if (ok) {
                uint64_t bytes = (uint64_t)TOTAL_SECTORS * 512;
                // KiB/s with integer math; frames elapsed = dt_us / 1000
                uint32_t kibps = (uint32_t)(bytes * 1000000ull / dt_us / 1024);
                uint32_t frames = (uint32_t)(dt_us / 1000);
                uint32_t pkts = (uint32_t)(bytes / 64);
                printf("seq read: 8 MiB in %lu ms -> %lu KiB/s (%lu.%02lu MiB/s)\n",
                       (unsigned long)(dt_us / 1000), (unsigned long)kibps,
                       (unsigned long)(kibps / 1024),
                       (unsigned long)((kibps % 1024) * 100 / 1024));
                printf("packets/frame: %lu.%lu (19 = saturated)\n",
                       (unsigned long)(pkts / frames),
                       (unsigned long)((pkts % frames) * 10 / frames));
            } else {
                printf("throughput read FAILED\n");
            }
        }
        if (!usb_msc_ready() && was_ready) printf("drive removed\n");
        was_ready = usb_msc_ready();
        sleep_ms(10);
    }
}
