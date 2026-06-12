#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "usb_hcd.h"

static void hexdump(const uint8_t *p, int n) {
    for (int i = 0; i < n; i++) printf("%02x ", p[i]);
    printf("\n");
}

int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    printf("usbmsc bench: stage 2 — device descriptor\n");
    hcd_init();

    for (;;) {
        while (hcd_port_speed() != HCD_SPEED_FULL) sleep_ms(10);
        printf("port: full-speed device, resetting\n");
        sleep_ms(100);                       // attach debounce
        hcd_bus_reset();

        // GET_DESCRIPTOR(device), first 8 bytes, address 0
        const uint8_t setup[8] = {0x80, 6, 0, 1, 0, 0, 8, 0};
        uint8_t desc[8];
        uint16_t len = sizeof desc;
        hcd_result_t r = hcd_control_xfer(0, setup, desc, &len);
        if (r == HCD_OK) {
            printf("device descriptor head (%u bytes): ", len);
            hexdump(desc, len);
            printf("bLength=%u bDescriptorType=%u bcdUSB=%x.%02x bMaxPacketSize0=%u\n",
                   desc[0], desc[1], desc[3], desc[2], desc[7]);
        } else {
            printf("control transfer failed: %d\n", (int)r);
        }
        while (hcd_port_speed() != HCD_SPEED_NONE) sleep_ms(10);
        printf("port: disconnected\n");
    }
}
