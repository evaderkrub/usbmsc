#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "usb_hcd.h"
#include "usb_core.h"

int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    printf("usbmsc bench: stage 3 — enumeration\n");
    hcd_init();

    for (;;) {
        while (hcd_port_speed() != HCD_SPEED_FULL) sleep_ms(10);
        printf("port: full-speed device, resetting\n");
        sleep_ms(100);
        hcd_bus_reset();
        usb_device_t dev;
        hcd_result_t r = core_enumerate(1, &dev);
        if (r != HCD_OK) {
            printf("enumeration failed: %d\n", (int)r);
        } else {
            printf("enumerated addr=%u vid=%04x pid=%04x ep0_mps=%u\n",
                   dev.addr, dev.vid, dev.pid, dev.ep0_mps);
            if (dev.cfg.is_msc)
                printf("MSC: itf=%u bulk_in=%02x bulk_out=%02x mps=%u/%u\n",
                       dev.cfg.msc_itf, dev.cfg.bulk_in, dev.cfg.bulk_out,
                       dev.cfg.bulk_in_mps, dev.cfg.bulk_out_mps);
            if (dev.cfg.is_hub)
                printf("HUB: int_ep=%02x mps=%u interval=%u\n",
                       dev.cfg.hub_int_ep, dev.cfg.hub_int_mps, dev.cfg.hub_int_interval);
        }
        while (hcd_port_speed() != HCD_SPEED_NONE) sleep_ms(10);
        printf("port: disconnected\n");
    }
}
