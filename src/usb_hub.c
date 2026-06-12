#include "usb_hub.h"

hcd_result_t usb_hub_attach(const usb_device_t *hub) { (void)hub; return HCD_ERR_TIMEOUT; }
hcd_result_t usb_hub_wait_drive(usb_device_t *drive, uint32_t timeout_ms) {
    (void)drive; (void)timeout_ms; return HCD_ERR_TIMEOUT;
}
bool usb_hub_drive_present(void) { return false; }
void usb_hub_detach(void) {}
