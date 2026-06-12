// RP2350 USB host controller driver. Polled, no interrupts.
// Datasheet findings (Task 4 Step 1), from SDK 2.2.0 headers:
//   - BUFF_STATUS host mapping: regs/usb.h gives EP0_IN = bit 0, EP0_OUT =
//     bit 1, EPn_IN = bit 2n, EPn_OUT = bit 2n+1. In host mode bit 0 reports
//     EPX completion, and hardware-polled interrupt endpoint N (1..15) uses
//     the EPn_IN/OUT positions: IN completion on bit 2N (OUT on 2N+1) --
//     i.e. polled endpoint N -> bit 2*N, as expected. Corroborated by
//     TinyUSB hcd_rp2040.c ("EPX is bit 0 & 1, IEP1 IN is bit 2, IEP1 OUT
//     is bit 3, IEP2 IN is bit 4, ... etc", bit = 1 << (i * 2 + j)).
//   - DPRAM 16-bit write support: structs/usb_dpram.h says of the DPSRAM:
//     "4K of DPSRAM at beginning. Note this supports 8, 16, and 32 bit
//     accesses" -- so io_rw_16 halfword writes to one half of a
//     double-buffered buffer-control word are architecturally fine (DPRAM is
//     SRAM-backed, not a register bus). The SDK structs themselves declare
//     the words io_rw_32; we may alias halves as io_rw_16 in Task 5/7.
//   - Names verified in SDK 2.2.0: usbh_dpram (usb_host_dpram_t, full 4 KB,
//     static_assert sizeof == USB_DPRAM_MAX == 4096) with fields
//     setup_packet[8], int_ep_ctrl[15].ctrl, epx_buf_ctrl, epx_ctrl,
//     int_ep_buffer_ctrl[15].ctrl, epx_data[] (offset 0x180); usb_hw fields
//     dev_addr_ctrl, int_ep_addr_ctrl[15], int_ep_ctrl, buf_status, muxing,
//     pwr, main_ctrl, sie_ctrl, sie_status; constants EP_CTRL_*,
//     USB_BUF_CTRL_*, USB_SIE_CTRL_*, USB_SIE_STATUS_*, USB_MAIN_CTRL_*,
//     USB_USB_MUXING_*, USB_USB_PWR_*, USB_ADDR_ENDP_ENDPOINT_LSB (=16).
//     All names in this file exist as written; no renames were needed.
#include "usb_hcd.h"
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/resets.h"
#include "hardware/structs/usb.h"   // also pulls in structs/usb_dpram.h

// SIE_CTRL bits present for every host operation: SOF/keep-alive generation
// and the host bus-pulldowns.
#define SIE_CTRL_BASE (USB_SIE_CTRL_SOF_EN_BITS | USB_SIE_CTRL_KEEP_ALIVE_EN_BITS | \
                       USB_SIE_CTRL_PULLDOWN_EN_BITS)

// DPRAM data-buffer layout (offsets from USBCTRL_DPRAM_BASE; the control/buffer
// register area occupies the bottom 0x180):
#define EPX_BUF_OFFSET    0x180u            // 2 x 64 bytes, EPX double buffer
#define INT_EP_BUF_OFFSET 0x200u            // 64 bytes, hub interrupt endpoint
// Task 5/10: data-buffer pointers, used by the transfer code added there.
// Commented out until then to avoid unused-variable warnings.
// static uint8_t *const epx_buf    = (uint8_t *)(USBCTRL_DPRAM_BASE + EPX_BUF_OFFSET);
// static uint8_t *const int_ep_buf = (uint8_t *)(USBCTRL_DPRAM_BASE + INT_EP_BUF_OFFSET);

static uint8_t ep0_mps = 8;

void hcd_set_ep0_mps(uint8_t mps) { ep0_mps = mps; }

void hcd_init(void) {
    reset_block_num(RESET_USBCTRL);
    unreset_block_num_wait_blocking(RESET_USBCTRL);
    // usb_host_dpram_t covers the full 4 KB DPRAM (static_assert in
    // usb_dpram.h), so this clears the data-buffer region at 0x180+ too.
    memset((void *)usbh_dpram, 0, sizeof(*usbh_dpram));

    // Connect controller to the on-chip PHY, force VBUS-detect high (our VBUS
    // is hardwired on), enable controller in host mode, start SOF generation.
    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS | USB_MAIN_CTRL_HOST_NDEVICE_BITS;
    usb_hw->sie_ctrl = SIE_CTRL_BASE;
    ep0_mps = 8;
}

hcd_speed_t hcd_port_speed(void) {
    return (hcd_speed_t)((usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS)
                         >> USB_SIE_STATUS_SPEED_LSB);
}

void hcd_bus_reset(void) {
    hw_set_bits(&usb_hw->sie_ctrl, USB_SIE_CTRL_RESET_BUS_BITS);
    sleep_ms(50);    // SIE drives SE0; bit self-clears when reset completes
    sleep_ms(10);    // post-reset recovery before first transfer
    ep0_mps = 8;
}
