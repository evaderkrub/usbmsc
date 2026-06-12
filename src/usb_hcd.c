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

// Clear-alias view of the USB registers (write-1-to-clear via the 0x3000
// atomic alias). SDK 2.2.0 defines no usb_hw_clear itself (verified:
// structs/usb.h only defines usb_hw); build from hw_clear_alias().
#define usb_hw_clear hw_clear_alias(usb_hw)

// SIE_CTRL bits present for every host operation: SOF/keep-alive generation
// and the host bus-pulldowns.
#define SIE_CTRL_BASE (USB_SIE_CTRL_SOF_EN_BITS | USB_SIE_CTRL_KEEP_ALIVE_EN_BITS | \
                       USB_SIE_CTRL_PULLDOWN_EN_BITS)

// DPRAM data-buffer layout (offsets from USBCTRL_DPRAM_BASE; the control/buffer
// register area occupies the bottom 0x180):
#define EPX_BUF_OFFSET    0x180u            // 2 x 64 bytes, EPX double buffer
#define INT_EP_BUF_OFFSET 0x200u            // 64 bytes, hub interrupt endpoint
static uint8_t *const epx_buf    = (uint8_t *)(USBCTRL_DPRAM_BASE + EPX_BUF_OFFSET);
// Task 10: hub interrupt-endpoint buffer pointer. Commented out until then to
// avoid an unused-variable warning.
// static uint8_t *const int_ep_buf = (uint8_t *)(USBCTRL_DPRAM_BASE + INT_EP_BUF_OFFSET);

// Endpoint-type encoding for the EP_CONTROL ENDPOINT_TYPE field (bits 27:26,
// EP_CTRL_BUFFER_TYPE_LSB). SDK 2.2.0 defines no USB_TRANSFER_TYPE_* names
// (verified: absent from regs/usb.h and structs/usb_dpram.h), so define them
// here per the RP2350 datasheet: 0 control, 1 isochronous, 2 bulk,
// 3 interrupt. Same values TinyUSB feeds this field (tusb_xfer_type_t).
#define USB_TRANSFER_TYPE_CONTROL   0u
#define USB_TRANSFER_TYPE_BULK      2u
#define USB_TRANSFER_TYPE_INTERRUPT 3u

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

// ---------------------------------------------------------------------------
// Shared transfer machinery
// ---------------------------------------------------------------------------

// Write one 16-bit half of the EPX buffer control. Datasheet requires the
// AVAILABLE bit to be set after the rest of the halfword has settled
// (>= 3 usb_clk cycles), or the SIE can read a torn value.
static void epx_buf_ctrl_write_half(int half, uint16_t val) {
    io_rw_16 *bc = (io_rw_16 *)&usbh_dpram->epx_buf_ctrl;
    if (val & USB_BUF_CTRL_AVAIL) {
        bc[half] = val & ~USB_BUF_CTRL_AVAIL;
        busy_wait_at_least_cycles(12);
    }
    bc[half] = val;
}

static uint16_t epx_buf_ctrl_read_half(int half) {
    io_rw_16 *bc = (io_rw_16 *)&usbh_dpram->epx_buf_ctrl;
    return bc[half];
}

// Start a transfer on EPX. dir_bits = SEND_SETUP / SEND_DATA / RECEIVE_DATA.
// Per RP2040/RP2350 erratum, START_TRANS must be set a few cycles after the
// rest of SIE_CTRL.
static void sie_start_transfer(uint32_t dir_bits) {
    usb_hw->sie_ctrl = SIE_CTRL_BASE | dir_bits;
    busy_wait_at_least_cycles(12);
    usb_hw->sie_ctrl = SIE_CTRL_BASE | dir_bits | USB_SIE_CTRL_START_TRANS_BITS;
}

// Check-and-clear SIE error flags. Returns HCD_OK if none set.
static hcd_result_t sie_check_errors(void) {
    uint32_t s = usb_hw->sie_status;
    if (s & USB_SIE_STATUS_STALL_REC_BITS) {
        usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;
        return HCD_ERR_STALL;
    }
    if (s & USB_SIE_STATUS_RX_TIMEOUT_BITS) {
        usb_hw_clear->sie_status = USB_SIE_STATUS_RX_TIMEOUT_BITS;
        return HCD_ERR_TIMEOUT;
    }
    uint32_t data_err = USB_SIE_STATUS_RX_OVERFLOW_BITS | USB_SIE_STATUS_DATA_SEQ_ERROR_BITS |
                        USB_SIE_STATUS_CRC_ERROR_BITS | USB_SIE_STATUS_BIT_STUFF_ERROR_BITS;
    if (s & data_err) {
        usb_hw_clear->sie_status = data_err;
        return HCD_ERR_DATA;
    }
    return HCD_OK;
}

// Wait (polling) until TRANS_COMPLETE, an error, disconnect, or timeout.
static hcd_result_t sie_wait_trans_complete(uint32_t timeout_ms) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    for (;;) {
        hcd_result_t err = sie_check_errors();
        if (err != HCD_OK) return err;
        if (usb_hw->sie_status & USB_SIE_STATUS_TRANS_COMPLETE_BITS) {
            usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;
            return HCD_OK;
        }
        if (hcd_port_speed() == HCD_SPEED_NONE) return HCD_ERR_DISCONNECT;
        if (time_reached(deadline)) return HCD_ERR_TIMEOUT;
    }
}

// Configure EPX for a single-buffered transfer of one packet and run it.
// Used by control transfers (small, simplicity over speed).
// dir_in: true = IN; buf/len: packet to send (OUT) or space to receive (IN).
// Returns bytes moved via *actual.
static hcd_result_t epx_single_packet(uint8_t dev_addr, uint8_t ep_num,
                                      bool dir_in, uint8_t toggle,
                                      void *buf, uint16_t len, uint16_t *actual,
                                      uint32_t timeout_ms) {
    usbh_dpram->epx_ctrl = EP_CTRL_ENABLE_BITS | EP_CTRL_INTERRUPT_PER_BUFFER
                         | (USB_TRANSFER_TYPE_CONTROL << EP_CTRL_BUFFER_TYPE_LSB)
                         | EPX_BUF_OFFSET;
    usb_hw->dev_addr_ctrl = (uint32_t)dev_addr
                          | ((uint32_t)ep_num << USB_ADDR_ENDP_ENDPOINT_LSB);

    uint16_t bc = (uint16_t)(len | USB_BUF_CTRL_LAST | USB_BUF_CTRL_AVAIL
                  | (toggle ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID));
    if (!dir_in) {
        if (len) memcpy(epx_buf, buf, len);
        bc |= USB_BUF_CTRL_FULL;
    }
    usb_hw_clear->buf_status = 1u;               // stale EPX buffer flag
    epx_buf_ctrl_write_half(0, bc);
    sie_start_transfer(dir_in ? USB_SIE_CTRL_RECEIVE_DATA_BITS
                              : USB_SIE_CTRL_SEND_DATA_BITS);
    hcd_result_t r = sie_wait_trans_complete(timeout_ms);
    if (r != HCD_OK) return r;

    if (dir_in) {
        uint16_t rx = epx_buf_ctrl_read_half(0) & USB_BUF_CTRL_LEN_MASK;
        if (rx > len) rx = len;
        memcpy(buf, epx_buf, rx);
        *actual = rx;
    } else {
        *actual = len;
    }
    usb_hw_clear->buf_status = 1u;
    return HCD_OK;
}

hcd_result_t hcd_control_xfer(uint8_t dev_addr, const uint8_t setup[8],
                              void *data, uint16_t *inout_len) {
    bool dir_in = (setup[0] & 0x80u) != 0;
    uint16_t wlen = (uint16_t)(setup[6] | (setup[7] << 8));

    // --- SETUP stage: 8 bytes from the dedicated DPRAM setup area, DATA0 ---
    memcpy((void *)usbh_dpram->setup_packet, setup, 8);
    usb_hw->dev_addr_ctrl = dev_addr;            // endpoint 0
    sie_start_transfer(USB_SIE_CTRL_SEND_SETUP_BITS);
    hcd_result_t r = sie_wait_trans_complete(100);
    if (r != HCD_OK) return r;

    // --- DATA stage: mps-sized packets, toggle starts at DATA1 ---
    uint16_t total = 0;
    if (data && inout_len && wlen) {
        uint16_t want = wlen < *inout_len ? wlen : *inout_len;
        uint8_t *p = (uint8_t *)data;
        uint8_t toggle = 1;
        while (total < want) {
            uint16_t n = (uint16_t)(want - total);
            if (n > ep0_mps) n = ep0_mps;
            uint16_t moved = 0;
            r = epx_single_packet(dev_addr, 0, dir_in, toggle,
                                  p + total, n, &moved, 100);
            if (r != HCD_OK) return r;
            total = (uint16_t)(total + moved);
            toggle ^= 1;
            if (dir_in && moved < ep0_mps) break;   // short packet ends stage
        }
    }
    if (inout_len) *inout_len = total;

    // --- STATUS stage: zero-length, always DATA1 ---
    // USB rule: status is IN unless the transfer had an IN data stage,
    // in which case status is OUT.
    bool status_in = !(dir_in && wlen && data);
    uint16_t zero = 0;
    uint8_t dummy[1];
    return epx_single_packet(dev_addr, 0, status_in, 1, dummy, 0, &zero, 100);
}
