# RP2350 USB MSC Host Driver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A from-scratch polled USB full-speed host driver for RP2350 exposing one USB mass-storage drive (direct or behind a hub) as a blocking block device at bus-saturating throughput (~19 packets/frame, ≥1.05 MiB/s).

**Architecture:** Five strictly layered C modules: `usb_parse` (pure functions), `usb_hcd` (RP2350 USB registers/DPRAM, EPX double-buffered pump, hardware-polled interrupt endpoint), `usb_core` (control transfers + enumeration), `usb_hub` (single-port passthrough), `usb_msc` (BOT + SCSI + public blocking API). Polled superloop — no interrupts, every wait has a timeout. See spec: `docs/superpowers/specs/2026-06-12-usb-msc-host-design.md`.

**Tech Stack:** C (C11), Pico SDK ≥ 2.0 (CMake, `hardware/structs/usb.h` register definitions only — NO TinyUSB), Ninja, arm-none-eabi-gcc, host C compiler for unit tests, CTest.

---

## Environment prerequisites (verify before Task 1)

- `$env:PICO_SDK_PATH` points at Pico SDK ≥ 2.0 (RP2350 support). Check: `Test-Path "$env:PICO_SDK_PATH/pico_sdk_init.cmake"` → True.
- `arm-none-eabi-gcc --version` and `ninja --version` and `cmake --version` all succeed.
- A host C compiler (`gcc` or MSVC `cl`) for the PC unit tests.
- Hardware steps need the user: custom RP2350 board (VBUS hardwired on), UART console attached, a USB flash drive, and a FS USB hub. Steps that need flashing say so explicitly — build the `.uf2`, then **ask the user** to flash (BOOTSEL copy or `picotool load -f build/bench.uf2`) and paste UART output. Do not mark those steps complete without the user's pasted output.

## Register-name ground rules

All register/field names come from Pico SDK headers: `hardware/structs/usb.h` (provides `usb_hw`, `usb_hw_set`, `usb_hw_clear`, `usbh_dpram`) and `hardware/regs/usb.h` (`USB_*_BITS`/`_LSB` constants). If a name in this plan fails to compile, find the equivalent in those headers — the intent (which register, which field) is stated in a comment at each use. Two facts to cross-check once against RP2350 datasheet §12.7 during Task 4 (they are flagged again where used):

1. **BUFF_STATUS bit mapping in host mode:** EPX uses bit 0; hardware-polled interrupt endpoint N (1-based) uses bit 2·N.
2. **DPRAM buffer-control halfword access:** this plan re-arms one half of the double-buffered EPX buffer control with a 16-bit write (`io_rw_16`) so the hardware-owned other half is never read-modify-written. Confirm DPRAM supports 16-bit writes; if it does not, the fallback (documented in Task 7) is to service both halves per poll iteration with a single 32-bit write.

---

### Task 1: Project skeleton — builds for RP2350 and on the host

**Files:**
- Create: `CMakeLists.txt`
- Create: `pico_sdk_import.cmake` (copied from SDK)
- Create: `apps/bench/main.c`
- Create: `src/usb_parse.h`, `src/usb_parse.c` (stubs)
- Create: `tests/CMakeLists.txt`, `tests/test_parse.c` (placeholder test that passes)
- Create: `.gitignore`

- [ ] **Step 1: Copy the SDK import shim**

```powershell
Copy-Item "$env:PICO_SDK_PATH/external/pico_sdk_import.cmake" pico_sdk_import.cmake
```

- [ ] **Step 2: Write `.gitignore`**

```gitignore
build/
build-tests/
```

- [ ] **Step 3: Write `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Platform")

include(pico_sdk_import.cmake)
project(usbmsc C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_library(usbmsc_host
    src/usb_parse.c
)
target_include_directories(usbmsc_host PUBLIC src)
target_link_libraries(usbmsc_host PUBLIC pico_stdlib hardware_resets)

add_executable(bench apps/bench/main.c)
target_link_libraries(bench usbmsc_host)
pico_enable_stdio_uart(bench 1)
pico_enable_stdio_usb(bench 0)   # we own the USB controller — never link stdio_usb
pico_add_extra_outputs(bench)
```

- [ ] **Step 4: Write `apps/bench/main.c` (stub)**

```c
#include <stdio.h>
#include "pico/stdlib.h"

int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    printf("usbmsc bench: skeleton OK\n");
    for (;;) tight_loop_contents();
}
```

- [ ] **Step 5: Write stub `src/usb_parse.h`**

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>
```

- [ ] **Step 6: Write stub `src/usb_parse.c`**

```c
#include "usb_parse.h"
```

- [ ] **Step 7: Write `tests/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.13)
project(usbmsc_tests C)
set(CMAKE_C_STANDARD 11)
enable_testing()

add_executable(test_parse test_parse.c ${CMAKE_CURRENT_SOURCE_DIR}/../src/usb_parse.c)
target_include_directories(test_parse PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
add_test(NAME parse COMMAND test_parse)
```

- [ ] **Step 8: Write `tests/test_parse.c` (scaffold)**

```c
#include <stdio.h>
#include "usb_parse.h"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

int main(void) {
    printf(failures ? "FAILED (%d)\n" : "all tests passed\n", failures);
    return failures ? 1 : 0;
}
```

- [ ] **Step 9: Build firmware**

Run: `cmake -S . -B build -G Ninja; cmake --build build`
Expected: succeeds, `build/bench.uf2` exists.

- [ ] **Step 10: Build and run host tests**

Run: `cmake -S tests -B build-tests; cmake --build build-tests; ctest --test-dir build-tests --output-on-failure`
Expected: `1/1 Test #1: parse ... Passed` (the scaffold passes trivially).

- [ ] **Step 11: Commit**

```bash
git add -A
git commit -m "chore: project skeleton — RP2350 firmware + host test builds"
```

---

### Task 2: usb_parse — configuration descriptor walker (TDD)

**Files:**
- Modify: `src/usb_parse.h`
- Modify: `src/usb_parse.c`
- Modify: `tests/test_parse.c`

- [ ] **Step 1: Add the API to `src/usb_parse.h`**

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define USB_CLASS_HUB     0x09
#define USB_CLASS_MSC     0x08
#define MSC_SUBCLASS_SCSI 0x06
#define MSC_PROTO_BOT     0x50

typedef struct {
    uint8_t  config_value;    // bConfigurationValue
    bool     is_hub;
    bool     is_msc;
    // valid when is_msc:
    uint8_t  msc_itf;         // bInterfaceNumber
    uint8_t  bulk_in;         // endpoint address (0x8x)
    uint8_t  bulk_out;        // endpoint address (0x0x)
    uint16_t bulk_in_mps;
    uint16_t bulk_out_mps;
    // valid when is_hub:
    uint8_t  hub_int_ep;      // status-change endpoint address (0x8x)
    uint16_t hub_int_mps;
    uint8_t  hub_int_interval;
} usb_cfg_info_t;

// Walk a full configuration descriptor (config + interface + endpoint TLVs).
// Returns false on malformed/truncated input.
bool usb_parse_config(const uint8_t *d, uint16_t len, usb_cfg_info_t *out);
```

- [ ] **Step 2: Write failing tests in `tests/test_parse.c`** (insert before `main`, and call from `main` before the summary printf)

```c
static const uint8_t MSC_CFG[] = {
    // config descriptor: wTotalLength=32, 1 interface, bConfigurationValue=1
    9, 2, 32, 0, 1, 1, 0, 0x80, 50,
    // interface: num 0, 2 eps, class 8 sub 6 proto 0x50
    9, 4, 0, 0, 2, 0x08, 0x06, 0x50, 0,
    // endpoint: 0x81 bulk, mps 64
    7, 5, 0x81, 0x02, 64, 0, 0,
    // endpoint: 0x02 bulk, mps 64
    7, 5, 0x02, 0x02, 64, 0, 0,
};

static const uint8_t HUB_CFG[] = {
    9, 2, 25, 0, 1, 1, 0, 0xE0, 50,
    9, 4, 0, 0, 1, 0x09, 0x00, 0x00, 0,
    7, 5, 0x81, 0x03, 1, 0, 0xFF,      // interrupt IN, mps 1, interval 255
};

static void test_parse_msc(void) {
    usb_cfg_info_t info;
    CHECK(usb_parse_config(MSC_CFG, sizeof MSC_CFG, &info));
    CHECK(info.is_msc && !info.is_hub);
    CHECK(info.config_value == 1);
    CHECK(info.msc_itf == 0);
    CHECK(info.bulk_in == 0x81 && info.bulk_in_mps == 64);
    CHECK(info.bulk_out == 0x02 && info.bulk_out_mps == 64);
}

static void test_parse_hub(void) {
    usb_cfg_info_t info;
    CHECK(usb_parse_config(HUB_CFG, sizeof HUB_CFG, &info));
    CHECK(info.is_hub && !info.is_msc);
    CHECK(info.hub_int_ep == 0x81 && info.hub_int_mps == 1 && info.hub_int_interval == 0xFF);
}

static void test_parse_malformed(void) {
    usb_cfg_info_t info;
    uint8_t zero_len[12];
    memcpy(zero_len, MSC_CFG, 12);
    zero_len[9] = 0;                              // interface bLength = 0
    CHECK(!usb_parse_config(zero_len, sizeof zero_len, &info));
    CHECK(!usb_parse_config(MSC_CFG, 3, &info));  // truncated header
    uint8_t overrun[sizeof MSC_CFG];
    memcpy(overrun, MSC_CFG, sizeof MSC_CFG);
    overrun[sizeof MSC_CFG - 7] = 200;            // last ep bLength runs past end
    CHECK(!usb_parse_config(overrun, sizeof overrun, &info));
}
```

Add `#include <string.h>` at the top, and in `main`: `test_parse_msc(); test_parse_hub(); test_parse_malformed();`

- [ ] **Step 3: Run tests to verify they fail**

Run: `cmake --build build-tests; ctest --test-dir build-tests --output-on-failure`
Expected: build FAILS — `usb_parse_config` undefined.

- [ ] **Step 4: Implement `usb_parse_config` in `src/usb_parse.c`**

```c
#include "usb_parse.h"
#include <string.h>

enum { DT_CONFIG = 2, DT_INTERFACE = 4, DT_ENDPOINT = 5 };

bool usb_parse_config(const uint8_t *d, uint16_t len, usb_cfg_info_t *out) {
    memset(out, 0, sizeof *out);
    if (len < 9 || d[0] < 9 || d[1] != DT_CONFIG) return false;
    out->config_value = d[5];

    uint8_t cur_class = 0, cur_itf = 0;
    bool in_msc_itf = false, in_hub_itf = false;
    uint16_t off = 0;
    while (off < len) {
        uint8_t dlen = d[off];
        if (dlen < 2 || (uint16_t)(off + dlen) > len) return false;
        const uint8_t *p = d + off;
        switch (p[1]) {
        case DT_INTERFACE:
            if (dlen < 9) return false;
            cur_itf = p[2];
            cur_class = p[5];
            in_msc_itf = (p[5] == USB_CLASS_MSC && p[6] == MSC_SUBCLASS_SCSI &&
                          p[7] == MSC_PROTO_BOT && !out->is_msc);
            in_hub_itf = (p[5] == USB_CLASS_HUB && !out->is_hub);
            if (in_msc_itf) { out->is_msc = true; out->msc_itf = cur_itf; }
            if (in_hub_itf) out->is_hub = true;
            break;
        case DT_ENDPOINT: {
            if (dlen < 7) return false;
            uint8_t  addr = p[2], attr = p[3] & 0x03;
            uint16_t mps  = (uint16_t)(p[4] | (p[5] << 8));
            if (in_msc_itf && attr == 0x02) {           // bulk
                if (addr & 0x80) { out->bulk_in = addr;  out->bulk_in_mps = mps; }
                else             { out->bulk_out = addr; out->bulk_out_mps = mps; }
            }
            if (in_hub_itf && attr == 0x03 && (addr & 0x80)) {  // interrupt IN
                out->hub_int_ep = addr;
                out->hub_int_mps = mps;
                out->hub_int_interval = p[6];
            }
            break;
        }
        default:
            break;
        }
        off += dlen;
    }
    (void)cur_class;
    if (out->is_msc && (!out->bulk_in || !out->bulk_out)) return false;
    if (out->is_hub && !out->hub_int_ep) return false;
    return out->is_msc || out->is_hub;
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cmake --build build-tests; ctest --test-dir build-tests --output-on-failure`
Expected: `parse ... Passed`, output contains `all tests passed`.

- [ ] **Step 6: Commit**

```bash
git add src/usb_parse.h src/usb_parse.c tests/test_parse.c
git commit -m "feat: configuration descriptor parser with hub/MSC detection"
```

---

### Task 3: usb_parse — CBW build / CSW validate (TDD)

**Files:**
- Modify: `src/usb_parse.h`
- Modify: `src/usb_parse.c`
- Modify: `tests/test_parse.c`

- [ ] **Step 1: Add the API to `src/usb_parse.h`** (append at end)

```c
#define MSC_CBW_LEN 31
#define MSC_CSW_LEN 13

// Fill a 31-byte Command Block Wrapper. cb_len <= 16.
void msc_build_cbw(uint8_t cbw[MSC_CBW_LEN], uint32_t tag, uint32_t data_len,
                   bool dir_in, uint8_t lun, const uint8_t *cb, uint8_t cb_len);

// Validate a 13-byte Command Status Wrapper (signature, tag, status <= 2).
// On success writes bCSWStatus (0=passed, 1=failed, 2=phase error) to *status.
bool msc_parse_csw(const uint8_t csw[MSC_CSW_LEN], uint32_t tag, uint8_t *status);
```

- [ ] **Step 2: Write failing tests in `tests/test_parse.c`** (insert before `main`; call both from `main`)

```c
static void test_cbw_build(void) {
    uint8_t cbw[MSC_CBW_LEN];
    const uint8_t read10[10] = {0x28, 0, 0, 0, 0x12, 0x34, 0, 0, 0x40, 0};
    msc_build_cbw(cbw, 0xDEADBEEF, 0x8000, true, 0, read10, sizeof read10);
    CHECK(cbw[0] == 0x55 && cbw[1] == 0x53 && cbw[2] == 0x42 && cbw[3] == 0x43); // 'USBC'
    CHECK(cbw[4] == 0xEF && cbw[5] == 0xBE && cbw[6] == 0xAD && cbw[7] == 0xDE); // tag LE
    CHECK(cbw[8] == 0x00 && cbw[9] == 0x80 && cbw[10] == 0 && cbw[11] == 0);     // len LE
    CHECK(cbw[12] == 0x80);                                                       // IN
    CHECK(cbw[13] == 0 && cbw[14] == 10);                                         // lun, cb_len
    CHECK(memcmp(cbw + 15, read10, 10) == 0);
    CHECK(cbw[25] == 0 && cbw[30] == 0);                                          // zero padding
}

static void test_csw_parse(void) {
    uint8_t status;
    uint8_t csw[MSC_CSW_LEN] = {0x55, 0x53, 0x42, 0x53,            // 'USBS'
        0xEF, 0xBE, 0xAD, 0xDE, 0, 0, 0, 0, 0};
    CHECK(msc_parse_csw(csw, 0xDEADBEEF, &status) && status == 0);
    csw[12] = 1;
    CHECK(msc_parse_csw(csw, 0xDEADBEEF, &status) && status == 1);
    csw[12] = 3;
    CHECK(!msc_parse_csw(csw, 0xDEADBEEF, &status));   // invalid status
    csw[12] = 0;
    CHECK(!msc_parse_csw(csw, 0x12345678, &status));   // wrong tag
    csw[0] = 0x56;
    CHECK(!msc_parse_csw(csw, 0xDEADBEEF, &status));   // bad signature
}
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cmake --build build-tests; ctest --test-dir build-tests --output-on-failure`
Expected: build FAILS — `msc_build_cbw` undefined.

- [ ] **Step 4: Implement in `src/usb_parse.c`** (append at end)

```c
static void put_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint32_t get_le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

void msc_build_cbw(uint8_t cbw[MSC_CBW_LEN], uint32_t tag, uint32_t data_len,
                   bool dir_in, uint8_t lun, const uint8_t *cb, uint8_t cb_len) {
    memset(cbw, 0, MSC_CBW_LEN);
    put_le32(cbw, 0x43425355u);          // dCBWSignature 'USBC'
    put_le32(cbw + 4, tag);
    put_le32(cbw + 8, data_len);
    cbw[12] = dir_in ? 0x80 : 0x00;
    cbw[13] = lun;
    cbw[14] = cb_len;
    memcpy(cbw + 15, cb, cb_len);
}

bool msc_parse_csw(const uint8_t csw[MSC_CSW_LEN], uint32_t tag, uint8_t *status) {
    if (get_le32(csw) != 0x53425355u) return false;   // dCSWSignature 'USBS'
    if (get_le32(csw + 4) != tag) return false;
    if (csw[12] > 2) return false;
    *status = csw[12];
    return true;
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cmake --build build-tests; ctest --test-dir build-tests --output-on-failure`
Expected: `parse ... Passed`.

- [ ] **Step 6: Commit**

```bash
git add src/usb_parse.h src/usb_parse.c tests/test_parse.c
git commit -m "feat: CBW build and CSW validation"
```

---

### Task 4: usb_hcd — controller init, port detect, bus reset

**Files:**
- Create: `src/usb_hcd.h`, `src/usb_hcd.c`
- Modify: `CMakeLists.txt` (add `src/usb_hcd.c` to `usbmsc_host` sources)
- Modify: `apps/bench/main.c`

- [ ] **Step 1: Datasheet cross-check (no code)**

Open RP2350 datasheet §12.7 (USB controller, host mode) and confirm the two flagged facts from "Register-name ground rules" at the top of this plan: host-mode BUFF_STATUS bit mapping, and DPRAM 16-bit write support for buffer-control halves. Also skim the host-mode transfer sequence description. Record findings as a comment block at the top of `usb_hcd.c` in Step 3.

- [ ] **Step 2: Write `src/usb_hcd.h`**

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    HCD_OK = 0,
    HCD_ERR_STALL,        // endpoint returned STALL
    HCD_ERR_TIMEOUT,      // device not responding / software timeout
    HCD_ERR_DATA,         // CRC, bit-stuff, DATA-seq, overflow
    HCD_ERR_DISCONNECT,   // device went away mid-transfer
} hcd_result_t;

typedef enum {
    HCD_SPEED_NONE = 0,
    HCD_SPEED_LOW  = 1,
    HCD_SPEED_FULL = 2,
} hcd_speed_t;

void        hcd_init(void);
hcd_speed_t hcd_port_speed(void);   // current SIE_STATUS.SPEED
void        hcd_bus_reset(void);    // drive USB reset + recovery delay

// EP0 max packet size used by control transfers (8 until first descriptor read).
void hcd_set_ep0_mps(uint8_t mps);

// Blocking control transfer. setup = 8-byte setup packet. data/inout_len for
// the data stage (NULL/0 for none); *inout_len in = buffer size, out = actual.
hcd_result_t hcd_control_xfer(uint8_t dev_addr, const uint8_t setup[8],
                              void *data, uint16_t *inout_len);

// Blocking bulk transfer on EPX, double-buffered. ep_addr bit 7 = IN.
// *toggle = DATA toggle for this pipe (0/1), updated on return.
hcd_result_t hcd_bulk_xfer(uint8_t dev_addr, uint8_t ep_addr, uint8_t *toggle,
                           void *buf, uint32_t len, uint32_t *actual,
                           uint32_t timeout_ms);

// Hardware-polled interrupt IN endpoint (hub status pipe). One supported.
void hcd_int_ep_install(uint8_t dev_addr, uint8_t ep_addr, uint16_t mps,
                        uint8_t interval_ms);
void hcd_int_ep_remove(void);
// >0: bytes copied to buf; 0: no new data; <0: error
int  hcd_int_ep_poll(uint8_t *buf, uint8_t maxlen);
```

- [ ] **Step 3: Write `src/usb_hcd.c` (init/port/reset only — transfer functions come in Tasks 5/7/10)**

```c
// RP2350 USB host controller driver. Polled, no interrupts.
// Datasheet findings (Task 4 Step 1):
//   - BUFF_STATUS host mapping: <record what §12.7 says>
//   - DPRAM 16-bit write support: <record what §12.7 says>
#include "usb_hcd.h"
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/resets.h"
#include "hardware/structs/usb.h"

// SIE_CTRL bits present for every host operation: SOF/keep-alive generation
// and the host bus-pulldowns.
#define SIE_CTRL_BASE (USB_SIE_CTRL_SOF_EN_BITS | USB_SIE_CTRL_KEEP_ALIVE_EN_BITS | \
                       USB_SIE_CTRL_PULLDOWN_EN_BITS)

// DPRAM data-buffer layout (offsets from USBCTRL_DPRAM_BASE; the control/buffer
// register area occupies the bottom 0x180):
#define EPX_BUF_OFFSET    0x180u            // 2 x 64 bytes, EPX double buffer
#define INT_EP_BUF_OFFSET 0x200u            // 64 bytes, hub interrupt endpoint
static uint8_t *const epx_buf    = (uint8_t *)(USBCTRL_DPRAM_BASE + EPX_BUF_OFFSET);
static uint8_t *const int_ep_buf = (uint8_t *)(USBCTRL_DPRAM_BASE + INT_EP_BUF_OFFSET);

static uint8_t ep0_mps = 8;

void hcd_set_ep0_mps(uint8_t mps) { ep0_mps = mps; }

void hcd_init(void) {
    reset_block_num(RESET_USBCTRL);
    unreset_block_num_wait_blocking(RESET_USBCTRL);
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
```

- [ ] **Step 4: Add `src/usb_hcd.c` to `CMakeLists.txt`**

In the `add_library(usbmsc_host ...)` block add a line `    src/usb_hcd.c` after `src/usb_parse.c`.

- [ ] **Step 5: Rewrite `apps/bench/main.c` as a connection monitor**

```c
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
```

- [ ] **Step 6: Build**

Run: `cmake --build build`
Expected: clean build. If any `USB_*` constant or `usbh_dpram` field name fails, fix per "Register-name ground rules".

- [ ] **Step 7: Hardware check — ASK THE USER**

Ask the user to flash `build/bench.uf2` and plug/unplug a USB flash drive directly into the port, pasting UART output.
Expected: `port: full-speed` on plug, `port: disconnected` on unplug. Do not proceed until confirmed.

- [ ] **Step 8: Commit**

```bash
git add -A
git commit -m "feat: HCD init, port speed detect, bus reset"
```

---

### Task 5: usb_hcd — control transfers

**Files:**
- Modify: `src/usb_hcd.c` (append transfer machinery + `hcd_control_xfer`)
- Modify: `apps/bench/main.c`

- [ ] **Step 1: Append shared transfer helpers to `src/usb_hcd.c`**

```c
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
        memcpy(epx_buf, buf, len);
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
```

- [ ] **Step 2: Append `hcd_control_xfer` to `src/usb_hcd.c`**

```c
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
```

(IN status = host receives a zero-length DATA1 packet; OUT status = host sends one. `epx_single_packet` with `len 0` does either, with `status_in` passed as its `dir_in` argument.)

- [ ] **Step 3: Rewrite `apps/bench/main.c` — stage 2: read a device descriptor**

```c
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
```

- [ ] **Step 4: Build**

Run: `cmake --build build`
Expected: clean build (fix any register-name mismatches per ground rules).

- [ ] **Step 5: Hardware check — ASK THE USER**

Ask the user to flash `build/bench.uf2`, plug in a flash drive, and paste UART output.
Expected: 8 hex bytes with `bLength=18 bDescriptorType=1` and `bMaxPacketSize0` of 64 (typical for flash drives). If this fails, debug here before proceeding — everything later depends on control transfers. Likely failure points: SETUP not acknowledged (check `dev_addr_ctrl` write), wrong status-stage direction (STALL on status), missing erratum delay before START_TRANS.

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "feat: HCD control transfers; bench reads device descriptor"
```

---

### Task 6: usb_core — enumeration to a configured device

**Files:**
- Create: `src/usb_core.h`, `src/usb_core.c`
- Modify: `CMakeLists.txt` (add `src/usb_core.c`)
- Modify: `apps/bench/main.c`

- [ ] **Step 1: Write `src/usb_core.h`**

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "usb_hcd.h"
#include "usb_parse.h"

typedef struct {
    uint8_t        addr;       // assigned device address
    uint8_t        ep0_mps;
    uint16_t       vid, pid;
    usb_cfg_info_t cfg;
} usb_device_t;

// Standard requests (all blocking, on the device's EP0)
hcd_result_t core_get_descriptor(uint8_t addr, uint8_t type, uint8_t index,
                                 void *buf, uint16_t len, uint16_t *actual);
hcd_result_t core_set_address(uint8_t new_addr);          // sent to address 0
hcd_result_t core_set_configuration(uint8_t addr, uint8_t config_value);
hcd_result_t core_clear_endpoint_halt(uint8_t addr, uint8_t ep_addr);

// Full enumeration of the device currently in default state (just reset,
// answering on address 0): reads descriptors, assigns `addr_to_assign`,
// parses config, sets configuration. Fills *dev.
hcd_result_t core_enumerate(uint8_t addr_to_assign, usb_device_t *dev);
```

- [ ] **Step 2: Write `src/usb_core.c`**

```c
#include "usb_core.h"
#include <string.h>
#include "pico/stdlib.h"

enum { DT_DEVICE = 1, DT_CONFIG = 2 };

hcd_result_t core_get_descriptor(uint8_t addr, uint8_t type, uint8_t index,
                                 void *buf, uint16_t len, uint16_t *actual) {
    const uint8_t setup[8] = {0x80, 6, index, type,
                              0, 0, (uint8_t)len, (uint8_t)(len >> 8)};
    uint16_t n = len;
    hcd_result_t r = hcd_control_xfer(addr, setup, buf, &n);
    if (actual) *actual = n;
    return r;
}

hcd_result_t core_set_address(uint8_t new_addr) {
    const uint8_t setup[8] = {0x00, 5, new_addr, 0, 0, 0, 0, 0};
    hcd_result_t r = hcd_control_xfer(0, setup, NULL, NULL);
    sleep_ms(10);     // SET_ADDRESS settle time (spec: 2 ms)
    return r;
}

hcd_result_t core_set_configuration(uint8_t addr, uint8_t config_value) {
    const uint8_t setup[8] = {0x00, 9, config_value, 0, 0, 0, 0, 0};
    return hcd_control_xfer(addr, setup, NULL, NULL);
}

hcd_result_t core_clear_endpoint_halt(uint8_t addr, uint8_t ep_addr) {
    const uint8_t setup[8] = {0x02, 1, 0, 0, ep_addr, 0, 0, 0};
    return hcd_control_xfer(addr, setup, NULL, NULL);
}

hcd_result_t core_enumerate(uint8_t addr_to_assign, usb_device_t *dev) {
    memset(dev, 0, sizeof *dev);
    hcd_set_ep0_mps(8);
    uint8_t buf[256];
    uint16_t got;

    // 1. First 8 bytes of device descriptor at address 0 -> bMaxPacketSize0
    hcd_result_t r = core_get_descriptor(0, DT_DEVICE, 0, buf, 8, &got);
    if (r != HCD_OK) return r;
    if (got < 8 || buf[1] != DT_DEVICE) return HCD_ERR_DATA;
    dev->ep0_mps = buf[7];
    hcd_set_ep0_mps(dev->ep0_mps);

    // 2. Assign address
    r = core_set_address(addr_to_assign);
    if (r != HCD_OK) return r;
    dev->addr = addr_to_assign;

    // 3. Full device descriptor (18 bytes)
    r = core_get_descriptor(dev->addr, DT_DEVICE, 0, buf, 18, &got);
    if (r != HCD_OK) return r;
    if (got < 18) return HCD_ERR_DATA;
    dev->vid = (uint16_t)(buf[8] | (buf[9] << 8));
    dev->pid = (uint16_t)(buf[10] | (buf[11] << 8));

    // 4. Config descriptor: header first for wTotalLength, then the whole thing
    r = core_get_descriptor(dev->addr, DT_CONFIG, 0, buf, 9, &got);
    if (r != HCD_OK) return r;
    uint16_t total = (uint16_t)(buf[2] | (buf[3] << 8));
    if (total > sizeof buf) total = sizeof buf;
    r = core_get_descriptor(dev->addr, DT_CONFIG, 0, buf, total, &got);
    if (r != HCD_OK) return r;

    // 5. Parse and configure
    if (!usb_parse_config(buf, got, &dev->cfg)) return HCD_ERR_DATA;
    return core_set_configuration(dev->addr, dev->cfg.config_value);
}
```

- [ ] **Step 3: Add `src/usb_core.c` to `CMakeLists.txt` sources**

- [ ] **Step 4: Rewrite `apps/bench/main.c` — stage 3: enumerate**

Replace the per-connection body (from `sleep_ms(100);` after the full-speed wait through the descriptor printing) with:

```c
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
```

Add `#include "usb_core.h"` and update the stage banner to `stage 3 — enumeration`.

- [ ] **Step 5: Build**

Run: `cmake --build build`
Expected: clean build.

- [ ] **Step 6: Hardware check — ASK THE USER**

Flash + plug a flash drive: expect `enumerated addr=1 vid=.... pid=....` and an `MSC:` line with plausible bulk endpoints (e.g. `bulk_in=81 bulk_out=02 mps=64/64`). Also plug a hub (nothing attached to it): expect a `HUB:` line. Paste output.

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "feat: usb_core enumeration — descriptors, address, configuration"
```

---

### Task 7: usb_hcd — double-buffered bulk pump (the bandwidth core)

**Files:**
- Modify: `src/usb_hcd.c` (append `hcd_bulk_xfer`)

How it works: EPX is configured double-buffered (two 64-byte halves). Both halves are primed before START_TRANS. The poll loop waits for `BUFF_STATUS` bit 0, which fires once per completed half; software services halves in strict alternation (a software `next_half` index tracks which one the hardware finished first — hardware always consumes half 0, then 1, then 0...). Servicing = copy data out (IN) or in (OUT), then re-arm the same half for the packet two slots ahead, marking it LAST if it is the final packet. The hardware sends the next token from the *other* half with no software in the loop, so the bus never idles. TRANS_COMPLETE fires when the LAST-marked buffer completes.

Fallback (only if Task 4 Step 1 found that DPRAM rejects 16-bit writes): service both halves inside one poll iteration and write the full 32-bit `epx_buf_ctrl` once — never read-modify-write while the hardware owns a half.

- [ ] **Step 1: Append `hcd_bulk_xfer` to `src/usb_hcd.c`**

```c
// Prime one half of the EPX double buffer for the next packet of the
// transfer. *toggle is the DATA toggle; *queued tracks bytes already
// assigned to buffers. Returns the buffer-control halfword written.
static void epx_prime_half(int half, bool dir_in, const uint8_t *src,
                           uint32_t len, uint32_t *queued, uint8_t *toggle) {
    uint32_t remaining = len - *queued;
    uint16_t n = remaining > 64 ? 64 : (uint16_t)remaining;
    uint16_t bc = (uint16_t)(n | USB_BUF_CTRL_AVAIL
                  | (*toggle ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID));
    if (n == remaining) bc |= USB_BUF_CTRL_LAST;
    if (!dir_in) {
        memcpy(epx_buf + half * 64, src + *queued, n);
        bc |= USB_BUF_CTRL_FULL;
    }
    *queued += n;
    *toggle ^= 1;
    epx_buf_ctrl_write_half(half, bc);
}

hcd_result_t hcd_bulk_xfer(uint8_t dev_addr, uint8_t ep_addr, uint8_t *toggle,
                           void *buf, uint32_t len, uint32_t *actual,
                           uint32_t timeout_ms) {
    bool dir_in = (ep_addr & 0x80u) != 0;
    uint8_t *p = (uint8_t *)buf;
    uint32_t queued = 0;     // bytes assigned to a buffer half so far
    uint32_t done = 0;       // bytes confirmed transferred
    if (actual) *actual = 0;

    usbh_dpram->epx_ctrl = EP_CTRL_ENABLE_BITS | EP_CTRL_DOUBLE_BUFFERED_BITS
                         | EP_CTRL_INTERRUPT_PER_BUFFER
                         | (USB_TRANSFER_TYPE_BULK << EP_CTRL_BUFFER_TYPE_LSB)
                         | EPX_BUF_OFFSET;
    usb_hw->dev_addr_ctrl = (uint32_t)dev_addr
                          | ((uint32_t)(ep_addr & 0x0F) << USB_ADDR_ENDP_ENDPOINT_LSB);
    usb_hw_clear->buf_status = 1u;               // stale EPX flag

    // Prime both halves (half 1 only if the transfer needs a second packet).
    epx_prime_half(0, dir_in, p, len, &queued, toggle);
    if (queued < len) epx_prime_half(1, dir_in, p, len, &queued, toggle);

    sie_start_transfer(dir_in ? USB_SIE_CTRL_RECEIVE_DATA_BITS
                              : USB_SIE_CTRL_SEND_DATA_BITS);

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    int next_half = 0;                           // hardware consumes 0,1,0,1,...
    bool complete = false;

    while (!complete || done < queued) {
        // TRANS_COMPLETE can arrive together with the final buffer flag;
        // latch it but keep draining buffers until done catches up.
        if (usb_hw->sie_status & USB_SIE_STATUS_TRANS_COMPLETE_BITS) {
            usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;
            complete = true;
        }
        hcd_result_t err = sie_check_errors();
        if (err != HCD_OK) return err;
        if (hcd_port_speed() == HCD_SPEED_NONE) return HCD_ERR_DISCONNECT;
        if (time_reached(deadline)) return HCD_ERR_TIMEOUT;
        if (!(usb_hw->buf_status & 1u)) continue;

        // At least one half finished. Service in strict alternation; check
        // the half's own status bits rather than trusting the flag count.
        usb_hw_clear->buf_status = 1u;
        for (;;) {
            uint16_t bc = epx_buf_ctrl_read_half(next_half);
            if (dir_in) {
                if (!(bc & USB_BUF_CTRL_FULL)) break;        // not done yet
                uint16_t rx = bc & USB_BUF_CTRL_LEN_MASK;
                memcpy(p + done, epx_buf + next_half * 64, rx);
                done += rx;
                // Clear FULL so we don't re-service; re-arm if more queued
                if (queued < len) {
                    epx_prime_half(next_half, true, p, len, &queued, toggle);
                } else {
                    epx_buf_ctrl_write_half(next_half, 0);
                }
                if (rx < 64) {                               // short packet ends transfer
                    if (actual) *actual = done;
                    // hardware raises TRANS_COMPLETE for short IN packets;
                    // consume it if already latched, else don't wait for it
                    usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;
                    return HCD_OK;
                }
            } else {
                if (bc & USB_BUF_CTRL_AVAIL) break;          // hw still owns it
                uint16_t tx = bc & USB_BUF_CTRL_LEN_MASK;
                done += tx;
                if (queued < len) {
                    epx_prime_half(next_half, false, p, len, &queued, toggle);
                } else {
                    epx_buf_ctrl_write_half(next_half, 0);
                }
            }
            next_half ^= 1;
        }
    }
    if (actual) *actual = done;
    return HCD_OK;
}
```

Two subtleties, called out so the implementer does not "fix" them:

1. **The DATA toggle is advanced at prime time, not completion time.** If a transfer aborts mid-flight the toggle is wrong — that is fine, because every error path in the MSC layer ends in CLEAR_FEATURE(halt) (which resets the device's toggle and our stored one) or bus reset.
2. **`done < queued` in the loop condition** keeps draining after TRANS_COMPLETE latches: the final two buffer completions and TRANS_COMPLETE can be observed in the same poll iteration.

- [ ] **Step 2: Build**

Run: `cmake --build build`
Expected: clean build. Functional verification happens in Task 8 (first real bulk traffic is the MSC INQUIRY); this task is compile + code-review only.

- [ ] **Step 3: Commit**

```bash
git add src/usb_hcd.c
git commit -m "feat: HCD double-buffered bulk transfer pump"
```

---

### Task 8: usb_msc — BOT engine, SCSI bring-up, ready state

**Files:**
- Create: `src/usb_msc.h`, `src/usb_msc.c`
- Modify: `CMakeLists.txt` (add `src/usb_msc.c`)
- Modify: `apps/bench/main.c`

- [ ] **Step 1: Write `src/usb_msc.h` (the project's public API, complete now — read/write bodies arrive in Task 9)**

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MSC_OK = 0,
    MSC_NOT_READY,        // no drive enumerated / drive still spinning up
    MSC_MEDIA_ERROR,      // command failed; sense data in usb_msc_last_sense()
    MSC_TRANSPORT_ERROR,  // unrecoverable USB-level failure
    MSC_DISCONNECTED,     // device removed
} msc_result_t;

void     usb_msc_init(void);     // hcd_init + state machine reset
void     usb_msc_task(void);     // call from main loop; drives hotplug + enum
bool     usb_msc_ready(void);
uint32_t usb_msc_block_count(void);
uint32_t usb_msc_block_size(void);

msc_result_t usb_msc_read (uint32_t lba, uint32_t count, void *buf);
msc_result_t usb_msc_write(uint32_t lba, uint32_t count, const void *buf);

// Last SCSI sense key/asc/ascq captured after MSC_MEDIA_ERROR (0 if none).
uint32_t usb_msc_last_sense(void);

// Drive identity from INQUIRY (valid when ready): "VENDOR  PRODUCT REV"
const char *usb_msc_drive_name(void);
```

- [ ] **Step 2: Write `src/usb_msc.c` — state, BOT engine, SCSI bring-up**

```c
#include "usb_msc.h"
#include "usb_core.h"
#include "usb_hub.h"          // Task 10; until then provide the stub below
#include "usb_parse.h"
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"

typedef enum { ST_DISCONNECTED, ST_READY, ST_FAILED } msc_state_t;

static msc_state_t  state = ST_DISCONNECTED;
static usb_device_t drive;          // the MSC device (addr 1 direct, 2 via hub)
static bool         via_hub;
static uint8_t      toggle_in, toggle_out;
static uint32_t     cbw_tag = 1;
static uint32_t     block_count_, block_size_;
static uint32_t     last_sense;
static char         drive_name[29];

uint32_t usb_msc_block_count(void) { return block_count_; }
uint32_t usb_msc_block_size(void)  { return block_size_; }
bool     usb_msc_ready(void)       { return state == ST_READY; }
uint32_t usb_msc_last_sense(void)  { return last_sense; }
const char *usb_msc_drive_name(void) { return drive_name; }

// ---------------------------------------------------------------------------
// BOT: CBW -> data -> CSW, with tier-1 (stall) recovery
// ---------------------------------------------------------------------------

// Returns MSC_OK with *csw_status filled (0 passed / 1 failed), or transport-
// level errors. Tier-2/3 recovery is the caller's job (Task 9).
static msc_result_t bot_command(const uint8_t *cb, uint8_t cb_len, bool dir_in,
                                void *data, uint32_t data_len,
                                uint8_t *csw_status, uint32_t timeout_ms) {
    uint8_t cbw[MSC_CBW_LEN], csw[MSC_CSW_LEN];
    uint32_t tag = cbw_tag++;
    uint32_t actual;
    msc_build_cbw(cbw, tag, data_len, dir_in, 0, cb, cb_len);

    hcd_result_t r = hcd_bulk_xfer(drive.addr, drive.cfg.bulk_out, &toggle_out,
                                   cbw, MSC_CBW_LEN, &actual, 1000);
    if (r == HCD_ERR_DISCONNECT) return MSC_DISCONNECTED;
    if (r != HCD_OK) return MSC_TRANSPORT_ERROR;

    bool data_stalled = false;
    if (data_len) {
        uint8_t ep = dir_in ? drive.cfg.bulk_in : drive.cfg.bulk_out;
        uint8_t *tog = dir_in ? &toggle_in : &toggle_out;
        r = hcd_bulk_xfer(drive.addr, ep, tog, data, data_len, &actual, timeout_ms);
        if (r == HCD_ERR_DISCONNECT) return MSC_DISCONNECTED;
        if (r == HCD_ERR_STALL) {
            // Tier 1: device rejected the data phase. Clear the halt and
            // fall through to read the CSW, which explains why.
            if (core_clear_endpoint_halt(drive.addr, ep) != HCD_OK)
                return MSC_TRANSPORT_ERROR;
            *tog = 0;
            data_stalled = true;
        } else if (r != HCD_OK) {
            return MSC_TRANSPORT_ERROR;
        }
    }

    r = hcd_bulk_xfer(drive.addr, drive.cfg.bulk_in, &toggle_in,
                      csw, MSC_CSW_LEN, &actual, 1000);
    if (r == HCD_ERR_STALL) {
        // CSW itself stalled: clear halt, retry CSW once (BOT spec figure 2)
        if (core_clear_endpoint_halt(drive.addr, drive.cfg.bulk_in) != HCD_OK)
            return MSC_TRANSPORT_ERROR;
        toggle_in = 0;
        r = hcd_bulk_xfer(drive.addr, drive.cfg.bulk_in, &toggle_in,
                          csw, MSC_CSW_LEN, &actual, 1000);
    }
    if (r == HCD_ERR_DISCONNECT) return MSC_DISCONNECTED;
    if (r != HCD_OK || actual != MSC_CSW_LEN) return MSC_TRANSPORT_ERROR;

    if (!msc_parse_csw(csw, tag, csw_status)) return MSC_TRANSPORT_ERROR;
    if (*csw_status == 2) return MSC_TRANSPORT_ERROR;   // phase error -> tier 2
    (void)data_stalled;
    return MSC_OK;
}

// ---------------------------------------------------------------------------
// SCSI bring-up
// ---------------------------------------------------------------------------

static msc_result_t scsi_request_sense(void) {
    uint8_t cb[6] = {0x03, 0, 0, 0, 18, 0};
    uint8_t sense[18] = {0};
    uint8_t st;
    msc_result_t r = bot_command(cb, 6, true, sense, 18, &st, 1000);
    if (r != MSC_OK) return r;
    last_sense = ((uint32_t)(sense[2] & 0x0F) << 16)   // sense key
               | ((uint32_t)sense[12] << 8)            // ASC
               | sense[13];                            // ASCQ
    return MSC_OK;
}

static msc_result_t scsi_bringup(void) {
    uint8_t st;

    // INQUIRY (36 bytes): identity string
    uint8_t inq_cb[6] = {0x12, 0, 0, 0, 36, 0};
    uint8_t inq[36] = {0};
    msc_result_t r = bot_command(inq_cb, 6, true, inq, 36, &st, 2000);
    if (r != MSC_OK) return r;
    memcpy(drive_name, inq + 8, 28);
    drive_name[28] = '\0';

    // TEST UNIT READY until the drive comes up (some sticks need ~1 s)
    uint8_t tur_cb[6] = {0};
    bool ready = false;
    for (int i = 0; i < 30; i++) {
        r = bot_command(tur_cb, 6, false, NULL, 0, &st, 1000);
        if (r != MSC_OK) return r;
        if (st == 0) { ready = true; break; }
        r = scsi_request_sense();                 // required after CHECK CONDITION
        if (r != MSC_OK) return r;
        sleep_ms(100);
    }
    if (!ready) return MSC_NOT_READY;

    // READ CAPACITY (10): last LBA + block size, big-endian
    uint8_t cap_cb[10] = {0x25, 0};
    uint8_t cap[8];
    r = bot_command(cap_cb, 10, true, cap, 8, &st, 2000);
    if (r != MSC_OK) return r;
    if (st != 0) return MSC_MEDIA_ERROR;
    block_count_ = (((uint32_t)cap[0] << 24) | ((uint32_t)cap[1] << 16) |
                    ((uint32_t)cap[2] << 8) | cap[3]) + 1;
    block_size_  = ((uint32_t)cap[4] << 24) | ((uint32_t)cap[5] << 16) |
                   ((uint32_t)cap[6] << 8) | cap[7];
    return MSC_OK;
}

// ---------------------------------------------------------------------------
// Hotplug / enumeration state machine
// ---------------------------------------------------------------------------

static absolute_time_t failed_retry_at;

static void teardown(void) {
    state = ST_DISCONNECTED;
    block_count_ = block_size_ = 0;
    toggle_in = toggle_out = 0;
    if (via_hub) usb_hub_detach();
    via_hub = false;
}

// Failed enumerations retry every 2 s while the device stays attached, so a
// drive plugged into an already-enumerated hub gets picked up.
static void enter_failed(void) {
    state = ST_FAILED;
    failed_retry_at = make_timeout_time_ms(2000);
}

void usb_msc_init(void) {
    hcd_init();
    teardown();
}

void usb_msc_task(void) {
    switch (state) {
    case ST_DISCONNECTED: {
        if (hcd_port_speed() != HCD_SPEED_FULL) return;
        sleep_ms(100);                            // attach debounce
        hcd_bus_reset();
        usb_device_t dev;
        if (core_enumerate(1, &dev) != HCD_OK) { enter_failed(); return; }

        if (dev.cfg.is_hub) {
            // Hub passthrough (Task 10): find a drive on a hub port,
            // reset it, enumerate it at address 2.
            if (usb_hub_attach(&dev) != HCD_OK) { enter_failed(); return; }
            if (usb_hub_wait_drive(&drive, 5000) != HCD_OK) { enter_failed(); return; }
            via_hub = true;
        } else if (dev.cfg.is_msc) {
            drive = dev;
            via_hub = false;
        } else {
            enter_failed();                       // not a drive, not a hub
            return;
        }
        toggle_in = toggle_out = 0;
        if (scsi_bringup() == MSC_OK) state = ST_READY;
        else enter_failed();
        return;
    }
    case ST_READY:
        if (hcd_port_speed() == HCD_SPEED_NONE) { teardown(); return; }
        if (via_hub && !usb_hub_drive_present()) { teardown(); return; }
        return;
    case ST_FAILED:
        if (hcd_port_speed() == HCD_SPEED_NONE) { teardown(); return; }
        if (time_reached(failed_retry_at)) teardown();   // retry from scratch
        return;
    }
}
```

- [ ] **Step 3: Create stub `src/usb_hub.h` so Task 8 links before Task 10 exists**

```c
#pragma once
#include "usb_core.h"

// Single-port hub passthrough (implemented in Task 10).
hcd_result_t usb_hub_attach(const usb_device_t *hub);
hcd_result_t usb_hub_wait_drive(usb_device_t *drive, uint32_t timeout_ms);
bool         usb_hub_drive_present(void);
void         usb_hub_detach(void);
```

And stub `src/usb_hub.c`:

```c
#include "usb_hub.h"

hcd_result_t usb_hub_attach(const usb_device_t *hub) { (void)hub; return HCD_ERR_TIMEOUT; }
hcd_result_t usb_hub_wait_drive(usb_device_t *drive, uint32_t timeout_ms) {
    (void)drive; (void)timeout_ms; return HCD_ERR_TIMEOUT;
}
bool usb_hub_drive_present(void) { return false; }
void usb_hub_detach(void) {}
```

Add both `src/usb_msc.c` and `src/usb_hub.c` to `CMakeLists.txt` sources.

- [ ] **Step 4: Rewrite `apps/bench/main.c` — stage 4: drive bring-up**

```c
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
```

- [ ] **Step 5: Build**

Run: `cmake --build build`
Expected: clean build.

- [ ] **Step 6: Hardware check — ASK THE USER**

Flash + plug a flash drive (directly, no hub). Expected output like:

```
drive ready: [SanDisk  Cruzer Blade    1.00]
capacity: 30031872 blocks x 512 bytes = 14664 MiB
```

This is the first real exercise of the Task 7 bulk pump (INQUIRY/CSW are short IN transfers; CBW is a short OUT). If it fails, debug the pump here — typical issues: DATA toggle desync (DATA_SEQ_ERROR status), short-packet path, LAST placement.

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "feat: MSC BOT engine, SCSI bring-up, hotplug state machine"
```

---

### Task 9: usb_msc — read/write API with the error-recovery ladder

**Files:**
- Modify: `src/usb_msc.c` (append)
- Modify: `apps/bench/main.c`

- [ ] **Step 1: Append BOT reset (tier 2) and read/write to `src/usb_msc.c`**

```c
// ---------------------------------------------------------------------------
// Tier-2 recovery: Bulk-Only Mass Storage Reset + clear both halts
// ---------------------------------------------------------------------------

static bool bot_reset_recovery(void) {
    // Class request: bmRequestType 0x21, bRequest 0xFF, wIndex = interface
    const uint8_t setup[8] = {0x21, 0xFF, 0, 0, drive.cfg.msc_itf, 0, 0, 0};
    if (hcd_control_xfer(drive.addr, setup, NULL, NULL) != HCD_OK) return false;
    if (core_clear_endpoint_halt(drive.addr, drive.cfg.bulk_in) != HCD_OK) return false;
    if (core_clear_endpoint_halt(drive.addr, drive.cfg.bulk_out) != HCD_OK) return false;
    toggle_in = toggle_out = 0;
    return true;
}

// ---------------------------------------------------------------------------
// Public read/write
// ---------------------------------------------------------------------------

#define MSC_MAX_SECTORS_PER_CMD 64u    // 32 KiB per READ(10)/WRITE(10)

static msc_result_t rw10(bool write, uint32_t lba, uint16_t count, void *buf) {
    uint8_t cb[10] = {
        write ? 0x2A : 0x28, 0,
        (uint8_t)(lba >> 24), (uint8_t)(lba >> 16),
        (uint8_t)(lba >> 8),  (uint8_t)lba,
        0,
        (uint8_t)(count >> 8), (uint8_t)count,
        0,
    };
    uint32_t bytes = (uint32_t)count * block_size_;
    uint8_t st;

    // One command, with one tier-2 retry on transport-level failure.
    for (int attempt = 0; attempt < 2; attempt++) {
        msc_result_t r = bot_command(cb, 10, !write, buf, bytes, &st, 5000);
        if (r == MSC_DISCONNECTED) return r;
        if (r == MSC_TRANSPORT_ERROR) {
            if (attempt == 0 && bot_reset_recovery()) continue;   // tier 2
            teardown();                                           // tier 3:
            return MSC_TRANSPORT_ERROR;   // next usb_msc_task() re-enumerates
        }
        if (st != 0) {                    // command failed: capture sense
            scsi_request_sense();
            return MSC_MEDIA_ERROR;
        }
        return MSC_OK;
    }
    return MSC_TRANSPORT_ERROR;
}

static msc_result_t rw(bool write, uint32_t lba, uint32_t count, void *buf) {
    if (state != ST_READY) return MSC_NOT_READY;
    if (lba + count < lba || lba + count > block_count_) return MSC_MEDIA_ERROR;
    uint8_t *p = (uint8_t *)buf;
    while (count) {
        uint16_t n = count > MSC_MAX_SECTORS_PER_CMD
                   ? (uint16_t)MSC_MAX_SECTORS_PER_CMD : (uint16_t)count;
        msc_result_t r = rw10(write, lba, n, p);
        if (r != MSC_OK) return r;
        lba += n;
        count -= n;
        p += (uint32_t)n * block_size_;
    }
    return MSC_OK;
}

msc_result_t usb_msc_read(uint32_t lba, uint32_t count, void *buf) {
    return rw(false, lba, count, buf);
}

msc_result_t usb_msc_write(uint32_t lba, uint32_t count, const void *buf) {
    return rw(true, lba, count, (void *)buf);
}
```

- [ ] **Step 2: Extend `apps/bench/main.c` — stage 5: data integrity**

Inside the `if (usb_msc_ready() && !was_ready)` block, after the capacity printf, add:

```c
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
```

Add `#include <string.h>` and update the banner to `stage 5 — integrity`.

- [ ] **Step 3: Build**

Run: `cmake --build build`
Expected: clean build.

- [ ] **Step 4: Hardware check — ASK THE USER**

Warn the user this stage **writes to the last 64 sectors of the drive** — use an expendable stick. Flash + plug in. Expected: `write ... : 0`, `read back: 0, compare: MATCH`.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: MSC read/write with three-tier error recovery"
```

---

### Task 10: usb_hub — single-port passthrough

**Files:**
- Modify: `src/usb_hcd.c` (append interrupt-endpoint functions)
- Modify: `src/usb_hub.c` (replace stub)

- [ ] **Step 1: Append interrupt-endpoint support to `src/usb_hcd.c`**

```c
// ---------------------------------------------------------------------------
// Hardware-polled interrupt endpoint (one supported; used by the hub)
// ---------------------------------------------------------------------------
// The controller polls registered interrupt endpoints autonomously every
// `interval` frames; completions appear in BUFF_STATUS. We use slot 0
// (= "interrupt endpoint 1" in register terms).
// VERIFY against datasheet (Task 4 Step 1): BUFF_STATUS bit for slot 0, and
// the host-interval field name in the EP control word.

static uint8_t  int_ep_toggle;
static uint16_t int_ep_mps;

#define INT_EP_BUF_STATUS_BIT (1u << 2)

static void int_ep_arm(void) {
    uint16_t bc = (uint16_t)(int_ep_mps | USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_LAST
                  | (int_ep_toggle ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID));
    io_rw_16 *p = (io_rw_16 *)&usbh_dpram->int_ep_buffer_ctrl[0].ctrl;
    p[0] = bc & ~USB_BUF_CTRL_AVAIL;
    busy_wait_at_least_cycles(12);
    p[0] = bc;
}

void hcd_int_ep_install(uint8_t dev_addr, uint8_t ep_addr, uint16_t mps,
                        uint8_t interval_ms) {
    int_ep_toggle = 0;
    int_ep_mps = mps;
    usb_hw->int_ep_addr_ctrl[0] = (uint32_t)dev_addr
        | ((uint32_t)(ep_addr & 0x0F) << USB_ADDR_ENDP_ENDPOINT_LSB);
    usbh_dpram->int_ep_ctrl[0].ctrl = EP_CTRL_ENABLE_BITS | EP_CTRL_INTERRUPT_PER_BUFFER
        | (USB_TRANSFER_TYPE_INTERRUPT << EP_CTRL_BUFFER_TYPE_LSB)
        | ((uint32_t)(interval_ms ? interval_ms - 1 : 0) << EP_CTRL_HOST_INTERRUPT_INTERVAL_LSB)
        | INT_EP_BUF_OFFSET;
    int_ep_arm();
    hw_set_bits(&usb_hw->int_ep_ctrl, 1u << 1);   // enable int-ep slot 0 ("EP1")
}

void hcd_int_ep_remove(void) {
    hw_clear_bits(&usb_hw->int_ep_ctrl, 1u << 1);
    usbh_dpram->int_ep_ctrl[0].ctrl = 0;
    usb_hw_clear->buf_status = INT_EP_BUF_STATUS_BIT;
}

int hcd_int_ep_poll(uint8_t *buf, uint8_t maxlen) {
    if (!(usb_hw->buf_status & INT_EP_BUF_STATUS_BIT)) return 0;
    usb_hw_clear->buf_status = INT_EP_BUF_STATUS_BIT;
    uint16_t bc = ((io_rw_16 *)&usbh_dpram->int_ep_buffer_ctrl[0].ctrl)[0];
    uint16_t n = bc & USB_BUF_CTRL_LEN_MASK;
    if (n > maxlen) n = maxlen;
    memcpy(buf, int_ep_buf, n);
    int_ep_toggle ^= 1;
    int_ep_arm();
    return (int)n;
}
```

- [ ] **Step 2: Replace `src/usb_hub.c` stub with the real driver**

```c
#include "usb_hub.h"
#include <string.h>
#include "pico/stdlib.h"

// USB 2.0 ch. 11: hub class requests and port features
enum {
    HUB_FEAT_PORT_RESET = 4,
    HUB_FEAT_PORT_POWER = 8,
    HUB_FEAT_C_PORT_CONNECTION = 16,
    HUB_FEAT_C_PORT_RESET = 20,
};
// wPortStatus bits
#define PORT_STAT_CONNECTION 0x0001u
#define PORT_STAT_LOW_SPEED  0x0200u
// wPortChange bits
#define PORT_CHG_CONNECTION  0x0001u

static usb_device_t hub;
static uint8_t      n_ports;
static int          drive_port = -1;    // 1-based; -1 = none
static bool         active;

static hcd_result_t hub_set_port_feature(uint8_t port, uint8_t feature) {
    const uint8_t setup[8] = {0x23, 3, feature, 0, port, 0, 0, 0};
    return hcd_control_xfer(hub.addr, setup, NULL, NULL);
}

static hcd_result_t hub_clear_port_feature(uint8_t port, uint8_t feature) {
    const uint8_t setup[8] = {0x23, 1, feature, 0, port, 0, 0, 0};
    return hcd_control_xfer(hub.addr, setup, NULL, NULL);
}

static hcd_result_t hub_get_port_status(uint8_t port, uint16_t *status,
                                        uint16_t *change) {
    const uint8_t setup[8] = {0xA3, 0, 0, 0, port, 0, 4, 0};
    uint8_t b[4];
    uint16_t len = 4;
    hcd_result_t r = hcd_control_xfer(hub.addr, setup, b, &len);
    if (r != HCD_OK) return r;
    if (len != 4) return HCD_ERR_DATA;
    *status = (uint16_t)(b[0] | (b[1] << 8));
    *change = (uint16_t)(b[2] | (b[3] << 8));
    return HCD_OK;
}

hcd_result_t usb_hub_attach(const usb_device_t *dev) {
    hub = *dev;
    drive_port = -1;

    // Hub descriptor (class type 0x29): bNbrPorts at offset 2,
    // bPwrOn2PwrGood (2 ms units) at offset 5
    const uint8_t setup[8] = {0xA0, 6, 0, 0x29, 0, 0, 9, 0};
    uint8_t hd[9];
    uint16_t len = 9;
    hcd_result_t r = hcd_control_xfer(hub.addr, setup, hd, &len);
    if (r != HCD_OK) return r;
    if (len < 7) return HCD_ERR_DATA;
    n_ports = hd[2];

    for (uint8_t p = 1; p <= n_ports; p++) {
        r = hub_set_port_feature(p, HUB_FEAT_PORT_POWER);
        if (r != HCD_OK) return r;
    }
    sleep_ms(2 * hd[5] + 20);   // power-on to power-good

    hcd_int_ep_install(hub.addr, hub.cfg.hub_int_ep, hub.cfg.hub_int_mps,
                       hub.cfg.hub_int_interval);
    active = true;
    return HCD_OK;
}

// Scan ports for a connected FS device; reset it; enumerate it at address 2.
hcd_result_t usb_hub_wait_drive(usb_device_t *drive, uint32_t timeout_ms) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!time_reached(deadline)) {
        for (uint8_t p = 1; p <= n_ports; p++) {
            uint16_t st, chg;
            if (hub_get_port_status(p, &st, &chg) != HCD_OK) continue;
            if (chg & PORT_CHG_CONNECTION)
                hub_clear_port_feature(p, HUB_FEAT_C_PORT_CONNECTION);
            if (!(st & PORT_STAT_CONNECTION)) continue;
            if (st & PORT_STAT_LOW_SPEED) continue;    // LS not supported

            sleep_ms(100);                              // connect debounce
            if (hub_set_port_feature(p, HUB_FEAT_PORT_RESET) != HCD_OK) continue;
            // Wait for reset-complete (C_PORT_RESET), max 500 ms
            absolute_time_t rst_deadline = make_timeout_time_ms(500);
            bool reset_done = false;
            while (!time_reached(rst_deadline)) {
                if (hub_get_port_status(p, &st, &chg) != HCD_OK) break;
                if (chg & (1u << 4)) {                  // C_PORT_RESET
                    hub_clear_port_feature(p, HUB_FEAT_C_PORT_RESET);
                    reset_done = true;
                    break;
                }
                sleep_ms(10);
            }
            if (!reset_done) continue;
            sleep_ms(20);                               // post-reset recovery

            if (core_enumerate(2, drive) == HCD_OK && drive->cfg.is_msc) {
                drive_port = p;
                return HCD_OK;
            }
        }
        sleep_ms(50);
    }
    return HCD_ERR_TIMEOUT;
}

// Called from usb_msc_task() while READY: consume hub status-change reports
// and re-check the drive's port on any change.
bool usb_hub_drive_present(void) {
    if (!active || drive_port < 0) return false;
    uint8_t bitmap[4];
    int n = hcd_int_ep_poll(bitmap, sizeof bitmap);
    if (n <= 0) return true;                            // no change reported
    // Bit (port N) of the bitmap = change on port N (bit 0 = hub itself)
    if (!(bitmap[drive_port / 8] & (1u << (drive_port % 8)))) return true;
    uint16_t st, chg;
    if (hub_get_port_status((uint8_t)drive_port, &st, &chg) != HCD_OK) return false;
    if (chg & PORT_CHG_CONNECTION)
        hub_clear_port_feature((uint8_t)drive_port, HUB_FEAT_C_PORT_CONNECTION);
    return (st & PORT_STAT_CONNECTION) != 0;
}

void usb_hub_detach(void) {
    if (active) hcd_int_ep_remove();
    active = false;
    drive_port = -1;
}
```

- [ ] **Step 3: Build**

Run: `cmake --build build`
Expected: clean build (watch the two VERIFY items from Step 1 if behavior is wrong later).

- [ ] **Step 4: Hardware check — ASK THE USER**

Flash; test all of: (a) drive direct → still works as in Task 9; (b) drive plugged into hub, hub into board → same `drive ready` + integrity MATCH output; (c) unplug drive from hub (hub stays) → `drive removed`; (d) re-plug drive into hub → comes ready again. Paste output for each.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: single-port hub passthrough with hardware-polled status endpoint"
```

---

### Task 11: Benchmark stage and acceptance

**Files:**
- Modify: `apps/bench/main.c`

- [ ] **Step 1: Add the throughput stage to `apps/bench/main.c`**

After the integrity check block (still inside the `ready && !was_ready` branch), add:

```c
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
```

Update the banner to `stage 6 — benchmark`.

- [ ] **Step 2: Build**

Run: `cmake --build build`
Expected: clean build.

- [ ] **Step 3: Acceptance run — ASK THE USER**

Flash; test with the fastest available stick, both direct and behind the hub. Paste output.

**Acceptance criteria (from spec):**
- `seq read` ≥ 1.05 MiB/s (1075 KiB/s) on a sufficiently fast drive, AND
- `packets/frame` ≥ 18.0 sustained during the data phase.

If packets/frame is high (≥18) but MiB/s is low, the drive is the bottleneck — acceptance passes on the packets/frame figure. If packets/frame is low (e.g. ~8–10), the pump is leaving bus idle: profile the poll loop (most likely cause: late half re-arm — check that both halves are primed before START_TRANS and that servicing never waits on TRANS_COMPLETE between buffers).

- [ ] **Step 4: Run full regression**

- Host tests: `ctest --test-dir build-tests --output-on-failure` → all pass.
- Hardware: direct attach ready + integrity MATCH; hub attach ready + integrity MATCH; unplug/replug both ways.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: benchmark stage; acceptance — bus-saturating sequential reads"
```

---

## Execution notes

- **Task order is strict** — each hardware stage builds on the previous one. Tasks 2–3 (pure functions) can run in parallel with Task 4 if desired; everything else is sequential.
- **Hardware-in-the-loop:** Tasks 4–6 and 8–11 end with a flash-and-observe step that requires the user. Stop and ask; do not claim the step passed without pasted UART output.
- **Debug aids:** if a hardware stage misbehaves, add temporary `printf`s of `usb_hw->sie_status` (hex) after each transfer — STALL_REC / RX_TIMEOUT / DATA_SEQ_ERROR pinpoint the failing transaction. Remove before committing.
- **Known-risk register details** (flagged in tasks; resolve once in Task 4 Step 1): host-mode BUFF_STATUS bit mapping, DPRAM 16-bit write support, `EP_CTRL_HOST_INTERRUPT_INTERVAL_LSB` field name, `int_ep_addr_ctrl`/`int_ep_ctrl` register names. All are confined to `usb_hcd.c`.





