# Agent Guide — usbmsc: RP2350 USB Mass-Storage Host Driver

This repo contains a **complete, hardware-verified** from-scratch USB full-speed
host driver for the RP2350 that exposes one USB flash drive as a blocking block
device. If you are here to *use* the driver (build an app, add FatFs, extend the
bench), read this file first. The design spec is in
`docs/superpowers/specs/2026-06-12-usb-msc-host-design.md`; implementation
history is in `docs/superpowers/plans/`.

Verified results (2026-06-12, real hardware): 1.12 MiB/s sequential read,
18.4 packets/frame (19 = theoretical FS max), 32 KiB write + read-back MATCH.
Do not degrade these without telling the user.

## Hardware facts you cannot discover from the code

- **The board has a hub chip soldered on it** (WCH, vid 1a86 pid 8091) between
  the RP2350 and the external USB-A port. Every attached device is *behind a
  hub*. There is no such thing as direct attach on this board. Never ask the
  user to "plug the drive in directly".
- VBUS is hardwired on; the driver fakes VBUS detect via override.
- **The board's UART is not wired to any USB-serial adapter.** Console output
  travels over **SEGGER RTT** through the Raspberry Pi Debug Probe
  (`pico_enable_stdio_rtt` is set in CMakeLists.txt). USB-serial COM ports may
  exist on the host PC, but none carries the board's stdio — don't go looking.
- Flashing is done via OpenOCD + the Debug Probe (BOOTSEL works too but the
  probe is faster and scriptable).

## Build

`PICO_SDK_PATH` may not be set globally. The official Pico VS Code extension
layout lives under `$env:USERPROFILE\.pico-sdk` (SDK 2.2.0, ARM GCC 14.2).
Every firmware configure/build needs (PowerShell):

```powershell
$env:PICO_SDK_PATH="$env:USERPROFILE\.pico-sdk\sdk\2.2.0"
$env:Path="$env:USERPROFILE\.pico-sdk\toolchain\14_2_Rel1\bin;$env:Path"
cmake -S . -B build -G Ninja     # first time only
cmake --build build              # produces build/bench.elf + bench.uf2
```

Host-side unit tests (pure-logic modules, run on the PC):

```powershell
cmake -S tests -B build-tests -G Ninja -DCMAKE_C_COMPILER=gcc   # first time
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
```

## Flash + console (RTT) recipe

Only ONE OpenOCD instance can own the probe — kill any old one first.

```powershell
Get-Process openocd -ErrorAction SilentlyContinue | Stop-Process -Force
$ocd="$env:USERPROFILE\.pico-sdk\openocd\0.12.0+dev"
& "$ocd\openocd.exe" -s "$ocd\scripts" -f interface/cmsis-dap.cfg -f target/rp2350.cfg `
  -c "adapter speed 5000" -c "init" -c "program build/bench.elf verify" -c "reset run" `
  -c "sleep 2500" -c "rtt setup 0x20000000 0x82000 {SEGGER RTT}" -c "rtt start" `
  -c "rtt server start 9090 0"
# run that in the background; then read the console with:
#   curl -s --max-time 10 telnet://localhost:9090
```

- GDB attaches on port 3333 (`arm-none-eabi-gdb build/bench.elf -ex "target extended-remote localhost:3333"`).
- If OpenOCD reports `CMSIS-DAP command CMD_INFO failed` on every attempt, the
  probe is wedged: the user must physically replug it (PnP restart needs admin).
- If the user power-cycles the board, the RTT session goes stale — restart
  OpenOCD (no reflash needed: drop the `program` line).

## Using the driver

```c
#include "usb_msc.h"

int main(void) {
    stdio_init_all();
    usb_msc_init();                      // takes over the USB controller
    for (;;) {
        usb_msc_task();                  // drives hotplug + enumeration
        if (usb_msc_ready()) {
            uint8_t buf[512];
            if (usb_msc_read(0, 1, buf) == MSC_OK) { /* sector 0 in buf */ }
        }
        sleep_ms(10);
    }
}
```

Semantics:

- `usb_msc_task()` must be called regularly from the main loop. It is cheap
  when idle, but a *connect event blocks for the whole enumeration* (hundreds
  of ms to seconds) — by design (polled superloop, see spec).
- `usb_msc_read/write(lba, count, buf)` **block** until done. Any `count` is
  fine; internally split into 32 KiB (64-sector) SCSI commands. ~28 ms per
  32 KiB chunk at full bus speed.
- Result codes: `MSC_OK`, `MSC_NOT_READY` (no drive / not enumerated),
  `MSC_MEDIA_ERROR` (drive rejected the command; `usb_msc_last_sense()` has
  key/ASC/ASCQ), `MSC_TRANSPORT_ERROR` (USB-level failure; the driver already
  tore down and will re-enumerate on the next task() call),
  `MSC_DISCONNECTED`.
- Error recovery is automatic and three-tiered (STALL clear → BOT reset →
  full re-enumeration). Callers just retry after `usb_msc_ready()` is true again.
- Adding FatFs: map `disk_read/disk_write/disk_ioctl(GET_SECTOR_COUNT/SIZE)`
  straight onto `usb_msc_read/write/block_count/block_size`. Keep FatFs off
  until `usb_msc_ready()`.

## Architecture (strict layering — each file calls only the layer below)

```
src/usb_msc.c   public API, BOT engine, SCSI, hotplug state machine
src/usb_hub.c   hub class: port power/reset, drive discovery, presence check
src/usb_core.c  enumeration, standard control requests
src/usb_hcd.c   ALL hardware access: registers, DPRAM, EPX pump, int endpoint
src/usb_parse.c pure functions (descriptor walk, CBW/CSW) — host-unit-tested
```

Rules that keep it correct:

- **No interrupts anywhere.** Everything polls, every wait has a timeout.
  Do not add IRQ handlers without revisiting every shared-flag assumption.
- **DATA toggles for the bulk pipes are owned by usb_msc.c** and passed by
  pointer into `hcd_bulk_xfer`. Toggles advance at prime time; short-IN exits
  fix them up from packets-actually-consumed. Every error path ends in a
  toggle reset (clear-halt or reset). Don't "simplify" this.
- **The one hardware interrupt endpoint is taken by the hub.** There is no
  second one in this driver; `hcd_int_ep_*` supports exactly one consumer.

## Landmines (each cost real debugging time — do not rediscover them)

1. **USB DPRAM is Device memory on the Cortex-M33.** `memcpy`/`memset` from
   newlib use unaligned accesses that **hardfault** against DPRAM
   (UNALIGNED UsageFault — verified with GDB). Every DPRAM copy must go
   through `dpram_copy()` in usb_hcd.c. If you add any DPRAM access, use it.
2. **TRANS_COMPLETE is shared** between EPX and the hardware-polled interrupt
   endpoint. The control-transfer wait disambiguates via BUFF_STATUS
   (`int_seen` parameter — sampled *before* `sie_start_transfer`); the bulk
   pump is immune by construction (`done == queued` accounting). Preserve
   both properties if you touch transfer code.
3. **EP_ABORT/EP_ABORT_DONE are device-mode-only** on RP2350. Host-mode abort
   is `SIE_CTRL.STOP_TRANS` (see `epx_abort()`).
4. `USB_TRANSFER_TYPE_*` and `usb_hw_clear` do **not** exist in the Pico SDK —
   they're defined locally in usb_hcd.c. Don't "fix" includes to find them.
5. The hub's status endpoint is installed only *after* the drive enumerates
   (usb_hub.c has the full rationale) — installing it earlier floods
   enumeration-time control transfers with spurious completions.

## Known issues (accepted, documented in the final review)

- UNIT ATTENTION after a BOT reset can surface a tier-2 retry as
  `MSC_MEDIA_ERROR` (sense 06/29/00). Workaround: retry the operation.
- `usb_hub_drive_present()` clears only connection-change bits; another
  latched change (e.g. a second device on a spare hub port) causes a
  re-report every 12 ms — functional, but wasteful. Fix is to clear every
  set change bit.
- Config descriptors > 256 bytes are rejected (by design; fails safe).
- TUR spin-up window is ~3 s — fine for sticks, short for spinning HDDs
  (they still come up via the 2 s failed-state retry loop).

## Bench app

`apps/bench/main.c` is a single evolving test app ("stage 6 — benchmark"):
bring-up report → **destructive** integrity test (overwrites the drive's last
64 sectors — warn the user before flashing it at a stick they care about) →
8 MiB sequential-read benchmark with packets/frame. Keep the packets/frame
metric in any rework: it's the honest measure of driver efficiency,
independent of how slow the stick is.
