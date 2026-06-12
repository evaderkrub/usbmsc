# usbmsc — a from-scratch USB mass-storage host driver for the RP2350

A USB full-speed **host** driver for the Raspberry Pi RP2350 that mounts a USB
flash drive as a simple block device — written from scratch against the USB
controller registers, with no TinyUSB. The Pico SDK is used only for
infrastructure (clocks, timers, build system); every USB byte on the wire is
this repo's own doing.

## Why

Mostly to see how fast full-speed USB can really go when the host controller
is driven directly. Answer: all the way.

| Metric | Result | Ceiling |
|---|---|---|
| Sequential read | **1.12 MiB/s** | ~1.16 MiB/s payload theoretical |
| Bus utilization | **18.4 packets/frame** | 19 packets/frame |
| Write integrity | 32 KiB write → read-back: bit-exact | — |

USB full-speed signals at 12 Mbit/s, but tokens, CRCs, handshakes and
inter-packet gaps mean the payload ceiling for bulk transfers is 19 × 64-byte
packets per 1 ms frame ≈ 1.16 MiB/s. At 18.4 packets/frame, the bus is
saturated — the residue is command turnaround and the flash stick thinking,
not the driver idling.

## What it supports

- One USB mass-storage drive (Bulk-Only Transport / transparent SCSI), plugged
  in directly **or behind a hub** (single hub tier, first drive wins)
- Hotplug: connect/disconnect any time; the driver re-enumerates by itself
- Blocking reads and writes of 512-byte sectors, any transfer size
- Three-tier automatic error recovery (endpoint STALL → BOT reset → full bus
  reset + re-enumeration), every wait bounded by a timeout
- Polled operation — **no interrupts anywhere**, which makes the whole driver
  single-steppable in a debugger

Out of scope by design: multiple drives, nested hubs, low-speed devices,
UASP, and filesystems (FatFs drops straight onto the block API if you want
one).

## Using it

```c
#include "usb_msc.h"

int main(void) {
    stdio_init_all();
    usb_msc_init();
    for (;;) {
        usb_msc_task();                       // drives hotplug + enumeration
        if (usb_msc_ready()) {
            static uint8_t sector[512];
            if (usb_msc_read(0, 1, sector) == MSC_OK) {
                // sector 0 of the drive is in `sector`
            }
        }
        sleep_ms(10);
    }
}
```

Call `usb_msc_task()` from your main loop. Once `usb_msc_ready()` returns
true, `usb_msc_read(lba, count, buf)` and `usb_msc_write(lba, count, buf)`
block until the data has moved; large transfers are split into 32 KiB SCSI
commands internally. `usb_msc_block_count()` / `usb_msc_block_size()` give
the drive geometry, `usb_msc_drive_name()` the INQUIRY identity string, and
`usb_msc_last_sense()` the SCSI sense code after a media error.

## Building

Requires the [Pico SDK](https://github.com/raspberrypi/pico-sdk) ≥ 2.0 and an
ARM GCC toolchain.

```sh
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S . -B build -G Ninja
cmake --build build          # → build/bench.uf2
```

The pure-logic modules (descriptor parsing, CBW/CSW framing) have host-side
unit tests that run on your PC:

```sh
cmake -S tests -B build-tests -G Ninja
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
```

`apps/bench/main.c` is the demo/test app: it reports the enumerated drive,
runs a write/read-back integrity check (**destructive** — it overwrites the
drive's last 64 sectors, use an expendable stick), then measures sequential
read throughput and packets-per-frame.

## How it works

```
usb_msc.c    public block API · BOT (CBW/data/CSW) · SCSI · hotplug state machine
usb_hub.c    hub class driver: port power, port reset, drive discovery
usb_core.c   enumeration and standard control requests
usb_hcd.c    the hardware layer: registers, DPRAM, transfer engines
usb_parse.c  pure functions: descriptor walking, CBW/CSW framing (unit-tested)
```

Strictly layered — each file only calls the one below it.

**Where the speed comes from.** The RP2350's host controller has one
transfer endpoint (EPX) with hardware double buffering: two 64-byte slots in
USB DPRAM. Both are primed before the transfer starts; while the hardware
fills one slot from the wire, the poll loop drains and re-arms the other, so
the controller issues the next IN token with zero idle bus time. On top of
that, reads and writes use 64-sector (32 KiB) SCSI commands, so the
per-command overhead (CBW, CSW, turnaround) is amortized over 512 packets.
The service deadline per buffer is ~52 µs; servicing takes well under 1 µs
at 150 MHz, so a polled loop keeps up with room to spare.

**Hub status without software cost.** The controller can hardware-poll up to
15 interrupt endpoints autonomously. The hub's status-change pipe rides one
of those, so during bulk streaming the hub costs no software time at all.

## Hard-won lessons (read before hacking on it)

- **USB DPRAM is Device memory on the Cortex-M33.** Newlib's optimized
  `memcpy` uses unaligned halfword accesses for odd-length tails — legal on
  SRAM, an instant UsageFault against DPRAM. All DPRAM traffic goes through
  the alignment-safe `dpram_copy()` in `usb_hcd.c`. (This was the project's
  one hardfault, found with GDB on the very first 31-byte CBW.)
- The RP2350's `EP_ABORT`/`EP_ABORT_DONE` registers are device-mode only;
  host-mode transfer cancellation is `SIE_CTRL.STOP_TRANS`.
- `TRANS_COMPLETE` is shared between EPX and the hardware-polled interrupt
  endpoints — completion handling must attribute it, or hub reports corrupt
  in-flight control transfers.
- DATA toggles advance at buffer-prime time, not packet-completion time; a
  transfer that ends in a short packet needs its toggle recomputed from
  packets actually consumed.

More operational detail (debug-probe workflow, board-specific facts, known
issues) lives in [AGENTS.md](AGENTS.md). Design rationale is in
[docs/superpowers/specs/](docs/superpowers/specs/).

## Status

Complete and hardware-verified (2026-06-12) on a custom RP2350 board with an
on-board hub and a SanDisk flash drive. See the known-issues list in
AGENTS.md for accepted minor limitations.
