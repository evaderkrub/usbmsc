# USB Mass Storage Host Driver for RP2350 — Design

**Date:** 2026-06-12
**Status:** Approved

## Goal

A from-scratch USB full-speed host driver for the RP2350 that exposes a single
USB mass-storage drive as a blocking block device, saturating the full-speed
bulk bandwidth ceiling (~19 × 64-byte packets per 1 ms frame, ≈1.16 MiB/s
theoretical payload).

## Requirements

- **Platform:** Custom RP2350 board, written in C, built with the Pico SDK
  (CMake). VBUS is hardwired on — no power switching in software.
- **From scratch:** No TinyUSB. The USB host controller driver is written
  directly against the RP2350 USB registers and DPRAM. The Pico SDK is used
  only for non-USB infrastructure (clocks, timers, UART, build system).
  Pico SDK / bootrom USB code must not touch the controller.
- **Topology:** One mass-storage drive, attached either directly or behind a
  single external hub (hub class driver included for passthrough). One drive
  active at a time; first MSC device found wins. No nested hubs. Low-speed
  devices are ignored (all MSC drives are full-speed).
- **Top of stack:** Block device API — read/write of 512-byte sectors via
  SCSI over Bulk-Only Transport. No filesystem layer.
- **API style:** Blocking. `usb_msc_read()` returns when the data is in the
  caller's buffer. A `usb_msc_task()` poll function drives hotplug and
  enumeration from the application's main loop.
- **Concurrency model:** Polled superloop. No interrupts. All hardware waits
  spin on status registers with timeouts. Chosen for debuggability
  (single-stepping works everywhere) at the cost of CPU time during waits.
- **Success bar:** Sustain ~19 bulk packets per frame on large sequential
  reads — ≥1.05 MiB/s measured against a sufficiently fast drive, with a
  packets-per-frame metric proving bus saturation independent of drive speed.

## Architecture

Five modules, strictly layered (each calls only the layer below):

```
apps/bench/main.c      demo + throughput benchmark
src/
  usb_msc.c/.h         public block API; MSC class (BOT framing + SCSI commands)
  usb_hub.c/.h         minimal hub class: port power, status, port reset, one port
  usb_core.c/.h        enumeration state machine, control transfers, pipe records
  usb_hcd.c/.h         hardware layer: registers, DPRAM, EPX pump, bus reset, speed
  usb_parse.c/.h       pure functions: descriptor parsing, CBW/CSW build+validate
tests/                 host-compiled (PC) unit tests for usb_parse
```

### usb_hcd — hardware layer and bandwidth strategy

The RP2350 host controller performs one control/bulk transfer at a time
through hardware endpoint **EPX**, plus up to 15 hardware-scheduled
**interrupt endpoints** that poll autonomously. One interrupt endpoint is
used for the hub's status-change pipe; it costs no software time during
bulk streaming.

Bandwidth comes from EPX **hardware double buffering**: two 64-byte DPRAM
buffer slots. While the SIE fills buffer 1 from the wire, software drains
buffer 0, flips its AVAILABLE bit and DATA-toggle PID, and the hardware
issues the next IN token with no idle bus time. The poll loop watches
`BUFF_STATUS`. Service deadline is ~52 µs per buffer at 19 packets/ms;
the actual service cost (64-byte copy + register writes) is well under
1 µs at 150 MHz.

The SIE handles NAK retries, CRC checking, and bus timeouts in hardware;
the pump deals only in completed buffers and error status bits.

HCD public surface (approximate):

- `hcd_init()` — controller to host mode, SOF generation on
- `hcd_port_status()` — connect/disconnect/speed from SIE_STATUS
- `hcd_bus_reset()` — drive reset, 10 ms, re-enable SOF
- `hcd_control_xfer(addr, setup, buf)` — blocking control transfer
- `hcd_bulk_xfer(addr, ep, pid_state, buf, len, timeout)` — blocking
  double-buffered bulk transfer, returns bytes transferred / error
- `hcd_int_ep_install(addr, ep, interval)` / `hcd_int_ep_check(buf)` —
  hardware-polled interrupt endpoint for the hub

DATA-toggle state per pipe is owned by the layer above and passed in/out,
keeping the HCD stateless across transfers.

### usb_core — enumeration

One explicit state machine advanced by `usb_msc_task()`:

1. Wait for connection (`SIE_STATUS.SPEED` ≠ 0).
2. Bus reset, settle delay.
3. GET_DESCRIPTOR(device, 8 bytes) at address 0 → EP0 max packet size.
4. SET_ADDRESS, then full device descriptor and full config descriptor.
5. Parse (via usb_parse). Dispatch:
   - **Hub (class 9):** SET_CONFIGURATION → hub descriptor → power all
     ports → install status-change interrupt endpoint → on port
     connect-change: PORT_RESET via hub, wait for completion, enumerate
     downstream device at address 2 with the same steps 3–5. Full-speed
     downstream devices only.
   - **MSC (interface 8/6/0x50):** SET_CONFIGURATION → record bulk IN/OUT
     endpoint addresses and max packet sizes → hand off to usb_msc.
6. Disconnect at any point (direct or hub-reported) tears down to state 1;
   `usb_msc_ready()` returns false.

### usb_msc — class driver and public API

BOT sequence per command: 31-byte CBW on bulk OUT → data phase → 13-byte
CSW on bulk IN, validated by signature and tag.

SCSI command set (minimal): INQUIRY, TEST UNIT READY + REQUEST SENSE
(retry loop during drive spin-up), READ CAPACITY(10), READ(10), WRITE(10).

Large transfers are split into **64-sector (32 KiB) READ(10)/WRITE(10)
commands** so CBW/CSW/turnaround overhead is amortized over 512 packets
(<1% protocol overhead).

Public API:

```c
void     usb_msc_init(void);
void     usb_msc_task(void);                 // call from main loop
bool     usb_msc_ready(void);
uint32_t usb_msc_block_count(void);
uint32_t usb_msc_block_size(void);           // typically 512
msc_result_t usb_msc_read (uint32_t lba, uint32_t count, void *buf);
msc_result_t usb_msc_write(uint32_t lba, uint32_t count, const void *buf);
```

`msc_result_t`: `MSC_OK`, `MSC_NOT_READY`, `MSC_MEDIA_ERROR` (CSW failed;
sense data captured), `MSC_TRANSPORT_ERROR` (USB-level failure),
`MSC_DISCONNECTED`.

## Error handling

Three escalation tiers per the MSC spec's recovery ladder:

1. **Endpoint STALL:** CLEAR_FEATURE(ENDPOINT_HALT), reset DATA toggle,
   read CSW, surface failure via REQUEST SENSE.
2. **BOT protocol confusion** (tag mismatch, malformed CSW): Bulk-Only
   Mass Storage Reset class request + clear both endpoint halts, retry
   the command once.
3. **Transport death** (timeout, repeated CRC errors, disconnect): full
   bus reset and re-enumeration; the in-flight call returns
   `MSC_TRANSPORT_ERROR` or `MSC_DISCONNECTED`.

Every hardware wait has a timeout; nothing can hang the superloop forever.

## Testing

- **Host-side unit tests** (PC build, CMake/CTest): usb_parse pure
  functions — descriptor walking, CBW construction, CSW validation —
  against crafted byte arrays including malformed/truncated input.
- **On-target bench app** (UART output), staged:
  1. Enumeration report: descriptors, drive identity (INQUIRY strings),
     capacity.
  2. Data integrity: write a known pattern to a scratch LBA region, read
     back, compare.
  3. Throughput: sequential read of 8 MiB, report MiB/s **and**
     packets-per-frame derived from SOF frame counts — the direct proof
     of bus saturation independent of drive speed.
- **Acceptance:** ≥1.05 MiB/s sequential read on a sufficiently fast
  drive; ~19 packets/frame sustained during the data phase.

## Out of scope

- Interrupt-driven or DMA operation (polled only)
- Multiple simultaneous drives, nested hubs, low-speed devices
- Non-BOT protocols (CBI), UASP, non-512-byte logical blocks beyond what
  READ CAPACITY reports
- Filesystem layer (FatFs can be bolted on later via the block API)
