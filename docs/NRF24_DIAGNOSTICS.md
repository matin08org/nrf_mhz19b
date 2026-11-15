# CO2 radio link, protocol v1

Both peers must be upgraded together. Legacy `CO2=...` text packets are rejected;
they must not become unvalidated numeric data. The receiver retains the latest
valid ppm on a sensor error but exposes the current error/freshness separately.

## Shared radio profile

- Address bytes in register/SPI order: `EE DD CC BB AA`.
- Channel 10 (2410 MHz), 1 Mbps, 0 dBm; 5-byte address.
- Fixed 32-byte payload, 2-byte radio CRC, Auto-ACK enabled on pipe 0.
- 750 us delay and 15 hardware retries (`SETUP_RETR=0x2F`).
- Dynamic payload and ACK payload disabled.
- Sender config `0x0E`; receiver config `0x0F`; RF_SETUP `0x06`.
- SPI Mode 0, 8-bit, MSB first, software CSN, 1 MHz.
- F030 sender: SPI1 PA5/PA6/PA7, CE PA3, CSN PA4.
- F412 receiver: SPI5 PB0/PA12/PA10, CE PA11, CSN PA15.

`NRF24L01.c`, `NRF24L01.h`, and `co2_link.h` are identical on both boards.
Only `nrf24_board.h` selects the SPI handle and bus clock. Copy changes to these
shared files together; host tests exercise both virtual peers using this code.

## Execution and failure handling

`NRF24_BoardInit` validates local SPI configuration and schedules radio startup.
Call `NRF24_Process` from one main/task owner. No other task or IRQ accesses this
radio or its SPI peripheral. The main loop owns the radio statistics and receive
cache; they are not concurrently updated from IRQ callbacks. A future RTOS
consumer must synchronize copies or consume snapshots in that owner task.

Power-on margin is 100 ms, power-up margin 5 ms. CE pulses use a cooperative
state (>=1 ms actual). No HAL_Delay or RF status busy loop exists in the driver.
TX polls at most once/ms and times out after 100 ms. RX drains at most 3 frames
per Process; the 3-frame application ring drops oldest on overflow, preserving
new telemetry and counting the loss. Polling FIFO rather than depending solely
on RX_DR handles arrivals racing flag clears. Normal reception never FLUSH_RXs
unread frames. Init/recovery deliberately flushes old radio FIFO contents.

SPI transactions remain synchronous and bounded, with one byte outstanding at
a time and CSN held low over the entire command. This avoids the per-byte IRQ
latency/overrun problem of external-flash execution. Each byte has a 2 ms HAL
timeout and each command a 5 ms elapsed-time budget checked between bytes.
Healthy frames take wire time plus software overhead; this is not fully DMA /
IRQ-asynchronous SPI. A stalled byte can exceed the command budget by its one
remaining 2 ms timeout. No sensor-conversion or ACK wait blocks the main loop.

Startup probes RF_CH with two patterns, verifies profile/address readbacks,
and rejects impossible STATUS bit 7 (including floating 0xFF MISO). Idle/RX
health checks verify CONFIG/channel/rate once/sec. Communication/configuration
faults deassert CE, reset SPI state on transport failure, and schedule retry
once/sec. Fault recovery discards queued telemetry; the app rebuilds a fresh
report. Queue is a latest-unsent slot, not guaranteed delivery storage.

Success requires TX_DS with Auto-ACK enabled. MAX_RT, timeout and SPI errors
are distinct; FIFO empty alone never indicates delivery. Radio ACK proves
peer radio reception, not application acceptance or LCD rendering.

## 32-byte application packet

All multi-byte integers are explicitly little endian; no packed C structure is
sent over the air. Offsets:

| Offset | Meaning |
|---|---|
| 0..1 | ASCII `NR` magic |
| 2 | Version, 1 |
| 3 | Type: 1 telemetry, 2 link test |
| 4 | Bit 0: has a valid measurement in history; other bits zero |
| 5 | Sensor status: 0 good, 1 warm-up, 2 timeout, 3 UART error, 4 bad header, 5 bad checksum, 6 not ready |
| 6..7 | Node ID, 1 |
| 8..11 | Packet sequence |
| 12..15 | Sender uptime in ms |
| 16..19 | Measurement age at report creation; UINT32_MAX before first good sample |
| 20..21 | Last valid CO2 ppm |
| 22..23 | Reserved, zero |
| 24..27 | Count of good sensor measurements |
| 28..29 | Reserved, zero |
| 30..31 | CRC16-CCITT-FALSE over bytes 0..29 (poly 0x1021, init 0xFFFF) |

Decoder checks magic/version/type, reserved fields, node and application CRC.
The application CRC is separate from the radio CRC; neither authenticates or
encrypts the link. This is a single trusted sender/node protocol, not an
anti-replay or adversarial radio security scheme.

Receiver deduplicates sequence+uptime without refreshing an old measurement's
sample timestamp. Sequence reset after a sender reboot and 32-bit tick wraps
are tolerated. Error reports preserve last good ppm and sample time. Data is
usable only when the latest accepted report has a good, present, <=6 s old
measurement and both local sample/link ages are <=6 s. Packet-carried sample
age prevents a fresh arrival from making an explicitly old sample fresh.
Timing is approximate: age is measured at report creation, not synchronized
one-way RF time, and the cooperative caller must service Process regularly.

## Application and diagnostics

The MH-Z19 UART DMA state machine remains intact. Warm-up is 180 s; measurement
and report periods are 2 s. Warm-up/error frames are transmitted too; no fake
zero ppm is presented as a valid sample. Sender uses WFI between SysTick/UART
interrupts. Debug `radio` exposes state, ready/detected, last result and stats:
SPI/config failures, recoveries, ACKs, MAX_RT, timeouts, completions and RX drops.
`watch_nrf_link_ack` is true only for actual ACK success.

Set `watch_nrf_test_request=1` to enqueue one protocol test frame. Observe
`watch_nrf_test_done` and `watch_nrf_test_result` (1 ACK, 2 MAX_RT, 3 SPI error,
4 timeout); a matching receiver counts it without replacing the cached CO2.
On the receiver, `co2_cache` / `CO2_GetSnapshot` provide ppm, status, sample/link
ages, validity and rejection/duplicate counters without doing SPI. The LCD
shows ppm, NRF OFF, NO LINK, WARMUP, CO2 ERR or OLD, never arbitrary radio text.

## Build and test

- Keil project remains supported with its original Cortex-M0 device/settings;
  install `Keil::STM32F0xx_DFP` and `ARM::CMSIS` packs first.
- Portable GNU build: `./tools/build.ps1 -ToolchainBin <GNU Arm bin directory>`.
  Linker limits are explicitly 16 KB Flash / 4 KB RAM, with 512 bytes reserved
  for stack. Outputs: `build/CO2_Node.elf`, `.hex`, `.bin`.
- Host tests: `./tools/test.ps1 -Compiler gcc`, or run the workflow's gcc command.
  They compile the real shared driver/protocol against a fake radio transport.

Tests cover profile matching, ACK/MAX_RT/timeout classification, missing chip /
SPI fault recovery, bounded RX draining, ring overflow, preserving FIFO packets,
all 256 single-bit corruptions of a packet, measurement age, error preservation,
duplicates, sender reboot and tick rollover. ARM builds test family/HAL portability
and memory budgets. CI runs the host tests under AddressSanitizer and UBSan.

No physical two-board RF test has been performed for this change. After
programming both peers, test a known CO2 report, warm-up/error displays, a link
test ACK, receiver off/MAX_RT, sender restart, lost sensor and >6 s stale state.
Check real CSN/CE/SCK timing, power integrity and radio range separately.
