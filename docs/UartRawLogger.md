# Circular Raw UART Logger

One hardware UART RX is recorded byte for byte to onboard Flash. No input
protocol is parsed. Every normal boot creates a new numbered capture. Recording
continues when storage fills by deleting the oldest logs. USB disk mode provides
numbered raw binary files that can be copied directly to a computer.

## Initial Setup

Use the updated HSRCF722_8C firmware. Connect source TX to a free UART RX and
connect grounds, using compatible 3.3 V logic. For UART1 the RX pin is PA10.
Run `serial` in Configurator CLI to confirm the selected UART is not assigned
to a receiver, GPS, MSP, ESC Sensor, or another function. Keep USB MSP enabled.

```text
set uart_logger_uart = UART1
set uart_logger_baud = 115200
set uart_logger_parity = 0
set uart_logger_stop_bits = 1
set blackbox_device = SPIFLASH
save
```

The logger defaults to disabled (`uart_logger_uart = NONE`). Supported baud is
1200..250000; zero refuses to start. Data is eight bits, non-inverted, RX only.
Parity 0 means none, 1 means even; stop bits can be 1 or 2. Supply appropriate
board power and start the data source after boot initialization completes.

No erase is needed between captures. A valid old version-1 UART log is retained
as `00000000.BIN`; new captures start at 1. Foreign Blackbox data or storage with
no recoverable UART header is not automatically reformatted. If initialization
refuses such storage, export anything needed, then use `flash_erase` and `save`.
An explicit whole-Flash erase destroys all logs and resets numbering to 1.

The feature compiles with `USE_FLASHFS`. It temporarily selects IRQ RX so
software buffer overruns can be counted, preserving the saved UART DMA setting.
Recording mode suppresses normal Blackbox writes, blocks arming, and blocks
motor enable calls. This is a bench capture mode, not a flight mode.

## Recording And Retention

- Each normal boot creates `00000001.BIN`, `00000002.BIN`, and so on. Sequence
  numbers are recovered from checksummed Flash headers and continue increasing
  when old logs are recycled. A boot with no incoming bytes still creates a log.
- Logs use as many erase sectors as needed, with a minimum of one sector per
  capture. Allocation follows a physical ring. Before reusing any sector owned
  by an old log, the oldest log is removed in its entirety.
- Up to 32 captures are retained. Reaching this directory limit also deletes
  the oldest capture, even if the Flash has free sectors. Short recordings thus
  favor a bounded directory; long recordings are limited by Flash capacity.
- A long recording automatically continues in the next numbered file at half
  the Flash capacity or the physical end of Flash, whichever comes first. This
  keeps the newest durable header intact while allocating its successor. Join
  the `.BIN` files in numeric order to reconstruct retained data across segments.
- Entering CLI stops and flushes acquisition. `uartlog` shows the sequence,
  retained-log count, state, and software dropped-byte count. Restart the board
  to begin the next capture; there is no need to erase first.
- The `msc` reboot enters download mode only and does not create another log.
  Returning to normal operation with a power cycle starts the next numbered log.
- Numbering stops rather than wraps after 99999999. Whole-Flash erase resets it.

## Copy Logs Over USB

1. Connect USB and open Configurator CLI.
2. Enter `msc`; the firmware stops recording, flushes Flash, and enters USB disk mode.
3. Open the `BETAFLT` disk in the computer's file manager.
4. Copy the desired numbered `.BIN` files to the computer.

No Python, serial downloader, or format conversion is required. The Blackbox
page's mass-storage activation action can also be used when available. Keep
`blackbox_device = SPIFLASH`, the HSRCF722_8C default.

For each sequence, `00000001.BIN` contains the original bytes and
`00000001.ULG` contains the checksummed log with block timestamps. `STATUS.TXT`
lists each retained log's size, completeness, and dropped-byte count. If power
was lost, its valid prefix is exported and marked INCOMPLETE. Corrupt blocks
and blocks carrying another session's sequence are excluded from the export.

The disk is read-only. Deleting its files does not erase onboard storage. Large
captures are scanned before USB enumeration and may take time to appear.
Unplug/reconnect to return to recording mode after copying. Plugging in USB alone
does not switch modes, preserving access to Configurator.

## Optional Timestamp Export

Copy a numbered `.ULG` from the USB disk, then optionally run:

```sh
python3 src/utils/uart_logger.py export 00000001.ULG capture.bin
```

This also writes `capture.bin.json` with block timestamps, sequence, byte offsets,
and dropped counts. Output paths must not already exist. The legacy script's
MSP `download` operation is for version-1 single captures only; use USB disk
mode for the circular format. Blackbox Explorer cannot decode these custom logs.

## Timing And Failure Handling

The receive ring buffers up to 4095 bytes. A data block holds up to 44 raw bytes;
the 1 kHz task writes at most one block per invocation, with partial blocks after
approximately 10 ms. Block timestamps are 32-bit microseconds at construction,
not individual byte arrival times; they include buffering delay and wrap after
about 71.6 minutes.

Sector erases run while RX interrupts continue buffering. Slow erase/program
operations or high input rates can overflow the ring, and that loss is counted.
Hardware UART overruns, framing/parity errors, and electrical errors are not
included. Validate actual sustained throughput on the board with a known byte
sequence. Erase times depend on the installed Flash chip.

At continuous 115200 8N1, 16 MiB holds at most roughly 16.7 minutes of raw input
before allowing for headers, sector slack, and whole-log eviction. Retained
duration varies as complete old segments are reclaimed. Short blocks add more
overhead. The logger stops at sequence exhaustion or invalid storage, not when
normal capacity is reached.

The header is committed in its own Flash page before reception starts. Partial
NAND pages are explicitly flushed at stop. Power loss can still lose the RAM
tail and a pending Flash page; an incomplete log does not imply its earlier
verified data is lost. Use `msc` before removing power for a clean end record.

## Format Version 2

All integers are little-endian. Each file begins at an erase-sector boundary.
Its first 64 bytes contain `UARTLOG` and version byte 2, baud (u32 at offset 8),
serial identifier/data bits/parity/stop bits (u8 at offsets 12..15), start time
(u32 at 16), sequence (u32 at 20), and payload start offset (u32 at 24). At offset
60 is CRC16-CCITT of bytes 0..59 stored as u32. The header area is padded to one
Flash page (at least 64 bytes) and committed before any payload records.

Records are 64 bytes: `UL` at 0..1; type at 2 (1 data, 2 end); length at 3
(0..44); timestamp at 4 (u32); dropped bytes at 8 (u32); CRC at 12 (u32); session
sequence at 16 (u32); payload at 20, padded to 44 bytes. CRC16-CCITT covers bytes
0..11 followed by 16..63, polynomial 0x1021, initial zero. End records have length
zero. A record is accepted only when its checksum and session sequence match.

Version 1 files remain readable: their payload starts at offset 64, records
carry up to 48 bytes at offset 16, and they have no sequence field. Recovery
scans sector-aligned headers independently of FlashFS's linear free-space cursor.

## Verification

```sh
make -C src/test CC=gcc CXX=g++ test_uart_logger_unittest
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/utils -p test_uart_logger.py -v
```

Tests cover reboot sequencing, circular retention, continuous segmentation,
power loss during recycling, old-block rejection, legacy migration, page-aligned
headers, overflow, binary fidelity, and random reads across multiple files.
HSRCF722_8C builds use the archived board configuration included with each
firmware bundle. Compilation and host tests do not replace physical-board tests.
