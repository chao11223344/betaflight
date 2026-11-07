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

New captures export only `00000001.BIN`, containing the original bytes with
no timestamps, headers, checksums, padding, or delimiters. Legacy v1/v2 captures
still export their `.ULG` alongside `.BIN` so old logs remain downloadable. `STATUS.TXT`
lists each retained log's size, completeness, and dropped-byte count. If power
was lost, its valid prefix is exported and marked INCOMPLETE. Corrupt blocks
and blocks carrying another session's sequence are excluded from the export.

The disk is read-only. Deleting its files does not erase onboard storage. Large
captures are scanned before USB enumeration and may take time to appear.
Unplug/reconnect to return to recording mode after copying. Plugging in USB alone
does not switch modes, preserving access to Configurator.

## Legacy Timestamp Export (v1/v2 only)

Copy a numbered `.ULG` from the USB disk, then optionally run:

```sh
python3 src/utils/uart_logger.py export 00000001.ULG capture.bin
```

This also writes `capture.bin.json` with block timestamps, sequence, byte offsets,
and dropped counts. Output paths must not already exist. The legacy script's
MSP `download` operation is for version-1 single captures only; use USB disk
mode for the circular format. Blackbox Explorer cannot decode these custom logs.

## Timing And Failure Handling

The receive ring buffers up to 4095 bytes. New v3 captures pack incoming bytes
into a physical Flash page, retaining only 16 bytes of recovery metadata per
page. A 2048-byte page stores 2032 raw bytes (99.22% utilization); a 256-byte
page stores 240 (93.75%). There are no block or per-byte timestamps.

Sparse data remains in RAM until a page fills or recording stops. There is no
idle-time flush creating padded small records. Enter CLI/`msc` before cutting
power to save a short recording; an abrupt power loss can lose all of a short
capture that has not filled one page. The 1 kHz task queues up to 64 physical
bytes per invocation, independent of incoming protocol/frame boundaries.

Sector erases run while RX interrupts continue buffering. Slow erase/program
operations or high input rates can overflow the ring, and that loss is counted.
Hardware UART overruns, framing/parity errors, and electrical errors are not
included. Validate actual sustained throughput on the board with a known byte
sequence. Erase times depend on the installed Flash chip.

At continuous 115200 8N1, 16 MiB has a theoretical packed capacity of about
24.1 minutes with 2048-byte pages, or 22.8 minutes with 256-byte pages, before
headers, final-page padding, sector slack and whole-log eviction. At lower
actual data rates the duration increases proportionally. Retained duration
varies as complete old segments are reclaimed. The logger stops at sequence
exhaustion or invalid storage, not when normal capacity is reached.

The header is committed in its own Flash page before reception starts. Partial
NAND pages are explicitly flushed at stop. Power loss can lose the RAM receive
ring, assembled page, and pending Flash writes. Recovery exports only complete,
checksummed pages preceding the first bad page; it never guesses length from
trailing 0xFF bytes. All byte values, including long 0xFF runs, are valid input.
The last saved dropped counter is cumulative for the boot, not per file, and
may be stale after power loss. A clean final page does not imply zero drops.

## Internal Format Version 3

This is internal Flash bookkeeping, not the USB `.BIN` format. Qt reads `.BIN`
as a plain byte stream and needs no container decoder. Old ULG parsers do not
understand v3. There is no `.ULG` export for newly recorded v3 captures.

The 64-byte little-endian header uses `UARTLOG` followed by version 3. Baud,
UART, data bits, parity, stop bits remain at offsets 8..15. Offset 16 is zero
(no start timestamp). Sequence is u32 at 20; data offset is u32 at 24; physical
page size is u32 at 28. The remaining reserved bytes are zero. Offset 60 holds
CRC16/XMODEM of bytes 0..59 in a u32 (upper half zero). Header padding fills
one physical page. Supported page sizes are multiples of 64, at most 2048,
and must divide the erase sector; a sector must hold at least three pages.

Each following physical page contains raw bytes at its beginning and a
16-byte footer at its end. Relative to that footer:

| Offset | Size | Meaning |
| --- | --- | --- |
| 0 | 2 | ASCII `UR` |
| 2 | 2 | Little-endian length; bit 15 marks the final page |
| 4 | 4 | Sequence matching the file header |
| 8 | 4 | Cumulative software dropped-byte count for this boot |
| 12 | 4 | CRC16/XMODEM over all preceding bytes of this page, stored in u32 |

Non-final pages must contain exactly `pageSize - 16` payload bytes. A final
page may contain 0 through that many bytes. Unused payload bytes are zero and
included in the CRC. A full page at the segment limit is marked final directly;
normal stop may emit an empty final page if the preceding full page was already
written. Only valid payload is exported, with all page footers/padding removed.
The physical layout and page checksums allow recovery of a verified prefix
without adding metadata to downloaded raw data. Sequence and geometry checks
prevent accepting pages from another capture or incompatible Flash geometry.

Existing v1/v2 logs remain readable and participate in the same retention ring.
They do not need to be erased for this upgrade; do not use old v2 firmware to
write a Flash containing v3 data. Export first if a later downgrade needs erase.

## Legacy Format Version 2


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
power loss during recycling, cross-session page rejection, v1/v2 migration, page-aligned
headers, sparse-data packing, torn NAND pages, overflow, all-0xFF binary
fidelity, and random reads across multiple files.
HSRCF722_8C builds use the archived board configuration included with each
firmware bundle. Compilation and host tests do not replace physical-board tests.
