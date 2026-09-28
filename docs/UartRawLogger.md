# Raw UART Logger Prototype

This branch records one hardware UART RX to the onboard FlashFS partition. It
does not parse protocols, delimiters, text, or ESC telemetry. USB MSP remains
available for downloading the capture. The feature is compiled when
`USE_FLASHFS` is enabled and defaults to disabled (`uart_logger_uart = NONE`).

## Configure

Use a board target with working onboard Flash and USB VCP MSP. Identify a free
UART RX pin and connect the source TX and ground using compatible logic levels.
UART1 is an example; it must not also be assigned to MSP, receiver, GPS, ESC
Sensor, or another function. Run `serial` to inspect assignments first.

In the Configurator CLI:

```text
set uart_logger_uart = UART1
set uart_logger_baud = 115200
set uart_logger_parity = 0
set uart_logger_stop_bits = 1
flash_erase
save
```

`flash_erase` deletes all logs in the FlashFS partition. Download anything needed
before using it. `save` restarts the board and starts recording on empty Flash.
Supply power appropriate for the board and start the external data source after
boot. USB power alone may not power all board components.

Supported settings:

| Setting | Values |
| --- | --- |
| `uart_logger_uart` | Hardware UART name, or `NONE` to disable |
| `uart_logger_baud` | 1200 to 250000, default 115200; zero refuses to start |
| `uart_logger_parity` | 0 = none, 1 = even |
| `uart_logger_stop_bits` | 1 or 2 |

Data is eight bits, non-inverted, RX only. The logger selects IRQ RX temporarily
when opening the UART to obtain explicit software-buffer overflow accounting.
The persistent UART DMA setting is preserved. STM32F405 is the initial build
target; other MCU UART implementations need hardware validation.

Enabling the logger suppresses normal Blackbox writes, blocks arming, and blocks
motor enable calls. Disabling it takes effect through the normal configuration
and reboot workflow. The logger does not drive the external device. This is a
bench capture mode; do not use it for flight.

## Stop And Download

Entering CLI stops acquisition and flushes pending bytes. `uartlog` reports
status, dropped bytes, and Flash bytes used. Reading Flash through MSP also
stops acquisition and flushes it before replying. Merely querying the MSP Flash
summary does not stop recording.

Close Configurator so it releases the USB serial port. Use the included tool:

```sh
python3 -m pip install pyserial
python3 src/utils/uart_logger.py download /dev/ttyACM0 capture.ulog
python3 src/utils/uart_logger.py export capture.ulog capture.bin
```

On Windows, substitute the USB serial name such as `COM5`. Download automatically
stops the logger with an initial Flash read, then queries the final stored length.
The exporter writes exact captured payload bytes to `capture.bin` and block
timestamps, offsets, and drop counters to `capture.bin.json`. Output paths must
not already exist. These are custom logs, not Blackbox `.bbl` files; Blackbox
Explorer and the existing MSC virtual log-file view are not supported.

To capture again, enter CLI, run `flash_erase`, then `save`. Erasing does not
automatically resume acquisition. Reboot preserves an existing log and refuses
to append; `uartlog` then reports `erase required`. Setting
`uart_logger_uart = NONE` and saving restores normal firmware behavior.

## Timing, Capacity, And Failure Handling

- A 4096-byte receive ring buffers up to 4095 bytes. The main task writes at most
  one data block per 1 ms invocation. Blocks hold up to 48 payload bytes; short
  blocks are emitted after approximately 10 ms or at stop.
- Timestamps are 32-bit microseconds at block construction, not per-byte arrival
  timestamps. They wrap after about 71.6 minutes and include buffering delay.
- Every block has a cumulative, saturating count of bytes discarded by the
  software receive ring. Pending bytes discarded when Flash fills are included
  in the final block. UART electrical/framing/parity errors and hardware overruns
  are not included in this count.
- Flash filling stops capture without overwriting old data. An end block is
  reserved. At 115200 8N1 continuously saturated input, 16 MiB holds at most about
  18 minutes with this format; short blocks increase overhead.
- No automatic erase occurs during capture. Sustainable rate depends on the
  actual Flash, UART interrupt load, and scheduler; test the chosen board with
  a known byte sequence and compare the exported binary.
- Power loss can lose the RAM tail and interrupt a Flash program operation.
  Checksums let the exporter recover the verified prefix. A missing end block
  or corrupt tail is reported as incomplete (exit code 2). Keep the `.ulog`
  image for recovery; incomplete does not imply that all earlier data is lost.

## On-Flash Format (Version 1)

All integers are little-endian. The file header and all records are 64 bytes.
Every record starts with non-erased bytes at a 64-byte boundary, including the
2048-byte boundaries used by the FlashFS boot scan. Thus arbitrary `0xff` input
cannot be mistaken for the end of the log. Boot scanning may round the used
length upward; the parser stops at the end record or erased tail.

Header: bytes 0..7 are `UARTLOG` followed by version byte 1; offset 8 is baud
(u32); offsets 12..15 are serial identifier, data bits, parity, stop bits (u8);
offset 16 is start time (u32); offset 60 is CRC16-CCITT of bytes 0..59 stored as
u32. Other bytes are zero. CRC uses polynomial 0x1021, initial value zero.

Record: offsets 0..1 are `UL`; offset 2 is type (1 data, 2 end); offset 3 is
payload length (0..48); offset 4 is time (u32); offset 8 is cumulative dropped
bytes (u32); offset 12 is CRC16-CCITT stored as u32, covering bytes 0..11 followed
by bytes 16..63; offset 16 starts payload, padded to 48 bytes with zeros. An end
record has zero payload length. No protocol framing is inferred.

## Verification

```sh
make TARGET=STM32F405 -j4
make -C src/test CC=gcc CXX=g++ test_uart_logger_unittest
python3 -m unittest discover -s src/utils -p test_uart_logger.py -v
```

Host tests cover binary fidelity, partial Flash writes, ring overflow, full
Flash, existing-log protection, UART conflicts, erased-looking input, torn
records, checksums, and MSP response validation. The generic F405 build checks
compilation; it does not establish compatibility with a particular board target.
