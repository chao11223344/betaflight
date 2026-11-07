/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "platform.h"

#ifdef USE_FLASHFS
#include "common/time.h"
#include "common/crc.h"
#include "drivers/time.h"
#ifdef USE_UART
#include "drivers/dma_reqmap.h"
#include "pg/serial_uart.h"
#endif
#include "io/flashfs.h"
#include "io/serial.h"
#include "io/serial_feature_map.h"
#include "io/uart_logger.h"
#include "io/uart_logger_store.h"
#include "pg/pg_ids.h"

PG_REGISTER_WITH_RESET_TEMPLATE(uartLoggerConfig_t, uartLoggerConfig, PG_UART_LOGGER_CONFIG, 0);
PG_RESET_TEMPLATE(uartLoggerConfig_t, uartLoggerConfig,
    .baud = 115200,
    .uart = SERIAL_PORT_NONE,
    .parity = 0,
    .stopBits = 1,
);

// One interrupt producer and one scheduler consumer. Publish indices last.
#define RX_SIZE 32768
#define RECORD_SIZE 64
// IRQ RX is CPU-only, so F405 CCM is safe and leaves DMA-capable SRAM free.
// Cover sector erase stalls (32767 bytes ~= 2.84 s at 115200 8N1).
// Other targets (notably F722) may already fill their small fast RAM region.
#if defined(STM32F405)
#define UART_LOGGER_RX_ATTRIBUTE FAST_DATA_ZERO_INIT
#else
#define UART_LOGGER_RX_ATTRIBUTE
#endif
static UART_LOGGER_RX_ATTRIBUTE volatile uint8_t rxBuffer[RX_SIZE];
static volatile uint32_t rxHead, rxTail, dropped;
static serialPort_t *loggerPort;
static volatile bool active;
static bool ownsFlash;
static uint8_t page[UART_LOGGER_MAX_PAGE_SIZE];
static unsigned payloadLength, pageSent;
static bool pageReady, pageEndsLog, sealed;
static const char *status = "disabled";

const char *uartLoggerStatus(void)
{
    return status;
}

uint32_t uartLoggerDropped(void)
{
    return dropped;
}

static void put32(uint8_t *p, uint32_t v)
{
    for (unsigned i = 0; i < 4; i++) {
        p[i] = v >> (8 * i);
    }
}

bool uartLoggerEnabled(void)
{
    return uartLoggerConfig()->uart != SERIAL_PORT_NONE;
}

static void receive(uint16_t data, void *context)
{
    (void)context;
    if (!active) {
        return;
    }
    const uint32_t next = (rxHead + 1) % RX_SIZE;
    if (next == rxTail) {
        if (dropped != UINT32_MAX) {
            dropped++;
        }
        return;
    }
    rxBuffer[rxHead] = data;
    rxHead = next;
}

static bool beginLog(void)
{
    uint8_t header[RECORD_SIZE] = { 'U', 'A', 'R', 'T', 'L', 'O', 'G', 3 };
    put32(header + 8, uartLoggerConfig()->baud);
    header[12] = uartLoggerConfig()->uart;
    header[13] = 8;
    header[14] = uartLoggerConfig()->parity;
    header[15] = uartLoggerConfig()->stopBits;
    if (!uartLogBegin(header)) return false;
    payloadLength = pageSent = 0;
    pageReady = pageEndsLog = sealed = false;
    return true;
}

void uartLoggerInit(void)
{
    if (ownsFlash) {
        return;
    }
    status = "disabled";
    if (!uartLoggerEnabled()) {
        return;
    }
    status = "flash unavailable";
    if (!flashfsIsSupported() || !flashfsIsReady()) {
        return;
    }
    const serialPortIdentifier_e port = uartLoggerConfig()->uart;
    status = "invalid baud";
    if (uartLoggerConfig()->baud < 1200 || uartLoggerConfig()->baud > 250000) {
        return;
    }
    status = "UART unavailable or assigned to another function";
    if (serialType(port) != SERIALTYPE_UART ||
        serialSynthesizeFunctionMask(port) != FUNCTION_UART_LOGGER) {
        return;
    }
    status = "unsupported or damaged storage; export logs before erasing";
    if (!uartLogScan()) {
        return;
    }
    status = "UART unavailable";
    rxHead = rxTail = dropped = 0;
#ifdef USE_UART
    const int resourceIndex = serialResourceIndex(port);
    if (resourceIndex < 0 || resourceIndex >= UARTDEV_CONFIG_MAX) {
        return;
    }
    // Callback delivery requires IRQ RX on STM32F405; leave persistent config alone.
    const int8_t rxDma = serialUartConfig(resourceIndex)->rxDmaopt;
    serialUartConfigMutable(resourceIndex)->rxDmaopt = DMA_OPT_UNUSED;
#endif
    portOptions_e options = SERIAL_NOT_INVERTED;
    if (uartLoggerConfig()->parity) {
        options |= SERIAL_PARITY_EVEN;
    }
    if (uartLoggerConfig()->stopBits == 2) {
        options |= SERIAL_STOPBITS_2;
    }
    loggerPort = openSerialPort(port, FUNCTION_UART_LOGGER, receive, NULL,
        uartLoggerConfig()->baud, MODE_RX, options);
#ifdef USE_UART
    serialUartConfigMutable(resourceIndex)->rxDmaopt = rxDma;
#endif
    if (!loggerPort) {
        return;
    }
    if (!(loggerPort->mode & MODE_RX)) {
        closeSerialPort(loggerPort);
        loggerPort = NULL;
        return;
    }
    if (!beginLog()) {
        closeSerialPort(loggerPort);
        loggerPort = NULL;
        status = "log sequence exhausted";
        return;
    }
    ownsFlash = true;
    active = true;
    status = "recording";
}

// Assemble an entire physical page before programming it. Sparse input stays
// in RAM instead of creating padded records every 10 ms. Stop seals a partial
// page; full pages carry only a 16-byte recovery footer, with no timestamps.
static bool drain(bool finish)
{
    unsigned count = (rxHead + RX_SIZE - rxTail) % RX_SIZE;
    if (sealed) {
        if (!count) return false;
        uartLogFlush();
        if (!beginLog()) {
            active = false;
            status = "log sequence exhausted";
            return false;
        }
    }
    const unsigned size = uartLogPageSize();
    const unsigned capacity = size - UART_LOGGER_PAGE_METADATA;
    if (!pageReady) {
        while (count && payloadLength < capacity) {
            page[payloadLength++] = rxBuffer[rxTail];
            rxTail = (rxTail + 1) % RX_SIZE;
            count--;
        }
        if (payloadLength < capacity && !finish) return false;
        pageEndsLog = (finish && !count) || uartLogRemaining() == size;
        memset(page + payloadLength, 0, size - payloadLength);
        uint8_t *meta = page + capacity;
        meta[0] = 'U';
        meta[1] = 'R';
        meta[2] = payloadLength;
        meta[3] = (payloadLength >> 8) | (pageEndsLog ? 0x80 : 0);
        put32(meta + 4, uartLogEntry(uartLogCount() - 1)->sequence);
        put32(meta + 8, dropped);
        put32(meta + 12, crc16_ccitt_update(0, page, size - 4));
        pageReady = true;
    }
    const unsigned free = flashfsGetWriteBufferFreeSpace();
    if (free < RECORD_SIZE) return false;
    if (!uartLogPrepareWrite()) return false;
    // Queue as much of this page as fits, rather than programming a NOR page
    // in four separate 64-byte operations. Never cross a physical page here.
    const unsigned remaining = size - pageSent;
    const unsigned length = remaining < free ? remaining : free / RECORD_SIZE * RECORD_SIZE;
    flashfsWrite(page + pageSent, length, false);
    pageSent += length;
    if (pageSent == size) {
        uartLogWritten();
        sealed = pageEndsLog;
        pageReady = false;
        payloadLength = pageSent = 0;
    }
    return true;
}

void uartLoggerStop(void)
{
    if (!ownsFlash) {
        return;
    }
    if (active) {
        status = "stopped";
    }
    active = false;
    if (loggerPort) {
        closeSerialPort(loggerPort);
        loggerPort = NULL;
    }
    // Called from USB/CLI before reads or erases, never from the RX interrupt.
    while (!flashfsFlushAsync(true)) {
        flashfsFlushSync();
    }
    while (drain(true)) {
        while (!flashfsFlushAsync(true)) {
            flashfsFlushSync();
        }
    }
    const uint32_t pending = (rxHead + RX_SIZE - rxTail) % RX_SIZE;
    dropped = UINT32_MAX - dropped < pending ? UINT32_MAX : dropped + pending;
    rxTail = rxHead;
    while (!flashfsFlushAsync(true)) {
        flashfsFlushSync();
    }
    uartLogFlush();
    ownsFlash = false;
}

void uartLoggerUpdate(void)
{
    if (!ownsFlash) {
        return;
    }
    flashfsFlushAsync(true);
    drain(false);
    if (!active) {
        uartLoggerStop();
    }
}
#endif
