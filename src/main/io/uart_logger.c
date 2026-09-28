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
#include "pg/pg_ids.h"

PG_REGISTER_WITH_RESET_TEMPLATE(uartLoggerConfig_t, uartLoggerConfig, PG_UART_LOGGER_CONFIG, 0);
PG_RESET_TEMPLATE(uartLoggerConfig_t, uartLoggerConfig,
    .baud = 115200,
    .uart = SERIAL_PORT_NONE,
    .parity = 0,
    .stopBits = 1,
);

// One interrupt producer and one scheduler consumer. Publish indices last.
#define RX_SIZE 4096
#define PAYLOAD_SIZE 48
#define RECORD_HEADER_SIZE 16
#define RECORD_SIZE 64
static volatile uint8_t rxBuffer[RX_SIZE];
static volatile uint32_t rxHead, rxTail, dropped;
static serialPort_t *loggerPort;
static volatile bool active;
static bool ownsFlash;
static timeUs_t lastRecordTime;
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

static void writeRecord(uint8_t type, const uint8_t *payload, unsigned length)
{
    uint8_t record[RECORD_HEADER_SIZE + PAYLOAD_SIZE] = { 'U', 'L', type, length };
    put32(record + 4, micros());
    put32(record + 8, dropped);
    put32(record + 12, 0);
    if (length) {
        memcpy(record + RECORD_HEADER_SIZE, payload, length);
    }
    const uint16_t crc = crc16_ccitt_update(0, record, 12);
    put32(record + 12, crc16_ccitt_update(crc, record + 16, PAYLOAD_SIZE));
    flashfsWrite(record, sizeof(record), false);
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
    // Never append a different format to an existing Blackbox or prior capture.
    // A new capture requires an explicit erase, so reboot cannot overwrite it.
    status = "erase required";
    if (flashfsGetOffset() != 0 || flashfsGetSize() < 2 * RECORD_SIZE) {
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
    // Every 64-byte record begins with a non-erased marker. This preserves
    // flashfs boot scanning even when the raw input is entirely 0xff.
    uint8_t header[RECORD_SIZE] = { 'U', 'A', 'R', 'T', 'L', 'O', 'G', 1 };
    put32(header + 8, uartLoggerConfig()->baud);
    header[12] = port;
    header[13] = 8;
    header[14] = uartLoggerConfig()->parity;
    header[15] = uartLoggerConfig()->stopBits;
    put32(header + 16, micros());
    put32(header + 60, crc16_ccitt_update(0, header, 60));
    flashfsWrite(header, sizeof(header), false);
    ownsFlash = true;
    active = true;
    lastRecordTime = micros();
    status = "recording";
}

static bool drain(void)
{
    const unsigned count = (rxHead + RX_SIZE - rxTail) % RX_SIZE;
    if (!count) {
        return false;
    }
    const unsigned length = count < PAYLOAD_SIZE ? count : PAYLOAD_SIZE;
    // Reserve space for a final record, even when the flash fills.
    if (flashfsGetSize() - flashfsGetOffset() < 2 * RECORD_SIZE) {
        active = false;
        status = "full";
        return false;
    }
    if (flashfsGetWriteBufferFreeSpace() < RECORD_SIZE) {
        return false;
    }
    uint8_t payload[PAYLOAD_SIZE];
    for (unsigned i = 0; i < length; i++) {
        payload[i] = rxBuffer[rxTail];
        rxTail = (rxTail + 1) % RX_SIZE;
    }
    writeRecord(1, payload, length);
    lastRecordTime = micros();
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
    while (drain()) {
        while (!flashfsFlushAsync(true)) {
            flashfsFlushSync();
        }
    }
    const uint32_t pending = (rxHead + RX_SIZE - rxTail) % RX_SIZE;
    dropped = UINT32_MAX - dropped < pending ? UINT32_MAX : dropped + pending;
    rxTail = rxHead;
    if (flashfsGetSize() - flashfsGetOffset() >= RECORD_SIZE) {
        writeRecord(2, NULL, 0);
    }
    while (!flashfsFlushAsync(true)) {
        flashfsFlushSync();
    }
    while (!flashfsIsReady()) {}
    ownsFlash = false;
}

void uartLoggerUpdate(void)
{
    if (!ownsFlash) {
        return;
    }
    flashfsFlushAsync(true);
    const unsigned count = (rxHead + RX_SIZE - rxTail) % RX_SIZE;
    if (count >= PAYLOAD_SIZE || (timeUs_t)(micros() - lastRecordTime) >= 10000) {
        drain();
    }
    if (!active) {
        uartLoggerStop();
    }
}
#endif
