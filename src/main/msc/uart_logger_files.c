/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "platform.h"
#if defined(USE_FLASHFS) && defined(USE_USB_MSC)
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "common/maths.h"
#include "common/printf.h"
#include "io/flashfs.h"
#include "io/uart_logger_store.h"
#include "msc/uart_logger_files.h"

#define INDEX_SIZE 256
static struct { uint32_t raw, flash; } indexEntries[INDEX_SIZE];
static unsigned indexCount, selectedLog;
static uint32_t cursorRaw, cursorFlash;
static uint8_t buffer[2048];
static char statusText[UART_LOGGER_MAX_LOGS * 96 + 128];

bool uartLoggerFilesInit(void)
{
    selectedLog = UINT32_MAX;
    if (!uartLogScan() || !uartLogCount()) return false;
    char *text = statusText;
    text += tfp_sprintf(text, "UART captures (oldest first)\r\nINCOMPLETE means no valid end record.\r\n");
    for (unsigned i = 0; i < uartLogCount(); i++) {
        const uartLogEntry_t *log = uartLogEntry(i);
        text += tfp_sprintf(text, "%08u.BIN: %s, bytes=%u, dropped=%u\r\n",
            log->sequence, log->complete ? "complete" : "INCOMPLETE", log->rawSize, log->dropped);
    }
    return true;
}

uint32_t uartLoggerFileSize(unsigned index, bool raw)
{
    const uartLogEntry_t *log = uartLogEntry(index);
    return log ? (raw ? log->rawSize : log->size) : 0;
}

const char *uartLoggerFileStatus(void) { return statusText; }

static void selectLog(unsigned index)
{
    selectedLog = index;
    indexCount = cursorRaw = 0;
    const uartLogEntry_t *log = uartLogEntry(index);
    cursorFlash = log->start + log->dataOffset;
    const unsigned stride = MAX(1U, (log->size / 64 + INDEX_SIZE - 1) / INDEX_SIZE);
    uint32_t raw = 0;
    for (uint32_t address = cursorFlash; address < log->start + log->size;) {
        const unsigned size = MIN(sizeof(buffer), log->start + log->size - address);
        if (flashfsReadAbs(address, buffer, size) != (int)size) return;
        for (unsigned i = 0; i + 64 <= size; i += 64) {
            const uint8_t *record = buffer + i;
            if (!uartLogRecordMatches(record, log) || record[2] == 2) return;
            const unsigned number = (address + i - cursorFlash) / 64;
            if (number % stride == 0 && indexCount < INDEX_SIZE) {
                indexEntries[indexCount].raw = raw;
                indexEntries[indexCount++].flash = address + i;
            }
            raw += record[3];
        }
        address += size;
    }
}

void uartLoggerReadLog(unsigned index, uint8_t *dest, unsigned size, uint32_t offset)
{
    memset(dest, 0, size);
    const uartLogEntry_t *log = uartLogEntry(index);
    if (log && offset < log->size) {
        flashfsReadAbs(log->start + offset, dest, MIN(size, log->size - offset));
    }
}

void uartLoggerReadRaw(unsigned index, uint8_t *dest, unsigned size, uint32_t offset)
{
    memset(dest, 0, size);
    const uartLogEntry_t *log = uartLogEntry(index);
    if (!log || offset >= log->rawSize) return;
    if (selectedLog != index) selectLog(index);
    size = MIN(size, log->rawSize - offset);
    if (cursorRaw > offset) {
        cursorRaw = 0;
        cursorFlash = log->start + log->dataOffset;
    }
    for (unsigned i = 0; i < indexCount && indexEntries[i].raw <= offset; i++) {
        if (indexEntries[i].raw > cursorRaw) {
            cursorRaw = indexEntries[i].raw;
            cursorFlash = indexEntries[i].flash;
        }
    }
    while (size && cursorFlash < log->start + log->size) {
        if (flashfsReadAbs(cursorFlash, buffer, 64) != 64 || !uartLogRecordMatches(buffer, log) || buffer[2] == 2) return;
        const unsigned length = buffer[3];
        if (offset < cursorRaw + length) {
            const unsigned skip = offset - cursorRaw;
            const unsigned count = MIN(size, length - skip);
            memcpy(dest, buffer + (log->version == 2 ? 20 : 16) + skip, count);
            dest += count;
            size -= count;
            offset += count;
            if (skip + count < length) return;
        }
        cursorRaw += length;
        cursorFlash += 64;
    }
}
#endif
