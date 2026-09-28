/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "platform.h"

#if defined(USE_FLASHFS) && defined(USE_USB_MSC)
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/crc.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/utils.h"
#include "io/flashfs.h"
#include "msc/uart_logger_files.h"

#define BLOCK_SIZE 64
#define INDEX_SIZE 256
// Sparse offsets bound random access without keeping an index for every block.
static struct { uint32_t raw, flash; } indexEntries[INDEX_SIZE];
static unsigned indexCount;
static uint32_t rawSize, logSize, dataEnd;
static uint32_t cursorRaw, cursorFlash;
static uint8_t buffer[2048];
static char statusText[160];

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static bool validRecord(const uint8_t *p)
{
    if (p[0] != 'U' || p[1] != 'L' ||
        !((p[2] == 1 && p[3] > 0 && p[3] <= 48) || (p[2] == 2 && p[3] == 0))) {
        return false;
    }
    const uint16_t crc = crc16_ccitt_update(0, p, 12);
    return get32(p + 12) == crc16_ccitt_update(crc, p + 16, 48);
}

bool uartLoggerFilesInit(void)
{
    rawSize = logSize = indexCount = cursorRaw = 0;
    dataEnd = cursorFlash = BLOCK_SIZE;
    const uint32_t used = MIN(flashfsGetOffset(), flashfsGetSize());
    if (used < BLOCK_SIZE || flashfsReadAbs(0, buffer, BLOCK_SIZE) != BLOCK_SIZE ||
        memcmp(buffer, "UARTLOG\1", 8)) {
        return false;
    }
    // Preserve the full stored image even if its header or tail is damaged.
    logSize = used;
    const bool headerValid = get32(buffer + 60) == crc16_ccitt_update(0, buffer, 60);
    bool complete = false;
    uint32_t dropped = 0;
    const unsigned stride = MAX(1U, (used / BLOCK_SIZE + INDEX_SIZE - 1) / INDEX_SIZE);
    if (headerValid) {
        for (uint32_t base = 0; base < used && !complete; base += sizeof(buffer)) {
            const unsigned size = MIN(sizeof(buffer), used - base);
            if (flashfsReadAbs(base, buffer, size) != (int)size) {
                break;
            }
            bool invalid = false;
            for (unsigned pos = base == 0 ? BLOCK_SIZE : 0; pos + BLOCK_SIZE <= size; pos += BLOCK_SIZE) {
                const uint8_t *record = buffer + pos;
                if (!validRecord(record)) {
                    invalid = true;
                    break;
                }
                dropped = get32(record + 8);
                if (record[2] == 2) {
                    complete = true;
                    logSize = base + pos + BLOCK_SIZE;
                    break;
                }
                const unsigned number = (base + pos) / BLOCK_SIZE - 1;
                if (number % stride == 0 && indexCount < INDEX_SIZE) {
                    indexEntries[indexCount].raw = rawSize;
                    indexEntries[indexCount++].flash = base + pos;
                }
                rawSize += record[3];
                dataEnd = base + pos + BLOCK_SIZE;
            }
            if (invalid) {
                break;
            }
        }
    }
    tfp_sprintf(statusText, "Capture: %s\r\nRaw bytes: %u\r\nDropped bytes: %u\r\n%s",
        complete ? "complete" : "INCOMPLETE", rawSize, dropped,
        complete ? "" : "Only verified data is in CAPTURE.BIN. Keep CAPTURE.ULG for recovery.\r\n");
    return true;
}

uint32_t uartLoggerFileSize(bool raw)
{
    return raw ? rawSize : logSize;
}

const char *uartLoggerFileStatus(void)
{
    return statusText;
}

void uartLoggerReadRaw(uint8_t *dest, unsigned size, uint32_t offset)
{
    memset(dest, 0, size);
    if (offset >= rawSize) {
        return;
    }
    size = MIN(size, rawSize - offset);
    if (cursorRaw > offset) {
        cursorRaw = 0;
        cursorFlash = BLOCK_SIZE;
    }
    for (unsigned i = 0; i < indexCount && indexEntries[i].raw <= offset; i++) {
        if (indexEntries[i].raw > cursorRaw) {
            cursorRaw = indexEntries[i].raw;
            cursorFlash = indexEntries[i].flash;
        }
    }
    while (size && cursorFlash < dataEnd) {
        if (flashfsReadAbs(cursorFlash, buffer, BLOCK_SIZE) != BLOCK_SIZE || !validRecord(buffer)) {
            return;
        }
        const unsigned length = buffer[3];
        if (offset < cursorRaw + length) {
            const unsigned skip = offset - cursorRaw;
            const unsigned count = MIN(size, length - skip);
            memcpy(dest, buffer + 16 + skip, count);
            dest += count;
            size -= count;
            offset += count;
            if (skip + count < length) {
                return;
            }
        }
        cursorRaw += length;
        cursorFlash += BLOCK_SIZE;
    }
}
#endif
