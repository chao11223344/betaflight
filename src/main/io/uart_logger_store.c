/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "platform.h"
#ifdef USE_FLASHFS
#include <string.h>
#include "common/crc.h"
#include "common/maths.h"
#include "drivers/flash/flash.h"
#include "io/flashfs.h"
#include "io/uart_logger_store.h"

static uartLogEntry_t logs[UART_LOGGER_MAX_LOGS];
static unsigned count;
static uint32_t sectorSize, volumeSize, pageSize, preparedSector, limit;
static uint8_t scanBuffer[2048];

uint32_t uartLogPageSize(void) { return pageSize; }

bool uartLogValidPage(const uint8_t *page, const uartLogEntry_t *log)
{
    const unsigned payloadSize = log->pageSize - UART_LOGGER_PAGE_METADATA;
    const uint8_t *meta = page + payloadSize;
    const unsigned length = meta[2] | (meta[3] & 0x7f) << 8;
    return meta[0] == 'U' && meta[1] == 'R' &&
        uartLogGet32(meta + 4) == log->sequence &&
        ((meta[3] & 0x80) ? length <= payloadSize : length == payloadSize) &&
        uartLogGet32(meta + 12) == crc16_ccitt_update(0, page, log->pageSize - 4);
}

uint32_t uartLogGet32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

void uartLogPut32(uint8_t *p, uint32_t value)
{
    for (unsigned i = 0; i < 4; i++) p[i] = value >> (8 * i);
}

bool uartLogValidRecord(const uint8_t *p)
{
    if (p[0] != 'U' || p[1] != 'L' ||
        !((p[2] == 1 && p[3] > 0 && p[3] <= 48) || (p[2] == 2 && p[3] == 0))) return false;
    const uint16_t crc = crc16_ccitt_update(0, p, 12);
    return uartLogGet32(p + 12) == crc16_ccitt_update(crc, p + 16, 48);
}

unsigned uartLogCount(void) { return count; }
const uartLogEntry_t *uartLogEntry(unsigned index) { return index < count ? &logs[index] : NULL; }

bool uartLogRecordMatches(const uint8_t *record, const uartLogEntry_t *log)
{
    return uartLogValidRecord(record) && (log->version == 1 ||
        (record[3] <= 44 && uartLogGet32(record + 16) == log->sequence));
}

static uint32_t alignSector(uint32_t address)
{
    return ((address + sectorSize - 1) / sectorSize) * sectorSize;
}

void uartLogFlush(void)
{
    while (!flashfsFlushAsync(true)) flashfsFlushSync();
    // NAND can still have an uncommitted partial page after FlashFS drains.
    flashFlush();
    while (!flashfsIsReady()) {}
}

static bool scanLog(uartLogEntry_t *log, uint32_t end)
{
    log->size = log->dataOffset;
    if (log->version == 3) {
        for (uint32_t address = log->start + log->dataOffset; address + log->pageSize <= end; address += log->pageSize) {
            if (flashfsReadAbs(address, scanBuffer, log->pageSize) != (int)log->pageSize) return false;
            if (!uartLogValidPage(scanBuffer, log)) return true;
            const uint8_t *meta = scanBuffer + log->pageSize - UART_LOGGER_PAGE_METADATA;
            log->size += log->pageSize;
            log->rawSize += meta[2] | (meta[3] & 0x7f) << 8;
            log->dropped = uartLogGet32(meta + 8);
            if (meta[3] & 0x80) {
                log->complete = true;
                return true;
            }
        }
        return true;
    }
    for (uint32_t address = log->start + log->dataOffset; address < end;) {
        const unsigned size = MIN(sizeof(scanBuffer), end - address);
        if (flashfsReadAbs(address, scanBuffer, size) != (int)size) return false;
        for (unsigned i = 0; i + 64 <= size; i += 64) {
            if (!uartLogRecordMatches(scanBuffer + i, log)) return true;
            log->size += 64;
            log->dropped = uartLogGet32(scanBuffer + i + 8);
            if (scanBuffer[i + 2] == 2) {
                log->complete = true;
                return true;
            }
            log->rawSize += scanBuffer[i + 3];
        }
        address += size;
    }
    return true;
}

bool uartLogScan(void)
{
    count = 0;
    preparedSector = UINT32_MAX;
    const flashGeometry_t *geometry = flashGetGeometry();
    volumeSize = flashfsGetSize();
    if (!geometry || !geometry->sectorSize || volumeSize / geometry->sectorSize < 2) return false;
    sectorSize = geometry->sectorSize;
    pageSize = MAX(64U, geometry->pageSize);
    if (sectorSize % pageSize || pageSize % 64 || pageSize > sizeof(scanBuffer) || 3 * pageSize > sectorSize) return false;
    bool nonempty = false;
    // A sector is reusable only after its previous file header is erased.
    // Recovery never trusts FlashFS's linear free-space cursor after wrapping.
    for (uint32_t address = 0; address < volumeSize; address += sectorSize) {
        if (flashfsReadAbs(address, scanBuffer, 64) != 64) return false;
        for (unsigned i = 0; i < 64; i++) nonempty |= scanBuffer[i] != 255;
        if (memcmp(scanBuffer, "UARTLOG", 7) || (scanBuffer[7] < 1 || scanBuffer[7] > 3) ||
            uartLogGet32(scanBuffer + 60) != crc16_ccitt_update(0, scanBuffer, 60)) continue;
        if (count == UART_LOGGER_MAX_LOGS) return false;
        uartLogEntry_t entry = {0};
        entry.version = scanBuffer[7];
        entry.sequence = scanBuffer[7] >= 2 ? uartLogGet32(scanBuffer + 20) : 0;
        entry.dataOffset = scanBuffer[7] >= 2 ? uartLogGet32(scanBuffer + 24) : 64;
        entry.pageSize = scanBuffer[7] == 3 ? uartLogGet32(scanBuffer + 28) : 0;
        if (entry.version == 3 && (entry.pageSize != pageSize || entry.dataOffset != pageSize)) return false;
        entry.start = address;
        if (entry.sequence > 99999999 || entry.dataOffset < 64 || entry.dataOffset % 64 || entry.dataOffset >= sectorSize) return false;
        // Validate the whole prefix before looking for more sector headers.
        // Arbitrary raw input can itself contain a checksummed UARTLOG header
        // at an erase boundary; it is still payload inside a valid page.
        if (!scanLog(&entry, volumeSize)) return false;
        unsigned pos = count++;
        while (pos && logs[pos - 1].sequence > entry.sequence) {
            logs[pos] = logs[pos - 1];
            pos--;
        }
        logs[pos] = entry;
        address = alignSector(entry.start + entry.size) - sectorSize;
    }
    if (!count) return !nonempty;
    for (unsigned i = 0; i < count; i++) {
        if (i && logs[i].sequence == logs[i - 1].sequence) return false;
    }
    return true;
}

static void eraseSector(uint32_t address)
{
    uartLogFlush();
    flashfsEraseRange(address, address + sectorSize);
    while (!flashfsIsReady()) {}
}

static void removeOldest(void)
{
    eraseSector(logs[0].start);
    memmove(logs, logs + 1, (--count) * sizeof(logs[0]));
}

static void reclaim(uint32_t address)
{
    // Files are allocated in sequence around the physical ring. Remove whole
    // oldest files before recycling any of their sectors, never just a suffix.
    bool erased = false;
    for (;;) {
        unsigned victim = count;
        for (unsigned i = 0; i < count; i++) {
            if (address >= logs[i].start && address < alignSector(logs[i].start + logs[i].size)) {
                victim = i;
                break;
            }
        }
        if (victim == count) break;
        erased |= logs[0].start == address;
        removeOldest();
    }
    if (!erased) eraseSector(address);
}

bool uartLogBegin(const uint8_t *template)
{
    if (!volumeSize || (count && logs[count - 1].sequence == 99999999)) return false;
    const uint32_t sequence = count ? logs[count - 1].sequence + 1 : 1;
    uint32_t start = count ? alignSector(logs[count - 1].start + logs[count - 1].size) : 0;
    if (start >= volumeSize) start = 0;
    if (count == UART_LOGGER_MAX_LOGS) removeOldest();
    reclaim(start);
    preparedSector = start;
    // Bound a segment to half the volume so the newest durable header always
    // survives while the next segment's sector is erased, even after power loss.
    limit = MIN(volumeSize, start + (volumeSize / sectorSize / 2) * sectorSize);
    flashfsSeekAbs(start);
    uint8_t header[64];
    memcpy(header, template, 64);
    header[7] = 3;
    uartLogPut32(header + 20, sequence);
    uartLogPut32(header + 24, pageSize);
    uartLogPut32(header + 28, pageSize);
    uartLogPut32(header + 60, crc16_ccitt_update(0, header, 60));
    flashfsWrite(header, 64, false);
    memset(header, 0, sizeof(header));
    // Commit the header in its own page before receiving bytes. In particular,
    // never reprogram a committed NAND header page when starting the payload.
    for (uint32_t offset = 64; offset < pageSize; offset += 64) {
        while (flashfsGetWriteBufferFreeSpace() < 64) flashfsFlushSync();
        flashfsWrite(header, 64, false);
    }
    uartLogFlush();
    logs[count++] = (uartLogEntry_t){ .sequence = sequence, .start = start, .size = pageSize, .dataOffset = pageSize, .pageSize = pageSize, .version = 3 };
    return true;
}

bool uartLogPrepareWrite(void)
{
    const uint32_t address = flashfsGetOffset();
    if (!count || address >= limit) return false;
    const uint32_t sector = address / sectorSize * sectorSize;
    if (sector != preparedSector) {
        reclaim(sector);
        preparedSector = sector;
    }
    return true;
}

uint32_t uartLogRemaining(void)
{
    return limit > flashfsGetOffset() ? limit - flashfsGetOffset() : 0;
}

void uartLogWritten(void)
{
    if (count) logs[count - 1].size = flashfsGetOffset() - logs[count - 1].start;
}
#endif
