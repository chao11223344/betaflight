/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <algorithm>
#include <vector>
#include <cstring>
#include "gtest/gtest.h"
extern "C" {
#include "platform.h"
#include "common/time.h"
#include "common/crc.h"
#include "drivers/flash/flash.h"
#include "io/serial.h"
#include "io/uart_logger.h"
#include "io/uart_logger_store.h"
#include "msc/uart_logger_files.h"
#include "pg/serial_uart.h"
#include "pg/pg_ids.h"
}

static std::vector<uint8_t> stored, pending, eraseSnapshot;
static uint32_t capacity, claims, cursor;
static bool writable, snapshotErase;
static serialReceiveCallbackPtr receiver;
static serialPort_t port;
static flashGeometry_t geometry;

extern "C" {
PG_REGISTER_ARRAY(serialUartConfig_t, UARTDEV_CONFIG_MAX, serialUartConfig, PG_SERIAL_UART_CONFIG, 0);
int serialResourceIndex(serialPortIdentifier_e) { return 0; }
timeUs_t micros(void) { return 123456; }
bool flashfsIsSupported(void) { return true; }
bool flashfsIsReady(void) { return true; }
uint32_t flashfsGetOffset(void) { return cursor + pending.size(); }
uint32_t flashfsGetSize(void) { return capacity; }
const flashGeometry_t *flashGetGeometry(void) { return &geometry; }
void flashFlush(void) { EXPECT_TRUE(pending.empty()); }
void flashfsSeekAbs(uint32_t offset) { EXPECT_TRUE(pending.empty()); cursor = offset; }
void flashfsEraseRange(uint32_t start, uint32_t end)
{
    ASSERT_EQ(0u, start % geometry.sectorSize);
    ASSERT_EQ(start + geometry.sectorSize, end);
    ASSERT_LE(end, stored.size());
    EXPECT_TRUE(pending.empty());
    std::fill(stored.begin() + start, stored.begin() + end, 255);
    if (snapshotErase) { eraseSnapshot = stored; snapshotErase = false; }
}
int flashfsReadAbs(uint32_t offset, uint8_t *dest, unsigned length)
{
    if (offset >= stored.size()) return 0;
    length = std::min<unsigned>(length, stored.size() - offset);
    memcpy(dest, stored.data() + offset, length);
    return length;
}
uint32_t flashfsGetWriteBufferFreeSpace(void) { return 127 - pending.size(); }
bool flashfsFlushAsync(bool)
{
    if (writable) {
        const auto size = std::min<size_t>(16, pending.size());
        for (unsigned i = 0; i < size; i++) {
            EXPECT_EQ(255, stored[cursor + i]) << "Programming non-erased Flash";
            stored[cursor + i] &= pending[i];
        }
        cursor += size;
        pending.erase(pending.begin(), pending.begin() + size);
    }
    return pending.empty();
}
void flashfsFlushSync(void) { flashfsFlushAsync(true); }
void flashfsWrite(const uint8_t *data, unsigned length, bool)
{
    EXPECT_LE(length, flashfsGetWriteBufferFreeSpace());
    EXPECT_LE(flashfsGetOffset() + length, capacity);
    pending.insert(pending.end(), data, data + length);
}
serialType_e serialType(serialPortIdentifier_e id)
{
    return id == SERIAL_PORT_USART1 ? SERIALTYPE_UART : SERIALTYPE_USB_VCP;
}
uint32_t serialSynthesizeFunctionMask(serialPortIdentifier_e) { return claims; }
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e function,
    serialReceiveCallbackPtr callback, void *, uint32_t baud, portMode_e mode, portOptions_e)
{
    EXPECT_EQ(FUNCTION_UART_LOGGER, function);
    EXPECT_EQ(115200u, baud);
    EXPECT_EQ(MODE_RX, mode);
    receiver = callback;
    port.mode = mode;
    return &port;
}
void closeSerialPort(serialPort_t *) { receiver = nullptr; }
}

class UartLoggerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        capacity = 32768; claims = FUNCTION_UART_LOGGER; writable = true;
        geometry = {}; geometry.sectorSize = 512; geometry.pageSize = 64;
        stored.assign(capacity, 255); pending.clear(); eraseSnapshot.clear(); cursor = 0;
        snapshotErase = false; receiver = nullptr;
        uartLoggerConfigMutable()->uart = SERIAL_PORT_USART1;
        uartLoggerConfigMutable()->baud = 115200;
        uartLoggerConfigMutable()->parity = 0;
        uartLoggerConfigMutable()->stopBits = 1;
    }
    void TearDown() override { writable = true; uartLoggerStop(); }
    void send(unsigned count, unsigned seed = 0)
    {
        ASSERT_NE(nullptr, receiver);
        for (unsigned i = 0; i < count; i++) receiver((i + seed) & 255, nullptr);
    }
    void ticks(unsigned count = 32) { while (count--) uartLoggerUpdate(); }
    std::vector<uint8_t> payload(unsigned index)
    {
        std::vector<uint8_t> result(uartLoggerFileSize(index, true));
        uartLoggerReadRaw(index, result.data(), result.size(), 0);
        return result;
    }
};

TEST_F(UartLoggerTest, StopDrainsBinaryDataAndPartialFlashWrites)
{
    uartLoggerInit(); send(257); uartLoggerStop();
    ASSERT_TRUE(uartLoggerFilesInit());
    const auto bytes = payload(0);
    ASSERT_EQ(257u, bytes.size());
    for (unsigned i = 0; i < bytes.size(); i++) EXPECT_EQ(i & 255, bytes[i]);
    EXPECT_TRUE(pending.empty());
    EXPECT_TRUE(uartLogEntry(0)->complete);
    EXPECT_EQ(0u, uartLoggerDropped());
    const auto before = stored; uartLoggerStop();
    EXPECT_EQ(before, stored);
}

TEST_F(UartLoggerTest, OverflowCountsDiscardedBytes)
{
    uartLoggerInit(); writable = false; send(5000);
    uartLoggerUpdate(); writable = true; uartLoggerStop();
    ASSERT_TRUE(uartLoggerFilesInit());
    EXPECT_EQ(905u, uartLoggerDropped());
    EXPECT_EQ(4095u, payload(0).size());
}

TEST_F(UartLoggerTest, EveryBootCreatesNextNumberAndPreservesPriorLogs)
{
    for (unsigned i = 0; i < 3; i++) {
        uartLoggerInit(); send(100 + i, i); uartLoggerStop();
    }
    ASSERT_TRUE(uartLoggerFilesInit());
    ASSERT_EQ(3u, uartLogCount());
    for (unsigned index : {2u, 0u, 1u, 2u}) {
        EXPECT_EQ(index + 1, uartLogEntry(index)->sequence);
        const auto data = payload(index);
        ASSERT_EQ(100u + index, data.size());
        for (unsigned i = 0; i < data.size(); i++) EXPECT_EQ((i + index) & 255, data[i]);
    }
}

TEST_F(UartLoggerTest, WrapRecyclesOnlyOldestLogsAndKeepsSequence)
{
    capacity = 4096; stored.assign(capacity, 255);
    for (unsigned i = 0; i < 12; i++) {
        uartLoggerInit(); send(100, i); uartLoggerStop();
    }
    ASSERT_TRUE(uartLoggerFilesInit());
    ASSERT_EQ(8u, uartLogCount());
    EXPECT_EQ(5u, uartLogEntry(0)->sequence);
    EXPECT_EQ(12u, uartLogEntry(7)->sequence);
    for (unsigned index = 0; index < 8; index++) {
        const auto data = payload(index);
        ASSERT_EQ(100u, data.size());
        EXPECT_EQ(index + 4, data[0]);
    }
}

TEST_F(UartLoggerTest, ContinuousCaptureRotatesAndRetainsNewestSuffix)
{
    capacity = 4096; stored.assign(capacity, 255);
    uartLoggerInit();
    std::vector<uint8_t> expected;
    for (unsigned i = 0; i < 100; i++) {
        send(96, i); ticks();
        for (unsigned b = 0; b < 96; b++) expected.push_back(i + b);
    }
    uartLoggerStop();
    ASSERT_TRUE(uartLoggerFilesInit());
    ASSERT_GT(uartLogEntry(0)->sequence, 1u);
    std::vector<uint8_t> actual;
    for (unsigned i = 0; i < uartLogCount(); i++) {
        const auto bytes = payload(i); actual.insert(actual.end(), bytes.begin(), bytes.end());
        EXPECT_TRUE(uartLogEntry(i)->complete);
    }
    ASSERT_LE(actual.size(), expected.size());
    EXPECT_TRUE(std::equal(actual.begin(), actual.end(), expected.end() - actual.size()));
    EXPECT_EQ(0u, uartLoggerDropped());
}

TEST_F(UartLoggerTest, InterruptedCaptureGetsNewNumberAndVerifiedPrefix)
{
    uartLoggerInit(); send(100); uartLoggerStop();
    const uint32_t footer = uartLogEntry(0)->start + uartLogEntry(0)->size - 64;
    std::fill(stored.begin() + footer, stored.begin() + footer + 64, 255);
    uartLoggerInit(); send(50); uartLoggerStop();
    ASSERT_TRUE(uartLoggerFilesInit());
    ASSERT_EQ(2u, uartLogCount());
    EXPECT_FALSE(uartLogEntry(0)->complete);
    EXPECT_EQ(100u, payload(0).size());
    EXPECT_EQ(2u, uartLogEntry(1)->sequence);
}

TEST_F(UartLoggerTest, PowerLossDuringReclamationDoesNotResetCounter)
{
    capacity = 4096; stored.assign(capacity, 255);
    for (unsigned i = 0; i < 8; i++) { uartLoggerInit(); send(10); uartLoggerStop(); }
    snapshotErase = true;
    uartLoggerInit(); uartLoggerStop();
    ASSERT_FALSE(eraseSnapshot.empty());
    stored = eraseSnapshot;
    uartLoggerInit(); send(12); uartLoggerStop();
    ASSERT_TRUE(uartLoggerFilesInit());
    EXPECT_EQ(9u, uartLogEntry(uartLogCount() - 1)->sequence);
    EXPECT_EQ(12u, payload(uartLogCount() - 1).size());
}

TEST_F(UartLoggerTest, CatalogLimitRecyclesOldestAndNumberNeverWraps)
{
    for (unsigned i = 0; i < 35; i++) { uartLoggerInit(); uartLoggerStop(); }
    ASSERT_TRUE(uartLoggerFilesInit());
    ASSERT_EQ(32u, uartLogCount());
    EXPECT_EQ(4u, uartLogEntry(0)->sequence);
    EXPECT_EQ(35u, uartLogEntry(31)->sequence);
    uint8_t *header = stored.data() + uartLogEntry(31)->start;
    uartLogPut32(header + 20, 99999999);
    uartLogPut32(header + 60, crc16_ccitt_update(0, header, 60));
    const auto before = stored;
    uartLoggerInit();
    EXPECT_EQ(nullptr, receiver);
    EXPECT_EQ(before, stored);
}

TEST_F(UartLoggerTest, RefusesForeignStorageAndConflictingPort)
{
    stored[0] = 'H'; uartLoggerInit();
    EXPECT_EQ(nullptr, receiver);
    stored[0] = 255; claims |= FUNCTION_MSP; uartLoggerInit();
    EXPECT_EQ(nullptr, receiver);
}

TEST_F(UartLoggerTest, PageSizedHeaderIsDurableBeforeAcquisition)
{
    geometry.pageSize = 2048; geometry.sectorSize = 8192;
    uartLoggerInit();
    EXPECT_TRUE(pending.empty());
    EXPECT_EQ(2048u, cursor);
    send(257); uartLoggerStop();
    ASSERT_TRUE(uartLoggerFilesInit());
    EXPECT_EQ(2048u, uartLogEntry(0)->dataOffset);
    EXPECT_EQ(257u, payload(0).size());
}

TEST_F(UartLoggerTest, UsbFilesRandomReadsAcrossLargeLogAndPadding)
{
    capacity = 262144; stored.assign(capacity, 255);
    uartLoggerInit();
    std::vector<uint8_t> expected;
    for (unsigned i = 0; i < 300; i++) {
        send(241);
        for (unsigned b = 0; b < 241; b++) expected.push_back(b);
        ticks();
    }
    uartLoggerStop();
    ASSERT_TRUE(uartLoggerFilesInit());
    ASSERT_EQ(1u, uartLogCount());
    EXPECT_EQ(expected.size(), uartLoggerFileSize(0, true));
    uint8_t result[512];
    for (unsigned offset : {70000u, 0u, 37000u, 17u, 72000u, 72290u, 100u}) {
        uartLoggerReadRaw(0, result, sizeof(result), offset);
        for (unsigned i = 0; i < sizeof(result); i++) {
            EXPECT_EQ(offset + i < expected.size() ? expected[offset + i] : 0, result[i]);
        }
    }
}

TEST_F(UartLoggerTest, UsbFilesRecoverVerifiedPrefixOnDamagedTail)
{
    uartLoggerInit(); send(100); uartLoggerStop();
    stored[128 + 16] ^= 1;
    ASSERT_TRUE(uartLoggerFilesInit());
    EXPECT_EQ(44u, uartLoggerFileSize(0, true));
    EXPECT_NE(nullptr, strstr(uartLoggerFileStatus(), "INCOMPLETE"));
}

TEST_F(UartLoggerTest, RecoveryRejectsChecksummedBlocksFromAnotherSession)
{
    uartLoggerInit(); send(44, 99); uartLoggerStop();
    std::vector<uint8_t> oldBlock(stored.begin() + 64, stored.begin() + 128);
    uartLoggerInit(); send(44, 1); uartLoggerStop();
    const auto *log = uartLogEntry(1);
    const uint32_t footer = log->start + log->size - 64;
    std::copy(oldBlock.begin(), oldBlock.end(), stored.begin() + footer);
    ASSERT_TRUE(uartLoggerFilesInit());
    EXPECT_FALSE(uartLogEntry(1)->complete);
    EXPECT_EQ(44u, payload(1).size());
    EXPECT_EQ(1, payload(1)[0]);
    uartLoggerInit(); uartLoggerStop();
    ASSERT_TRUE(uartLoggerFilesInit());
    EXPECT_EQ(3u, uartLogEntry(2)->sequence);
}

TEST_F(UartLoggerTest, LegacyVersionOneCaptureIsPreservedDuringUpgrade)
{
    uint8_t header[64] = { 'U', 'A', 'R', 'T', 'L', 'O', 'G', 1 };
    uartLogPut32(header + 60, crc16_ccitt_update(0, header, 60));
    std::copy(header, header + 64, stored.begin());
    uint8_t record[64] = { 'U', 'L', 1, 3 };
    memcpy(record + 16, "abc", 3);
    uint16_t crc = crc16_ccitt_update(0, record, 12);
    uartLogPut32(record + 12, crc16_ccitt_update(crc, record + 16, 48));
    std::copy(record, record + 64, stored.begin() + 64);
    uartLoggerInit(); send(10); uartLoggerStop();
    ASSERT_TRUE(uartLoggerFilesInit());
    ASSERT_EQ(2u, uartLogCount());
    EXPECT_EQ(0u, uartLogEntry(0)->sequence);
    EXPECT_EQ((std::vector<uint8_t>{'a', 'b', 'c'}), payload(0));
    EXPECT_EQ(1u, uartLogEntry(1)->sequence);
}

TEST_F(UartLoggerTest, FullLogReadsUseSessionOffsetsAndClampAtFileEnd)
{
    for (unsigned i = 0; i < 3; i++) { uartLoggerInit(); send(50, i); uartLoggerStop(); }
    ASSERT_TRUE(uartLoggerFilesInit());
    uint8_t result[512];
    for (unsigned index : {2u, 0u, 1u}) {
        const auto *log = uartLogEntry(index);
        uartLoggerReadLog(index, result, sizeof(result), 0);
        EXPECT_EQ(0, memcmp(result, stored.data() + log->start, log->size));
        EXPECT_EQ(log->sequence, uartLogGet32(result + 20));
        for (unsigned i = log->size; i < sizeof(result); i++) EXPECT_EQ(0, result[i]);
        uartLoggerReadLog(index, result, sizeof(result), log->size);
        EXPECT_TRUE(std::all_of(result, result + sizeof(result), [](uint8_t v) { return v == 0; }));
    }
}
