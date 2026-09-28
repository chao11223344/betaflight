/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <algorithm>
#include <vector>
#include <cstring>
#include "gtest/gtest.h"
extern "C" {
#include "platform.h"
#include "common/time.h"
#include "common/crc.h"
#include "io/serial.h"
#include "io/uart_logger.h"
#include "pg/serial_uart.h"
#include "pg/pg_ids.h"
}

static std::vector<uint8_t> stored, pending;
static uint32_t capacity, claims;
static bool writable;
static serialReceiveCallbackPtr receiver;
static serialPort_t port;

extern "C" {
PG_REGISTER_ARRAY(serialUartConfig_t, UARTDEV_CONFIG_MAX, serialUartConfig, PG_SERIAL_UART_CONFIG, 0);
int serialResourceIndex(serialPortIdentifier_e) { return 0; }
timeUs_t micros(void) { return 123456; }
bool flashfsIsSupported(void) { return true; }
bool flashfsIsReady(void) { return true; }
uint32_t flashfsGetOffset(void) { return stored.size() + pending.size(); }
uint32_t flashfsGetSize(void) { return capacity; }
uint32_t flashfsGetWriteBufferFreeSpace(void) { return 127 - pending.size(); }
bool flashfsFlushAsync(bool)
{
    if (writable) {
        const auto count = std::min<size_t>(16, pending.size());
        stored.insert(stored.end(), pending.begin(), pending.begin() + count);
        pending.erase(pending.begin(), pending.begin() + count);
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
        stored.clear(); pending.clear();
        capacity = 32768; claims = FUNCTION_UART_LOGGER; writable = true;
        receiver = nullptr;
        uartLoggerConfigMutable()->uart = SERIAL_PORT_USART1;
        uartLoggerConfigMutable()->baud = 115200;
        uartLoggerConfigMutable()->parity = 0;
        uartLoggerConfigMutable()->stopBits = 1;
    }
    void TearDown() override { writable = true; uartLoggerStop(); }
    void send(unsigned count)
    {
        ASSERT_NE(nullptr, receiver);
        for (unsigned i = 0; i < count; i++) receiver(i & 255, nullptr);
    }
    std::vector<uint8_t> payload()
    {
        std::vector<uint8_t> result;
        for (size_t i = 64; i < stored.size(); i += 64) {
            EXPECT_EQ('U', stored[i]); EXPECT_EQ('L', stored[i + 1]);
            uint16_t crc = crc16_ccitt_update(0, stored.data() + i, 12);
            crc = crc16_ccitt_update(crc, stored.data() + i + 16, 48);
            EXPECT_EQ(crc, stored[i + 12] | stored[i + 13] << 8);
            result.insert(result.end(), stored.begin() + i + 16,
                stored.begin() + i + 16 + stored[i + 3]);
        }
        return result;
    }
};

TEST_F(UartLoggerTest, StopDrainsBinaryDataAndPartialFlashWrites)
{
    uartLoggerInit(); send(257); uartLoggerStop();
    const auto bytes = payload();
    ASSERT_EQ(257u, bytes.size());
    for (unsigned i = 0; i < bytes.size(); i++) EXPECT_EQ(i & 255, bytes[i]);
    EXPECT_TRUE(pending.empty());
    EXPECT_EQ(2, stored[stored.size() - 62]);
    EXPECT_EQ(0u, uartLoggerDropped());
    const auto size = stored.size(); uartLoggerStop();
    EXPECT_EQ(size, stored.size());
}

TEST_F(UartLoggerTest, OverflowCountsDiscardedBytes)
{
    uartLoggerInit(); writable = false; send(5000);
    uartLoggerUpdate(); writable = true; uartLoggerStop();
    EXPECT_EQ(905u, uartLoggerDropped());
    EXPECT_EQ(4095u, payload().size());
}

TEST_F(UartLoggerTest, FullFlashPreservesEndRecordAndCountsPendingBytes)
{
    capacity = 192;
    uartLoggerInit(); send(100);
    for (unsigned i = 0; i < 30; i++) uartLoggerUpdate();
    EXPECT_EQ(capacity, stored.size());
    EXPECT_EQ(48u, payload().size());
    EXPECT_EQ(52u, uartLoggerDropped());
    EXPECT_STREQ("full", uartLoggerStatus());
}

TEST_F(UartLoggerTest, RefusesExistingLogAndConflictingPort)
{
    stored.resize(64, 0xff); uartLoggerInit();
    EXPECT_EQ(nullptr, receiver);
    EXPECT_STREQ("erase required", uartLoggerStatus());
    stored.clear(); claims |= FUNCTION_MSP; uartLoggerInit();
    EXPECT_EQ(nullptr, receiver); EXPECT_TRUE(stored.empty());
}

TEST_F(UartLoggerTest, ErasedLookingInputKeepsBootScanMarkers)
{
    uartLoggerInit();
    for (unsigned i = 0; i < 3000; i++) receiver(255, nullptr);
    uartLoggerStop();
    for (size_t i = 2048; i < stored.size(); i += 2048) EXPECT_EQ('U', stored[i]);
    const auto bytes = payload();
    EXPECT_EQ(3000u, bytes.size());
    EXPECT_TRUE(std::all_of(bytes.begin(), bytes.end(), [](uint8_t b) { return b == 255; }));
}
