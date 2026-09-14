#include <cstring>

extern "C" {
#include "platform.h"
#include "common/axis.h"
#include "drivers/serial.h"
#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "io/gps.h"
#include "io/serial.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "rx/rx.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "telemetry/custom.h"
#include "telemetry/telemetry.h"

PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
gyro_t gyro;
acc_t acc;
gpsSolutionData_t gpsSol;
rxRuntimeState_t rxRuntimeState;
uint8_t armingFlags;
uint8_t stateFlags;
serialPort_t *telemetrySharedPort;
const uint32_t baudRates[BAUD_COUNT] = {};

static uint32_t nowMs;
static uint8_t captured[44];
static int capturedLength;
static serialPort_t port;

timeMs_t millis(void) { return nowMs; }
bool sensors(uint32_t) { return false; }
bool gyroIsCalibrationComplete(void) { return false; }
bool baroIsCalibrated(void) { return false; }
float getBaroAltitude(void) { return 0; }
serialPortIdentifier_e telemetryProviderPort(uint8_t) { return SERIAL_PORT_USART1; }
uint8_t telemetryProviderBaud(uint8_t, serialPortIdentifier_e) { return BAUD_115200; }
portSharing_e determinePortSharing(serialPortIdentifier_e, serialPortFunction_e) { return PORTSHARING_NOT_SHARED; }
bool telemetryCheckRxPortShared(serialPortIdentifier_e, SerialRXType) { return false; }
bool telemetryDetermineEnabledState(portSharing_e) { return true; }
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr,
    void *, uint32_t, portMode_e, portOptions_e) { return &port; }
void closeSerialPort(serialPort_t *) {}
uint32_t serialTxBytesFree(const serialPort_t *) { return sizeof(captured); }
void serialWriteBuf(serialPort_t *, const uint8_t *data, int count)
{
    capturedLength = count;
    if (count <= (int)sizeof(captured)) {
        memcpy(captured, data, count);
    }
}
}

#include "gtest/gtest.h"

static uint32_t readU32(unsigned offset)
{
    return (uint32_t)captured[offset] | ((uint32_t)captured[offset + 1] << 8)
        | ((uint32_t)captured[offset + 2] << 16) | ((uint32_t)captured[offset + 3] << 24);
}

static void receiveFrame(void)
{
    static uint32_t schedulerUs;
    schedulerUs += 20000;
    capturedLength = 0;
    handleCustomTelemetry(schedulerUs);
    ASSERT_EQ(44, capturedLength);
    EXPECT_EQ(0xAA, captured[0]);
    EXPECT_EQ(0x55, captured[1]);
    EXPECT_EQ(2, captured[2]);
    EXPECT_EQ(44, captured[3]);
    uint16_t crc = 0xFFFF;
    for (unsigned i = 2; i < 42; ++i) {
        crc ^= (uint16_t)captured[i] << 8;
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    EXPECT_EQ(crc, captured[42] | ((uint16_t)captured[43] << 8));
    EXPECT_EQ(nowMs, readU32(6));
}

TEST(TelemetryCustomTest, FlightTimerLifecycleAndWrap)
{
    initCustomTelemetry();
    checkCustomTelemetryState();
    nowMs = 500;
    receiveFrame();
    EXPECT_EQ(0U, readU32(38));

    customTelemetryOnArm();
    receiveFrame();
    EXPECT_EQ(0U, readU32(38));
    nowMs += 12345;
    receiveFrame();
    EXPECT_EQ(12345U, readU32(38));

    // No telemetry calls between the final frame and disarming.
    nowMs += 1000;
    customTelemetryOnDisarm();
    nowMs += 5000;
    customTelemetryOnDisarm();
    receiveFrame();
    EXPECT_EQ(13345U, readU32(38));

    customTelemetryOnArm();
    receiveFrame();
    EXPECT_EQ(0U, readU32(38));
    nowMs += 200;
    receiveFrame();
    EXPECT_EQ(200U, readU32(38));
    customTelemetryOnDisarm();

    nowMs = UINT32_MAX - 100;
    customTelemetryOnArm();
    nowMs = 150;
    receiveFrame();
    EXPECT_EQ(251U, readU32(38));
    customTelemetryOnDisarm();
    nowMs = 300;
    receiveFrame();
    EXPECT_EQ(251U, readU32(38));
}
