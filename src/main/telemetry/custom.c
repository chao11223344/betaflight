/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software and/or
 * modify this software under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_TELEMETRY_CUSTOM

#include "common/axis.h"
#include "common/crc.h"
#include "common/maths.h"

#include "drivers/serial.h"

#include "fc/runtime_config.h"

#include "io/gps.h"
#include "io/serial.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/pitot.h"
#include "sensors/sensors.h"

#include "telemetry/custom.h"
#include "telemetry/telemetry.h"

#define CUSTOM_TELEMETRY_FRAME_SIZE 40
#define CUSTOM_TELEMETRY_RATE_HZ 50

enum {
    CUSTOM_VALID_GPS_SENSOR = 1 << 0,
    CUSTOM_VALID_GPS_FIX = 1 << 1,
    CUSTOM_VALID_BARO = 1 << 2,
    CUSTOM_VALID_AIRSPEED = 1 << 3,
    CUSTOM_VALID_GYRO = 1 << 4,
    CUSTOM_VALID_ACCEL = 1 << 5,
    CUSTOM_STATUS_ARMED = 1 << 6,
};

static serialPort_t *customPort;
static serialPortIdentifier_e customPortIdentifier = SERIAL_PORT_NONE;
static portSharing_e customPortSharing;
static bool customEnabled;
static uint16_t customSequence;
static uint32_t lastFrameTimeUs;

static void putU16(uint8_t *buffer, unsigned *offset, uint16_t value)
{
    buffer[(*offset)++] = value;
    buffer[(*offset)++] = value >> 8;
}

static void putU32(uint8_t *buffer, unsigned *offset, uint32_t value)
{
    putU16(buffer, offset, value);
    putU16(buffer, offset, value >> 16);
}

static int16_t scaledInt16(float value, float scale)
{
    return (int16_t)lrintf(constrainf(value * scale, INT16_MIN, INT16_MAX));
}

static void sendCustomFrame(uint32_t currentTimeUs)
{
    uint8_t frame[CUSTOM_TELEMETRY_FRAME_SIZE];
    unsigned offset = 0;
    uint16_t validFlags = 0;

    if (sensors(SENSOR_GYRO) && gyroIsCalibrationComplete()) {
        validFlags |= CUSTOM_VALID_GYRO;
    }
#ifdef USE_ACC
    if (sensors(SENSOR_ACC) && acc.isAccelUpdatedAtLeastOnce) {
        validFlags |= CUSTOM_VALID_ACCEL;
    }
#endif
#ifdef USE_GPS
    if (sensors(SENSOR_GPS)) {
        validFlags |= CUSTOM_VALID_GPS_SENSOR;
        if (STATE(GPS_FIX)) {
            validFlags |= CUSTOM_VALID_GPS_FIX;
        }
    }
#endif
#ifdef USE_BARO
    if (sensors(SENSOR_BARO) && baroIsCalibrated()) {
        validFlags |= CUSTOM_VALID_BARO;
    }
#endif
#ifdef USE_PITOT
    if (sensors(SENSOR_PITOT) && pitotIsCalibrated()) {
        validFlags |= CUSTOM_VALID_AIRSPEED;
    }
#endif
    if (ARMING_FLAG(ARMED)) {
        validFlags |= CUSTOM_STATUS_ARMED;
    }

    frame[offset++] = 0xAA;
    frame[offset++] = 0x55;
    frame[offset++] = 1; // Protocol version.
    frame[offset++] = CUSTOM_TELEMETRY_FRAME_SIZE;
    putU16(frame, &offset, customSequence++);
    putU32(frame, &offset, currentTimeUs / 1000);
    putU16(frame, &offset, validFlags);

#ifdef USE_GPS
    putU16(frame, &offset, gpsSol.groundSpeed);
    putU32(frame, &offset, gpsSol.llh.altCm);
#else
    putU16(frame, &offset, 0);
    putU32(frame, &offset, 0);
#endif
#ifdef USE_BARO
    putU32(frame, &offset, lrintf(getBaroAltitude()));
#else
    putU32(frame, &offset, 0);
#endif
#ifdef USE_PITOT
    putU32(frame, &offset, lrintf(pitotGetAirspeed()));
#else
    putU32(frame, &offset, 0);
#endif

    for (unsigned axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        putU16(frame, &offset, scaledInt16(gyro.gyroADCf[axis], 10.0f));
    }
#ifdef USE_ACC
    for (unsigned axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        putU16(frame, &offset, scaledInt16(acc.accADC.v[axis] * acc.dev.acc_1G_rec, 1000.0f));
    }
#else
    for (unsigned axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        putU16(frame, &offset, 0);
    }
#endif

    const uint16_t crc = crc16_ccitt_update(0xFFFF, &frame[2], offset - 2);
    putU16(frame, &offset, crc);

    if (serialTxBytesFree(customPort) >= offset) {
        serialWriteBuf(customPort, frame, offset);
    }
}

static void freeCustomTelemetryPort(void)
{
    closeSerialPort(customPort);
    customPort = NULL;
    customEnabled = false;
}

static void configureCustomTelemetryPort(void)
{
    if (customPortIdentifier == SERIAL_PORT_NONE) {
        return;
    }

    baudRate_e baudRateIndex = telemetryProviderBaud(TELEMETRY_PROTOCOL_CUSTOM, customPortIdentifier);
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_115200;
    }

    customPort = openSerialPort(customPortIdentifier, FUNCTION_TELEMETRY_CUSTOM, NULL, NULL,
        baudRates[baudRateIndex], MODE_TX,
        telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);
    if (customPort) {
        customEnabled = true;
    }
}

void initCustomTelemetry(void)
{
    customPortIdentifier = telemetryProviderPort(TELEMETRY_PROTOCOL_CUSTOM);
    customPortSharing = determinePortSharing(customPortIdentifier, FUNCTION_TELEMETRY_CUSTOM);
}

void checkCustomTelemetryState(void)
{
    if (customPortIdentifier != SERIAL_PORT_NONE
        && telemetryCheckRxPortShared(customPortIdentifier, rxRuntimeState.serialrxProvider)) {
        if (!customEnabled && telemetrySharedPort) {
            customPort = telemetrySharedPort;
            customEnabled = true;
        }
        return;
    }

    const bool shouldEnable = telemetryDetermineEnabledState(customPortSharing);
    if (shouldEnable == customEnabled) {
        return;
    }

    if (shouldEnable) {
        configureCustomTelemetryPort();
    } else {
        freeCustomTelemetryPort();
    }
}

void handleCustomTelemetry(uint32_t currentTimeUs)
{
    if (!customEnabled || !customPort) {
        return;
    }

    if ((uint32_t)(currentTimeUs - lastFrameTimeUs) >= (1000000U / CUSTOM_TELEMETRY_RATE_HZ)) {
        lastFrameTimeUs = currentTimeUs;
        sendCustomFrame(currentTimeUs);
    }
}

#endif
