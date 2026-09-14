/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software and/or
 * modify this software under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 */

#pragma once

#include <stdint.h>

/*
 * CUSTOM telemetry protocol version 2
 *
 * All multi-byte fields are little-endian. The 44-byte frame is sent at 50 Hz:
 *
 * Offset  Size  Type     Field / unit
 *   0       2   uint8    Sync bytes 0xAA, 0x55
 *   2       1   uint8    Protocol version (2)
 *   3       1   uint8    Total frame length (44)
 *   4       2   uint16   Sequence number
 *   6       4   uint32   Time since boot, ms
 *  10       2   uint16   Valid/status flags
 *  12       2   uint16   GPS ground speed, cm/s
 *  14       4   int32    GPS altitude above MSL, cm
 *  18       4   int32    Barometer altitude from power-on zero, cm
 *  22       4   int32    Indicated airspeed, cm/s
 *  26       6   int16[3] Body gyro X/Y/Z, 0.1 deg/s
 *  32       6   int16[3] Body acceleration X/Y/Z including gravity, mg
 *  38       4   uint32   Armed duration, ms; reset on arm, retained on disarm
 *  42       2   uint16   CRC-16/CCITT over bytes 2..41, initial value 0xFFFF
 *
 * Valid/status flags: bit 0 GPS sensor, bit 1 GPS fix, bit 2 calibrated baro,
 * bit 3 calibrated airspeed, bit 4 calibrated gyro, bit 5 accelerometer data,
 * bit 6 armed. Consumers must use these flags instead of treating zero as
 * invalid sensor data.
 */

void initCustomTelemetry(void);
void checkCustomTelemetryState(void);
void handleCustomTelemetry(uint32_t currentTimeUs);
void customTelemetryOnArm(void);
void customTelemetryOnDisarm(void);
