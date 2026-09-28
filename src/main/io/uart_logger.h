/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "pg/pg.h"

typedef struct uartLoggerConfig_s {
    uint32_t baud;
    int8_t uart;
    uint8_t parity;
    uint8_t stopBits;
} uartLoggerConfig_t;

#ifdef USE_FLASHFS
PG_DECLARE(uartLoggerConfig_t, uartLoggerConfig);
bool uartLoggerEnabled(void);
void uartLoggerInit(void);
void uartLoggerUpdate(void);
void uartLoggerStop(void);
const char *uartLoggerStatus(void);
uint32_t uartLoggerDropped(void);
#else
static inline bool uartLoggerEnabled(void) { return false; }
static inline void uartLoggerInit(void) {}
static inline void uartLoggerUpdate(void) {}
static inline void uartLoggerStop(void) {}
#endif
