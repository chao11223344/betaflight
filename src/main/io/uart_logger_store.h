/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define UART_LOGGER_MAX_LOGS 32
#define UART_LOGGER_BLOCK_SIZE 64

typedef struct uartLogEntry_s {
    uint32_t sequence;
    uint32_t start;
    uint32_t size;
    uint32_t dataOffset;
    uint32_t rawSize;
    uint32_t dropped;
    uint8_t version;
    bool complete;
} uartLogEntry_t;

bool uartLogScan(void);
unsigned uartLogCount(void);
const uartLogEntry_t *uartLogEntry(unsigned index);
bool uartLogBegin(const uint8_t *header);
bool uartLogPrepareWrite(void);
uint32_t uartLogRemaining(void);
void uartLogWritten(void);
void uartLogFlush(void);
bool uartLogValidRecord(const uint8_t *record);
bool uartLogRecordMatches(const uint8_t *record, const uartLogEntry_t *log);
uint32_t uartLogGet32(const uint8_t *p);
void uartLogPut32(uint8_t *p, uint32_t value);
