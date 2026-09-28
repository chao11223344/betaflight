/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once

#include <stdbool.h>
#include <stdint.h>

bool uartLoggerFilesInit(void);
uint32_t uartLoggerFileSize(unsigned index, bool raw);
void uartLoggerReadRaw(unsigned index, uint8_t *dest, unsigned size, uint32_t offset);
void uartLoggerReadLog(unsigned index, uint8_t *dest, unsigned size, uint32_t offset);
const char *uartLoggerFileStatus(void);
