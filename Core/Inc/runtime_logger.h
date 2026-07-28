#ifndef RUNTIME_LOGGER_H
#define RUNTIME_LOGGER_H

#include <stdint.h>

/* =================================================================================
 * CONFIGURATION FOR STM32G431 (128KB Flash)
 * Total pages = 64. We use the last 10 pages (Pages 54 to 63).
 * 10 pages * 2048 bytes = 20 KB dedicated to wear leveling.
 * Expected lifespan at 1 save/minute = >48 years of continuous 24/7 operation.
 * ================================================================================= */
#define FLASH_LOGGER_START_ADDR  0x0801B000 // Address for Page 54
#define FLASH_LOGGER_START_PAGE  54
#define FLASH_LOGGER_NUM_PAGES   10

#define LOGGER_MAGIC_WORD        0xAA550000
#define PAGE_SIZE_BYTES          2048
#define SLOTS_PER_PAGE           (PAGE_SIZE_BYTES / 8) // 256 slots per page
#define TOTAL_SLOTS              (SLOTS_PER_PAGE * FLASH_LOGGER_NUM_PAGES) // 2560 total slots

/* Initializes the logger by scanning Flash for the latest saved minute.
 * Must be called in main.c BEFORE FreeRTOS starts. */
void Logger_Init(void);

/* Increments the total runtime and safely saves it to Flash.
 * Handles page erasing and wear leveling automatically. */
void Logger_SaveMinute(void);

/* Returns the total saved minutes from Flash */
uint32_t Logger_GetTotalMinutes(void);

#endif /* RUNTIME_LOGGER_H */
