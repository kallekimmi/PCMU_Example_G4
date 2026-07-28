#ifndef RUNTIME_LOGGER_H
#define RUNTIME_LOGGER_H

#include <stdint.h>

/* Initializes the logger by scanning Flash for the latest saved minute.
 * Must be called in main.c BEFORE FreeRTOS starts. */
void Logger_Init(void);

/* Increments the total runtime and safely saves it to Flash.
 * Handles page erasing and wear leveling automatically. */
void Logger_SaveMinute(void);

/* Returns the total saved minutes from Flash */
uint32_t Logger_GetTotalMinutes(void);

#endif /* RUNTIME_LOGGER_H */
