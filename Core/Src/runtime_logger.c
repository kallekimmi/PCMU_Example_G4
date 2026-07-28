#include "runtime_logger.h"
#include "main.h" // Ensures HAL and device specific headers are included

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

static uint32_t total_runtime_minutes = 0;
static uint32_t next_write_slot_index = 0;

void Logger_Init(void) {
    total_runtime_minutes = 0;
    next_write_slot_index = 0;

    uint64_t *flash_ptr = (uint64_t *)FLASH_LOGGER_START_ADDR;

    /* Scan all 10 dedicated Flash pages to find the highest (latest) minute value.
       This ensures we recover correctly even after sudden power loss. */
    for (uint32_t i = 0; i < TOTAL_SLOTS; i++) {
        uint64_t data = flash_ptr[i];

        // Check if the memory slot is not empty (0xFFFFFFFFFFFFFFFF)
        if (data != 0xFFFFFFFFFFFFFFFFULL) {
            uint32_t magic = (uint32_t)(data >> 32);
            uint32_t minutes = (uint32_t)(data & 0xFFFFFFFF);

            // Verify signature and find the absolute maximum value
            if ((magic == LOGGER_MAGIC_WORD) && (minutes >= total_runtime_minutes)) {
                total_runtime_minutes = minutes;
                next_write_slot_index = i + 1; // Point to the next available slot
            }
        }
    }

    // Wrap around to index 0 if we perfectly filled the very last slot before power off
    if (next_write_slot_index >= TOTAL_SLOTS) {
        next_write_slot_index = 0;
    }
}

void Logger_SaveMinute(void) {
    total_runtime_minutes++;

    // Calculate geometric location in the circular buffer
    uint32_t current_page_offset = next_write_slot_index / SLOTS_PER_PAGE;
    uint32_t current_slot_in_page = next_write_slot_index % SLOTS_PER_PAGE;

    HAL_FLASH_Unlock();

    /* WEAR LEVELING LOGIC:
       If we are at the very first slot of ANY page, we must erase that specific page
       before we can write new data to it. */
    if (current_slot_in_page == 0) {
        FLASH_EraseInitTypeDef EraseInitStruct;
        uint32_t PageError;

        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.Banks = FLASH_BANK_1;
        EraseInitStruct.Page = FLASH_LOGGER_START_PAGE + current_page_offset;
        EraseInitStruct.NbPages = 1;

        HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    }

    // Combine the 32-bit signature and the 32-bit runtime into a single 64-bit word
    uint64_t data_to_write = ((uint64_t)LOGGER_MAGIC_WORD << 32) | total_runtime_minutes;
    uint32_t write_address = FLASH_LOGGER_START_ADDR + (next_write_slot_index * 8);

    // Program the data into Flash memory
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, write_address, data_to_write);
    HAL_FLASH_Lock();

    // Advance the pointer and wrap around circularly if we hit the end of the 10 pages
    next_write_slot_index++;
    if (next_write_slot_index >= TOTAL_SLOTS) {
        next_write_slot_index = 0;
    }
}

uint32_t Logger_GetTotalMinutes(void) {
    return total_runtime_minutes;
}
