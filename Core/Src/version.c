#include "version.h"
#include "main.h"       // Required for LED1_Pin, LED1_GPIO_Port, HAL_GPIO_WritePin
#include "buffer.h"     // Required to read muxData for the HW version
#include "cmsis_os.h"   // Required for FreeRTOS osDelay()

/* Helper function to blink a specific number of times (private to this file) */
static void BlinkNumber(uint8_t count) {
    if (count == 0) {
        /* Short distinct blip for number zero */
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); // ON
        BSP_LED_On(LED_GREEN);
        osDelay(40);
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);   // OFF
        BSP_LED_Off(LED_GREEN);
        osDelay(300);
        return;
    }

    for (uint8_t i = 0; i < count; i++) {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); // ON
        BSP_LED_On(LED_GREEN);
        osDelay(200);
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);   // OFF
        BSP_LED_Off(LED_GREEN);
        osDelay(250);
    }
    osDelay(500); /* Pause between parts of the same version */
}

/* Main sequence function called from the FreeRTOS default task */
void Version_BlinkSequence(void) {
    /* Wait 1.5 seconds at startup so task_rs422.c has time to read MUX3 first */
    osDelay(1500);
    /* Initialize leds */
    BSP_LED_Init(LED_GREEN);

    /* Decode the 4-bit Hardware Revision from MUX3 */
    uint8_t hw_version = (muxData.fields.hw_addr_3_msb << 3) |
                         (muxData.fields.hw_addr_2     << 2) |
                         (muxData.fields.hw_addr_1     << 1) |
                         (muxData.fields.hw_addr_0_lsb);

    /* --- VISUAL BLINK SEQUENCE RUNS ONCE AT STARTUP --- */

    /* PHASE 1: Blink Hardware Version */
    BlinkNumber(hw_version);

    /* Clean 1.5-second delay to separate HW and SW */
    osDelay(1500);

    /* PHASE 2: Blink Software Version (Major.Minor.Patch) */
    BlinkNumber(SW_VERSION_MAJOR);
    BlinkNumber(SW_VERSION_MINOR);
    BlinkNumber(SW_VERSION_PATCH);

    /* Make sure LED is turned off after the sequence is finished */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    BSP_LED_Off(LED_GREEN);
    BSP_LED_DeInit(LED_GREEN);
}

uint32_t getUnique32bitID(void) {
    // UID0: X and Y coordinates on the wafer
    uint32_t uid0 = HAL_GetUIDw0();
    // UID1: Wafer number and the first part of the lot number
    uint32_t uid1 = HAL_GetUIDw1();
    // UID2: The remaining part of the lot number
    uint32_t uid2 = HAL_GetUIDw2();
    // Compress the 96-bit UID into a 32-bit ID using bitwise XOR
    return (uid0 ^ uid1 ^ uid2);
}

/**
  * @brief  Retrieves the full, uncompressed 96-bit Unique Device ID.
  * @param  id_buffer: Pointer to a uint32_t array of at least 3 elements where the ID will be stored.
  * @retval None
  */
void getUnique96bitID(uint32_t *id_buffer) {
    if (id_buffer != NULL) {
        // UID0: X and Y coordinates on the wafer
        id_buffer[0] = HAL_GetUIDw0();
        // UID1: Wafer number and the first part of the lot number
        id_buffer[1] = HAL_GetUIDw1();
        // UID2: The remaining part of the lot number
        id_buffer[2] = HAL_GetUIDw2();
    }
}
