#include "main.h"   // Required for HAL RCC macros
#include "buffer.h"

// Define and initialize the variables in memory.
ResetCause_t last_reset_cause = RST_BOR; /* Default fallback */
volatile uint8_t daq_is_alive = 0;
DaqBuffer_t pingPongBuf[2];
volatile uint8_t activeBufIdx = 0;
MuxData_t muxData;
const uint8_t INA_I2C1_ADDRS[NUM_INA260] = {
	I2C1_ADR_INA_MC, I2C1_ADR_INA_FAN, I2C1_ADR_INA_CUST,
	I2C1_ADR_INA_LPRC, I2C1_ADR_INA_AUTH, I2C1_ADR_INA_INT, I2C1_ADR_INA_CHR
};
const uint8_t INA_I2C3_ADDRS[NUM_INA260] = {
		I2C3_ADR_INA_FP1, I2C3_ADR_INA_FP2, I2C3_ADR_INA_MD,
		I2C3_ADR_INA_MS1, I2C3_ADR_INA_MS2, I2C3_ADR_INA_FW1, I2C3_ADR_INA_FW2
};

// ResetBuffer function implementation
void ResetBuffer(uint8_t bufIdx) {
    // Reset INA260
    for(int i = 0; i < NUM_INA260; i++) {
        pingPongBuf[bufIdx].i2c1_ina_voltage[i].sum = 0;
        pingPongBuf[bufIdx].i2c1_ina_voltage[i].count = 0;
        pingPongBuf[bufIdx].i2c1_ina_voltage[i].min = INT32_MAX;
        pingPongBuf[bufIdx].i2c1_ina_voltage[i].max = INT32_MIN;

        pingPongBuf[bufIdx].i2c1_ina_current[i].sum = 0;
        pingPongBuf[bufIdx].i2c1_ina_current[i].count = 0;
        pingPongBuf[bufIdx].i2c1_ina_current[i].min = INT32_MAX;
        pingPongBuf[bufIdx].i2c1_ina_current[i].max = INT32_MIN;

        pingPongBuf[bufIdx].i2c3_ina_voltage[i].sum = 0;
        pingPongBuf[bufIdx].i2c3_ina_voltage[i].count = 0;
        pingPongBuf[bufIdx].i2c3_ina_voltage[i].min = INT32_MAX;
        pingPongBuf[bufIdx].i2c3_ina_voltage[i].max = INT32_MIN;

        pingPongBuf[bufIdx].i2c3_ina_current[i].sum = 0;
        pingPongBuf[bufIdx].i2c3_ina_current[i].count = 0;
        pingPongBuf[bufIdx].i2c3_ina_current[i].min = INT32_MAX;
        pingPongBuf[bufIdx].i2c3_ina_current[i].max = INT32_MIN;
    }

    //Reset Temperature and Humidity
    pingPongBuf[bufIdx].temperature_raw = 0;
	pingPongBuf[bufIdx].humidity_raw = 0;

    // Reset DAQ loop timing stats
	pingPongBuf[bufIdx].daq_loop_time.sum = 0;
	pingPongBuf[bufIdx].daq_loop_time.count = 0;
	pingPongBuf[bufIdx].daq_loop_time.min = INT32_MAX;
	pingPongBuf[bufIdx].daq_loop_time.max = INT32_MIN;
}

/**
  * @brief  Evaluates the STM32 hardware registers to determine the cause of the last reset.
  * It checks all available flags independently without merging states.
  * @retval ResetCause_t The specific reason the system last restarted.
  */
ResetCause_t GetSystemResetCause(void) {
    ResetCause_t cause = RST_OTH;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) {
        cause = RST_IWDG;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET) {
        cause = RST_WWDG;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET) {
        cause = RST_SFT;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET) {
        cause = RST_LPWR;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST) != RESET) {
        cause = RST_OBL;
    }
    //else if (__HAL_RCC_GET_FLAG(RCC_FLAG_FWRST) != RESET) {
    //    cause = RST_FWR;
    //}
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET) {
        cause = RST_BOR;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET) {
        cause = RST_PIN;
    }
    else {
        cause = RST_OTH;
    }

    /* Clear the reset flags immediately so the next reset is read correctly */
    __HAL_RCC_CLEAR_RESET_FLAGS();

    return cause;
}
