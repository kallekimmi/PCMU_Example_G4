#include "main.h"   // Required for HAL RCC macros
#include "buffer.h"

// Define and initialize the variables in memory.
ResetCause_t last_reset_cause = RST_BOR; /* Default fallback */
volatile uint8_t daq_is_alive = 0;
DaqBuffer_t pingPongBuf[2];
volatile uint8_t activeBufIdx = 0;
MuxData_t muxData;
const uint8_t INA_I2C_ADDRS[NUM_INA260] = {
    INA_I2C_ADR_MC, INA_I2C_ADR_FAN, INA_I2C_ADR_CUST,
    INA_I2C_ADR_LPRC, INA_I2C_ADR_AUTH, INA_I2C_ADR_INT, INA_I2C_ADR_CHR
};


// ResetBuffer function implementation
void ResetBuffer(uint8_t bufIdx) {
    // Reset INA260
    for(int i = 0; i < NUM_INA260; i++) {
        pingPongBuf[bufIdx].ina_voltage[i].sum = 0;
        pingPongBuf[bufIdx].ina_voltage[i].count = 0;
        pingPongBuf[bufIdx].ina_voltage[i].min = INT32_MAX;
        pingPongBuf[bufIdx].ina_voltage[i].max = INT32_MIN;

        pingPongBuf[bufIdx].ina_current[i].sum = 0;
        pingPongBuf[bufIdx].ina_current[i].count = 0;
        pingPongBuf[bufIdx].ina_current[i].min = INT32_MAX;
        pingPongBuf[bufIdx].ina_current[i].max = INT32_MIN;
    }

    // Reset SPI ADCs
    for(int adc = 0; adc < NUM_SPI_ADC; adc++) {
        for(int ch = 0; ch < SPI_ADC_CHANNELS; ch++) {
            pingPongBuf[bufIdx].spi_adc[adc][ch].sum = 0;
            pingPongBuf[bufIdx].spi_adc[adc][ch].count = 0;
            pingPongBuf[bufIdx].spi_adc[adc][ch].min = INT32_MAX;
            pingPongBuf[bufIdx].spi_adc[adc][ch].max = INT32_MIN;
        }
    }
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
