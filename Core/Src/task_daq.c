#include <daq.h>
#include "buffer.h"
#include "main.h"       // Required for STM32 HAL-definitions
#include "FreeRTOS.h"   // Required for FreeRTOS (TickType_t)
#include "task.h"       // Required for FreeRTOS (vTaskDelayUntil)

// Tell this file that the I2C and SPI handles exist in main.c
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

/* * Helper function to select the correct ADC via a 2-to-4 decoder.
 * Assumes A0 and A1 select ADC 0, 1 or 2, and setting both HIGH disables all.
 */
static void Set_SPI_CS(uint8_t adc_idx, uint8_t enable) {
    if (!enable) {
        // Deselect all (Pulling both address lines HIGH typically disables the output)
        HAL_GPIO_WritePin(SPI_CS_A1_GPIO_Port, SPI_CS_A1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SPI_CS_A0_GPIO_Port, SPI_CS_A0_Pin, GPIO_PIN_SET);
        return;
    }

    // Select specific ADC
    switch (adc_idx) {
        case 0:
            HAL_GPIO_WritePin(SPI_CS_A1_GPIO_Port, SPI_CS_A1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(SPI_CS_A0_GPIO_Port, SPI_CS_A0_Pin, GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(SPI_CS_A1_GPIO_Port, SPI_CS_A1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(SPI_CS_A0_GPIO_Port, SPI_CS_A0_Pin, GPIO_PIN_SET);
            break;
        case 2:
            HAL_GPIO_WritePin(SPI_CS_A1_GPIO_Port, SPI_CS_A1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(SPI_CS_A0_GPIO_Port, SPI_CS_A0_Pin, GPIO_PIN_RESET);
            break;
    }
}


void StartTask_DAQ(void * argument) {
    // Initialize both buffers at startup
    ResetBuffer(0);
    ResetBuffer(1);

    /* --- HARDWARE INITIALIZATION --- */
	/* Configure all 7 INA260 sensors for continuous 140us conversion (0x6007).
	   Register 0x00 is the Configuration Register. */
	uint8_t inaConfigPayload[3] = {0x00, 0x60, 0x07};

	for(int i = 0; i < NUM_INA260; i++) {
		/* INA260 addresses are 7-bit, HAL requires them shifted by 1 bit left */
		HAL_I2C_Master_Transmit(&hi2c1, (INA_I2C_ADDRS[i] << 1), inaConfigPayload, 3, 5);
	}

	/* Enable DWT Cycle Counter hardware register for microsecond task timing.
	   Bit 0 of DWT->CTRL is the CYCCNT enable bit. */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= 1UL;

    /* --- MAIN TASK LOOP --- */
    while(1) {
    	// Capture the RTOS tick count at the very start of the cycle to calculate our dynamic sleep time later.
    	TickType_t start_tick = xTaskGetTickCount();

    	// Start execution timing for this cycle
    	uint32_t start_cycles = DWT->CYCCNT;

        // Get the index of the active buffer
        uint8_t idx = activeBufIdx;

        /* --- 1. READ INA260 OVER I2C --- */
		for(int i = 0; i < NUM_INA260; i++) {
			uint8_t i2cBuf[2];

			// Read Voltage Register (0x02) - Timeout set to 2 ms
			if (HAL_I2C_Mem_Read(&hi2c1, (INA_I2C_ADDRS[i] << 1), INA_REG_VOLTAGE, I2C_MEMADD_SIZE_8BIT, i2cBuf, 2, 2) == HAL_OK) {
				int16_t rawVoltage = (int16_t)((i2cBuf[0] << 8) | i2cBuf[1]);
                pingPongBuf[idx].ina_voltage[i].sum += rawVoltage;
                pingPongBuf[idx].ina_voltage[i].count++;
                if(rawVoltage < pingPongBuf[idx].ina_voltage[i].min) pingPongBuf[idx].ina_voltage[i].min = rawVoltage;
                if(rawVoltage > pingPongBuf[idx].ina_voltage[i].max) pingPongBuf[idx].ina_voltage[i].max = rawVoltage;
			}

			// Read Current Register (0x01) - Timeout set to 2 ms
			if (HAL_I2C_Mem_Read(&hi2c1, (INA_I2C_ADDRS[i] << 1), INA_REG_CURRENT, I2C_MEMADD_SIZE_8BIT, i2cBuf, 2, 2) == HAL_OK) {
				int16_t rawCurrent = (int16_t)((i2cBuf[0] << 8) | i2cBuf[1]);
                pingPongBuf[idx].ina_current[i].sum += rawCurrent;
                pingPongBuf[idx].ina_current[i].count++;
                if(rawCurrent < pingPongBuf[idx].ina_current[i].min) pingPongBuf[idx].ina_current[i].min = rawCurrent;
                if(rawCurrent > pingPongBuf[idx].ina_current[i].max) pingPongBuf[idx].ina_current[i].max = rawCurrent;
			}
        }

		/* --- 2. READ 3x SPI ADC (ADC128S022) --- */
		for(int adc = 0; adc < NUM_SPI_ADC; adc++) {

		    // --- DUMMY READ ---
		    // Pre-load the ADC multiplexer with Channel 0 for the first iteration.
		    uint16_t dummyTx = (0 << 11);
		    uint16_t dummyRx = 0;

		    Set_SPI_CS(adc, 1);
		    for(volatile int nop = 0; nop < 50; nop++); // Allow decoder time to pull CS low

		    // Transmit 1 frame of 16 bits[cite: 1]
		    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&dummyTx, (uint8_t*)&dummyRx, 1, 2);

		    Set_SPI_CS(adc, 0);
		    for(volatile int nop = 0; nop < 50; nop++); // CS hold time

		    // --- READ ALL CHANNELS ---
		    for(int ch = 0; ch < SPI_ADC_CHANNELS; ch++) {

		        // Pipeline mechanics: request the NEXT channel, receive the CURRENT channel.
		        // Wrap around to 0 after reading channel 7.
		        uint8_t nextCh = (ch == 7) ? 0 : (ch + 1);

		        // Place the 3-bit channel address in bits 13, 12, and 11 of the 16-bit word[cite: 1]
		        uint16_t spiTx = (nextCh << 11);
		        uint16_t spiRx = 0;

		        Set_SPI_CS(adc, 1);
		        for(volatile int nop = 0; nop < 50; nop++);

		        HAL_StatusTypeDef spi_status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&spiTx, (uint8_t*)&spiRx, 1, 2);

		        Set_SPI_CS(adc, 0);
		        for(volatile int nop = 0; nop < 50; nop++);

		        if (spi_status == HAL_OK) {
		            // DOUT clocks out 4 leading zeros followed by the 12-bit result[cite: 1].
		            // Mask out the leading zeros to isolate the data.
		            uint16_t rawAdcValue = spiRx & 0x0FFF;

		            pingPongBuf[idx].spi_adc[adc][ch].sum += rawAdcValue;
		            pingPongBuf[idx].spi_adc[adc][ch].count++;
		            if(rawAdcValue < pingPongBuf[idx].spi_adc[adc][ch].min) pingPongBuf[idx].spi_adc[adc][ch].min = rawAdcValue;
		            if(rawAdcValue > pingPongBuf[idx].spi_adc[adc][ch].max) pingPongBuf[idx].spi_adc[adc][ch].max = rawAdcValue;
		        }
		    }
		}

        /* Calculate loop execution duration in microseconds */
		uint32_t elapsed_cycles = DWT->CYCCNT - start_cycles;
		int32_t elapsed_us = elapsed_cycles / (SystemCoreClock / 1000000U);

		/* Commit telemetry metrics for processing by the 1 Hz thread */
		pingPongBuf[idx].daq_loop_time.sum += elapsed_us;
		pingPongBuf[idx].daq_loop_time.count++;
		if (elapsed_us < pingPongBuf[idx].daq_loop_time.min) pingPongBuf[idx].daq_loop_time.min = elapsed_us;
		if (elapsed_us > pingPongBuf[idx].daq_loop_time.max) pingPongBuf[idx].daq_loop_time.max = elapsed_us;

		/* Feed software watchdog status flag */
        daq_is_alive = 1;

        /* --- 3. DYNAMIC YIELD / BREATHING ROOM --- */
		/* Calculate how many RTOS ticks the hardware sampling actually took */
		TickType_t execution_time = xTaskGetTickCount() - start_tick;

		if (execution_time < pdMS_TO_TICKS(5)) {
			/* BEST CASE: Execution was fast! Hardware responded in time.
			   Sleep the remaining time up to our 5 ms (200Hz) budget. */
			vTaskDelay(pdMS_TO_TICKS(5) - execution_time);
		} else {
			/* FAILSAFE: Loop took 5 ms or longer (likely due to timeouts).
			   Force a hard 5 ms sleep to guarantee that lower priority tasks
			   (like Telemetry) and the Watchdog get CPU time! */
			vTaskDelay(pdMS_TO_TICKS(50));
		}
    }
}
