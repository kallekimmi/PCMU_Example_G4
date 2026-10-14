#include <daq.h>
#include "buffer.h"
#include "main.h"       // Required for STM32 HAL-definitions
#include "FreeRTOS.h"   // Required for FreeRTOS (TickType_t)
#include "task.h"       // Required for FreeRTOS (vTaskDelayUntil)

// Tell this file that the I2C and SPI handles exist in main.c
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;


void StartTask_DAQ(void * argument) {
    // Initialize both buffers at startup
    ResetBuffer(0);
    ResetBuffer(1);

    /* --- HARDWARE INITIALIZATION --- */
	/* Configure all 7 INA260 sensors for continuous 140us conversion (0x6007).
	   Register 0x00 is the Configuration Register. */
	uint8_t inaConfigPayload[3] = {0x00, 0x60, 0x07};


	// Initialize devices on I2C1
	for(int i = 0; i < NUM_INA260; i++) {
		/* INA260 addresses are 7-bit, HAL requires them shifted by 1 bit left */
		HAL_I2C_Master_Transmit(&hi2c1, (INA_I2C1_ADDRS[i] << 1), inaConfigPayload, 3, 5);
	}

	// Initialize devices on I2C3
	for(int i = 0; i < NUM_INA260; i++) {
		/* INA260 addresses are 7-bit, HAL requires them shifted by 1 bit left */
		HAL_I2C_Master_Transmit(&hi2c3, (INA_I2C3_ADDRS[i] << 1), inaConfigPayload, 3, 5);
	}


	/* Enable DWT Cycle Counter hardware register for microsecond task timing.
	   Bit 0 of DWT->CTRL is the CYCCNT enable bit. */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= 1UL;

	TickType_t sht_trigger_tick = 0; //Temperature sensor timer (to know how long to wait)
	uint8_t sht_state = 0; // 0 = Idle/Waiting for sync, 1 = Waiting for conversion

    /* --- MAIN TASK LOOP --- */
    while(1) {
    	// Capture the RTOS tick count at the very start of the cycle to calculate our dynamic sleep time later.
    	TickType_t start_tick = xTaskGetTickCount();

    	// Start execution timing for this cycle
    	uint32_t start_cycles = DWT->CYCCNT;

        // Get the index of the active buffer
        uint8_t idx = activeBufIdx;

        /* --- 1. SHT41A Temperature sensor 1Hz STATE MACHINE --- */
        if (sht_state == 0) {
			// Trigger exactly when the telemetry task clears the flag
			if (daq_is_alive == 0) {
				uint8_t shtCmd = 0xFD; // High precision command
				if (HAL_I2C_Master_Transmit(&hi2c3, (I2C3_ADR_TEMP << 1), &shtCmd, 1, 2) == HAL_OK) {
					sht_state = 1;
					sht_trigger_tick = start_tick; // Record the tick so we know when to read
				}
			}
		}
		else if (sht_state == 1) {
			// High precision conversion takes up to 8.3 ms. Wait at least 20 ms.
			if ((start_tick - sht_trigger_tick) >= pdMS_TO_TICKS(20)) {
				uint8_t shtRx[6];
				if (HAL_I2C_Master_Receive(&hi2c3, (I2C3_ADR_TEMP << 1), shtRx, 6, 2) == HAL_OK) {
					pingPongBuf[idx].temperature_raw = (uint16_t)((shtRx[0] << 8) | shtRx[1]);
					pingPongBuf[idx].humidity_raw = (uint16_t)((shtRx[3] << 8) | shtRx[4]);
				}

				// Return to idle state until the next time daq_is_alive becomes 0
				sht_state = 0;
			}
		}

        /* --- 1. READ I2C1 SENSORS --- */
		for(int i = 0; i < NUM_INA260; i++) {
			uint8_t i2cBuf[2];

			// Read Voltage Register (0x02) - Timeout set to 2 ms
			if (HAL_I2C_Mem_Read(&hi2c1, (INA_I2C1_ADDRS[i] << 1), INA_REG_VOLTAGE, I2C_MEMADD_SIZE_8BIT, i2cBuf, 2, 2) == HAL_OK) {
				int16_t rawVoltage = (int16_t)((i2cBuf[0] << 8) | i2cBuf[1]);
                pingPongBuf[idx].i2c1_ina_voltage[i].sum += rawVoltage;
                pingPongBuf[idx].i2c1_ina_voltage[i].count++;
                if(rawVoltage < pingPongBuf[idx].i2c1_ina_voltage[i].min) pingPongBuf[idx].i2c1_ina_voltage[i].min = rawVoltage;
                if(rawVoltage > pingPongBuf[idx].i2c1_ina_voltage[i].max) pingPongBuf[idx].i2c1_ina_voltage[i].max = rawVoltage;
			}

			// Read Current Register (0x01) - Timeout set to 2 ms
			if (HAL_I2C_Mem_Read(&hi2c1, (INA_I2C1_ADDRS[i] << 1), INA_REG_CURRENT, I2C_MEMADD_SIZE_8BIT, i2cBuf, 2, 2) == HAL_OK) {
				int16_t rawCurrent = (int16_t)((i2cBuf[0] << 8) | i2cBuf[1]);
                pingPongBuf[idx].i2c1_ina_current[i].sum += rawCurrent;
                pingPongBuf[idx].i2c1_ina_current[i].count++;
                if(rawCurrent < pingPongBuf[idx].i2c1_ina_current[i].min) pingPongBuf[idx].i2c1_ina_current[i].min = rawCurrent;
                if(rawCurrent > pingPongBuf[idx].i2c1_ina_current[i].max) pingPongBuf[idx].i2c1_ina_current[i].max = rawCurrent;
			}
        }

		/* --- 2. READ I2C3 SENSORS --- */
		for(int i = 0; i < NUM_INA260; i++) {
			uint8_t i2cBuf[2];

			// Read Voltage Register
			if (HAL_I2C_Mem_Read(&hi2c3, (INA_I2C3_ADDRS[i] << 1), INA_REG_VOLTAGE, I2C_MEMADD_SIZE_8BIT, i2cBuf, 2, 2) == HAL_OK) {
				int16_t rawVoltage = (int16_t)((i2cBuf[0] << 8) | i2cBuf[1]);
				pingPongBuf[idx].i2c3_ina_voltage[i].sum += rawVoltage;
				pingPongBuf[idx].i2c3_ina_voltage[i].count++;
				if(rawVoltage < pingPongBuf[idx].i2c3_ina_voltage[i].min) pingPongBuf[idx].i2c3_ina_voltage[i].min = rawVoltage;
				if(rawVoltage > pingPongBuf[idx].i2c3_ina_voltage[i].max) pingPongBuf[idx].i2c3_ina_voltage[i].max = rawVoltage;
			}

			// Read Current Register
			if (HAL_I2C_Mem_Read(&hi2c3, (INA_I2C3_ADDRS[i] << 1), INA_REG_CURRENT, I2C_MEMADD_SIZE_8BIT, i2cBuf, 2, 2) == HAL_OK) {
				int16_t rawCurrent = (int16_t)((i2cBuf[0] << 8) | i2cBuf[1]);
				pingPongBuf[idx].i2c3_ina_current[i].sum += rawCurrent;
				pingPongBuf[idx].i2c3_ina_current[i].count++;
				if(rawCurrent < pingPongBuf[idx].i2c3_ina_current[i].min) pingPongBuf[idx].i2c3_ina_current[i].min = rawCurrent;
				if(rawCurrent > pingPongBuf[idx].i2c3_ina_current[i].max) pingPongBuf[idx].i2c3_ina_current[i].max = rawCurrent;
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
