/**
  ******************************************************************************
  * @file           : task_telemetry.c
  * @brief          : 1 Hz Processing and RS422 JSON Telemetry Output Task
  ******************************************************************************
  */

#include <daq.h>
#include "buffer.h"
#include "version.h"
#include "main.h"       // Required for STM32 HAL definitions (MUX_SEL_A_Pin, etc.)
#include "FreeRTOS.h"   // Required for FreeRTOS types (TickType_t)
#include "task.h"       // Required for FreeRTOS task control (vTaskDelayUntil)
#include <stdio.h>
#include <string.h>     // Required for strlen(), strcpy(), strcat(), strstr()
#include <telemetry_labels.h>     // Required for string lookup table, Side_t, LabelID_t, Error definitions

/* External handles initialized by STM32CubeMX in main.c */
extern UART_HandleTypeDef huart1; // RS422
extern UART_HandleTypeDef huart2; // USB Debug
extern IWDG_HandleTypeDef hiwdg;  // Watchdog
extern ADC_HandleTypeDef hadc2;	  // STM32 internal ADC

/**
  * @brief  Task responsible for buffer swapping, processing accumulated raw
  * samples into engineering units, and transmitting a JSON telemetry log.
  * @param  argument: Not used
  * @retval None
  */
void StartTask_Telemetry(void * argument) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // Exactly 1 Hz execution rate
    const uint32_t uid32 = getUnique32bitID();

    /* ========================================================================= */
	/* --- ONE-TIME STARTUP HARDWARE READ & LOCKING ---------------------------- */
	/* ========================================================================= */
	/* Temporary buffer for the initial startup MUX read */
	float startup_mux_voltages[NUM_MUX][MUX_CHANNELS] = {0};

	/* Sample all MUX channels once at boot to lock static hardware strapping configuration */
	for(uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
		HAL_GPIO_WritePin(MUX_SEL_A_GPIO_Port, MUX_SEL_A_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MUX_SEL_B_GPIO_Port, MUX_SEL_B_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MUX_SEL_C_GPIO_Port, MUX_SEL_C_Pin, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

		for(volatile int nop = 0; nop < 10; nop++);

		HAL_ADC_Start(&hadc2);

		for(int m = 0; m < NUM_MUX; m++) {
			if (HAL_ADC_PollForConversion(&hadc2, 1) == HAL_OK) {
				uint32_t raw_value = HAL_ADC_GetValue(&hadc2);
				startup_mux_voltages[m][ch] = ((float)raw_value * 3.3f) / 4095.0f;
			} else {
				startup_mux_voltages[m][ch] = -999.0f;
			}
		}

		/* Populate the global muxData union immediately so that version.c
		   can decode hw_version for its LED blink sequence after the 1.5s delay */
		// Convert ADC data to binary. Threshold voltage = 1.5V
		muxData.raw[0][ch] = (startup_mux_voltages[0][ch] > 1.5f) ? 1 : 0;
		muxData.raw[1][ch] = (startup_mux_voltages[1][ch] > 1.5f) ? 1 : 0;
		muxData.raw[2][ch] = (startup_mux_voltages[2][ch] > 1.5f) ? 1 : 0;
	}


	/* Translate the latest STM32 hardware reset cause into a 3-character string
	   to be logged in the JSON telemetry. This value is locked in at startup
	   and remains static for the duration of the current uptime. */
	const char* rst_str = "OTH";
	switch(last_reset_cause) {
		case RST_BOR:  rst_str = "BOR"; break; // Brown-Out: Normal power cycle or voltage drop
		case RST_OBL:  rst_str = "OBL"; break; // Option Byte Loader: Memory configuration reload
		case RST_PIN:  rst_str = "PIN"; break; // Pin Reset: External NRST pin was pulled low
		case RST_SFT:  rst_str = "SFT"; break; // Software Reset: Code explicitly rebooted the MCU
		case RST_IWDG: rst_str = "WDG"; break; // Independent Watchdog: Safety timer forced a reboot
		case RST_WWDG: rst_str = "WWD"; break; // Window Watchdog: Timing violation during refresh
		case RST_LPWR: rst_str = "LPW"; break; // Low-Power Reset: Illegal deep sleep mode entry
		case RST_FWR:  rst_str = "FWR"; break; // Firewall Reset: Unauthorized memory access blocked
		default:       rst_str = "OTH"; break; // Other/Unknown: Unmapped hardware reset event
	}


	/* Decode and lock static hardware configuration parameters as constants */
	const uint8_t hw_version = (muxData.fields.hw_addr_3_msb << 3) |
							   (muxData.fields.hw_addr_2     << 2) |
							   (muxData.fields.hw_addr_1     << 1) |
							   (muxData.fields.hw_addr_0_lsb);

	/* Read and invert signals so that 1 = connected (active), 0 = floating (inactive) */
	const uint8_t prim = !muxData.fields.prim_side_sel_n;
	const uint8_t sec  = !muxData.fields.sec_side_sel_n;

	/* Combine the bits: sec becomes bit 1 (value 2), prim becomes bit 0 (value 1)
	   00 = 0 (NONE)
	   01 = 1 (PRIMARY)
	   10 = 2 (SECONDARY)
	   11 = 3 (UNKNOWN) */
	const uint8_t side_val = (sec << 1) | prim;

	/* Since side_val (0-3) perfectly matches our Side_t enum (0-3),
	   we can cast it directly without any conditional statements! */
	Side_t active_variant = (Side_t)side_val;

	/* ========================================================================= */
	/* --- MACROS FOR STRING LOOKUP -------------------------------------------- */
	/* ========================================================================= */

	/* Helper macro to fetch string pointers seamlessly from the LUT */
	#define L(id) StringTable[active_variant][id]

	/* Helper macro to calculate a numerical source ID from two active-low valid signals.
	   Since inputs are active LOW (_n), we invert them so that 1 = active, 0 = inactive.
	   - Returns 0 (00 in binary) if neither source is active.
	   - Returns 1 (01 in binary) if valid1_n is active and valid2_n is inactive.
	   - Returns 2 (10 in binary) if valid2_n is active and valid1_n is inactive.
	   - Returns 3 (11 in binary) if both sources are active (Fault state). */
	#define GET_SRC_ID(valid1_n, valid2_n) ( ((!(valid2_n)) << 1) | (!(valid1_n)) )

	/* Helper macro to dynamically resolve power source strings instead of integers.
	   - Returns "NONE" if neither source is active (1, 1).
	   - Returns "UNKNOWN" if both sources are active simultaneously (0, 0).
	   - Otherwise returns the mapped name of the single active source. */
	#define GET_SRC_STR(valid1_n, valid2_n) \
		(((valid1_n) == (valid2_n)) ? ((valid1_n) ? "NONE" : "UNKNOWN") : \
		(!(valid1_n) ? L(LBL_28V_IN_1) : L(LBL_28V_IN_2)))

	/* ========================================================================= */
	/* --- MAIN TASK LOOP (RUNS AT 1 HZ) --------------------------------------- */
	/* ========================================================================= */
    while(1) {
    	/* Capture the exact tick-count when the telemetry task wakes up */
    	TickType_t loop_start_tick = xTaskGetTickCount();

    	/* Reset the Error ID bitfield at the start of each evaluation second */
		uint32_t current_error_id = 0;

		/* Check if the processing and UART transmission of the PREVIOUS loop
		   took longer than the 1 Hz budget (1000ms). If it did, log a Loop Timeout. */
		if ((xTaskGetTickCount() - xLastWakeTime) > xFrequency) {
			current_error_id |= ERR_TEL_TASK_TO; /* Set Telemetry task timeout bit */
		}

		/* Wait until exactly 1000ms have passed since the last execution */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

        /* --- 1. SWAP BUFFER (Ping-Pong Architecture) --- */
        /* Capture the active buffer index that the high-speed task has been filling */
        uint8_t processIdx = activeBufIdx;

        /* Instantly flip the index so Task 1 can write to the alternate clean buffer
           while this task processes the stable snapshot */
        activeBufIdx = !activeBufIdx;

        /* --- 2. SAMPLE MUX CHANNELS (ANALOG READ & BINARY CONVERSION) --- */
		float mux_voltages[NUM_MUX][MUX_CHANNELS] = {0};
		/* Iterate through all 8 channels of the SN74LV4051A-Q1 multiplexers */
		for(uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
			/* Set the 3-bit address lines (A, B, C) on the hardware MUX chips */
			HAL_GPIO_WritePin(MUX_SEL_A_GPIO_Port, MUX_SEL_A_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX_SEL_B_GPIO_Port, MUX_SEL_B_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX_SEL_C_GPIO_Port, MUX_SEL_C_Pin, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

			/* Brief delay to guarantee signal settling through the analog MUX */
			for(volatile int nop = 0; nop < 10; nop++);

			/* Start the conversion sequence (triggers Rank 1, 2, and 3 automatically) */
			HAL_ADC_Start(&hadc2);

			/* Read all 3 channels in sequence using the built-in sequencer */
			for(int m = 0; m < NUM_MUX; m++) {
				/* Ensure the conversion was successful before fetching the value */
				if (HAL_ADC_PollForConversion(&hadc2, 1) == HAL_OK) {
					/* Fetch the raw 12-bit values and convert them to Volts (3.3V VREF) */
					uint32_t raw_value = HAL_ADC_GetValue(&hadc2);
					mux_voltages[m][ch] = ((float)raw_value * 3.3f) / 4095.0f;
				} else {
					/* If the ADC hangs, flag it with a distinct error value */
					mux_voltages[m][ch] = -999.0f;
					current_error_id |= ERR_INT_ADC_TO; /* Set Internal ADC timeout bit */
				}
			}
		}

		/* --- 3. CALCULATE AND APPLY CONVERSION MACROS --- */

		// --- INA260 Arrays for JSON loop ---
		float ina_v_avg[NUM_INA260] = {0};
		float ina_v_min[NUM_INA260] = {0};
		float ina_v_max[NUM_INA260] = {0};
		float ina_i_avg[NUM_INA260] = {0};
		float ina_i_min[NUM_INA260] = {0};
		float ina_i_max[NUM_INA260] = {0};

		for (int i = 0; i < NUM_INA260; i++) {
			/* If count > 0, the sensor responded normally during the second */
			if (pingPongBuf[processIdx].ina_voltage[i].count > 0) {
				float v_avg_raw = (float)pingPongBuf[processIdx].ina_voltage[i].sum / pingPongBuf[processIdx].ina_voltage[i].count;

				// Convert to Volts using macro from adc.h
				ina_v_avg[i] = INA_RAW_TO_V(v_avg_raw);
				ina_v_min[i] = INA_RAW_TO_V(pingPongBuf[processIdx].ina_voltage[i].min);
				ina_v_max[i] = INA_RAW_TO_V(pingPongBuf[processIdx].ina_voltage[i].max);
			}
			else {
				// Sensor timed out or failed to communicate
				ina_v_avg[i] = -999.0f;
				ina_v_min[i] = -999.0f;
				ina_v_max[i] = -999.0f;

				/* Dynamically map the failed I2C index (0-6) into the reserved bit block */
				current_error_id |= (1UL << (ERR_I2C_START_BIT + i));
			}

			if (pingPongBuf[processIdx].ina_current[i].count > 0) {
				float i_avg_raw = (float)pingPongBuf[processIdx].ina_current[i].sum / pingPongBuf[processIdx].ina_current[i].count;

				// Convert to Amperes using macro from adc.h
				ina_i_avg[i] = INA_RAW_TO_A(i_avg_raw);
				ina_i_min[i] = INA_RAW_TO_A(pingPongBuf[processIdx].ina_current[i].min);
				ina_i_max[i] = INA_RAW_TO_A(pingPongBuf[processIdx].ina_current[i].max);
			}
			else {
				ina_i_avg[i] = -999.0f;
				ina_i_min[i] = -999.0f;
				ina_i_max[i] = -999.0f;
			}
		}

		// --- SPI ADC Arrays for JSON loop ---
		float adc_avg[NUM_SPI_ADC][SPI_ADC_CHANNELS] = {0};
		float adc_min[NUM_SPI_ADC][SPI_ADC_CHANNELS] = {0};
		float adc_max[NUM_SPI_ADC][SPI_ADC_CHANNELS] = {0};

		for (int adc = 0; adc < NUM_SPI_ADC; adc++) {
			for (int ch = 0; ch < SPI_ADC_CHANNELS; ch++) {
				if (pingPongBuf[processIdx].spi_adc[adc][ch].count > 0) {
					float raw_avg = (float)pingPongBuf[processIdx].spi_adc[adc][ch].sum / pingPongBuf[processIdx].spi_adc[adc][ch].count;
					float v_avg = ADC_RAW_TO_V(raw_avg);
					float v_min = ADC_RAW_TO_V(pingPongBuf[processIdx].spi_adc[adc][ch].min);
					float v_max = ADC_RAW_TO_V(pingPongBuf[processIdx].spi_adc[adc][ch].max);

					if (adc == 0) { // ADC1: Voltage Sens
						float div = 1.0f;
						switch(ch) {
							case 0: div = V_DIV_ADC1_CH0; break;
							case 1: div = V_DIV_ADC1_CH1; break;
							case 2: div = V_DIV_ADC1_CH2; break;
							case 3: div = V_DIV_ADC1_CH3; break;
							case 4: div = V_DIV_ADC1_CH4; break;
							case 5: div = V_DIV_ADC1_CH5; break;
							case 6: div = V_DIV_ADC1_CH6; break;
							default: div = 1.0f; break;
						}
						adc_avg[adc][ch] = (v_avg * div);
						adc_min[adc][ch] = (v_min * div);
						adc_max[adc][ch] = (v_max * div);
					}
					else if (adc == 1) { // ADC2: Current Sens & Temp
						if (ch == 7) {
							adc_avg[adc][ch] = ADC_V_TO_CELSIUS(v_avg);
							adc_min[adc][ch] = ADC_V_TO_CELSIUS(v_min);
							adc_max[adc][ch] = ADC_V_TO_CELSIUS(v_max);
						} else {
							switch(ch) {
								case 0:
									adc_avg[adc][ch] = ADC_V_TO_A_MD(v_avg);
									adc_min[adc][ch] = ADC_V_TO_A_MD(v_min);
									adc_max[adc][ch] = ADC_V_TO_A_MD(v_max);
									break;
								case 1: case 2:
									adc_avg[adc][ch] = ADC_V_TO_A_FP(v_avg);
									adc_min[adc][ch] = ADC_V_TO_A_FP(v_min);
									adc_max[adc][ch] = ADC_V_TO_A_FP(v_max);
									break;
								default:
									adc_avg[adc][ch] = ADC_V_TO_A_FWSW(v_avg);
									adc_min[adc][ch] = ADC_V_TO_A_FWSW(v_min);
									adc_max[adc][ch] = ADC_V_TO_A_FWSW(v_max);
									break;
							}
						}
					}
					else if (adc == 2) { // ADC3: Voltage Sens
						float div = 1.0f;
						switch(ch) {
							case 0: div = V_DIV_ADC3_CH0; break;
							case 1: div = V_DIV_ADC3_CH1; break;
							case 2: div = V_DIV_ADC3_CH2; break;
							case 3: div = V_DIV_ADC3_CH3; break;
							case 4: div = V_DIV_ADC3_CH4; break;
							case 5: div = V_DIV_ADC3_CH5; break;
							case 6: div = V_DIV_ADC3_CH6; break;
							case 7: div = V_DIV_ADC3_CH7; break;
						}
						adc_avg[adc][ch] = (v_avg * div);
						adc_min[adc][ch] = (v_min * div);
						adc_max[adc][ch] = (v_max * div);
					}
				}
				else {
					//Error handling for dropped SPI communications
					adc_avg[adc][ch] = -999.0f;
					adc_min[adc][ch] = -999.0f;
					adc_max[adc][ch] = -999.0f;

					/* SPI Timeout! If channel 0 is missing data, we assume the entire
					   chip failed. We only flag the error bit when ch == 0 to avoid overlaps. */
					if (ch == 0) {
						/* Dynamically map the failed ADC index (0-2) into the reserved bit block */
						current_error_id |= (1UL << (ERR_SPI_START_BIT + adc));
					}
				}
			}
		}

		/* --- 4. ERROR LOGIC & RECONSTRUCTION --- */
		/* Check if the high-priority DAQ task failed to report in this second */
		if (daq_is_alive == 0) {
			current_error_id |= ERR_DAQ_TASK_TO;
		}

		char err_str[256] = ""; /* String buffer to preserve backward compatible error formats */

		/* Translate the populated bitfield back into the string array for JSON output */
		if (current_error_id == 0) {
			strcpy(err_str, "NONE"); /* No errors detected */
		} else {
			/* Check fixed single bits */
			if (current_error_id & ERR_TEL_TASK_TO) strcat(err_str, "TEL_TASK_TO ");
			if (current_error_id & ERR_INT_ADC_TO)  strcat(err_str, "INT_ADC_TO ");

			/* Reconstruct I2C failure strings dynamically based on bits set */
			for (int i = 0; i < NUM_INA260; i++) {
				if (current_error_id & (1UL << (ERR_I2C_START_BIT + i))) {
					char to_str[16];
					snprintf(to_str, sizeof(to_str), "I2C_0x%02X_TO ", INA_I2C_ADDRS[i]);
					strcat(err_str, to_str);
				}
			}

			/* Reconstruct SPI failure strings dynamically based on bits set */
			for (int adc = 0; adc < NUM_SPI_ADC; adc++) {
				if (current_error_id & (1UL << (ERR_SPI_START_BIT + adc))) {
					char to_str[16];
					snprintf(to_str, sizeof(to_str), "SPI_ADC%d_TO ", adc + 1); /* Creates SPI_ADC1_TO etc. */
					strcat(err_str, to_str);
				}
			}

			/* Check final DAQ task timeout bit */
			if (current_error_id & ERR_DAQ_TASK_TO) strcat(err_str, "DAQ_TASK_TO ");

			/* Remove the trailing space applied during concatation for a clean output */
			size_t err_len = strlen(err_str);
			if (err_len > 0) {
				err_str[err_len - 1] = '\0';
			}
		}


		/* --- 5. FORMAT AND SEND UART (JSON LOG) --- */
		static char txBuffer[4096];
		uint32_t uptime_s = xTaskGetTickCount() / 1000;
		int len = 0;

		/* Calculate High-Speed DAQ task execution metrics for the past second */
		float daq_samples = 0, daq_avg_ms = 0.0f, daq_min_ms = 0.0f, daq_max_ms = 0.0f;
		if (pingPongBuf[processIdx].daq_loop_time.count > 0) {
			daq_samples = pingPongBuf[processIdx].daq_loop_time.count;
			float avg_raw_us = (float)pingPongBuf[processIdx].daq_loop_time.sum / daq_samples;
			daq_avg_ms = avg_raw_us / 1000.0f;
			daq_min_ms = (float)pingPongBuf[processIdx].daq_loop_time.min / 1000.0f;
			daq_max_ms = (float)pingPongBuf[processIdx].daq_loop_time.max / 1000.0f;
		}

		/* 5a. Format JSON: Start directly with the core telemetry payload (MS & FW) */
		/* Note: We inject both ErrorID and SideID alongside their string equivalents */
		len += snprintf(txBuffer + len, sizeof(txBuffer) - len,
			"{\"Uptime\":%lu,\"Rst\":\"%s\",\"ErrorID\":%lu,\"Error\":\"%s\",\"HW_ver\":\"0x%02X\",\"SW_ver\":\"%d.%d.%d\",\"UID\":\"0x%08lX\",\"SideID\":%d,\"Side\":\"%s\","
			"\"%s\":{\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},"
			"\"%s\":{\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},",
			uptime_s,
			rst_str,
			(unsigned long)current_error_id,
			err_str,
			hw_version, SW_VERSION_MAJOR, SW_VERSION_MINOR, SW_VERSION_PATCH,
			(unsigned long)uid32,
			side_val,	 // Provides physical side strap ID (0, 1, 2, or 3)
			L(LBL_SIDE), // Translates dynamically to "NONE", "PRIMARY", "SECONDARY", "UNKNOWN"

			// --- MS ---
			L(LBL_MS),
			L(LBL_V1), adc_min[0][6], adc_max[0][6], adc_avg[0][6],
			L(LBL_I1), adc_min[1][6], adc_max[1][6], adc_avg[1][6],
			L(LBL_V2), adc_min[0][5], adc_max[0][5], adc_avg[0][5],
			L(LBL_I2), adc_min[1][5], adc_max[1][5], adc_avg[1][5],

			// --- FW ---
			L(LBL_FW),
			L(LBL_V1), adc_min[0][4], adc_max[0][4], adc_avg[0][4],
			L(LBL_I1), adc_min[1][4], adc_max[1][4], adc_avg[1][4],
			L(LBL_V2), adc_min[0][3], adc_max[0][3], adc_avg[0][3],
			L(LBL_I2), adc_min[1][3], adc_max[1][3], adc_avg[1][3]
		);

		/* 5b. Format JSON: FP, MD, AUX1, CHR */
		/* Note: CHR uses GET_SRC_ID for numerical ID and GET_SRC_STR for the string representation */
		len += snprintf(txBuffer + len, sizeof(txBuffer) - len,
			"\"%s\":{\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},"
			"\"%s\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},"
			"\"%s\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},"
			"\"%s\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"SrcID\":%d,\"Src\":\"%s\"},",

			// --- FP ---
			L(LBL_FP),
			L(LBL_V1), adc_min[0][2], adc_max[0][2], adc_avg[0][2],
			L(LBL_I1), adc_min[1][2], adc_max[1][2], adc_avg[1][2],
			L(LBL_V2), adc_min[0][1], adc_max[0][1], adc_avg[0][1],
			L(LBL_I2), adc_min[1][1], adc_max[1][1], adc_avg[1][1],

			// --- MD ---
			L(LBL_MD),
			adc_min[0][0], adc_max[0][0], adc_avg[0][0], // V
			adc_min[1][0], adc_max[1][0], adc_avg[1][0], // I

			// --- AUX1 ---
			L(LBL_AUX1),
			ina_v_min[2], ina_v_max[2], ina_v_avg[2],    // V
			ina_i_min[2], ina_i_max[2], ina_i_avg[2],    // I

			// --- CHR (Dynamically maps to AUX2 on Secondary) ---
			L(LBL_CHR),
			ina_v_min[6], ina_v_max[6], ina_v_avg[6],    // V
			ina_i_min[6], ina_i_max[6], ina_i_avg[6],    // I
			GET_SRC_ID(muxData.fields.chr1_valid_n, muxData.fields.chr2_valid_n), // Evaluates cleanly to integer values 0,1,2,3
			GET_SRC_STR(muxData.fields.chr1_valid_n, muxData.fields.chr2_valid_n) // Evaluates to string "UPS1", "UPS2", etc.
		);

		/* 5c. Format JSON: AS, MC, FAN, PSEC, LPRC */
		len += snprintf(txBuffer + len, sizeof(txBuffer) - len,
			"\"%s\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"SrcID\":%d,\"Src\":\"%s\"},"
			"\"%s\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},"
			"\"%s\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},"
			"\"%s\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},"
			"\"%s\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},",

			// --- AS (Dynamically maps to AUX3 on Secondary) ---
			L(LBL_AS),
			ina_v_min[4], ina_v_max[4], ina_v_avg[4],
			ina_i_min[4], ina_i_max[4], ina_i_avg[4],
			GET_SRC_ID(muxData.fields.as1_valid_n, muxData.fields.as2_valid_n),
			GET_SRC_STR(muxData.fields.as1_valid_n, muxData.fields.as2_valid_n),

			// --- MC ---
			L(LBL_MC), ina_v_min[0], ina_v_max[0], ina_v_avg[0], ina_i_min[0], ina_i_max[0], ina_i_avg[0],

			// --- FAN ---
			L(LBL_FAN), ina_v_min[1], ina_v_max[1], ina_v_avg[1], ina_i_min[1], ina_i_max[1], ina_i_avg[1],

			// --- PSEC ---
			L(LBL_PSEC), ina_v_min[5], ina_v_max[5], ina_v_avg[5], ina_i_min[5], ina_i_max[5], ina_i_avg[5],

			// --- LPRC ---
			L(LBL_LPRC), ina_v_min[3], ina_v_max[3], ina_v_avg[3], ina_i_min[3], ina_i_max[3], ina_i_avg[3]
		);

		/* 5d. Format JSON: Internal Power Architecture */
		len += snprintf(txBuffer + len, sizeof(txBuffer) - len,
			"\"%s\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},"
			"\"%s\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},"
			"\"28V_INT\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"SrcID\":%d,\"Src\":\"%s\"},"
			"\"7V5\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},"
			"\"5V\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},"
			"\"3V3\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},",

			// --- 28V Inputs (Dynamically swaps UPS1/UPS2 on secondary side) ---
			L(LBL_28V_IN_1), adc_min[2][0], adc_max[2][0], adc_avg[2][0],
			L(LBL_28V_IN_2), adc_min[2][1], adc_max[2][1], adc_avg[2][1],

			// --- Static internal/low voltages ---
			adc_min[2][2], adc_max[2][2], adc_avg[2][2], // 28V INT
			GET_SRC_ID(muxData.fields.int1_valid_n, muxData.fields.int2_valid_n),
			GET_SRC_STR(muxData.fields.int1_valid_n, muxData.fields.int2_valid_n),
			adc_min[2][3], adc_max[2][3], adc_avg[2][3], // 7.5V
			adc_min[2][4], adc_max[2][4], adc_avg[2][4], // 5V
			adc_min[2][5], adc_max[2][5], adc_avg[2][5]  // 3.3V
		);

		/* 5e. Format JSON: Remaining internal rails and PCP */
		len += snprintf(txBuffer + len, sizeof(txBuffer) - len,
			"\"5V_MCU\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},"
			"\"3V3_MCU\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},"
			"\"Temp\":{\"T\":{\"Min\":%.1f,\"Max\":%.1f,\"Avg\":%.1f}},"
			"\"PCP\":{\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%d,\"DIR\":%d,\"OMNI\":%d,\"SEC\":%d}",

			adc_min[2][6], adc_max[2][6], adc_avg[2][6], // 5V_MCU
			adc_min[2][7], adc_max[2][7], adc_avg[2][7], // 3.3V_MCU
			adc_min[1][7], adc_max[1][7], adc_avg[1][7], // Temp

			// --- PCP Digital States using mapped keys ---
			L(LBL_MAIN1),  muxData.fields.di_main1_state,
			L(LBL_MAIN2),  muxData.fields.di_main2_state,
			L(LBL_FP1),    muxData.fields.di_fp1_state,
			L(LBL_FP2),    muxData.fields.di_fp2_state,
			L(LBL_MC_LED), muxData.fields.di_mc_led_on,
			muxData.fields.di_dir_state,
			muxData.fields.di_omni_state,
			muxData.fields.di_sec_state
		);

		int base_len = len;

		/* ========================================================================= */
		/* --- RS422 TRANSMISSION (WITH SYSLOG RFC 5424 HEADER) -------------------- */
		/* ========================================================================= */
		char syslogHeader[180];
        int header_len = snprintf(syslogHeader, sizeof(syslogHeader),
				"{\"PRI\":\"14\",\"VERSION\":\"1\",\"TIMESTAMP\":\"-\",\"HOSTNAME\":\"%s\",\"APP-NAME\":\"PCMU\",\"PROCID\":\"-\",\"MSGID\":\"HW_STATUS\",\"STRUCTURED-DATA\":\"-\",\"MSG\":{",
				L(LBL_PCMU)); // Automatically inserts "PCMU", "PPCMU" or "SPCMU"

		/* RS422: Send Syslog header + Telemetry (without leading {) + Closing brackets */
        HAL_UART_Transmit(&huart1, (uint8_t*)syslogHeader, header_len, 100);
        HAL_UART_Transmit(&huart1, (uint8_t*)(txBuffer + 1), base_len - 1, 500);
        HAL_UART_Transmit(&huart1, (uint8_t*)"}}}\r\n", 4, 50);

        /* ========================================================================= */
        /* --- USB VERBOSE OUTPUT: APPEND FLATTENED DEBUG DATA (PURE JSON) --------- */
        /* ========================================================================= */
        len = base_len;

        /* Calculate how many milliseconds this loop has executed so far */
        uint32_t execution_time_ms = (xTaskGetTickCount() - loop_start_tick) * portTICK_PERIOD_MS;

        /* Read the full uncompressed 96-bit hardware UID array */
        uint32_t uid96[3];
        getUnique96bitID(uid96);

        len += snprintf(txBuffer + len, sizeof(txBuffer) - len,
        		",\"DEBUG\":{"
				"\"UID96\":\"0x%08lX%08lX%08lX\","
				"\"hw_addr3_v\":%.3f,\"hw_addr2_v\":%.3f,\"hw_addr1_v\":%.3f,\"hw_addr0_v\":%.3f,"
				"\"tp2_v\":%.3f,\"tp5_v\":%.3f,\"prim_sel_v\":%.3f,\"sec_sel_v\":%.3f,\"tp9_v\":%.3f,\"tp10_v\":%.3f,"
				"\"main1_v\":%.3f,\"main2_v\":%.3f,\"fp_s1_v\":%.3f,\"fp_s2_v\":%.3f,\"led_v\":%.3f,\"dir_v\":%.3f,\"omni_v\":%.3f,\"sec_v\":%.3f,"
				"\"as_s1_v\":%.3f,\"as_s2_v\":%.3f,\"chr_s1_v\":%.3f,\"chr_s2_v\":%.3f,\"pcmu_s1_v\":%.3f,\"pcmu_s2_v\":%.3f,"
				"\"Tel_Loop_Time_ms\":%lu,"
				"\"Daq_Samples\":%lu,"
				"\"Daq_Loop_Time_ms\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}"
				"}}}\r\n",

			// --- Full 96-bit UID array words (Printed MSB to LSB order) ---
			(unsigned long)uid96[2], (unsigned long)uid96[1], (unsigned long)uid96[0],

			// --- Hardware Board Strapping Address ---
			mux_voltages[2][3], mux_voltages[2][2], mux_voltages[2][1], mux_voltages[2][0],

			// --- Explicit Dedicated Test Points & Side Straps ---
			mux_voltages[0][7], mux_voltages[1][7],
			mux_voltages[2][4], // prim_sel_v (Formerly tp7_v)
			mux_voltages[2][5], // sec_sel_v  (Formerly tp8_v)
			mux_voltages[2][6], mux_voltages[2][7],

			// --- Core Power Control Input Line Voltages ---
			mux_voltages[0][0], mux_voltages[1][0], // main
			mux_voltages[0][1], mux_voltages[1][1], // fp
			mux_voltages[0][2],                     // led
			mux_voltages[0][3],                     // dir
			mux_voltages[1][3],                     // omni
			mux_voltages[1][2],                     // sec

			// --- Redundant Channel Hardware Valid Signal Voltages ---
			mux_voltages[0][4], mux_voltages[1][4], // as
			mux_voltages[0][5], mux_voltages[1][5], // chr
			mux_voltages[0][6], mux_voltages[1][6],  // pcmu

			(unsigned long)execution_time_ms,	//Loop time of Telemetry task
			(unsigned long)daq_samples,         //Number of DAQ samples this second
			daq_min_ms, daq_max_ms, daq_avg_ms  //Loop time of DAQ task
		);

        /* USB: Send manual JSON/MSG wrapper, then payload (without leading {) */
		HAL_UART_Transmit(&huart2, (uint8_t*)"{\"MSG\":{", 8, 100);
		HAL_UART_Transmit(&huart2, (uint8_t*)(txBuffer + 1), len - 1, 500);

		/* --- 6. RESET THE PROCESSED BUFFER FOR THE NEXT ROUND --- */
		/* Wipe sum, count and re-initialize min/max thresholds for the clean buffer */
		ResetBuffer(processIdx);

		/* --- 7. FEED THE WATCHDOG --- */
		/* Reload the 3-second IWDG timer. Since this RS422 task runs exactly once
		   per second (1 Hz), we have a safe 2-second margin. If the system hangs,
		   this function is never called, and the STM32 performs a hardware reset. */
		/* Verify that the high-priority DAQ task is still alive
		   before allowing the system to continue ticking. */
		if (daq_is_alive == 1) {
			HAL_IWDG_Refresh(&hiwdg);
			daq_is_alive = 0; // Reset the flag so the DAQ task must set it again
		}
	} // End of while(1)
} // End of function
