/**
  ******************************************************************************
  * @file           : task_telemetry.c
  * @brief          : 1 Hz Processing and RS422 JSON Telemetry Output Task
  ******************************************************************************
  * This task runs exactly once per second. It swaps the high-speed DAQ buffer,
  * processes the accumulated raw samples into engineering units (Volts, Amps),
  * evaluates system health/errors, and transmits a formatted JSON payload
  * over RS422 and USB. It also feeds the hardware watchdog.
  ******************************************************************************
  */

#include <daq.h>
#include "buffer.h"
#include "version.h"
#include "main.h"       // Required for STM32 HAL definitions (MUX_SEL_A_Pin, etc.)
#include "FreeRTOS.h"   // Required for FreeRTOS types (TickType_t)
#include "task.h"       // Required for FreeRTOS task control (vTaskDelayUntil)
#include <stdio.h>
#include <string.h>     // Required for strlen(), strcpy(), strcat()
#include <telemetry_labels.h> // String lookup table, Side_t, LabelID_t, Error definitions

/* External handles initialized by STM32CubeMX in main.c */
extern UART_HandleTypeDef huart1; // RS422 interface for Main Telemetry
extern UART_HandleTypeDef huart2; // USB Virtual COM Port for Verbose Debugging
extern IWDG_HandleTypeDef hiwdg;  // Independent Hardware Watchdog
extern ADC_HandleTypeDef hadc2;	  // STM32 internal ADC (used for MUX reading)

/**
  * @brief  Main task function for Telemetry processing.
  */
void StartTask_Telemetry(void * argument) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // Enforce exactly 1 Hz execution rate

    // Retrieve the unique 32-bit hardware serial number of the STM32 chip
    const uint32_t uid32 = getUnique32bitID();

    // Flag to remember if the previous loop took longer than 1 second to execute
    uint8_t tel_timeout_flag = 0;

    /* ========================================================================= */
	/* --- ONE-TIME STARTUP HARDWARE READ & LOCKING ---------------------------- */
	/* ========================================================================= */

    float startup_mux_voltages[NUM_MUX][MUX_CHANNELS] = {0};

	/* Sample all multiplexer channels once at boot to lock in the static
       hardware strapping configuration (Board address, Primary/Secondary role). */
	for(uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
        // Set the 3-bit physical address lines (A, B, C) on the hardware MUX chips
		HAL_GPIO_WritePin(MUX_SEL_A_GPIO_Port, MUX_SEL_A_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MUX_SEL_B_GPIO_Port, MUX_SEL_B_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MUX_SEL_C_GPIO_Port, MUX_SEL_C_Pin, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        // Brief delay to allow the analog signal to settle across the MUX
		for(volatile int nop = 0; nop < 10; nop++);
		HAL_ADC_Start(&hadc2);

        // Read all 3 MUX chips (connected to ADC channels in a sequence)
		for(int m = 0; m < NUM_MUX; m++) {
			if (HAL_ADC_PollForConversion(&hadc2, 1) == HAL_OK) {
				uint32_t raw_value = HAL_ADC_GetValue(&hadc2);
                // Convert 12-bit raw ADC value to Volts (assuming 3.3V reference)
				startup_mux_voltages[m][ch] = ((float)raw_value * 3.3f) / 4095.0f;
			} else {
				startup_mux_voltages[m][ch] = -999.0f; // Mark as error if ADC fails
			}
		}

		/* Populate the global muxData union immediately. This is required so that
           version.c can decode the hardware version and blink the LED accordingly.
		   We use a 1.5V threshold to convert the analog voltage into a digital 0 or 1. */
		muxData.raw[0][ch] = (startup_mux_voltages[0][ch] > 1.5f) ? 1 : 0;
		muxData.raw[1][ch] = (startup_mux_voltages[1][ch] > 1.5f) ? 1 : 0;
		muxData.raw[2][ch] = (startup_mux_voltages[2][ch] > 1.5f) ? 1 : 0;
	}

	/* Translate the latest STM32 hardware reset cause into a 3-character string.
	   This value is locked in at startup and remains static for the current uptime. */
	const char* rst_str = "OTH";
	switch(last_reset_cause) {
		case RST_BOR:  rst_str = "BOR"; break; // Brown-Out Reset (Power loss)
		case RST_OBL:  rst_str = "OBL"; break; // Option Byte Loader Reset
		case RST_PIN:  rst_str = "PIN"; break; // External Reset Pin pulled low
		case RST_SFT:  rst_str = "SFT"; break; // Software forced reset
		case RST_IWDG: rst_str = "WDG"; break; // Independent Watchdog timeout
		case RST_WWDG: rst_str = "WWD"; break; // Window Watchdog violation
		case RST_LPWR: rst_str = "LPW"; break; // Low-Power sleep violation
		case RST_FWR:  rst_str = "FWR"; break; // Firewall security reset
		default:       rst_str = "OTH"; break; // Unknown reset cause
	}

	/* Decode hardware board version (4-bit binary address read from MUX) */
	const uint8_t hw_version = (muxData.fields.hw_addr_3_msb << 3) |
							   (muxData.fields.hw_addr_2     << 2) |
							   (muxData.fields.hw_addr_1     << 1) |
							   (muxData.fields.hw_addr_0_lsb);

	/* Determine board role (Primary or Secondary).
       Signals are active LOW (_n), so we invert them: 1 = Active, 0 = Inactive. */
	const uint8_t prim = !muxData.fields.prim_side_sel_n;
	const uint8_t sec  = !muxData.fields.sec_side_sel_n;

    // Combine bits into a single value: 0=NONE, 1=PRIMARY, 2=SECONDARY, 3=UNKNOWN
	const uint8_t side_val = (sec << 1) | prim;

    // Cast the side value directly to our enumerated type for string table lookups
	Side_t active_variant = (Side_t)side_val;

	/* ========================================================================= */
	/* --- MACROS FOR STRING LOOKUP & SAFE APPENDING --------------------------- */
	/* ========================================================================= */

    /* Macro to easily fetch string labels based on the active hardware configuration. */
    #define L(id) StringTable[active_variant][id]

    /* Macro to generate a 2-bit numerical ID from redundant power sources.
       Returns: 0 (No source), 1 (Source 1), 2 (Source 2), 3 (Both sources/Fault). */
	#define GET_SRC_ID(valid1_n, valid2_n) ( ((!(valid2_n)) << 1) | (!(valid1_n)) )

    /* Macro to resolve which redundant source is currently active as a string. */
	#define GET_SRC_STR(valid1_n, valid2_n) \
		(((valid1_n) == (valid2_n)) ? ((valid1_n) ? "NONE" : "UNKNOWN") : \
		(!(valid1_n) ? L(LBL_28V_IN_1) : L(LBL_28V_IN_2)))

	/* SAFE BUFFER APPEND MACRO
	 * Safely writes formatted string data into the txBuffer. It calculates the
     * remaining space and ignores the write if the buffer is full, preventing
     * out-of-bounds memory corruption (HardFaults).
	 */
	#define APPEND_JSON(...) do { \
		if (len < sizeof(txBuffer) - 1) { \
			int space_left = sizeof(txBuffer) - len; \
			int ret = snprintf(txBuffer + len, space_left, __VA_ARGS__); \
			if (ret > 0) { len += ret; } \
		} \
	} while(0)

	/* ========================================================================= */
	/* --- MAIN TASK LOOP (RUNS AT 1 HZ) --------------------------------------- */
	/* ========================================================================= */
    while(1) {
        // Record exact start time to measure task execution duration later
		TickType_t start_of_processing = xTaskGetTickCount();

		/* 1. SLEEP UNTIL NEXT CYCLE (Enforces strict 1 Hz timing) */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

    	/* 2. EVALUATE ERRORS FOR THE CURRENT SECOND */
		uint32_t current_error_id = 0; // Reset error bitfield

        // Check structural hardware strapping faults
		if (side_val == SIDE_NONE) current_error_id |= ERR_PCP_NC;
		else if (side_val == SIDE_UNKNOWN) current_error_id |= ERR_SIDE_INV;

        // Check if the telemetry task took too long during the previous loop
		if (tel_timeout_flag) {
			current_error_id |= ERR_TEL_TASK_TO;
			tel_timeout_flag = 0; // Clear the flag after logging it
		}

        // Check if the High-Speed DAQ task failed to report in
		if (daq_is_alive == 0) {
			current_error_id |= ERR_DAQ_TASK_TO;
		}

        /* 3. SWAP BUFFER (Ping-Pong Architecture) */
        // Grab the buffer index that the DAQ task just finished filling
        uint8_t processIdx = activeBufIdx;
        // Immediately flip the active index so the DAQ task can start filling the clean buffer
        activeBufIdx = !activeBufIdx;

        /* 4. SAMPLE MUX CHANNELS (ANALOG READ) */
		float mux_voltages[NUM_MUX][MUX_CHANNELS] = {0};

        for(uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
            // Set hardware address lines
			HAL_GPIO_WritePin(MUX_SEL_A_GPIO_Port, MUX_SEL_A_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX_SEL_B_GPIO_Port, MUX_SEL_B_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX_SEL_C_GPIO_Port, MUX_SEL_C_Pin, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

            // Wait for analog voltage to propagate through the multiplexer
			for(volatile int nop = 0; nop < 10; nop++);
			HAL_ADC_Start(&hadc2);

            // Fetch the 3 corresponding analog values
			for(int m = 0; m < NUM_MUX; m++) {
				if (HAL_ADC_PollForConversion(&hadc2, 1) == HAL_OK) {
					uint32_t raw_value = HAL_ADC_GetValue(&hadc2);
					mux_voltages[m][ch] = ((float)raw_value * 3.3f) / 4095.0f;
				} else {
                    // Internal ADC timeout occurred
					mux_voltages[m][ch] = -999.0f;
					current_error_id |= ERR_INT_ADC_TO;
				}
			}
		}

		/* 5. EVALUATE REDUNDANT SOURCE FLAGS */
        // Chronometer Source
		uint8_t chr_src_id = GET_SRC_ID(muxData.fields.chr1_valid_n, muxData.fields.chr2_valid_n);
		const char* chr_src_str = GET_SRC_STR(muxData.fields.chr1_valid_n, muxData.fields.chr2_valid_n);

        // Authentication Server Source
		uint8_t as_src_id = GET_SRC_ID(muxData.fields.as1_valid_n, muxData.fields.as2_valid_n);
		const char* as_src_str = GET_SRC_STR(muxData.fields.as1_valid_n, muxData.fields.as2_valid_n);

        // Internal 28V Bus Source
		uint8_t int_src_id = GET_SRC_ID(muxData.fields.int1_valid_n, muxData.fields.int2_valid_n);
		const char* int_src_str = GET_SRC_STR(muxData.fields.int1_valid_n, muxData.fields.int2_valid_n);

        // Flag errors if a source is completely lost (0) or overlapping/shorted (3)
		if (chr_src_id == 0) current_error_id |= ERR_CHR_NO_SRC;
		else if (chr_src_id == 3) current_error_id |= ERR_CHR_UNKN_SRC;

		if (as_src_id == 0) current_error_id |= ERR_AS_NO_SRC;
		else if (as_src_id == 3) current_error_id |= ERR_AS_UNKN_SRC;

		if (int_src_id == 0) current_error_id |= ERR_INT_NO_SRC;
		else if (int_src_id == 3) current_error_id |= ERR_INT_UNKN_SRC;

		/* 6. CONVERT RAW DATA TO ENGINEERING UNITS */

        // --- Process INA260 I2C Sensors ---
        float ina_v_avg[NUM_INA260] = {0}, ina_v_min[NUM_INA260] = {0}, ina_v_max[NUM_INA260] = {0};
		float ina_i_avg[NUM_INA260] = {0}, ina_i_min[NUM_INA260] = {0}, ina_i_max[NUM_INA260] = {0};

		for (int i = 0; i < NUM_INA260; i++) {
            // Check if the sensor successfully responded during this second
			if (pingPongBuf[processIdx].ina_voltage[i].count > 0) {
				float v_avg_raw = (float)pingPongBuf[processIdx].ina_voltage[i].sum / pingPongBuf[processIdx].ina_voltage[i].count;
				ina_v_avg[i] = INA_RAW_TO_V(v_avg_raw);
				ina_v_min[i] = INA_RAW_TO_V(pingPongBuf[processIdx].ina_voltage[i].min);
				ina_v_max[i] = INA_RAW_TO_V(pingPongBuf[processIdx].ina_voltage[i].max);
			} else {
                // Sensor missing/timeout - Flag the specific I2C address bit
				ina_v_avg[i] = -999.0f; ina_v_min[i] = -999.0f; ina_v_max[i] = -999.0f;
				current_error_id |= (1UL << (ERR_I2C_START_BIT + i));
			}

			if (pingPongBuf[processIdx].ina_current[i].count > 0) {
				float i_avg_raw = (float)pingPongBuf[processIdx].ina_current[i].sum / pingPongBuf[processIdx].ina_current[i].count;
				ina_i_avg[i] = INA_RAW_TO_A(i_avg_raw);
				ina_i_min[i] = INA_RAW_TO_A(pingPongBuf[processIdx].ina_current[i].min);
				ina_i_max[i] = INA_RAW_TO_A(pingPongBuf[processIdx].ina_current[i].max);
			} else {
				ina_i_avg[i] = -999.0f; ina_i_min[i] = -999.0f; ina_i_max[i] = -999.0f;
			}
		}

        // --- Process SPI ADC Sensors ---
		float adc_avg[NUM_SPI_ADC][SPI_ADC_CHANNELS] = {0};
		float adc_min[NUM_SPI_ADC][SPI_ADC_CHANNELS] = {0};
		float adc_max[NUM_SPI_ADC][SPI_ADC_CHANNELS] = {0};

		for (int adc = 0; adc < NUM_SPI_ADC; adc++) {
			for (int ch = 0; ch < SPI_ADC_CHANNELS; ch++) {
                // Check if the DAQ task successfully captured data for this channel
				if (pingPongBuf[processIdx].spi_adc[adc][ch].count > 0) {
					float raw_avg = (float)pingPongBuf[processIdx].spi_adc[adc][ch].sum / pingPongBuf[processIdx].spi_adc[adc][ch].count;
					float v_avg = ADC_RAW_TO_V(raw_avg), v_min = ADC_RAW_TO_V(pingPongBuf[processIdx].spi_adc[adc][ch].min), v_max = ADC_RAW_TO_V(pingPongBuf[processIdx].spi_adc[adc][ch].max);

                    // Apply specific hardware voltage dividers or current sense formulas
					if (adc == 0) { // ADC1: High Voltage Sensing
						float div = 1.0f;
						switch(ch) {
							case 0: div = V_DIV_ADC1_CH0; break; case 1: div = V_DIV_ADC1_CH1; break;
							case 2: div = V_DIV_ADC1_CH2; break; case 3: div = V_DIV_ADC1_CH3; break;
							case 4: div = V_DIV_ADC1_CH4; break; case 5: div = V_DIV_ADC1_CH5; break;
							case 6: div = V_DIV_ADC1_CH6; break;
						}
						adc_avg[adc][ch] = (v_avg * div); adc_min[adc][ch] = (v_min * div); adc_max[adc][ch] = (v_max * div);
					}
					else if (adc == 1) { // ADC2: Current Sensing & Temperature
						if (ch == 7) {
							adc_avg[adc][ch] = ADC_V_TO_CELSIUS(v_avg); adc_min[adc][ch] = ADC_V_TO_CELSIUS(v_min); adc_max[adc][ch] = ADC_V_TO_CELSIUS(v_max);
						} else {
							switch(ch) {
								case 0:
									adc_avg[adc][ch] = ADC_V_TO_A_MD(v_avg); adc_min[adc][ch] = ADC_V_TO_A_MD(v_min); adc_max[adc][ch] = ADC_V_TO_A_MD(v_max); break;
								case 1: case 2:
									adc_avg[adc][ch] = ADC_V_TO_A_FP(v_avg); adc_min[adc][ch] = ADC_V_TO_A_FP(v_min); adc_max[adc][ch] = ADC_V_TO_A_FP(v_max); break;
								default:
									adc_avg[adc][ch] = ADC_V_TO_A_FWSW(v_avg); adc_min[adc][ch] = ADC_V_TO_A_FWSW(v_min); adc_max[adc][ch] = ADC_V_TO_A_FWSW(v_max); break;
							}
						}
					}
					else if (adc == 2) { // ADC3: Internal Low Voltage Sensing
						float div = 1.0f;
						switch(ch) {
							case 0: div = V_DIV_ADC3_CH0; break; case 1: div = V_DIV_ADC3_CH1; break;
							case 2: div = V_DIV_ADC3_CH2; break; case 3: div = V_DIV_ADC3_CH3; break;
							case 4: div = V_DIV_ADC3_CH4; break; case 5: div = V_DIV_ADC3_CH5; break;
							case 6: div = V_DIV_ADC3_CH6; break; case 7: div = V_DIV_ADC3_CH7; break;
						}
						adc_avg[adc][ch] = (v_avg * div); adc_min[adc][ch] = (v_min * div); adc_max[adc][ch] = (v_max * div);
					}
				}
				else {
					// Handle dropped SPI communications
					adc_avg[adc][ch] = -999.0f; adc_min[adc][ch] = -999.0f; adc_max[adc][ch] = -999.0f;

                    // If channel 0 is missing, assume the entire chip failed to avoid redundant errors
					if (ch == 0) current_error_id |= (1UL << (ERR_SPI_START_BIT + adc));
				}
			}
		}

		/* 7. DYNAMIC ERROR STRING RECONSTRUCTION */
		char err_str[256] = "";
		if (current_error_id == 0) {
			strcpy(err_str, "NONE"); // Clean system
		} else {
            // Append single-bit system faults
			if (current_error_id & ERR_TEL_TASK_TO)  strcat(err_str, "TEL_TASK_TO ");
			if (current_error_id & ERR_INT_ADC_TO)   strcat(err_str, "INT_ADC_TO ");
			if (current_error_id & ERR_PCP_NC)       strcat(err_str, "PCP_NC ");
			if (current_error_id & ERR_SIDE_INV)     strcat(err_str, "SIDE_INV ");

            // Append Source mapping faults
            if (current_error_id & ERR_CHR_NO_SRC)   strcat(err_str, "CHR_NO_SRC ");
			if (current_error_id & ERR_CHR_UNKN_SRC) strcat(err_str, "CHR_UNKN_SRC ");
			if (current_error_id & ERR_AS_NO_SRC)    strcat(err_str, "AS_NO_SRC ");
			if (current_error_id & ERR_AS_UNKN_SRC)  strcat(err_str, "AS_UNKN_SRC ");
			if (current_error_id & ERR_INT_NO_SRC)   strcat(err_str, "INT_NO_SRC ");
			if (current_error_id & ERR_INT_UNKN_SRC) strcat(err_str, "INT_UNKN_SRC ");

			// Reconstruct specific I2C chip timeouts dynamically
			for (int i = 0; i < NUM_INA260; i++) {
				if (current_error_id & (1UL << (ERR_I2C_START_BIT + i))) {
					char to_str[16];
					snprintf(to_str, sizeof(to_str), "I2C_0x%02X_TO ", INA_I2C_ADDRS[i]);
					strcat(err_str, to_str);
				}
			}

			// Reconstruct specific SPI ADC chip timeouts dynamically
			for (int adc = 0; adc < NUM_SPI_ADC; adc++) {
				if (current_error_id & (1UL << (ERR_SPI_START_BIT + adc))) {
					char to_str[16];
					snprintf(to_str, sizeof(to_str), "SPI_ADC%d_TO ", adc + 1);
					strcat(err_str, to_str);
				}
			}

			if (current_error_id & ERR_DAQ_TASK_TO) strcat(err_str, "DAQ_TASK_TO ");

			// Remove the final trailing space for a clean string output
			size_t err_len = strlen(err_str);
			if (err_len > 0) err_str[err_len - 1] = '\0';
		}


		/* 8. FORMAT AND ASSEMBLE JSON PAYLOAD */
        /* By breaking the formatting into smaller segments, we prevent the MCU's
		   system heap/stack from overflowing when formatting multiple floats simultaneously. */
		static char txBuffer[4096];
		uint32_t uptime_s = xTaskGetTickCount() / 1000;
		size_t len = 0; // Tracks the current index in the txBuffer

        // Calculate execution statistics for the High-Speed DAQ task
		float daq_samples = 0, daq_avg_ms = 0.0f, daq_min_ms = 0.0f, daq_max_ms = 0.0f;
		if (pingPongBuf[processIdx].daq_loop_time.count > 0) {
			daq_samples = pingPongBuf[processIdx].daq_loop_time.count;
			float avg_raw_us = (float)pingPongBuf[processIdx].daq_loop_time.sum / daq_samples;
			daq_avg_ms = avg_raw_us / 1000.0f;
			daq_min_ms = (float)pingPongBuf[processIdx].daq_loop_time.min / 1000.0f;
			daq_max_ms = (float)pingPongBuf[processIdx].daq_loop_time.max / 1000.0f;
		}

		/* 8a. Core Header Variables */
		APPEND_JSON("{\"Uptime\":%lu,\"Rst\":\"%s\",\"ErrorID\":%lu,\"Error\":\"%s\",", (unsigned long)uptime_s, rst_str, (unsigned long)current_error_id, err_str);
		APPEND_JSON("\"HW_ver\":\"0x%02X\",\"SW_ver\":\"%d.%d.%d\",\"UID\":\"0x%08lX\",", hw_version, SW_VERSION_MAJOR, SW_VERSION_MINOR, SW_VERSION_PATCH, (unsigned long)uid32);
		APPEND_JSON("\"SideID\":%d,\"Side\":\"%s\",", side_val, L(LBL_SIDE));

		/* 8b. Main Switch (MS) */
		APPEND_JSON("\"%s\":{", L(LBL_MS));
		APPEND_JSON("\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", L(LBL_V1), adc_min[0][6], adc_max[0][6], adc_avg[0][6]);
		APPEND_JSON("\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", L(LBL_I1), adc_min[1][6], adc_max[1][6], adc_avg[1][6]);
		APPEND_JSON("\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", L(LBL_V2), adc_min[0][5], adc_max[0][5], adc_avg[0][5]);
		APPEND_JSON("\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", L(LBL_I2), adc_min[1][5], adc_max[1][5], adc_avg[1][5]);

		/* 8c. Firewall (FW) */
		APPEND_JSON("\"%s\":{", L(LBL_FW));
		APPEND_JSON("\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", L(LBL_V1), adc_min[0][4], adc_max[0][4], adc_avg[0][4]);
		APPEND_JSON("\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", L(LBL_I1), adc_min[1][4], adc_max[1][4], adc_avg[1][4]);
		APPEND_JSON("\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", L(LBL_V2), adc_min[0][3], adc_max[0][3], adc_avg[0][3]);
		APPEND_JSON("\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", L(LBL_I2), adc_min[1][3], adc_max[1][3], adc_avg[1][3]);

		/* 8d. Flight Panel (FP) */
		APPEND_JSON("\"%s\":{", L(LBL_FP));
		APPEND_JSON("\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", L(LBL_V1), adc_min[0][2], adc_max[0][2], adc_avg[0][2]);
		APPEND_JSON("\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", L(LBL_I1), adc_min[1][2], adc_max[1][2], adc_avg[1][2]);
		APPEND_JSON("\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", L(LBL_V2), adc_min[0][1], adc_max[0][1], adc_avg[0][1]);
		APPEND_JSON("\"%s\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", L(LBL_I2), adc_min[1][1], adc_max[1][1], adc_avg[1][1]);

		/* 8e. Mission Display & Auxiliary Output 1 (MD & AUX1) */
		APPEND_JSON("\"%s\":{", L(LBL_MD));
		APPEND_JSON("\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", adc_min[0][0], adc_max[0][0], adc_avg[0][0]);
		APPEND_JSON("\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", adc_min[1][0], adc_max[1][0], adc_avg[1][0]);

		APPEND_JSON("\"%s\":{", L(LBL_AUX1));
		APPEND_JSON("\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", ina_v_min[2], ina_v_max[2], ina_v_avg[2]);
		APPEND_JSON("\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", ina_i_min[2], ina_i_max[2], ina_i_avg[2]);

		/* 8f. Chronometer & Authentication Server (CHR & AS) */
		APPEND_JSON("\"%s\":{", L(LBL_CHR));
		APPEND_JSON("\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", ina_v_min[6], ina_v_max[6], ina_v_avg[6]);
		APPEND_JSON("\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", ina_i_min[6], ina_i_max[6], ina_i_avg[6]);
		APPEND_JSON("\"SrcID\":%d,\"Src\":\"%s\"},", chr_src_id, chr_src_str);

		APPEND_JSON("\"%s\":{", L(LBL_AS));
		APPEND_JSON("\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", ina_v_min[4], ina_v_max[4], ina_v_avg[4]);
		APPEND_JSON("\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", ina_i_min[4], ina_i_max[4], ina_i_avg[4]);
		APPEND_JSON("\"SrcID\":%d,\"Src\":\"%s\"},", as_src_id, as_src_str);

		/* 8g. Mission Computer, Fans, Power SEC, Link Power RC (MC, FAN, PSEC, LPRC) */
		APPEND_JSON("\"%s\":{", L(LBL_MC));
		APPEND_JSON("\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", ina_v_min[0], ina_v_max[0], ina_v_avg[0]);
		APPEND_JSON("\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", ina_i_min[0], ina_i_max[0], ina_i_avg[0]);

		APPEND_JSON("\"%s\":{", L(LBL_FAN));
		APPEND_JSON("\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", ina_v_min[1], ina_v_max[1], ina_v_avg[1]);
		APPEND_JSON("\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", ina_i_min[1], ina_i_max[1], ina_i_avg[1]);

		APPEND_JSON("\"%s\":{", L(LBL_PSEC));
		APPEND_JSON("\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", ina_v_min[5], ina_v_max[5], ina_v_avg[5]);
		APPEND_JSON("\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", ina_i_min[5], ina_i_max[5], ina_i_avg[5]);

		APPEND_JSON("\"%s\":{", L(LBL_LPRC));
		APPEND_JSON("\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},", ina_v_min[3], ina_v_max[3], ina_v_avg[3]);
		APPEND_JSON("\"I\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", ina_i_min[3], ina_i_max[3], ina_i_avg[3]);

		/* 8h. Internal Power Architecture (28V Inputs & Logic Voltages) */
		APPEND_JSON("\"%s\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", L(LBL_28V_IN_1), adc_min[2][0], adc_max[2][0], adc_avg[2][0]);
		APPEND_JSON("\"%s\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", L(LBL_28V_IN_2), adc_min[2][1], adc_max[2][1], adc_avg[2][1]);

		APPEND_JSON("\"28V_INT\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f},\"SrcID\":%d,\"Src\":\"%s\"},", adc_min[2][2], adc_max[2][2], adc_avg[2][2], int_src_id, int_src_str);
		APPEND_JSON("\"7V5\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", adc_min[2][3], adc_max[2][3], adc_avg[2][3]);
		APPEND_JSON("\"5V\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", adc_min[2][4], adc_max[2][4], adc_avg[2][4]);
		APPEND_JSON("\"3V3\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", adc_min[2][5], adc_max[2][5], adc_avg[2][5]);
		APPEND_JSON("\"5V_MCU\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", adc_min[2][6], adc_max[2][6], adc_avg[2][6]);
		APPEND_JSON("\"3V3_MCU\":{\"V\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}},", adc_min[2][7], adc_max[2][7], adc_avg[2][7]);
		APPEND_JSON("\"Temp\":{\"T\":{\"Min\":%.1f,\"Max\":%.1f,\"Avg\":%.1f}},", adc_min[1][7], adc_max[1][7], adc_avg[1][7]);

		/* 8i. Digital Power Control Panel States (PCP) */
		APPEND_JSON("\"PCP\":{\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%d,\"DIR\":%d,\"OMNI\":%d,\"SEC\":%d}",
			L(LBL_MAIN1),  muxData.fields.di_main1_state,
			L(LBL_MAIN2),  muxData.fields.di_main2_state,
			L(LBL_FP1),    muxData.fields.di_fp1_state,
			L(LBL_FP2),    muxData.fields.di_fp2_state,
			L(LBL_MC_LED), muxData.fields.di_mc_led_on,
			muxData.fields.di_dir_state,
			muxData.fields.di_omni_state,
			muxData.fields.di_sec_state
		);

		/* Save base_len. This indicates where the primary operational payload ends.
           Everything added after this is strictly for the USB debug output. */
		size_t base_len = len;

		/* ========================================================================= */
		/* --- RS422 TRANSMISSION (WITH SYSLOG RFC 5424 HEADER) -------------------- */
		/* ========================================================================= */

        char syslogHeader[180];
        int header_len = snprintf(syslogHeader, sizeof(syslogHeader),
				"{\"PRI\":\"14\",\"VERSION\":\"1\",\"TIMESTAMP\":\"-\",\"HOSTNAME\":\"%s\",\"APP-NAME\":\"PCMU\",\"PROCID\":\"-\",\"MSGID\":\"HW_STATUS\",\"STRUCTURED-DATA\":\"-\",\"MSG\":{",
				L(LBL_PCMU));

		/* Output over RS422 interface.
           We send the Syslog Header, then the JSON payload (skipping the leading '{'),
           and finally append the closing brackets. Timeout set to 1 second. */
		if (base_len > 0) {
			HAL_UART_Transmit(&huart1, (uint8_t*)syslogHeader, header_len, 100);
			HAL_UART_Transmit(&huart1, (uint8_t*)(txBuffer + 1), base_len - 1, 1000);
			HAL_UART_Transmit(&huart1, (uint8_t*)"}}}\r\n", 4, 100);
		}

        /* ========================================================================= */
        /* --- USB VERBOSE OUTPUT: APPEND FLATTENED DEBUG DATA (PURE JSON) --------- */
        /* ========================================================================= */

        // Calculate the physical time (in ms) it took for this task to compute the JSON
        uint32_t execution_time_ms = (xTaskGetTickCount() - start_of_processing) * portTICK_PERIOD_MS;

        // Fetch uncompressed hardware UID for deep diagnostics
        uint32_t uid96[3];
        getUnique96bitID(uid96);

		APPEND_JSON(",\"DEBUG\":{");
		APPEND_JSON("\"Tx_Msg_Len\":%lu,\"Tx_Buffer_Margin\":%ld,", (unsigned long)len, (long)(sizeof(txBuffer) - len));
		APPEND_JSON("\"UID96\":\"0x%08lX%08lX%08lX\",", (unsigned long)uid96[2], (unsigned long)uid96[1], (unsigned long)uid96[0]);

        // Hardware Board Strapping Address (Analog Voltages)
		APPEND_JSON("\"hw_addr3_v\":%.3f,\"hw_addr2_v\":%.3f,\"hw_addr1_v\":%.3f,\"hw_addr0_v\":%.3f,", mux_voltages[2][3], mux_voltages[2][2], mux_voltages[2][1], mux_voltages[2][0]);

        // Explicit Dedicated Test Points & Side Straps
        APPEND_JSON("\"tp2_v\":%.3f,\"tp5_v\":%.3f,\"prim_sel_v\":%.3f,\"sec_sel_v\":%.3f,", mux_voltages[0][7], mux_voltages[1][7], mux_voltages[2][4], mux_voltages[2][5]);
		APPEND_JSON("\"tp9_v\":%.3f,\"tp10_v\":%.3f,\"main1_v\":%.3f,\"main2_v\":%.3f,", mux_voltages[2][6], mux_voltages[2][7], mux_voltages[0][0], mux_voltages[1][0]);

        // Core Power Control Input Line Voltages
		APPEND_JSON("\"fp_s1_v\":%.3f,\"fp_s2_v\":%.3f,\"led_v\":%.3f,\"dir_v\":%.3f,", mux_voltages[0][1], mux_voltages[1][1], mux_voltages[0][2], mux_voltages[0][3]);
		APPEND_JSON("\"omni_v\":%.3f,\"sec_v\":%.3f,\"as_s1_v\":%.3f,\"as_s2_v\":%.3f,", mux_voltages[1][3], mux_voltages[1][2], mux_voltages[0][4], mux_voltages[1][4]);

        // Redundant Channel Hardware Valid Signal Voltages
        APPEND_JSON("\"chr_s1_v\":%.3f,\"chr_s2_v\":%.3f,\"pcmu_s1_v\":%.3f,\"pcmu_s2_v\":%.3f,", mux_voltages[0][5], mux_voltages[1][5], mux_voltages[0][6], mux_voltages[1][6]);

        // Execution Times and Sampling Frequencies
		APPEND_JSON("\"Tel_Loop_Time_ms\":%lu,\"Daq_Samples\":%lu,", (unsigned long)execution_time_ms, (unsigned long)daq_samples);
		APPEND_JSON("\"Daq_Loop_Time_ms\":{\"Min\":%.3f,\"Max\":%.3f,\"Avg\":%.3f}", daq_min_ms, daq_max_ms, daq_avg_ms);
		APPEND_JSON("}}}\r\n");

        /* USB output wrapper. It mimics the syslog 'MSG' format so external parsers don't break. */
		if (len > 0) {
			HAL_UART_Transmit(&huart2, (uint8_t*)"{\"MSG\":{", 8, 100);
			HAL_UART_Transmit(&huart2, (uint8_t*)(txBuffer + 1), len - 1, 1000);
		}

		/* Did this entire processing block take longer than 1 second?
           Flag it so it can be logged as an error on the next cycle. */
		if ((xTaskGetTickCount() - start_of_processing) > xFrequency) {
			tel_timeout_flag = 1;
		}

		/* 9. RESET BUFFER & FEED WATCHDOG */
		// Clear statistics from the buffer we just read, preparing it for the DAQ task
		ResetBuffer(processIdx);

        // Verify the DAQ loop is still updating its alive flag
		if (daq_is_alive == 1) {
            // Pet the hardware watchdog to prevent system reboot
			HAL_IWDG_Refresh(&hiwdg);
			daq_is_alive = 0; // Clear the flag, forcing the DAQ task to set it again
		}
	} // End of infinite while loop
} // End of function
