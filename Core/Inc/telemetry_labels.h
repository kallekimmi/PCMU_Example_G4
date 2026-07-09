/*
 * serial.h
 *
 *  Created on: 7 juli 2026
 *      Author: johan.abrahamsson
 */

#ifndef INC_TELEMETRY_LABELS_H_
#define INC_TELEMETRY_LABELS_H_

/* 1. Define hardware variants */
typedef enum {
    SIDE_NONE      = 0,  /* Neither side active */
    SIDE_PRIMARY   = 1,  /* Primary configuration */
    SIDE_SECONDARY = 2,  /* Secondary configuration */
    SIDE_UNKNOWN   = 3,  /* Both sides active (Fault/Unknown) */
    NUM_VARIANTS
} Side_t;

/* 2. Define unique IDs for all telemetry labels */
typedef enum {
	LBL_SIDE,
	LBL_PCMU,
    LBL_MS,
    LBL_FW,
    LBL_FP,
    LBL_MD,
    LBL_AUX1,
    LBL_CHR,
    LBL_AS,
    LBL_MC,
    LBL_FAN,
    LBL_PSEC,
    LBL_LPRC,
    LBL_28V_IN_1,
    LBL_28V_IN_2,
    LBL_MAIN1,
    LBL_MAIN2,
    LBL_FP1,
    LBL_FP2,
	LBL_MC_LED,
    LBL_V1,
    LBL_I1,
    LBL_V2,
    LBL_I2,
    NUM_LABELS
} LabelID_t;

/* 3. Define Telemetry ErrorID Bitfields */
/* These bits map directly to the ErrorID output in the telemetry JSON. */
#define ERR_TEL_TASK_TO   (1UL << 0)  /* Telemetry task missed its 1 Hz deadline */
#define ERR_INT_ADC_TO    (1UL << 1)  /* STM32 Internal ADC failed to respond */
#define ERR_I2C_START_BIT 2           /* Bits 2-8 used dynamically for the 7 INA260 I2C sensors */
#define ERR_SPI_START_BIT 9           /* Bits 9-11 used dynamically for the 3 SPI ADC chips */
#define ERR_DAQ_TASK_TO   (1UL << 12) /* High-priority DAQ task failed to report alive */
#define ERR_PCP_NC        (1UL << 13) /* Side is 0 (None) - Power Control Panel disconnected */
#define ERR_SIDE_INV      (1UL << 14) /* Side is 3 (Invalid / Unknown / Both) - Hardware strapping fault in PCP side_sel*/
#define ERR_CHR_NO_SRC    (1UL << 15) /* CHR has no valid power source (0) */
#define ERR_CHR_UNKN_SRC  (1UL << 16) /* CHR has both power sources active simultaneously (3) */
#define ERR_AS_NO_SRC     (1UL << 17) /* AS has no valid power source (0) */
#define ERR_AS_UNKN_SRC   (1UL << 18) /* AS has both power sources active simultaneously (3) */
#define ERR_INT_NO_SRC    (1UL << 19) /* 28V_INT has no valid power source (0) */
#define ERR_INT_UNKN_SRC  (1UL << 20) /* 28V_INT has both power sources active simultaneously (3) */

/* 4. Declare the string lookup table for external linkage */
extern const char* const StringTable[NUM_VARIANTS][NUM_LABELS];

#endif /* INC_TELEMETRY_LABELS_H_ */
