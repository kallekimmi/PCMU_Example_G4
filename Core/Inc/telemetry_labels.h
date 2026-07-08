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
    /* Add additional labels here as needed */
    NUM_LABELS
} LabelID_t;

/* 3. Declare the string lookup table for external linkage */
extern const char* const StringTable[NUM_VARIANTS][NUM_LABELS];

#endif /* INC_TELEMETRY_LABELS_H_ */
