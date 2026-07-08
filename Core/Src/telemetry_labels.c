/*
 * serial.c
 *
 *  Created on: 7 juli 2026
 *      Author: johan.abrahamsson
 */

#include <telemetry_labels.h>

/* * String Lookup Table (LUT):
 * Rows correspond to the hardware variants (Side_t).
 * Columns correspond to the specific string labels (LabelID_t).
 * Declared as 'const char* const' to ensure both pointers and strings
 * are stored entirely in Flash memory to save RAM.
 */
const char* const StringTable[NUM_VARIANTS][NUM_LABELS] = {

		/* ========================================================================= */
		/* --- 0: NONE (Standard PCMU on Bench / No straps connected) ----------------------- */
		/* ========================================================================= */
    [SIDE_NONE] = {
        [LBL_SIDE]     = "NONE",
        [LBL_PCMU]     = "PCMU",
        [LBL_MS]       = "MS",
        [LBL_FW]       = "FW",
        [LBL_FP]       = "FP",
        [LBL_MD]       = "MD",
        [LBL_AUX1]     = "AUX1",
        [LBL_CHR]      = "CHR",
        [LBL_AS]       = "AS",
        [LBL_MC]       = "MC",
        [LBL_FAN]      = "FAN",
        [LBL_PSEC]     = "PSEC",
        [LBL_LPRC]     = "LPRC",
        [LBL_28V_IN_1] = "28V_IN_1",	//28V_IN_1 since we don't know what UPS
        [LBL_28V_IN_2] = "28V_IN_2",	//28V_IN_2 since we don't know what UPS
        [LBL_MAIN1]    = "MAIN1",
        [LBL_MAIN2]    = "MAIN2",
        [LBL_FP1]      = "PFP",
        [LBL_FP2]      = "SFP",
        [LBL_MC_LED]   = "MC",
        [LBL_V1]       = "V1",
        [LBL_I1]       = "I1",
		[LBL_V2]       = "V2",
		[LBL_I2]       = "I2"
    },

	/* ========================================================================= */
	/* --- 1: PRIMARY (PPCMU Configuration) ------------------------------------ */
	/* ========================================================================= */
    [SIDE_PRIMARY] = {
        [LBL_SIDE]     = "PRIMARY",
        [LBL_PCMU]     = "PCMU",
        [LBL_MS]       = "MS",
        [LBL_FW]       = "FW",
        [LBL_FP]       = "FP",
        [LBL_MD]       = "MD",
        [LBL_AUX1]     = "AUX1",
        [LBL_CHR]      = "CHR",
        [LBL_AS]       = "AS",
        [LBL_MC]       = "MC",
        [LBL_FAN]      = "FAN",
        [LBL_PSEC]     = "PSEC",
        [LBL_LPRC]     = "LPRC",
        [LBL_28V_IN_1] = "UPS1",
        [LBL_28V_IN_2] = "UPS2",
        [LBL_MAIN1]    = "MAIN1",
        [LBL_MAIN2]    = "MAIN2",
        [LBL_FP1]      = "PFP",
        [LBL_FP2]      = "SFP",
        [LBL_MC_LED]   = "PMC",
        [LBL_V1]       = "V1",
        [LBL_I1]       = "I1",
		[LBL_V2]       = "V2",
		[LBL_I2]       = "I2"
    },

	/* ========================================================================= */
	/* --- 2: SECONDARY (SPCMU Configuration) ---------------------------------- */
	/* ========================================================================= */
    [SIDE_SECONDARY] = {
        [LBL_SIDE]     = "SECONDARY",
        [LBL_PCMU]     = "PCMU",
        [LBL_MS]       = "MS",
        [LBL_FW]       = "FW",
        [LBL_FP]       = "FP",
        [LBL_MD]       = "MD",
        [LBL_AUX1]     = "AUX1",
        [LBL_CHR]      = "AUX2",		//Changed to AUX2 since secondary side.
        [LBL_AS]       = "AUX3",		//Changed to AUX3 since secondary side.
        [LBL_MC]       = "MC",
        [LBL_FAN]      = "FAN",
        [LBL_PSEC]     = "PSEC",
        [LBL_LPRC]     = "LPRC",
        [LBL_28V_IN_1] = "UPS2",		//Switched place since secondary side.
        [LBL_28V_IN_2] = "UPS1",		//Switched place since secondary side.
        [LBL_MAIN1]    = "MAIN2",		//Switched place since secondary side.
        [LBL_MAIN2]    = "MAIN1",		//Switched place since secondary side.
        [LBL_FP1]      = "SFP",
        [LBL_FP2]      = "PFP",
        [LBL_MC_LED]   = "SMC",
        [LBL_V1]       = "V2",			//Switched place since secondary side.
        [LBL_I1]       = "I2",			//Switched place since secondary side.
		[LBL_V2]       = "V1",			//Switched place since secondary side.
		[LBL_I2]       = "I1"			//Switched place since secondary side.
    },

	/* ========================================================================= */
	/* --- 3: UNKNOWN (Fault/Unknown state, both straps connected) ---------------- */
	/* ========================================================================= */
	[SIDE_UNKNOWN] = {
		[LBL_SIDE]     = "UNKNOWN",
		[LBL_PCMU]     = "PCMU",
		[LBL_MS]       = "MS",
		[LBL_FW]       = "FW",
		[LBL_FP]       = "FP",
		[LBL_MD]       = "MD",
		[LBL_AUX1]     = "AUX1",
		[LBL_CHR]      = "CHR",
		[LBL_AS]       = "AS",
		[LBL_MC]       = "MC",
		[LBL_FAN]      = "FAN",
		[LBL_PSEC]     = "PSEC",
		[LBL_LPRC]     = "LPRC",
		[LBL_28V_IN_1] = "28V_IN_1",	//28V_IN_1 since we don't know what UPS
		[LBL_28V_IN_2] = "28V_IN_2",	//28V_IN_2 since we don't know what UPS
		[LBL_MAIN1]    = "MAIN1",
		[LBL_MAIN2]    = "MAIN2",
		[LBL_FP1]      = "PFP",
		[LBL_FP2]      = "SFP",
		[LBL_MC_LED]   = "MC",
		[LBL_V1]       = "V1",
		[LBL_I1]       = "I1",
		[LBL_V2]       = "V2",
		[LBL_I2]       = "I2"
	}
};
