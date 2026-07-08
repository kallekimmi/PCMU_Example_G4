#ifndef INC_BUFFER_H_
#define INC_BUFFER_H_

#include <daq.h>
#include <stdint.h>
#include <limits.h>

// Define sizes
#define NUM_INA260 7
#define NUM_SPI_ADC 3
#define SPI_ADC_CHANNELS 8
#define NUM_MUX 3
#define MUX_CHANNELS 8

// Structure for a single measurement point over 1 second
typedef struct {
    uint32_t sum;
    int32_t min;
    int32_t max;
    uint32_t count;
} SensorStats_t;

// Our entire DAQ buffer
typedef struct {
    SensorStats_t ina_voltage[NUM_INA260];
    SensorStats_t ina_current[NUM_INA260];
    SensorStats_t spi_adc[NUM_SPI_ADC][SPI_ADC_CHANNELS];
    SensorStats_t daq_loop_time;
} DaqBuffer_t;

/* * ============================================================================
 * MULTIPLEXER DATA MAPPING (TYPE PUNNING)
 * ============================================================================
 * In C, a 2D array like raw[3][8] is physically stored as 24 continuous
 * bytes in memory, not as a grid.
 * * By defining a struct with exactly 24 consecutive uint8_t variables, and
 * putting both the struct and the array inside the same 'union', we force
 * them to share the exact same physical memory address.
 * * This gives us two ways to look at the exact same data:
 * 1. The ARRAY view (.raw): Used by the hardware for-loop to iterate easily.
 * 2. The STRUCT view (.fields): Used for human-readable JSON formatting.
 * * Changing raw[0][0] instantly changes fields.di_main1_state, because they
 * are literally the same byte in RAM.
 * ============================================================================
 */
// Structure with explicit names mapped exactly to the schematic (U8, U10, U11)
// Total memory footprint: 24 bytes (3 MUXes * 8 channels)
typedef struct {
    /* --- MUX1 (COM0): Maps sequentially to raw[0][0] through raw[0][7] --- */
    uint8_t di_main1_state; // Byte 0  (raw[0][0])
    uint8_t di_fp1_state;   // Byte 1  (raw[0][1])
    uint8_t di_mc_led_on;   // Byte 2  (raw[0][2])
    uint8_t di_dir_state;   // Byte 3  (raw[0][3])
    uint8_t as1_valid_n;    // Byte 4  (raw[0][4])
    uint8_t chr1_valid_n;   // Byte 5  (raw[0][5])
    uint8_t int1_valid_n;   // Byte 6  (raw[0][6])
    uint8_t mux1_tp2;       // Byte 7  (raw[0][7]) - Y7 tied to GND via resistor

    /* --- MUX2 (COM1): Maps sequentially to raw[1][0] through raw[1][7] --- */
    uint8_t di_main2_state; // Byte 8  (raw[1][0])
    uint8_t di_fp2_state;   // Byte 9  (raw[1][1])
    uint8_t di_sec_state;   // Byte 10 (raw[1][2])
    uint8_t di_omni_state;  // Byte 11 (raw[1][3])
    uint8_t as2_valid_n;    // Byte 12 (raw[1][4])
    uint8_t chr2_valid_n;   // Byte 13 (raw[1][5])
    uint8_t int2_valid_n;   // Byte 14 (raw[1][6])
    uint8_t mux2_tp5;       // Byte 15 (raw[1][7]) - Y7 tied to GND via resistor

    /* --- MUX3 (COM2): Maps sequentially to raw[2][0] through raw[2][7] --- */
    uint8_t hw_addr_0_lsb;  // Byte 16 (raw[2][0])
    uint8_t hw_addr_1;      // Byte 17 (raw[2][1])
    uint8_t hw_addr_2;      // Byte 18 (raw[2][2])
    uint8_t hw_addr_3_msb;  // Byte 19 (raw[2][3])
    uint8_t prim_side_sel_n;// Byte 20 (raw[2][4]) (mux3_tp7)
    uint8_t sec_side_sel_n; // Byte 21 (raw[2][5]) (mux3_tp8)
    uint8_t mux3_tp9;       // Byte 22 (raw[2][6])
    uint8_t mux3_tp10;      // Byte 23 (raw[2][7]) - Y7 tied to GND via resistor
} MuxNamedFields_t;

/* Define all explicit STM32 reset causes for telemetry */
typedef enum {
    RST_BOR = 0, /* Brown-Out Reset (Power cycle/drop) */
    RST_OBL,     /* Option Byte Loader Reset (Memory configuration change) */
    RST_PIN,     /* Hardware Reset Pin (External button/signal) */
    RST_SFT,     /* Software Reset (NVIC_SystemReset) */
    RST_IWDG,    /* Independent Watchdog Reset */
    RST_WWDG,    /* Window Watchdog Reset */
    RST_LPWR,    /* Low-Power Reset */
    RST_FWR,     /* Firewall Reset (Unauthorized memory access) */
    RST_OTH      /* Other / Unknown Reset */
} ResetCause_t;

extern ResetCause_t last_reset_cause;

/* Function prototype for reset cause evaluation */
ResetCause_t GetSystemResetCause(void);

// Union that overlays the 24-byte array directly on top of the 24-byte struct
typedef union {
    uint8_t raw[NUM_MUX][MUX_CHANNELS]; // Write-friendly view for hardware loop
    MuxNamedFields_t fields;            // Read-friendly view for JSON output
} MuxData_t;
// Global variable declaration
extern MuxData_t muxData;

// Global variables for the Ping-Pong buffer (Declarations ONLY)
extern DaqBuffer_t pingPongBuf[2];
extern volatile uint8_t activeBufIdx;

// Address array for INA260 from your header (Declaration ONLY)
extern const uint8_t INA_I2C_ADDRS[NUM_INA260];

// Watchdog safety flag
extern volatile uint8_t daq_is_alive;

void ResetBuffer(uint8_t bufIdx);

#endif /* INC_BUFFER_H_ */
