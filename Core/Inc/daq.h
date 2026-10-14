#ifndef INC_DAQ_H_
#define INC_DAQ_H_

/* --- Voltage Dividers MUX4 --- */
// Formula: V = V_adc * (R1 + R2) / R2
#define V_DIV_28V_IN_1        ((120.0f + 12.0f) / 12.0f) // 28V_IN_1
#define V_DIV_28V_IN_2        ((120.0f + 12.0f) / 12.0f) // 28V_IN_2
#define V_DIV_28V_INT         ((120.0f + 12.0f) / 12.0f) // 28V_INT
#define V_DIV_7V5             ((27.0f + 12.0f) / 12.0f)  // 7V5
#define V_DIV_3V3             ((10.0f + 33.0f) / 33.0f)  // 3V3
#define V_DIV_MCU_5V          ((27.0f + 27.0f) / 27.0f)  // 5V
#define V_DIV_MCU_3V3         ((10.0f + 33.0f) / 33.0f)  // 3V3

/* --- SHT41A Parameters --- */
// Formulas from SHT4xA datasheet section 4.5
#define SHT_RAW_TO_TEMP_C(raw)     (-45.0f + 175.0f * ((float)(raw) / 65535.0f))
#define SHT_RAW_TO_HUMID_PERC(raw) (-6.0f + 125.0f * ((float)(raw) / 65535.0f))

/* --- INA260 Parameters --- */
#define INA_CURRENT_LSB    				1.25f 	// mA per bit
#define INA_VOLTAGE_LSB    				1.25f 	// mV per bit
#define INA_POWER_LSB    				10.0f 	// mW per bit
#define INA_RAW_TO_MA(raw)     ((float)(raw) * INA_CURRENT_LSB)	//mA
#define INA_RAW_TO_MV(raw)     ((float)(raw) * INA_VOLTAGE_LSB)	//mV
#define INA_RAW_TO_MW(raw)     ((float)(raw) * INA_POWER_LSB)	//mW
#define INA_RAW_TO_A(raw)      ((float)(raw) * INA_CURRENT_LSB / 1000.0f)	//mA
#define INA_RAW_TO_V(raw)      ((float)(raw) * INA_VOLTAGE_LSB / 1000.0f)	//mV
#define INA_RAW_TO_W(raw)      ((float)(raw) * INA_POWER_LSB / 1000.0f)	//mW

#define INA_REG_CONFIG    		0x00	 // Configuration Register
#define INA_REG_CURRENT   		0x01	 // Current Register
#define INA_REG_VOLTAGE   		0x02	 // Bus Voltage Register
#define INA_REG_POWER     		0x03	 // Power Register
#define INA_REG_MASK     		0x06	 // Mask/Enable Register
#define INA_REG_ALERT_LIMIT     0x07	 // Alert Limit Register
#define INA_REG_MFG_ID    		0xFE	 // Manufacturer ID Register
#define INA_REG_DIE_ID	 		0xFF	 // Die ID Register

#define I2C1_ADR_INA_MC		    0x40	 // I2C Address Mission Computer
#define I2C1_ADR_INA_FAN		0x41	 // I2C Address Fan
#define I2C1_ADR_INA_CUST		0x42     // I2C Address Customer Specific
#define I2C1_ADR_INA_LPRC		0x43	 // I2C Link Power Remote Controller
#define I2C1_ADR_INA_AUTH		0x46	 // I2C Address Authentication Server
#define I2C1_ADR_INA_INT		0x47	 // I2C Address Internal PCMU + Serial To Ethernet Converter
#define I2C1_ADR_INA_CHR		0x48	 // I2C Address Chronometer

#define I2C3_ADR_INA_FP1		0x49	 // I2C Address Flight Panel supply 1
#define I2C3_ADR_INA_FP2		0x4A	 // I2C Address Flight Panel supply 2
#define I2C3_ADR_INA_MD 		0x4B     // I2C Address Mission Display
#define I2C3_ADR_INA_MS1 		0x4C	 // I2C Address Mission Switch supply 1
#define I2C3_ADR_INA_MS2 		0x4D	 // I2C Address Mission Switch supply 2
#define I2C3_ADR_INA_FW1 		0x4E	 // I2C Address Firewall supply 1
#define I2C3_ADR_INA_FW2 		0x4F	 // I2C Address Firewall supply 2

#define I2C3_ADR_TEMP			0x44	 // I2C Address Temperature and Humidity Sensor

void INA260_Init(void);

#endif /* INC_DAQ_H_ */
