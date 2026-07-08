#ifndef INC_DAQ_H_
#define INC_DAQ_H_

/* --- ADC ADC128S022 --- */
#define V_REF_ADC_MV          5000.0f  // ADC reference voltage (mV)
#define ADC_MAX_VAL           4095.0f  // 12-bit ADC
#define ADC_LSB               (V_REF_ADC_MV / ADC_MAX_VAL)
#define ADC_RAW_TO_MV(raw)    ((float)(raw) * ADC_LSB)
#define ADC_RAW_TO_V(raw)    ((float)(raw) * ADC_LSB / 1000.0f)

/* --- TMP235 Temperature sensor --- */
// T = (V_adc - Voffs) / Tc
// For -40C to +100C, Tc = 10 (mV/C), Voffs = 500 (mV)
#define TMP235_V_OFFS		  500.0f //mV
#define TMP235_TC			  10.0f  // mV/C
#define ADC_MV_TO_CELSIUS(v)  ( ((float)(v) - TMP235_V_OFFS) / TMP235_TC )
#define ADC_V_TO_CELSIUS(v)   ADC_MV_TO_CELSIUS((float)(v) * 1000.0f)

/* --- Voltage Dividers ADC1 --- */
// Formula: V = V_adc * (R1 + R2) / R2
#define V_DIV_ADC1_CH0        ((120.0f + 20.0f) / 20.0f) // MD_U_SENSE
#define V_DIV_ADC1_CH1        ((120.0f + 20.0f) / 20.0f) // FP2_U_SENSE
#define V_DIV_ADC1_CH2        ((120.0f + 20.0f) / 20.0f) // FP1_U_SENSE
#define V_DIV_ADC1_CH3        ((120.0f + 20.0f) / 20.0f) // FW2_U_SENSE
#define V_DIV_ADC1_CH4        ((120.0f + 20.0f) / 20.0f) // FW1_U_SENSE
#define V_DIV_ADC1_CH5        ((120.0f + 20.0f) / 20.0f) // MS2_U_SENSE
#define V_DIV_ADC1_CH6        ((120.0f + 20.0f) / 20.0f) // MS1_U_SENSE
#define V_DIV_ADC1_CH7        1					   //Unused (TP1)

/* --- Voltage to Current ADC2 --- */
// Formula BTH50030-1LUA: I = ((V_adc * k_ILIS) / R_IS)
#define K_ILIS				  34.0f // Current sense differential ratio
#define R_IS_MD				  33.0f // kOhm IS resistor for Mission Display
#define R_IS_FP				  20.0f // kOhm IS resistor for Flight Panel
#define R_IS_FWSW			  82.0f // kOhm IS resistor for Firewall and Switch
#define ADC_V_TO_MA_MD(v) 	  ( ((float)(v) * K_ILIS) / R_IS_MD ) //mA
#define ADC_V_TO_MA_FP(v) 	  ( ((float)(v) * K_ILIS) / R_IS_FP ) //mA
#define ADC_V_TO_MA_FWSW(v)   ( ((float)(v) * K_ILIS) / R_IS_FWSW ) //mA
#define ADC_V_TO_A_MD(v)      ( ((float)(v) * K_ILIS) / (R_IS_MD * 1000.0f) )   // A
#define ADC_V_TO_A_FP(v)      ( ((float)(v) * K_ILIS) / (R_IS_FP * 1000.0f) )   // A
#define ADC_V_TO_A_FWSW(v)    ( ((float)(v) * K_ILIS) / (R_IS_FWSW * 1000.0f) ) // A


// ADC2_CH0		MD_I_SENSE
// ADC2_CH1		FP2_I_SENSE
// ADC2_CH2		FP1_I_SENSE
// ADC2_CH3		FW2_I_SENSE
// ADC2_CH4		FW1_I_SENSE
// ADC2_CH5		MS2_I_SENSE
// ADC2_CH6		MS1_I_SENSE
// ADC2_CH7 	Used for temperature sensor

/* --- Voltage Dividers ADC3 --- */
// Formula: V = V_adc * (R1 + R2) / R2
#define V_DIV_ADC3_CH0        ((120.0f + 20.0f) / 20.0f) // 28V_IN_1
#define V_DIV_ADC3_CH1        ((120.0f + 20.0f) / 20.0f) // 28V_IN_2
#define V_DIV_ADC3_CH2        ((120.0f + 20.0f) / 20.0f) // 28V_INT
#define V_DIV_ADC3_CH3        ((13.0f + 15.0f) / 15.0f)  // 7V5
#define V_DIV_ADC3_CH4        ((3.3f + 15.0f) / 15.0f) // 5V
#define V_DIV_ADC3_CH5        1					   // 3V3
#define V_DIV_ADC3_CH6        ((3.3f + 15.0f) / 15.0f) // 5V
#define V_DIV_ADC3_CH7        1					   // 3V3

/* --- INA260 Parametrar --- */
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

#define INA_I2C_ADR_MC			  0x40	// I2C Address Mission Computer
#define INA_I2C_ADR_FAN			  0x41	// I2C Address Fan
#define INA_I2C_ADR_CUST		  0x42	// I2C Address Customer Specific
#define INA_I2C_ADR_LPRC		  0x44	// I2C Link Power Remote Controller
#define INA_I2C_ADR_AUTH		  0x45	// I2C Address Authentication Server
#define INA_I2C_ADR_INT			  0x48	// I2C Address Internal PCMU + Serial To Ethernet Converter
#define INA_I2C_ADR_CHR			  0x4A	// I2C Address Chronometer

void INA260_Init(void);

#endif /* INC_DAQ_H_ */
