/*
 * GPSDO.c
 *
 * Created: 2020-11-22 00:40:06
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 * AVR/GNU C Compiler : 4.8.1
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <avr/eeprom.h>
#include "types.h"
#include "macro.h"
#include "config.h"
#include "led.h"
#include "twi_master_driver.h"
#include "mcp47x6.h"
#include "usart_driver.h"
#include "cbuffer.h"
#include "terminal.h"
#include "terminal_commands.h"
#include "adc.h"
#include "delays.h"
#include "lowpass_filter.h"


/*
GPSDO - GPS Disciplined Oscillator - 40MHz, 0.1ppm
Hardware resources: 
TWI Master:
    TWIC	DAC
USART:
    C0		GPS
    D0		Terminal
ADC:
    MUX1	Temperature
Event:
    CH0MUX	Frequency counter
    CH1MUX	Timer D0 overflow
Timer:
    D0		Frequency counter, lo word
    D1		Frequency counter, hi word
Interrupt Levels:
    TWIC	    Medium      DAC
    PORTC INT0	    Low         GPS 1PPS gate
    RTC	OVF	    Low         Common trigger
    ADCA CH0	    Low		Temperature
    USARTC0 RXC	    Low		GPS
    USARTD0 RXC	    Low		Terminal
*/

#define VERSION_MAJOR				"1"
#define VERSION_MINOR				"1"
#define DEVICE_INFO_SIZE			64
const char DEVICE_INFO[DEVICE_INFO_SIZE]	PROGMEM = "GPSDO "VERSION_MAJOR "." VERSION_MINOR " (build: " __DATE__ " " __TIME__ ")\n\r";

#define MAIN_RTC_PERIOD_ms			50
#define MAIN_TIMEOUT_MAINCLOCK_ms		200

#define TWI_CLK					100000
#define TWI_BAUDSETTING				TWI_BAUD(F_CPU, TWI_CLK)

#define DAC_VREF_mv				2500
#define DAC_VALUE_LOW				(uint16_t)(((uint32_t)(CONFIG_VCXO_VOLTAGE_CONTROL_LOW_mv) * 4096) / DAC_VREF_mv)
#define DAC_VALUE_HIGH				(uint16_t)(((uint32_t)(CONFIG_VCXO_VOLTAGE_CONTROL_HIGH_mv) * 4096) / DAC_VREF_mv)
#define DAC_VALUE_BASE				(uint16_t)((DAC_VALUE_HIGH + DAC_VALUE_LOW) / 2)
#define DAC_CORRECTION_FACTOR			1.8
#define DAC_FILTER_SCALE			2
#define DAC_WORD_LENGTH				12

#define VCXO_FREQUENCY_LOW			(CONFIG_VCXO_NOMINAL_FREQUENCY - (2 * CONFIG_VCXO_ADJUST_RANGE_ppm * CONFIG_VCXO_NOMINAL_FREQUENCY) / 1000000)
#define VCXO_FREQUENCY_HIGH			(CONFIG_VCXO_NOMINAL_FREQUENCY + (2 * CONFIG_VCXO_ADJUST_RANGE_ppm * CONFIG_VCXO_NOMINAL_FREQUENCY) / 1000000)

#define SEQUENTIAL_TO_FIX_CALIBRATION		(uint8_t)(16)
#define TARGET_PRECISION_ppm			0.1

#define CSV_DELIMITER				";"

enum {
	MAIN_ERROR_MAIN_CLOCK,
	MAIN_ERROR_ADC_TIMEOUT,
	MAIN_ERROR_FREQ_OUT_OF_RANGE
};


#define FREQUENCY_DEVIATION_ARRAY_SIZE		500
#define FREQUENCY_DEVIATION_TEMP_BASE_K10	(uint16_t)(2831)
#define FREQUENCY_DEVIATION_MASK		0x001F
#define FREQUENCY_DEVIATION_MASK_LENGTH		5
#define FREQUENCY_DEVIATION_EMPTY_CELL		0xFFFF
// most significant bits - DAC value (base)
// least significant bits - half of the DAC deviation value 
uint16_t EEMEM frequency_deviation[FREQUENCY_DEVIATION_ARRAY_SIZE];

#if (DAC_WORD_LENGTH > (16 - FREQUENCY_DEVIATION_MASK_LENGTH))
#define DAC_WORD_SHRINK				(uint8_t)(DAC_WORD_LENGTH - (16 - FREQUENCY_DEVIATION_MASK_LENGTH))
#else
#define DAC_WORD_SHRINK				0
#endif 



GPSDO_State_t gpsdo_state = {
	.device_info = DEVICE_INFO,
	.gps_fixed = false,
	.calibrated = false,
	.frequency_measured = false,
	.sequential_calibrated = 0,
	.count_checks = 0,
	.count_calibrated = 0,
	.count_loss_calibration = 0,
	.max_diff_above_nominal = 0,
	.max_diff_below_nominal = 0,
	.forward_gps_message = true,
	.show_gps_message = false,
	.show_gpsdo_message = false,
	.gpsdo_status_format_csv = false,
	.disable_frequency_correction = false,
	.disable_gps_correction = false,
	.errors = 0,
	.frequency_deviation = frequency_deviation,
	.show_frequency_deviation = false,
	.clear_frequency_deviation = false,
	.clear_frequency_deviation_keep_base = true,
	};

volatile bool main_request_led = false;
volatile bool main_request_adc = false;
volatile bool main_request_frq = false;
bool main_request_show_state = false;

LED_t led_D1;
LED_t led_D2;

TWI_Master_t twiMaster;
MCP47X6_t dac;
USART_DRIVER_t usart_terminal;
USART_DRIVER_t usart_gps;

#define BUFFER_TERMINAL_DATA_LENGTH		150
volatile char buffer_terminal_data[BUFFER_TERMINAL_DATA_LENGTH];
CircularBuffer_t buffer_terminal;
TERMINAL_t terminal;

#define BUFFER_GPS_DATA_LENGTH			80
volatile char buffer_gps_data[BUFFER_GPS_DATA_LENGTH];
CircularBuffer_t buffer_gps;

#define SWITCH_TERMINAL_PIPE_CHAR		(0x40)
#define SWITCH_TERMINAL_PIPE_REPEATED_KEY	8
uint8_t switch_terminal_pipe_repeated_key_count = 0;

LPFu16_t filter_temperature;
#define FILTER_TEMPRATURE_LENGTH		4
LPFu16_t filter_dac_value;
#define FILTER_DAC_VALUE_LENGTH			3

#define MAIN_TIMER_ADCCONVERSION_ms		1000
DELAY_t timeout_adc_conversion_ms;
#define MAIN_TIMER_PPS_SIGNAL_ms		2500
DELAY_t timeout_pps_signal_lost_ms;
#define MAIN_TIMER_STATE_SHOW_ms		2000
DELAY_t timeout_show_state_ms;
#define MAIN_TIMER_CORRECTION_ms		500
DELAY_t timeout_correction_on_temp_ms;


// Prototypes of functions
void CCPWrite(volatile uint8_t *address, uint8_t value);
void main_RC32MClockInit(uint8_t timeout_ms);
void main_RTCInit(void);
void main_PowerReduction(void);
void main_LedUpdate(void);
void main_ADCInitConversion(void);
void main_FrequencyDividerReset(void);
void main_ClearFrequencyCounter(void);
bool main_IsFrequencyInRange(uint32_t f);
void main_GPSModuleReset(void);
uint16_t main_NTCTermistorToKelvin(uint16_t adc_raw);
void main_ClearSyncState(void);
void main_ShowTemperature(void);
void main_ShowErrorPpm(void);
void main_ShowStatus(void);
void main_SetFixPin(bool fixed);
void main_SendGPSBuffer(void);
void main_SetDAC(uint16_t val);
bool main_GetFrequencyDeviationArrayIndex(uint16_t temp_k10, uint16_t *idx);
bool main_ReadEEPROMDeviation(uint16_t idx, uint16_t *min, uint16_t *max);
bool main_GetFrequencyDeviationBase(uint16_t idx, uint16_t *base);
void main_UpdateEEPROMDeviation(uint16_t idx, uint16_t min, uint16_t max);
void main_CatchFrequencyDeviation(void);
void main_ShowEEPROMDeviation(void);
void main_ClearEEPROMDeviation(bool keep_base);
float main_ErrorFrequencyToPPM(int f);


int main(void)
{	
 	// Clock initialization
 	main_RC32MClockInit(MAIN_TIMEOUT_MAINCLOCK_ms);
	main_RTCInit();

	// Power reduction
	main_PowerReduction();
	
	// Initialize LED 
#ifdef CONFIG_LED_D1_port
	led_Init(&led_D1, &CONFIG_LED_D1_port, CONFIG_LED_D1_pin);
#endif
#ifdef CONFIG_LED_D2_port
	led_Init(&led_D2, &CONFIG_LED_D2_port, CONFIG_LED_D2_pin);
	led_SetTimers(&led_D2, 10, 4);
#endif
	led_SetLed(&led_D1, true, false);
	led_SetLed(&led_D2, true, false);
	main_LedUpdate();
	
	// Initialize interrupt
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_RREN_bm;
	sei();

	// Initialize TWI master
	TWI_MasterInit(&twiMaster, &TWIC, TWI_MASTER_INTLVL_MED_gc, TWI_BAUDSETTING); 
	
	// Initialize DAC
	mcp47x6_Init(&dac, &twiMaster, 0x00);
	mcp47x6_SetConfiguration(&dac, MCP47X6_GAIN_1X, MCP47X6_VREF_VREFPIN_BUFFERED);
	uint16_t saved_level;
	mcp47x6_ReadSavedLevel(&dac, &saved_level);
	if (saved_level != DAC_VALUE_BASE) {
		mcp47x6_SaveToEEPROM(&dac, DAC_VALUE_BASE);
	}
	gpsdo_state.dac_value = DAC_VALUE_BASE;
	lpfilter_Set(&filter_dac_value, FILTER_DAC_VALUE_LENGTH);
	lpfilter_Fill(&filter_dac_value, (DAC_VALUE_BASE << DAC_FILTER_SCALE));
	
	// Initialize Terminal
	usart_Init(&usart_terminal, &USARTD0, &PORTD, PIN3_bm, PIN2_bm);
	usart_SetBaudrate(&usart_terminal, BAUD_RATE_19200);
	usart_RxEnable(&usart_terminal, true);
	usart_TxEnable(&usart_terminal, true);
 	cbuffer_Init(&buffer_terminal, buffer_terminal_data, BUFFER_TERMINAL_DATA_LENGTH);
 	terminal_Init(&terminal, &usart_terminal, &buffer_terminal);
	terminal_commands_Init(&gpsdo_state);
	terminal_SendNL(&terminal, true);
	terminal_PrintPM(&terminal, DEVICE_INFO, true);
	terminal_SendNL(&terminal, true);

	// Initialize GPS
	setbit(CONFIG_GPS_RESET_port.DIRCLR, CONFIG_GPS_RESET_pin);
	*(&(CONFIG_GPS_RESET_port.PIN0CTRL) + CONFIG_GPS_RESET_pin) = PORT_OPC_WIREDANDPULL_gc;
	cbuffer_Init(&buffer_gps, buffer_gps_data, BUFFER_GPS_DATA_LENGTH);
	usart_Init(&usart_gps, &USARTC0, &PORTC, PIN3_bm, PIN2_bm);
	usart_SetBaudrate(&usart_gps, BAUD_RATE_9600);
	usart_RxEnable(&usart_gps, true);
	usart_TxEnable(&usart_gps, true);

	// First part initialization complete
	_delay_ms(10);
	led_SetLed(&led_D1, false, false);
	led_SetLed(&led_D2, false, true);
	main_LedUpdate();

	// Initialize ADC
	delays_Init(&timeout_adc_conversion_ms, MAIN_TIMER_ADCCONVERSION_ms);
	delays_Pause(&timeout_adc_conversion_ms);
	adc_Setup(glue3(ADC_REFSEL_, CONFIG_ADC_REFERENCE, _gc));
	adc_SelectInput(ADC_CH_MUXPOS_PIN1_gc);
	adc_BindMuxInput(1, glue3(ADC_CH_MUXPOS_PIN, CONFIG_ADC_TEMPERATURE_VCXO_mux, _gc));
	lpfilter_Set(&filter_temperature, FILTER_TEMPRATURE_LENGTH);
	main_ADCInitConversion();

	// Initialize 1PPS detector
	// 74LVC1G80 positive-edge triggered D-type flip-flop
	// U-BLOX-M8_ReceiverDescrProtSpec_(UBX-13003221).pdf - C.20 Timepulse Settings (UBX-CFG-TP5)
	delays_Init(&timeout_pps_signal_lost_ms, MAIN_TIMER_PPS_SIGNAL_ms);
	*(&(PORTC.PIN0CTRL) + CONFIG_GPS_1PPS_GATE_pin) = PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc;
	PORTC.INT0MASK = (1 << CONFIG_GPS_1PPS_GATE_pin);
	PORTC.INTCTRL = PORT_INT0LVL_LO_gc;
	PORTC.INTFLAGS = PORT_INT0IF_bm;
	
	// Initialize frequency counter
	setbit(CONFIG_FREQUENCY_MR_port.DIRSET, CONFIG_FREQUENCY_MR_pin);
	setbit(CONFIG_FREQUENCY_MR_port.OUT, CONFIG_FREQUENCY_MR_pin);
	*(&(CONFIG_FREQUENCY_Q3_port.PIN0CTRL) + CONFIG_FREQUENCY_Q3_pin) = PORT_ISC_FALLING_gc;
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTB_PIN0_gc | CONFIG_FREQUENCY_Q3_pin;
	EVSYS.CH1MUX = EVSYS_CHMUX_TCD0_OVF_gc;
	TCD0.CTRLA = TC_CLKSEL_EVCH0_gc;
	TCD0.CNT = 0;
	TCD0.CTRLD = TC_EVACT_CAPT_gc | TC_EVSEL_CH0_gc;
	TCD0.CTRLB = TC0_CCAEN_bm;
	TCD1.CTRLA = TC_CLKSEL_EVCH1_gc;
	TCD1.CNT = 0;
	TCD1.CTRLD = TC_EVACT_CAPT_gc | TC_EVSEL_CH1_gc | TC1_EVDLY_bm;
	TCD1.CTRLB = TC1_CCAEN_bm;
	  	
	// Initialize 'Fixed Frequency' pin
	setbit(CONFIG_FREQUENCY_FIXED_port.DIRSET, CONFIG_FREQUENCY_FIXED_pin);	
	main_SetFixPin(gpsdo_state.calibrated);
	
	delays_Init(&timeout_show_state_ms, MAIN_TIMER_STATE_SHOW_ms);
	delays_Init(&timeout_correction_on_temp_ms, MAIN_TIMER_CORRECTION_ms);
	uint8_t command_length;
	uint16_t temperature_adc;
	uint32_t temperature_filter_ready = 1 << FILTER_TEMPRATURE_LENGTH;
	
	led_SetLed(&led_D1, true, false);
	while(1) {
		// Update LED
		if (main_request_led) {
			main_request_led = false;
			main_LedUpdate();
		}
		// Read ADC and start next conversion
		if (main_request_adc) {
			temperature_adc = adc_ReadInput();
			gpsdo_state.temperature_raw = lpfilter_Filter(&filter_temperature, temperature_adc);
			gpsdo_state.temperature_k100 = main_NTCTermistorToKelvin(gpsdo_state.temperature_raw);
			if (temperature_filter_ready > 0) temperature_filter_ready--;
			main_ADCInitConversion();
		}
		// Read frequency
		if (main_request_frq) {
			main_request_frq = false;
			gpsdo_state.gps_fixed = true;
			_delay_us(1);
			gpsdo_state.frequency = (((uint32_t)TCD1.CNT << 16) | TCD0.CNT) << 4;
			if (testbit(CONFIG_FREQUENCY_Q3_port.IN, CONFIG_FREQUENCY_Q3_pin)) gpsdo_state.frequency |= (1 << 3);
			if (testbit(CONFIG_FREQUENCY_Q2_port.IN, CONFIG_FREQUENCY_Q2_pin)) gpsdo_state.frequency |= (1 << 2);
			if (testbit(CONFIG_FREQUENCY_Q1_port.IN, CONFIG_FREQUENCY_Q1_pin)) gpsdo_state.frequency |= (1 << 1);
			if (testbit(CONFIG_FREQUENCY_Q0_port.IN, CONFIG_FREQUENCY_Q0_pin)) gpsdo_state.frequency |= 1;
			delays_Reset(&timeout_pps_signal_lost_ms);
 			main_FrequencyDividerReset();
			if (main_IsFrequencyInRange(gpsdo_state.frequency)) {
				gpsdo_state.frequency_measured = true;
				main_request_show_state = true;
				gpsdo_state.error_frequency = gpsdo_state.frequency - CONFIG_VCXO_NOMINAL_FREQUENCY;
				if (!gpsdo_state.disable_gps_correction) {
					main_SetDAC(gpsdo_state.dac_value - DAC_CORRECTION_FACTOR * gpsdo_state.error_frequency);
				}
				gpsdo_state.error_ppm = main_ErrorFrequencyToPPM(gpsdo_state.error_frequency);
				if (gpsdo_state.error_ppm <= TARGET_PRECISION_ppm) {
					if (gpsdo_state.sequential_calibrated >= SEQUENTIAL_TO_FIX_CALIBRATION) {
						if (!(gpsdo_state.calibrated)) {
							gpsdo_state.calibrated = true;
							gpsdo_state.max_diff_above_nominal = 0;
							gpsdo_state.max_diff_below_nominal = 0;
						}
					} else {
						gpsdo_state.sequential_calibrated++;	
					}
				} 
				if (gpsdo_state.error_ppm > 2 * TARGET_PRECISION_ppm) {
					if (gpsdo_state.calibrated) gpsdo_state.count_loss_calibration++;
					main_ClearSyncState();
				}
				if (gpsdo_state.calibrated) {
					gpsdo_state.count_calibrated++;
					if (gpsdo_state.error_frequency > gpsdo_state.max_diff_above_nominal) {
						gpsdo_state.max_diff_above_nominal = gpsdo_state.error_frequency;
					}
					if (gpsdo_state.error_frequency < gpsdo_state.max_diff_below_nominal) {
						gpsdo_state.max_diff_below_nominal = gpsdo_state.error_frequency;
					}
					if (gpsdo_state.error_ppm <= TARGET_PRECISION_ppm) main_CatchFrequencyDeviation();
				}
			} else {
				gpsdo_state.frequency_measured = false;
				if (gpsdo_state.show_gpsdo_message) terminal_Print(&terminal, "Frequency out of range\n\r", false);
				setbit(gpsdo_state.errors, MAIN_ERROR_FREQ_OUT_OF_RANGE);
				if (gpsdo_state.calibrated) gpsdo_state.count_loss_calibration++;
				main_ClearSyncState();
			}
			main_ClearFrequencyCounter();	
			gpsdo_state.count_checks++;
		}
		// Correction based on temperature
		if ((!gpsdo_state.frequency_measured || gpsdo_state.disable_gps_correction) && temperature_filter_ready == 0)	{
			if (delays_Check(&timeout_correction_on_temp_ms)) {
				uint16_t idx;
				uint16_t base;
				if (main_GetFrequencyDeviationArrayIndex(gpsdo_state.temperature_k100, &idx)) {
					if (main_GetFrequencyDeviationBase(idx, &base)) {
						main_SetDAC(base);
					}
				}
				delays_Reset(&timeout_correction_on_temp_ms);
			}
		}
		// Check terminal buffer
		terminal_SendOutBuffer(&terminal);
		if (terminal_IsBufferFull(&terminal)) terminal_DropInputBuffer(&terminal);
		if (terminal_FindNewLine(&terminal, &command_length)) terminal_ParseCommand(&terminal, command_length);
		if (gpsdo_state.clear_frequency_deviation) main_ClearEEPROMDeviation(gpsdo_state.clear_frequency_deviation_keep_base);
		if (gpsdo_state.show_frequency_deviation) main_ShowEEPROMDeviation();
		// Check GPS buffer
		 main_SendGPSBuffer();
		// Check timers
		if (delays_Check(&timeout_adc_conversion_ms)) {
			main_ADCInitConversion();
			led_SetLed(&led_D1, true, true);
			setbit(gpsdo_state.errors, MAIN_ERROR_ADC_TIMEOUT);
		}
		if (delays_Check(&timeout_pps_signal_lost_ms)) {
			gpsdo_state.gps_fixed = false;
			gpsdo_state.frequency_measured = false;
			if (gpsdo_state.calibrated) gpsdo_state.count_loss_calibration++;
			main_ClearSyncState();
 			main_FrequencyDividerReset();
			_delay_us(1);
			main_ClearFrequencyCounter();
		}
		// Show state
		if ((!gpsdo_state.frequency_measured && delays_Check(&timeout_show_state_ms)) || main_request_show_state) {
			if (gpsdo_state.show_gpsdo_message) main_ShowStatus();
			delays_Reset(&timeout_show_state_ms);
			main_request_show_state = false;
		}
		// Show "Fixed Frequency"
		main_SetFixPin(gpsdo_state.calibrated);
		led_SetLed(&led_D2, gpsdo_state.gps_fixed, !(gpsdo_state.calibrated));		
	}
}

// Timers
ISR(RTC_OVF_vect)
{
	main_request_led = true;
	delays_Update(&timeout_adc_conversion_ms, MAIN_RTC_PERIOD_ms);
	delays_Update(&timeout_pps_signal_lost_ms, MAIN_RTC_PERIOD_ms);
	delays_Update(&timeout_show_state_ms, MAIN_RTC_PERIOD_ms);
	delays_Update(&timeout_correction_on_temp_ms, MAIN_RTC_PERIOD_ms);
}

// ADC conversion complete
ISR(ADCA_CH0_vect)
{
	main_request_adc = true;
}

// TWIC Master Interrupt vector.
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twiMaster);
}

ISR(USARTC0_RXC_vect)
{
	char c = usart_ReadChar(&usart_gps);
	if (gpsdo_state.forward_gps_message) {
		if (c == 0x24) gpsdo_state.show_gps_message = true; 
		if (gpsdo_state.show_gps_message) cbuffer_AppendChar(&buffer_terminal, c);
	}
}

ISR(USARTD0_RXC_vect)
{
	char c = usart_ReadChar(terminal.usart);
	if (gpsdo_state.forward_gps_message) {
		if (c == SWITCH_TERMINAL_PIPE_CHAR) {
			switch_terminal_pipe_repeated_key_count++;
		} else {
			switch_terminal_pipe_repeated_key_count = 0;
		}
		if (switch_terminal_pipe_repeated_key_count >= SWITCH_TERMINAL_PIPE_REPEATED_KEY) {
			switch_terminal_pipe_repeated_key_count = 0;
			gpsdo_state.forward_gps_message = false;
			gpsdo_state.show_gps_message = false;
			terminal_SendWelcome(&terminal);
			command_SystemVersion(&terminal);
		}
		if (!cbuffer_IsFull(&buffer_gps)) {
			cbuffer_AppendChar(&buffer_gps, c);
		}
	} else {		
		gpsdo_state.show_gpsdo_message = false;
		if (!terminal_IsBufferFull(&terminal)) {
			terminal_ParseChar(&terminal, c);
		}
	}
}

ISR(PORTC_INT0_vect) 
{   
	main_request_frq = true;
}

// From Application Note AVR1003
void CCPWrite(volatile uint8_t *address, uint8_t value)
{
	uint8_t volatile saved_sreg = SREG;
	cli();

	#ifdef __ICCAVR__
	asm("movw r30, r16");
	#ifdef RAMPZ
	RAMPZ = 0;
	#endif
	asm("ldi  r16,  0xD8 \n"
	"out  0x34, r16  \n"
	#if (__MEMORY_MODEL__ == 1)
	"st     Z,  r17  \n");
	#elif (__MEMORY_MODEL__ == 2)
	"st     Z,  r18  \n");
	#else /* (__MEMORY_MODEL__ == 3) || (__MEMORY_MODEL__ == 5) */
	"st     Z,  r19  \n");
	#endif /* __MEMORY_MODEL__ */

	#elif defined __GNUC__
	volatile uint8_t * tmpAddr = address;
	#ifdef RAMPZ
	RAMPZ = 0;
	#endif
	asm volatile(
	"movw r30,  %0"	      "\n\t"
	"ldi  r16,  %2"	      "\n\t"
	"out   %3, r16"	      "\n\t"
	"st     Z,  %1"       "\n\t"
	:
	: "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
	: "r16", "r30", "r31"
	);

	#endif
	SREG = saved_sreg;
}

void main_RC32MClockInit(uint8_t timeout_ms) 
{
	OSC.CTRL = OSC_RC32MEN_bm;
	while (timeout_ms) {
		_delay_ms(1);
		if (OSC.STATUS & OSC_RC32MRDY_bm) {
			CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);
			break;
		}
		timeout_ms--;
		if (!timeout_ms) {
			setbit(gpsdo_state.errors, MAIN_ERROR_MAIN_CLOCK);
		}
	}
}

void main_RTCInit(void)
{
	CLK.RTCCTRL = CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm;
	while (RTC.STATUS & RTC_SYNCBUSY_bm) {};
	RTC.PER = (uint16_t)MAIN_RTC_PERIOD_ms;
	RTC.INTCTRL = RTC_OVFINTLVL_LO_gc;
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;
}

void main_PowerReduction(void) 
{
	// Power reduction: Stop unused peripherals
	PR.PRGEN = PR_USB_bm | PR_AES_bm | PR_EBI_bm | PR_DMA_bm;
	PR.PRPA = PR_DAC_bm | PR_AC_bm;
	PR.PRPB = PR_DAC_bm | PR_ADC_bm | PR_AC_bm;	
	PR.PRPC = PR_USART1_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC0_bm | PR_TC1_bm;
	PR.PRPD = PR_USART1_bm | PR_TWI_bm | PR_SPI_bm | PR_HIRES_bm;
	PR.PRPE = PR_USART0_bm | PR_USART1_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC0_bm | PR_TC1_bm;
}

void main_LedUpdate(void)
{
	led_Update(&led_D1);
	led_Update(&led_D2);
}

void main_ADCInitConversion(void)
{
 	main_request_adc = false;
	adc_StartConversion();
	delays_Reset(&timeout_adc_conversion_ms);
}

void main_FrequencyDividerReset(void)
{
	setbit(CONFIG_FREQUENCY_MR_port.OUTCLR, CONFIG_FREQUENCY_MR_pin);
	_delay_us(1);
	setbit(CONFIG_FREQUENCY_MR_port.OUTSET, CONFIG_FREQUENCY_MR_pin);
}

bool main_IsFrequencyInRange(uint32_t f)
{
	if (f < VCXO_FREQUENCY_LOW) return false;
	if (f > VCXO_FREQUENCY_HIGH) return false;
	return true;
}

void main_ClearFrequencyCounter(void)
{
	TCD0.CNT = 0;
	_delay_us(1);
	TCD1.CNT = 0;
}

void main_GPSModuleReset(void)
{
	setbit(CONFIG_GPS_RESET_port.OUTCLR, CONFIG_GPS_RESET_pin);
	_delay_us(1);
	setbit(CONFIG_GPS_RESET_port.OUTSET, CONFIG_GPS_RESET_pin);
}

uint16_t main_NTCTermistorToKelvin(uint16_t adc_raw)
{
	// temperature LSB = 0.01K
	#define vref_adc	2.5	// 2.5V
	#define vref_ntc	2.5	// 2.5V
	#define adc_range	4096
	#define r1		10000L	// R1
	#define beta		3435L	// NTC B=3435
	#define ntc		10000L	// R2 NTC 10k
	#define t0		298.15	// 298.15K = 25C
	
	uint16_t result = 0;
	float volt = (float)adc_raw * vref_adc / adc_range;	
	if ((vref_ntc - volt) > 0) {
		float rz = volt * r1 / (vref_ntc - volt);
		float t = t0 * beta / (beta + (t0 * log(rz / ntc)));
		result = (uint16_t)(100 * t);
	}
	return result;
}

void main_ClearSyncState(void)
{
	gpsdo_state.calibrated = false;
	gpsdo_state.sequential_calibrated = 0;
	gpsdo_state.count_calibrated = 0;
}

void main_ShowTemperature(void)
{
	// shows temperature in degrees Celsius
	if (!gpsdo_state.gpsdo_status_format_csv) {
		terminal_Print(&terminal, "Temperature=", false);
	}
	char buffer[8];
	ftoa((float)(gpsdo_state.temperature_k100 - 27315) / 100, buffer, 8);
	terminal_Print(&terminal, buffer, false);
	if (!gpsdo_state.gpsdo_status_format_csv) {
		terminal_Print(&terminal, "\xF8""C" , false);
	}
}

void main_ShowErrorPpm(void)
{
	if (gpsdo_state.error_ppm < 0.1) {
		terminal_Print(&terminal, "<0.1", false);
	} else {
		if (!gpsdo_state.gpsdo_status_format_csv) {
			terminal_Print(&terminal, " ", false);
		}
		char ppm_buffer[8];
		ftoa(gpsdo_state.error_ppm, ppm_buffer, 8);
		terminal_Print(&terminal, ppm_buffer, false);
	}
	if (!gpsdo_state.gpsdo_status_format_csv) {
		terminal_Print(&terminal, "ppm", false);
	}
}

void main_ShowStatus(void)
{
	if (gpsdo_state.gpsdo_status_format_csv) {
		if (gpsdo_state.frequency_measured) terminal_PrintULong(&terminal, gpsdo_state.count_checks, false);
		terminal_Print(&terminal, CSV_DELIMITER, false);
		if (gpsdo_state.frequency_measured) terminal_PrintULong(&terminal, gpsdo_state.count_calibrated, false);
		terminal_Print(&terminal, CSV_DELIMITER, false);
		if (gpsdo_state.frequency_measured) terminal_PrintULong(&terminal, gpsdo_state.count_loss_calibration, false);
		terminal_Print(&terminal, CSV_DELIMITER, false);
		if (gpsdo_state.frequency_measured) terminal_PrintInt(&terminal, gpsdo_state.max_diff_below_nominal, false);
		terminal_Print(&terminal, CSV_DELIMITER, false);
		if (gpsdo_state.frequency_measured) terminal_PrintInt(&terminal, gpsdo_state.max_diff_above_nominal, false);
		terminal_Print(&terminal, CSV_DELIMITER , false);
		if (gpsdo_state.frequency_measured) terminal_PrintULong(&terminal, gpsdo_state.frequency, false);
		terminal_Print(&terminal, CSV_DELIMITER, false);
		if (gpsdo_state.frequency_measured) terminal_PrintInt(&terminal, gpsdo_state.error_frequency, false);
		terminal_Print(&terminal, CSV_DELIMITER, false);
		if (gpsdo_state.frequency_measured) main_ShowErrorPpm();
		terminal_Print(&terminal, CSV_DELIMITER, false);
		main_ShowTemperature();
		terminal_Print(&terminal, CSV_DELIMITER, false);
		terminal_PrintInt(&terminal, gpsdo_state.dac_value_diff_last, false);
		terminal_SendNL(&terminal, false);	
	} else {
		if (gpsdo_state.frequency_measured) {
			terminal_Print(&terminal, "CHK=", false);
			terminal_PrintULong(&terminal, gpsdo_state.count_checks, false);
			terminal_Print(&terminal, ", ", false);
			if (gpsdo_state.disable_gps_correction) terminal_Print(&terminal, "*", false);
			terminal_Print(&terminal, "SYN=", false);
			terminal_PrintULong(&terminal, gpsdo_state.count_calibrated, false);
			terminal_Print(&terminal, ", LST=", false);
			terminal_PrintULong(&terminal, gpsdo_state.count_loss_calibration, false);
			terminal_Print(&terminal, ", LOF=", false);
			terminal_PrintInt(&terminal, gpsdo_state.max_diff_below_nominal, false);
			terminal_Print(&terminal, ", HIF=", false);
			terminal_PrintInt(&terminal, gpsdo_state.max_diff_above_nominal, false);
			terminal_SendNL(&terminal, false);
			terminal_Print(&terminal, "  f=", false);
			terminal_PrintULong(&terminal, gpsdo_state.frequency, false);
			terminal_Print(&terminal, "Hz, error: ", false);
			if (gpsdo_state.error_frequency >= 0) terminal_Print(&terminal, " ", false);
			terminal_PrintInt(&terminal, gpsdo_state.error_frequency, false);
			terminal_Print(&terminal, "Hz, ", false);
			main_ShowErrorPpm();
			terminal_SendNL(&terminal, false);
			terminal_Print(&terminal, "  " , false);
		}
		main_ShowTemperature();
		terminal_Print(&terminal, ", ", false);
		if (gpsdo_state.disable_frequency_correction) terminal_Print(&terminal, "*", false);
		terminal_Print(&terminal, "DAC correction: ", false);
		terminal_PrintInt(&terminal, gpsdo_state.dac_value_diff_last, false);
		terminal_SendNL(&terminal, false);
	}
}

void main_SetFixPin(bool fixed)
{
	if (fixed) {
		setbit(CONFIG_FREQUENCY_FIXED_port.OUTSET, CONFIG_FREQUENCY_FIXED_pin);
	} else {
		setbit(CONFIG_FREQUENCY_FIXED_port.OUTCLR, CONFIG_FREQUENCY_FIXED_pin);		
	}
}

void main_SendGPSBuffer(void)
{
	bool send_status = true;
	while (send_status && !(cbuffer_IsEmpty(&buffer_gps))) {
		send_status = usart_SendChar(&usart_gps, cbuffer_ReadChar(&buffer_gps));
		if (send_status) cbuffer_DropChar(&buffer_gps);
	}
}

void main_SetDAC(uint16_t val)
{
	gpsdo_state.dac_value_diff_last = gpsdo_state.dac_value - DAC_VALUE_BASE;	
	if (gpsdo_state.disable_frequency_correction) {
		gpsdo_state.dac_value = DAC_VALUE_BASE;
	} else {
		val = (1 << DAC_FILTER_SCALE) * val;
		gpsdo_state.dac_value = (lpfilter_Filter(&filter_dac_value, val) >> DAC_FILTER_SCALE);
	}
	mcp47x6_SetOutputLevel(&dac, gpsdo_state.dac_value);
};

bool main_GetFrequencyDeviationArrayIndex(uint16_t temperature, uint16_t *idx)
{	
	temperature = (uint16_t)(temperature / 10);
	if ((temperature >= FREQUENCY_DEVIATION_TEMP_BASE_K10) && (temperature < FREQUENCY_DEVIATION_TEMP_BASE_K10 + FREQUENCY_DEVIATION_ARRAY_SIZE)) {
		*idx = (uint16_t)(temperature - FREQUENCY_DEVIATION_TEMP_BASE_K10);
		return true;
	}
	return false;
}

bool main_ReadEEPROMDeviation(uint16_t idx, uint16_t *min, uint16_t *max)
{
	uint16_t base;
	uint16_t dev;
	
	uint16_t v = eeprom_read_word(&frequency_deviation[idx]);
	if (v == FREQUENCY_DEVIATION_EMPTY_CELL) {
		*min = 0xFFFF;
		*max = 0x0000;
		return false;
	}
	base = v >> FREQUENCY_DEVIATION_MASK_LENGTH;
	dev = (v & FREQUENCY_DEVIATION_MASK);
	*min = base - dev;
	*max = base + dev;
	return true;
}

bool main_GetFrequencyDeviationBase(uint16_t idx, uint16_t *base)
{
	uint16_t v = eeprom_read_word(&frequency_deviation[idx]);
	if (v == FREQUENCY_DEVIATION_EMPTY_CELL) return false;
	*base = (v >> FREQUENCY_DEVIATION_MASK_LENGTH) << DAC_WORD_SHRINK;
	return true;
}

void main_UpdateEEPROMDeviation(uint16_t idx, uint16_t min, uint16_t max)
{
	uint32_t base;
	uint16_t dev;
	
	if (max < min) return;
	base = (max + min) >> 1;
	dev = (max - min) >> 1;
	if (dev > FREQUENCY_DEVIATION_MASK) dev = FREQUENCY_DEVIATION_MASK;
	uint16_t v = (uint16_t)(base << FREQUENCY_DEVIATION_MASK_LENGTH) | dev;
	if (v != FREQUENCY_DEVIATION_EMPTY_CELL) {
		uint16_t last = eeprom_read_word(&frequency_deviation[idx]);
		if (last != v) {
			eeprom_write_word(&frequency_deviation[idx], v); 
		}
	}
}

void main_CatchFrequencyDeviation(void)
{
	uint16_t idx;
	uint16_t min;
	uint16_t max;
	
	if (main_GetFrequencyDeviationArrayIndex(gpsdo_state.temperature_k100, &idx)) {
		main_ReadEEPROMDeviation(idx, &min, &max);	
		uint16_t val = gpsdo_state.dac_value >> DAC_WORD_SHRINK;
		if ((val < min) || (val > max)) {
			if (val < min) min = val;
			if (val > max) max = val;
			main_UpdateEEPROMDeviation(idx, min, max);
		}
	}
}

void main_ShowEEPROMDeviation(void)
{
	static uint16_t i = 0;
	
	if (i == 0) i = FREQUENCY_DEVIATION_ARRAY_SIZE;
	if (cbuffer_IsEmpty(&buffer_terminal)) {
		if (i) {
			uint16_t cell = FREQUENCY_DEVIATION_ARRAY_SIZE - i;
			uint16_t v = eeprom_read_word(&frequency_deviation[cell]);
			if (v != FREQUENCY_DEVIATION_EMPTY_CELL) {
				terminal_PrintInt(&terminal, cell, false);
				terminal_Print(&terminal, CSV_DELIMITER, false);
				terminal_PrintInt(&terminal, (int)(v >> FREQUENCY_DEVIATION_MASK_LENGTH), false);
				terminal_Print(&terminal, CSV_DELIMITER, false);
				terminal_PrintInt(&terminal, (int)(v & FREQUENCY_DEVIATION_MASK), false);
				terminal_SendNL(&terminal, false);
			}
			i--;
			if (i == 0) gpsdo_state.show_frequency_deviation = false;
		}
	}
}

void main_ClearEEPROMDeviation(bool keep_base)
{
	static uint16_t i = 0;
	
	if (i == 0) i = FREQUENCY_DEVIATION_ARRAY_SIZE;
	if (i) {
		i--;
		uint16_t v = FREQUENCY_DEVIATION_EMPTY_CELL;
		if (keep_base) {
			v = eeprom_read_word(&frequency_deviation[i]) & ~(FREQUENCY_DEVIATION_MASK);
		}
		eeprom_write_word(&frequency_deviation[i], v);
		if (i == 0) {
			gpsdo_state.clear_frequency_deviation_keep_base = true;
			gpsdo_state.clear_frequency_deviation = false;
			terminal_PrintNL(&terminal, "Done", false);
		}
	}
}

float main_ErrorFrequencyToPPM(int f)
{
	float result = f;
	if (f < 0) result = -f;
	result++;
	result /= (CONFIG_VCXO_NOMINAL_FREQUENCY / 1000000);
	return result;
}
