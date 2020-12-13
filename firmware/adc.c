/*
 * adc.c
 *
 * Created: 2019-01-12 20:46:53
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include "adc.h"
#include <avr/pgmspace.h>
#include <avr/eeprom.h>


uint8_t adc_input[ADC_MAX_INPUT_DEVICES];


uint8_t adc_ReadCalibrationByte(uint8_t index);
uint8_t current_mux_input = 0;


void adc_Setup(ADC_REFSEL_t reference)
{
	ADCA.REFCTRL = reference;
	ADCA.CALL = adc_ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
	ADCA.CALH = adc_ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
	ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc;
	ADCA.PRESCALER = ADC_PRESCALER_DIV256_gc;
	ADCA.CTRLA = ADC_ENABLE_bm;
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.CH0.INTCTRL = ADC_CH_INTLVL_LO_gc;
	for (uint8_t i = 0; i < ADC_MAX_INPUT_DEVICES; i++) adc_input[i] = ADC_CH_MUXPOS_PIN0_gc;
}

void adc_BindMuxInput(uint8_t device, ADC_CH_MUXPOS_t multiplexer_input)
{
	if (device > ADC_MAX_INPUT_DEVICES) return;
	adc_input[device - 1] = multiplexer_input;
}

void adc_SelectInput(uint8_t input)
{
	ADCA.CH0.MUXCTRL = input;
}

void adc_StartConversion(void)
{
	ADCA.CH0.CTRL |= ADC_CH_START_bm;
}

uint16_t adc_ReadInput(void) 
{
	uint16_t result = 0;
	result = (uint16_t)(ADCA.CH0.RES);
	return result;
}

uint8_t adc_CurrentInput(void) 
{
	if (current_mux_input >= sizeof(adc_input)) current_mux_input = 0;
	return adc_input[current_mux_input];
}

uint8_t adc_NextInput(void)
{
	current_mux_input++;
	if (current_mux_input >= sizeof(adc_input)) current_mux_input = 0;
	return adc_input[current_mux_input];
}

uint8_t adc_ReadCalibrationByte(uint8_t index) {
	uint8_t result = 0;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);
	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return result;
}
