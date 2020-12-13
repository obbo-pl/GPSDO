/*
 * adc.h
 *
 * Created: 2019-01-12 20:46:39
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>


#define ADC_MAX_INPUT_DEVICES		1


void adc_Setup(ADC_REFSEL_t reference);
void adc_BindMuxInput(uint8_t device, ADC_CH_MUXPOS_t multiplexer_input);
void adc_SelectInput(uint8_t input);
void adc_StartConversion(void);
uint16_t adc_ReadInput(void);
uint8_t adc_CurrentInput(void);
uint8_t adc_NextInput(void);


#endif /* ADC_H_ */