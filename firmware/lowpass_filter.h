/*
 * lowpass_filter.h
 *
 * Created: 2019-01-13 23:32:51
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 * AVR/GNU C Compiler : 4.8.1
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef LOWPASS_FILTER_H_
#define LOWPASS_FILTER_H_

#include <avr/io.h>


typedef struct {
	uint8_t length;
	uint16_t average;
	uint32_t sum;
} LPFu16_t;


void lpfilter_Set(LPFu16_t *filter, uint8_t length);
void lpfilter_Fill(LPFu16_t *filter, uint16_t val);
uint16_t lpfilter_Filter(LPFu16_t *filter, uint16_t new_value);


#endif /* LOWPASS_FILTER_H_ */