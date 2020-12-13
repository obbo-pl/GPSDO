/*
 * lowpass_filter.c
 *
 * Created: 2019-01-13 23:33:01
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 * AVR/GNU C Compiler : 4.8.1
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include "lowpass_filter.h"



void lpfilter_Set(LPFu16_t *filter, uint8_t length)
{
	filter->length = length;
	if (filter->length > 16) filter->length = 16;
}

void lpfilter_Fill(LPFu16_t *filter, uint16_t val)
{
	filter->sum = (uint32_t)val << filter->length;
	filter->average = val;
}

uint16_t lpfilter_Filter(LPFu16_t *filter, uint16_t new_value)
{
	filter->sum = filter->sum - filter->average + new_value;
	filter->average = filter->sum >> filter->length;
	return filter->average;
}

