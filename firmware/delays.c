/*
 * delays.c
 *
 * Created: 2019-01-13 18:49:02
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include "delays.h"


void delays_Update(DELAY_t *delay, uint16_t period)
{
	if (!delay->pause) {
		if (delay->counter > period) delay->counter -= period;
		else delay->counter = 0;
	}
}

void delays_Init(DELAY_t *delay, uint16_t val)
{
	delay->length = val;
	delay->counter = val;
	delay->pause = false;
}

void delays_Reset(DELAY_t *delay)
{
	delay->counter = delay->length;
	delay->pause = false;
}

bool delays_Check(DELAY_t *delay)
{
	if (delay->counter == 0) return true;
	else return false;
}

void delays_Pause(DELAY_t *delay)
{
	delay->pause = true;
}
