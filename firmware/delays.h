/*
 * delays.h
 *
 * Created: 2019-01-13 18:48:50
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef DELAYS_H_
#define DELAYS_H_

#include <avr/io.h>
#include <stdbool.h>



typedef struct delay {
	uint16_t length;
	volatile uint16_t counter;
	bool pause;
} DELAY_t;


void delays_Update(DELAY_t *delay, uint16_t period);
void delays_Init(DELAY_t *delay, uint16_t val);
void delays_Reset(DELAY_t *delay);
bool delays_Check(DELAY_t *delay);
void delays_Pause(DELAY_t *delay);

#endif /* DELAYS_H_ */