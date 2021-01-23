/*
 * terminal_commands.h
 *
 * Created: 2020-06-15 19:32:51
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 * AVR/GNU C Compiler : 4.8.1
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef TERMINAL_COMMANDS_H_
#define TERMINAL_COMMANDS_H_

#include <avr/io.h>
#include "terminal.h"
#include "types.h"


void ftoa(float f, char *buffer, uint8_t length);

void terminal_commands_Init(GPSDO_State_t *state);
void command_SystemVersion(TERMINAL_t *terminal);




#endif /* TERMINAL_COMMANDS_H_ */