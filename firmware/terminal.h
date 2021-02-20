/*
 * terminal.h
 *
 * Created: 27.08.2019 13:53:31
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 * AVR/GNU C Compiler : 4.8.1
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef TERMINAL_H_
#define TERMINAL_H_

#include <stdbool.h>
#include <avr/io.h>
#include "cbuffer.h"
#include "usart_driver.h"


#define TERMINAL_NEW_LINE		(0x0D)
#define TERMINAL_NEW_COMMAND		(0x40)
#define TERMINAL_HELP			(0x3F)
#define TERMINAL_SPACE			(0x20)

#define TERMINAL_INPUT_BUFFER_LENGTH	(24)
#define TERMINAL_OPTION_LENGTH		(16)

#define COMMAND_LENGTH			(4)

//#define TERMINAL_MESSAGE_IN_EEPROM

typedef struct TERMINAL {
	volatile char input_buffer[TERMINAL_INPUT_BUFFER_LENGTH];
	char command_option[TERMINAL_OPTION_LENGTH];
	uint8_t input_buffer_position;
	bool change_to_write;
	USART_DRIVER_t *usart;
	CircularBuffer_t *output_buffer;
} TERMINAL_t;


typedef struct terminal_command {
	const char *pattern;
	void (*callback)(TERMINAL_t *);
} TERMINAL_COMMAND_t;



void terminal_Init(TERMINAL_t *terminal, USART_DRIVER_t *usart, CircularBuffer_t *cbuffer);
void terminal_BindCommands(TERMINAL_COMMAND_t *commands, uint8_t length);

bool terminal_FindNewLine(TERMINAL_t *terminal, uint8_t *len);
void terminal_ParseCommand(TERMINAL_t *terminal, uint8_t len);
void terminal_InterruptHandler(TERMINAL_t *terminal);
void terminal_ParseChar(TERMINAL_t *terminal, char c);
bool terminal_IsBufferEmpty(TERMINAL_t *terminal);
bool terminal_IsBufferFull(TERMINAL_t *terminal);
void terminal_DropInputBuffer(TERMINAL_t *terminal);
void terminal_FlushOutBuffer(TERMINAL_t *terminal);
void terminal_SendOutBuffer(TERMINAL_t *terminal);
void terminal_SendWelcome(TERMINAL_t *terminal);
void terminal_SendBadArgument(TERMINAL_t *terminal);
void terminal_SendOK(TERMINAL_t *terminal);
void terminal_Print(TERMINAL_t *terminal, const char *src, bool flush);
void terminal_PrintEE(TERMINAL_t *terminal, const char *src, bool flush);
void terminal_PrintPM(TERMINAL_t *terminal, const char *src, bool flush);
void terminal_PrintNL(TERMINAL_t *terminal, const char *src, bool flush);
void terminal_PrintInt(TERMINAL_t *terminal, int val, bool flush);
void terminal_PrintULong(TERMINAL_t *terminal, uint32_t val, bool flush);
void terminal_PrintHex(TERMINAL_t *terminal, uint8_t val, bool flush);
void terminal_PrintBin(TERMINAL_t *terminal, uint8_t val, bool flush);
void terminal_PrintHexAndBin(TERMINAL_t *terminal, uint8_t val, bool flush);
void terminal_PrintBuf(TERMINAL_t *terminal, uint8_t *buffer, uint8_t len, bool flush);
void terminal_PrintEEBufDump(TERMINAL_t *terminal, uint8_t *buffer, uint8_t len, bool flush);
void terminal_SendNL(TERMINAL_t *terminal, bool flush);




#endif /* TERMINAL_H_ */