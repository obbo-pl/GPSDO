/*
 * cbuffer.c
 *
 * Created: 2020-06-23 00:02:04
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 * AVR/GNU C Compiler : 4.8.1
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include "cbuffer.h"


uint8_t cbuffer_NextItem(CircularBuffer_t *cbuffer, uint8_t pointer);



void cbuffer_Init(CircularBuffer_t *cbuffer)
{
	cbuffer->start = 0;
	cbuffer->end = 0;
	for (uint8_t i = 0; i < CBUFFER_LENGTH; i++) cbuffer->data[i] = 0x00;
}

uint8_t cbuffer_NextItem(CircularBuffer_t *cbuffer, uint8_t item)
{
	item++;
	if (item >= sizeof(cbuffer->data)) item = 0;
	return item;
}

bool cbuffer_IsFull(CircularBuffer_t *cbuffer)
{
	if (cbuffer->start == cbuffer_NextItem(cbuffer, cbuffer->end)) return true;
	return false;
}

bool cbuffer_IsEmpty(CircularBuffer_t *cbuffer)
{
	if (cbuffer->start == cbuffer->end) return true;
	return false;
}

char cbuffer_ReadChar(CircularBuffer_t *cbuffer)
{
	return cbuffer->data[cbuffer->start];
}

void cbuffer_DropChar(CircularBuffer_t *cbuffer)
{
	if (cbuffer->start != cbuffer->end) cbuffer->start = cbuffer_NextItem(cbuffer, cbuffer->start);
}

void cbuffer_AppendChar(CircularBuffer_t *cbuffer, const char src)
{
	cbuffer->data[cbuffer->end] = src;
	cbuffer->end = cbuffer_NextItem(cbuffer, cbuffer->end);
	if (cbuffer->end == cbuffer->start) cbuffer->start = cbuffer_NextItem(cbuffer, cbuffer->end);
}

void cbuffer_AppendString(CircularBuffer_t *cbuffer, const char *src)
{
	while (*src != 0x00) cbuffer_AppendChar(cbuffer, *src++);
}

void cbuffer_AppendBuffer(CircularBuffer_t *cbuffer, const char *src, uint8_t length)
{
	while (length--) cbuffer_AppendChar(cbuffer, *src++);
}

void cbuffer_AppendEEString(CircularBuffer_t *cbuffer, const char *src)
{
	while (1) {
		char item = (char)eeprom_read_byte((const uint8_t *)(src++));
		if (!item) break;
		cbuffer_AppendChar(cbuffer, item);
	} 
}

void cbuffer_AppendEEBuffer(CircularBuffer_t *cbuffer, const char *src, uint8_t length)
{
	while (length--) cbuffer_AppendChar(cbuffer, (char)eeprom_read_byte((const uint8_t *)(src++)));
}

void cbuffer_AppendPMString(CircularBuffer_t *cbuffer, const char *src)
{
	while (1) {
		char item = (char)pgm_read_byte((const uint8_t *)(src++));
		if (!item) break;
		cbuffer_AppendChar(cbuffer, item);
	}
}
