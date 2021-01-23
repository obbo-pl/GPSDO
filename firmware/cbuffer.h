/*
 * cbuffer.h
 *
 * Created: 2020-06-23 00:01:47
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 * AVR/GNU C Compiler : 4.8.1
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef CBUFFER_H_
#define CBUFFER_H_

#include <avr/io.h>

//#define CBUFFER_INTERNAL_DATA_STORE

#ifdef CBUFFER_INTERNAL_DATA_STORE
#define CBUFFER_LENGTH			(150)
typedef struct CircularBuffer {
	volatile char data[CBUFFER_LENGTH];
	uint8_t length;
	volatile uint8_t start;
	volatile uint8_t end;
} CircularBuffer_t;
#else
typedef struct CircularBuffer {
	volatile char *data;
	uint8_t length;
	volatile uint8_t start;
	volatile uint8_t end;
} CircularBuffer_t;
#endif //CBUFFER_INTERNAL_DATA_STORE

#ifdef CBUFFER_INTERNAL_DATA_STORE
void cbuffer_Init(CircularBuffer_t *cbuffer);
#else
void cbuffer_Init(CircularBuffer_t *cbuffer, volatile char *data, uint8_t length);
#endif //CBUFFER_INTERNAL_DATA_STORE
bool cbuffer_IsFull(CircularBuffer_t *cbuffer);
bool cbuffer_IsEmpty(CircularBuffer_t *cbuffer);
char cbuffer_ReadChar(CircularBuffer_t *cbuffer);
void cbuffer_DropChar(CircularBuffer_t *cbuffer);
void cbuffer_AppendChar(CircularBuffer_t *cbuffer, const char src);
void cbuffer_AppendString(CircularBuffer_t *cbuffer, const char *src);
void cbuffer_AppendBuffer(CircularBuffer_t *cbuffer, const char *src, uint8_t length);
void cbuffer_AppendEEString(CircularBuffer_t *cbuffer, const char *src);
void cbuffer_AppendEEBuffer(CircularBuffer_t *cbuffer, const char *src, uint8_t length);
void cbuffer_AppendPMString(CircularBuffer_t *cbuffer, const char *src);

#endif /* CBUFFER_H_ */