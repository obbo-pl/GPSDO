/*
 * usart_driver.h
 *
 * Created: 2020-06-22 19:47:41
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 * AVR/GNU C Compiler : 4.8.1
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef USART_DRIVER_H_
#define USART_DRIVER_H_


#include <stdbool.h>
#include <avr/io.h>

typedef enum {
	BAUD_RATE_4800,
	BAUD_RATE_9600,
	BAUD_RATE_14400,
	BAUD_RATE_19200,
	BAUD_RATE_28800,
	BAUD_RATE_38400,
	BAUD_RATE_57600
} BAUD_RATE_t;


#define USART_BAUDRATE					(9600)



#if (defined(__AVR_XMEGA__))
typedef struct {
	USART_t *module;
} USART_DRIVER_t;
#elif (defined(__AVR_MEGA__))
typedef struct {
} USART_DRIVER_t;
#endif

#if (defined(__AVR_XMEGA__))
void usart_Init(USART_DRIVER_t *usart, USART_t *module, PORT_t *port, uint8_t tx_pin_bm, uint8_t rx_pin_bm);
void usart_RxEnable(USART_DRIVER_t *usart, bool enable);
void usart_TxEnable(USART_DRIVER_t *usart, bool enable);
void usart_SetDataFormat(USART_DRIVER_t *usart, USART_CHSIZE_t char_size, USART_PMODE_t parity_mode, bool two_stop_bits);
#elif (defined(__AVR_MEGA__))
void usart_Init(USART_DRIVER_t *usart);
#endif
void usart_SetBaudrate(USART_DRIVER_t *usart, BAUD_RATE_t baud_rate);
bool usart_SendChar(USART_DRIVER_t *usart, char data);
char usart_ReadChar(USART_DRIVER_t *usart);




#endif /* USART_DRIVER_H_ */