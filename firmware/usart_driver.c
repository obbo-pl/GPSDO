/*
 * usart_driver.c
 *
 * Created: 2020-06-22 19:47:41
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 * AVR/GNU C Compiler : 4.8.1
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include "usart_driver.h"
#include <avr/pgmspace.h> 

#if (F_CPU == 32000000UL)
#define BSCALE_RATE_4800	5
#define BSCALE_RATE_9600	4
#define BSCALE_RATE_14400	0
#define BSCALE_RATE_19200	3
#define BSCALE_RATE_28800	-1
#define BSCALE_RATE_38400	2
#define BSCALE_RATE_57600	-2

#define BSEL_RATE_4800		12
#define BSEL_RATE_9600		12
#define BSEL_RATE_14400		138
#define BSEL_RATE_19200		12
#define BSEL_RATE_28800		137
#define BSEL_RATE_38400		12
#define BSEL_RATE_57600		135
#endif

const int bscale_rate_factor[7] PROGMEM = {
	BSCALE_RATE_4800, 
	BSCALE_RATE_9600, 
	BSCALE_RATE_14400, 
	BSCALE_RATE_19200, 
	BSCALE_RATE_28800, 
	BSCALE_RATE_38400,
	BSCALE_RATE_57600
};

const uint16_t bsel_rate_period[7] PROGMEM = {
	BSEL_RATE_4800,
	BSEL_RATE_9600,
	BSEL_RATE_14400,
	BSEL_RATE_19200,
	BSEL_RATE_28800,
	BSEL_RATE_38400,
	BSEL_RATE_57600
};

#define USART_PRESCALE			(((F_CPU / (16UL * USART_BAUDRATE))) - 1)


#define USART_SERIAL_PORT		D
#define USART_SERIAL_Tx_PIN		1
#define USART_SERIAL_Rx_PIN		0



#if (defined(__AVR_XMEGA__))
void usart_Init(USART_DRIVER_t *usart, USART_t *module, PORT_t *port, uint8_t tx_pin_bm, uint8_t rx_pin_bm)
{
	usart->module = module;
	port->DIRSET = tx_pin_bm;
	port->DIRCLR = rx_pin_bm;
	usart_SetDataFormat(usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	usart->module->CTRLA |= USART_RXCINTLVL_LO_gc;
}

bool usart_SendChar(USART_DRIVER_t *usart, char data)
{
	if (usart->module->STATUS & USART_DREIF_bm) {
		usart->module->DATA = data;
		return true;
	}
	return false;
}

char usart_ReadChar(USART_DRIVER_t *usart)
{
	return usart->module->DATA;
}

void usart_SetBaudrate(USART_DRIVER_t *usart, BAUD_RATE_t baud_rate)
{
	int bscale = pgm_read_byte(&bscale_rate_factor[baud_rate]);
	uint16_t bsel = pgm_read_byte(&bsel_rate_period[baud_rate]);
	usart->module->BAUDCTRLA = (uint8_t)bsel;
	usart->module->BAUDCTRLB = (bscale << 4) | (bsel >> 8);

}

void usart_RxEnable(USART_DRIVER_t *usart, bool enable)
{
	if (enable) {
		usart->module->CTRLB |= USART_RXEN_bm;
	} else {
		usart->module->CTRLB &= ~USART_RXEN_bm;
	}
}

void usart_TxEnable(USART_DRIVER_t *usart, bool enable)
{
	if (enable) {
		usart->module->CTRLB |= USART_TXEN_bm;
	} else {
		usart->module->CTRLB &= ~USART_TXEN_bm;
	}
}

void usart_SetDataFormat(USART_DRIVER_t *usart, USART_CHSIZE_t char_size, USART_PMODE_t parity_mode, bool two_stop_bits)
{
	usart->module->CTRLC = char_size | parity_mode | (two_stop_bits ? USART_SBMODE_bm : 0);
}

#elif (defined(__AVR_MEGA__))
void usart_Init(USART_DRIVER_t *usart)
{
	SET_PIN_AS_OUT_(USART_SERIAL_PORT,USART_SERIAL_Tx_PIN);
	SET_PIN_AS_IN_(USART_SERIAL_PORT,USART_SERIAL_Rx_PIN);
	SET_PIN_PULLUP_(USART_SERIAL_PORT,USART_SERIAL_Rx_PIN);

	#if (defined(__AVR_ATmega8__))
	UBRRH = (uint8_t)(USART_PRESCALE >> 8);
	UBRRL = (uint8_t)(USART_PRESCALE);
	UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);		// RX Complete Interrupt Enable, Receiver Enable, Transmitter Enable
	UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);		// Asynchronous Operation, Parity Mode Disabled, Stop Bit: 1-bit, Character Size: 8-bit
	#elif (defined(__AVR_ATmega88A__) || defined(__AVR_ATmega88PA__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__))
	UBRR0H = (uint8_t)(USART_PRESCALE >> 8);
	UBRR0L = (uint8_t)(USART_PRESCALE);
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);		// RX Complete Interrupt Enable, Receiver Enable, Transmitter Enable
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);				// Asynchronous Operation, Parity Mode Disabled, Stop Bit: 1-bit, Character Size: 8-bit
	#endif
}

bool usart_SendChar(USART_DRIVER_t *usart, char data)
{
	#if (defined(__AVR_ATmega8__))
	if (UCSRA & (1 << UDRE)) {
		UDR = data;
		return true;
	}
	#elif (defined(__AVR_ATmega88A__) || defined(__AVR_ATmega88PA__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__))
	if (UCSR0A & (1 << UDRE0)) {
		UDR0 = data;
		return true;
	}
	#endif
	return false;
}

char usart_ReadChar(USART_DRIVER_t *usart)
{
	#if (defined(__AVR_ATmega8__))
	return UDR;
	#elif (defined(__AVR_ATmega88A__) || defined(__AVR_ATmega88PA__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__))
	return UDR0;
	#endif
}
#endif

