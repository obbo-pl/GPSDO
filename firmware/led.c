/*
 * led.c
 *
 * Created: 2018-12-25 22:39:03
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include "led.h"
#include "macro.h"
#include <avr/interrupt.h>


enum led_setup {
	LED_STATE_On,
	LED_STATE_Blink,
	LED_STATE_Invert
};


#if (defined(__AVR_XMEGA__))
#define LED_LOW			{char cSREG = SREG; cli(); clrbit(led->port.port->OUT, led->port.pin); SREG = cSREG;}
#define LED_HIGH		{char cSREG = SREG; cli(); setbit(led->port.port->OUT, led->port.pin); SREG = cSREG;}
#elif (defined(__AVR_MEGA__))
#define LED_LOW			{char cSREG = SREG; cli(); clrbit(*(led->port.regport), led->port.pin); SREG = cSREG;}
#define LED_HIGH		{char cSREG = SREG; cli(); setbit(*(led->port.regport), led->port.pin); SREG = cSREG;}
#endif



#if (defined(__AVR_XMEGA__))
void led_Init(LED_t *led, PORT_t volatile *port, uint8_t pin)
{
	led->port.port = port;
	led->port.pin = pin;
	setbit(led->port.port->DIR, led->port.pin);
	led->counter = 0;
	led->period = LED_DEFAULT_PERIOD_tic;
	led->duty = LED_DEFAULT_DUTY_tic;
	led->setup = 0x00;
	led->configured = true;
}
#elif (defined(__AVR_MEGA__))
void led_Init(LED_t *led, uint8_t volatile *led_regddr, uint8_t volatile *led_regport, uint8_t pin)
{
	led->port.regddr = led_regddr;
	led->port.regport = led_regport;
	led->port.pin = pin;
	setbit(*led->port.regddr, led->port.pin);
	led->counter = 0;
	led->period = LED_DEFAULT_PERIOD_tic;
	led->duty = LED_DEFAULT_DUTY_tic;
	led->setup = 0x00;
	led->configured = true;
}
#endif


void led_Update(LED_t *led)
{
	if (led->configured) {
		if (testbit(led->setup, LED_STATE_On)) {
			if (testbit(led->setup, LED_STATE_Blink)) {
				led->counter += 1;
				if (led->counter >= led->period) led->counter = 0;
				if (testbit(led->setup, LED_STATE_Invert)) {
					if (led->duty > led->counter) {
						LED_HIGH;
					} else {
						LED_LOW;
					}
				} else {
					if (led->duty < led->counter) {
						LED_HIGH;
					} else {
						LED_LOW;
					}
				}
			} else {
				LED_HIGH;
			}
		} else {
			LED_LOW;
		}
	}
}

void led_Clear(LED_t *led)
{
	led->counter = 0;
}

void led_SetEnabled(LED_t *led, bool enabled)
{
	if (enabled) {
		setbit(led->setup, LED_STATE_On);
	} else {
		clrbit(led->setup, LED_STATE_On);
	}
}

// Sets LED light blinking or solid
void led_SetBlink(LED_t *led, bool blink)
{
	if (blink) {
		setbit(led->setup, LED_STATE_Blink);
	} else {
		clrbit(led->setup, LED_STATE_Blink);
	}
}

void led_SetLed(LED_t *led, bool enabled, bool blink)
{
	led_SetEnabled(led, enabled);
	led_SetBlink(led, blink);
}

void led_SetTimers(LED_t *led, uint8_t period, uint8_t duty)
{
	led->period = period;
	led->duty = duty;
}

// Sets LED blinking behavior
void led_SetBlinkInverted(LED_t *led, bool state)
{
	if (state) {
		setbit(led->setup, LED_STATE_Invert);
	} else {
		clrbit(led->setup, LED_STATE_Invert);
	}
}
