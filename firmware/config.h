/*
 * config.h
 *
 * Created: 2020-11-22 00:40:06
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 * AVR/GNU C Compiler : 4.8.1
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_


// LED
#define CONFIG_LED_D1_pin			6
#define CONFIG_LED_D1_port			PORTC
#define CONFIG_LED_D2_pin			7
#define CONFIG_LED_D2_port			PORTC

// Temperature
#define CONFIG_ADC_REFERENCE			AREFA
#define CONFIG_ADC_TEMPERATURE_VCXO_mux		1

// GPS 
#define CONFIG_GPS_1PPS_GATE_pin		5
#define CONFIG_GPS_1PPS_GATE_port		PORTC
#define CONFIG_GPS_RESET_pin			6
#define CONFIG_GPS_RESET_port			PORTA

// Frequency counter
#define CONFIG_FREQUENCY_MR_pin			7
#define CONFIG_FREQUENCY_MR_port		PORTA
#define CONFIG_FREQUENCY_Q0_pin			0
#define CONFIG_FREQUENCY_Q0_port		PORTB
#define CONFIG_FREQUENCY_Q1_pin			1
#define CONFIG_FREQUENCY_Q1_port		PORTB
#define CONFIG_FREQUENCY_Q2_pin			2
#define CONFIG_FREQUENCY_Q2_port		PORTB
#define CONFIG_FREQUENCY_Q3_pin			3
#define CONFIG_FREQUENCY_Q3_port		PORTB

// TCVCXO
#define CONFIG_VCXO_NOMINAL_FREQUENCY		40000000UL
#define CONFIG_VCXO_VOLTAGE_CONTROL_LOW_mv	500
#define CONFIG_VCXO_VOLTAGE_CONTROL_HIGH_mv	2500
#define CONFIG_VCXO_ADJUST_RANGE_ppm		5
#define CONFIG_VCXO_FREQUENCY_STABILITY_ppm	2			
#define CONFIG_VCXO_FREQUENCY_AGING_ppm		1

// Fixed Frequency
#define CONFIG_FREQUENCY_FIXED_pin		0
#define CONFIG_FREQUENCY_FIXED_port		PORTD


#endif /* CONFIG_H_ */

