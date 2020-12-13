/*
 * macro.h
 *
 * Created: 2020-05-10 23:51:39
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 * AVR/GNU C Compiler : 4.8.1
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef MACRO_H_
#define MACRO_H_


#define	hibyte(x) (uint8_t)(x >> 8)
#define	lobyte(x) (uint8_t)(x & 0x00FF)
#define setbit(value, bit) ((value) |= (uint8_t)(1 << bit))
#define clrbit(value, bit) ((value) &= (uint8_t)~(1 << bit))
#define testbit(value, bit) (uint8_t)(((uint8_t)value >> bit) & 0x01)
#define togglebit(value, bit) ((value) ^= (uint8_t)(1 << bit))
#define bitmask(bit) ((uint8_t)(1 << bit))

//#define nop()					asm volatile ("nop");

#define glue2_(x, y)				x ## y
#define glue2(x, y)				glue2_(x, y)
#define glue3_(x, y, z)				x ## y ## z
#define glue3(x, y, z)				glue3_(x, y, z)


#define SET_PIN_AS_OUT_(port,pin)		(glue2(DDR, port) |= (1 << glue3(DD, port, pin)))
#define SET_PIN_AS_IN_(port,pin)		(glue2(DDR, port) &= ~(1 << glue3(DD, port, pin)))
#define SET_PIN_PULLUP_(port,pin)		(glue2(PORT, port) |= (1 << glue3(P, port, pin)))
#define SET_PIN_LEVEL_HIGH_(port,pin)		(glue2(PORT, port) |= (1 << glue3(P, port, pin)))
#define SET_PIN_LEVEL_LOW_(port,pin)		(glue2(PORT, port) &= ~(1 << glue3(P, port, pin)))
#define TOGGLE_PIN_LEVEL_(port,pin)		(glue2(PORT, port) ^= (1 << glue3(P, port, pin)))
#define READ_PIN_(port,pin)			((glue2(PIN, port) >> glue3(P, port, pin)) & 0x01)

#define SET_PIN_AS_OUT(out)			SET_PIN_AS_OUT_(out)
#define SET_PIN_AS_IN(out)			SET_PIN_AS_IN_(out)
#define SET_PIN_PULLUP(out)			SET_PIN_PULLUP_(out)
#define SET_PIN_LEVEL_HIGH(out)			SET_PIN_LEVEL_HIGH_(out)
#define SET_PIN_LEVEL_LOW(out)			SET_PIN_LEVEL_LOW_(out)
#define TOGGLE_PIN_LEVEL(out)			TOGGLE_PIN_LEVEL_(out)
#define READ_PIN(out)				READ_PIN_(out)


#define PORT_REG_IO_(port,pin)			glue2(&PIN, port), glue2(&DDR, port), glue2(&PORT, port), pin
#define PORT_REG_I_(port,pin)			glue2(&PIN, port), glue2(&DDR, port), pin
#define PORT_REG_O_(port,pin)			glue2(&DDR, port), glue2(&PORT, port), pin
#define PORT_PIN_(port,pin)			pin
#define PORT_REG_PIN_(port,pin)			glue2(PIN, port)
#define PORT_REG_DDR_(port,pin)			glue2(DDR, port)
#define PORT_REG_PORT_(port,pin)		glue2(PORT, port)

#define PORT_REG_IO(out)			PORT_REG_IO_(out)
#define PORT_REG_I(out)				PORT_REG_I_(out)
#define PORT_REG_O(out)				PORT_REG_O_(out)
#define PORT_PIN(out)				PORT_PIN_(out)
#define PORT_REG_PIN(out)			PORT_REG_PIN_(out)
#define PORT_REG_DDR(out)			PORT_REG_DDR_(out)
#define PORT_REG_PORT(out)			PORT_REG_PORT_(out)




#endif /* MACRO_H_ */