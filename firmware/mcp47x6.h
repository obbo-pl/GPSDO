/*
 * mcp47x6.h
 *
 * Created: 2020-11-24 22:42:20
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#ifndef MCP47X6_H_
#define MCP47X6_H_


// Atmel XMEGA TWI driver AVR1308.zip
#include "twi_master_driver.h"
#include <stdbool.h>


#define MCP47X6_BASE_ADDRESS		(0x60)
#define MCP47X6_ADDRESS_MASK		(0x07)

// Programmable Gain
#define MCP47X6_GAIN_MASK		(0x01)
#define MCP47X6_GAIN_1X			(0x00)
#define MCP47X6_GAIN_2X			(0x01)

// Power Down Mode
#define MCP47X6_PWRDN_MASK		(0x06)
#define MCP47X6_PWRDN_UP		(0x00)
#define MCP47X6_PWRDN_1K		(0x02)
#define MCP47X6_PWRDN_100K		(0x04)
#define MCP47X6_PWRDN_500K		(0x06)

// Reference Voltage
#define MCP47X6_VREF_MASK		(0x18)
#define MCP47X6_VREF_VDD		(0x00)
#define MCP47X6_VREF_VREFPIN_UNBUFFERED	(0x10)
#define MCP47X6_VREF_VREFPIN_BUFFERED	(0x18)

// Command
#define MCP47X6_CMD_MASK		(0xE0)
#define MCP47X6_CMD_VOLDAC		(0x00)
#define MCP47X6_CMD_VOLALL		(0x40)
#define MCP47X6_CMD_ALLMEM		(0x60)
#define MCP47X6_CMD_VOLCONFIG		(0x80)


#define MCP47X6_TWI_TIMEOUT_MS		(uint8_t)(50)



typedef struct {
	TWI_Master_t *twi;
	uint8_t address;
        uint8_t config;
} MCP47X6_t;


void mcp47x6_Init(MCP47X6_t *chip, TWI_Master_t *twi, uint8_t address);
bool mcp47x6_SetConfiguration(MCP47X6_t *chip, uint8_t gain, uint8_t vref);
bool mcp47x6_SetOutputLevel(MCP47X6_t *chip, uint16_t level);
bool mcp47x6_SaveToEEPROM(MCP47X6_t *chip, uint16_t level);
bool mcp47x6_ReadSavedLevel(MCP47X6_t *chip, uint16_t *level);


#endif /* MCP47X6_H_ */
