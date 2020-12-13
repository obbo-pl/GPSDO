/*
 * mcp47x6.c
 *
 * Created: 2020-11-24 22:42:20
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include "mcp47x6.h"
#include <util/delay.h>
#include "macro.h"


uint8_t buffer[3];


void mcp47x6_Init(MCP47X6_t *chip, TWI_Master_t *twi, uint8_t address)
{
	chip->twi = twi;
	chip->address = (MCP47X6_BASE_ADDRESS) | ( address & MCP47X6_ADDRESS_MASK);
	chip->config = 0x00;
}

bool mcp47x6_SetConfiguration(MCP47X6_t *chip, uint8_t gain, uint8_t vref) 
{
	uint16_t timeout = 10 * MCP47X6_TWI_TIMEOUT_MS;

	chip->config &= ~MCP47X6_GAIN_MASK;
	chip->config |= gain & MCP47X6_GAIN_MASK;
	chip->config &= ~MCP47X6_VREF_MASK;
	chip->config |= vref & MCP47X6_VREF_MASK;
	buffer[0] = MCP47X6_CMD_VOLCONFIG | chip->config;
	bool result = TWI_MasterWriteRead(chip->twi, chip->address, buffer, 1, 0);
	while ((chip->twi->status != TWIM_STATUS_READY) && (timeout > 0)) {
		if (timeout > 0) timeout--;
		_delay_us(100);
	}
	if (!timeout) result &= false;
	return result;
}

bool mcp47x6_SetOutputLevel(MCP47X6_t *chip, uint16_t level)
{
	uint16_t timeout = 10 * MCP47X6_TWI_TIMEOUT_MS;

	uint8_t config = (chip->config & MCP47X6_PWRDN_MASK) << 2;
	buffer[0] = MCP47X6_CMD_VOLDAC | config | (uint8_t)(level >> 8);
	buffer[1] = (uint8_t)(level & 0xff);
	bool result = TWI_MasterWriteRead(chip->twi, chip->address, buffer, 2, 0);
	while ((chip->twi->status != TWIM_STATUS_READY) && (timeout > 0)) {
		if (timeout > 0) timeout--;
		_delay_us(100);
	}
	if (!timeout) result &= false;
	return result;
}

bool mcp47x6_SaveToEEPROM(MCP47X6_t *chip, uint16_t level)
{
	uint16_t timeout = 10 * MCP47X6_TWI_TIMEOUT_MS;

	buffer[0] = MCP47X6_CMD_ALLMEM | chip->config;
	buffer[1] = (uint8_t)(level >> 4);
	buffer[2] = (uint8_t)(level << 4);
	bool result = TWI_MasterWriteRead(chip->twi, chip->address, buffer, 3, 0);
	while ((chip->twi->status != TWIM_STATUS_READY) && (timeout > 0)) {
		if (timeout > 0) timeout--;
		_delay_us(100);
	}
	if (!timeout) result &= false;
	return result;
}

bool mcp47x6_ReadSavedLevel(MCP47X6_t *chip, uint16_t *level)
{
	uint16_t saved_level = 0;
	uint16_t timeout = 10 * MCP47X6_TWI_TIMEOUT_MS;

	bool result = TWI_MasterWriteRead(chip->twi, chip->address, buffer, 0, 6);
	while ((chip->twi->status != TWIM_STATUS_READY) && (timeout > 0)) {
		if (timeout > 0) timeout--;
		_delay_us(100);
	}
	if (!timeout) result &= false;
	if (result)  {
		saved_level = chip->twi->readData[4];
		saved_level = (saved_level << 8) | chip->twi->readData[5];
		saved_level = saved_level >> 4;
	}
	*level = saved_level;
	return result;
}

