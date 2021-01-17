/*
 * terminal_commands.cpp
 *
 * Created: 2020-06-15 19:33:07
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 * AVR/GNU C Compiler : 4.8.1
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include <stdlib.h>
#include <avr/eeprom.h>
#include <ctype.h>
#include "terminal_commands.h"
#include "macro.h"



uint8_t hex2int(const char hex);

void command_SystemVersion(TERMINAL_t *terminal);
void command_EndTerminalSession(TERMINAL_t *terminal);
void command_ShowGPSDOStatus(TERMINAL_t *terminal);
void command_ShowStatusCSVFormat(TERMINAL_t *terminal);
void command_DisableFrequencyCorrection(TERMINAL_t *terminal);


#define TERMINAL_BASE_COMMANDS_COUNT		5
#define TERMINAL_COMMANDS_COUNT			(TERMINAL_BASE_COMMANDS_COUNT)

TERMINAL_COMMAND_t terminal_commands[TERMINAL_COMMANDS_COUNT] = {
	{ .pattern = "@VER", .callback = command_SystemVersion,},
	{ .pattern = "@ETS", .callback = command_EndTerminalSession,},
	{ .pattern = "@SGS", .callback = command_ShowGPSDOStatus,},
	{ .pattern = "@SSC", .callback = command_ShowStatusCSVFormat,},
	{ .pattern = "@DFC", .callback = command_DisableFrequencyCorrection,},		
};

GPSDO_State_t *state_gpsdo;



void terminal_commands_Init(GPSDO_State_t *state)
{
	terminal_BindCommands(terminal_commands, TERMINAL_COMMANDS_COUNT);
	state_gpsdo = state;
}

// in special cases an alternative to sprintf
void ftoa(float f, char *buffer, uint8_t length)
{
	char *start = buffer;
	f += 0.05;
	int major = (int)f;
	itoa(major, start, 10);
	for (uint8_t i = 0; i < length; i++) {
		start++;
		if (*start == 0x00) break;
	}
	*(start++) = '.';
	int minor = (int)((f - major) * 10);
	itoa(minor, start, 10);
}

uint8_t hex2int(const char hex) {
	uint8_t result = 0;
	if (hex >= '0' && hex <= '9') result = hex - '0';
	else if (hex >= 'a' && hex <='f') result = hex - 'a' + 10;
	else if (hex >= 'A' && hex <='F') result = hex - 'A' + 10;
	return result;
}

void command_SystemVersion(TERMINAL_t *terminal)
{
	cbuffer_AppendPMString(terminal->output_buffer, state_gpsdo->device_info);
}

void command_EndTerminalSession(TERMINAL_t *terminal)
{
	cbuffer_AppendString(terminal->output_buffer, "Bye!\n\r");
	state_gpsdo->forward_gps_message = true;
}

void command_ShowGPSDOStatus(TERMINAL_t *terminal)
{
	state_gpsdo->show_gpsdo_status = true;
}

void command_ShowStatusCSVFormat(TERMINAL_t *terminal)
{
	if (terminal->command_option[0] == TERMINAL_SPACE) {
		int temp = atoi(terminal->command_option);
		if (temp < 1) {
			state_gpsdo->gpsdo_status_format_csv = false;
		} else {
			state_gpsdo->gpsdo_status_format_csv = true;
		}
	}
}

void command_DisableFrequencyCorrection(TERMINAL_t *terminal)
{
	if (terminal->command_option[0] == TERMINAL_SPACE) {
		int temp = atoi(terminal->command_option);
		if (temp < 1) {
			state_gpsdo->disable_frequency_correction = false;
		} else {
			state_gpsdo->disable_frequency_correction = true;
		}
	}
}
